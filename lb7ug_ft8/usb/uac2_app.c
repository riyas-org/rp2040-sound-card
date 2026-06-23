/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Jerzy Kasenberg
 * Copyright (c) 2022 Angel Molina 
 * Copyright (c) 2023 Dhiru Kholia 
 * Copyright (c) 2026 Riyas Vettukattil 
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
 
#if UAC2_DEBUG
#define UAC_LOG(...) printf(__VA_ARGS__)
#else
#define UAC_LOG(...) do{}while(0)
#endif 

#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "common.h"
#include "cdc_app.h"
#include "uac2_app.h"

// ========================================================================
// NEW: transceiver headers (replace wm8731.h)
// ========================================================================
#include "config.h"
#include "hal/adc_audio.h"
#include "radio/vfo.h"
#include "radio/trx.h"
#include "radio/agc.h"
#include "dsp/goertzel.h"
#include "dsp/ssb_mod.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTOTYPES
//--------------------------------------------------------------------+

#define N_SAMPLE_RATES                          TU_ARRAY_SIZE(sample_rates)
#define MIC_BUF_SIZE                            (AUDIO_BUF_SAMPLES * 2)  // stereo

//--------------------------------------------------------------------+
// GLOBAL VARIABLES (kept from working version)
//--------------------------------------------------------------------+

static bool     codec_initialized = false;   // will be set when transceiver hardware is ready

uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;
int8_t   mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];
int16_t  volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];
int32_t  spk_buf[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4];
int32_t  mic_buf[MIC_BUF_SIZE];
uint16_t spk_data_size;

const uint8_t resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {
    CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX
};

uint8_t current_resolution;
static bool streaming_spk_active = false;
static bool streaming_mic_active = false;
uint32_t sample_rates[] = {48000}; 
uint32_t current_sample_rate = 48000;

// ------------------------------------------------------------------------
// NEW: transceiver state (replaces wm8731_* globals)
// ------------------------------------------------------------------------
static bool transceiver_audio_ready = false;

// ------------------------------------------------------------------------
// AUDIO RATE CHANGE HANDLER (kept from working version, but without wm8731)
// ------------------------------------------------------------------------



void uac2_set_codec_ready(bool ready) {
    codec_initialized = ready;
}

void audio_apply_pending_rate_change(void) {
    if (!sample_rate_change_pending) return;
    sample_rate_change_pending = false;

    uint32_t new_rate = pending_sample_rate;
    UAC_LOG("Switching to %lu Hz — re-enumerating USB\r\n", new_rate);

    // Stop streaming cleanly before touching hardware
    if (codec_initialized) {
        streaming_spk_active = false;
        streaming_mic_active = false;
    }

    // Update sample rate (transceiver hardware uses fixed 48kHz, so no-op here)
    current_sample_rate = new_rate;

    // Force host to re-enumerate so it learns the new clock rate
    tud_disconnect();
    sleep_ms(200);
    tud_connect();
}

//--------------------------------------------------------------------+
// NEW AUDIO TASK (replaces WM8731 audio_task)
// Called from main loop – handles USB <-> transceiver data flow
//--------------------------------------------------------------------+

void audio_task(void) {
    if (!codec_initialized) return;

    // -----------------------------------------------------------------
    // 1. SPEAKER PATH (USB OUT -> radio) – FT8/WSPR/SSB TX
    // -----------------------------------------------------------------
    if (streaming_spk_active) {
        uint32_t n = tud_audio_read(spk_buf, sizeof(spk_buf));
        if (n > 0) {
            // Unpack 2-channel 24-in-32 to mono 24-bit
            int samples = n / 8;  // 2ch × 4 bytes
            if (samples > AUDIO_BUF_SAMPLES) samples = AUDIO_BUF_SAMPLES;
            
            static int32_t mono[AUDIO_BUF_SAMPLES];
            for (int i = 0; i < samples; i++) {
                mono[i] = spk_buf[i * 2] >> 8;   // left channel, unpack 24-bit
            }

            radio_mode_t mode = vfo.mode;

            if (mode == MODE_FT8 || mode == MODE_WSPR) {
#ifdef FEATURE_FT8_WSPR
                goertzel_fsk_feed(mono, samples, mode);
#endif
            } else if (mode == MODE_USB || mode == MODE_LSB) {
#ifdef FEATURE_SSB_TX
                for (int i = 0; i < samples; i++)
                    ssb_mod_sample(mono[i]);
#endif
            }
        }
    }

    // -----------------------------------------------------------------
    // 2. MICROPHONE PATH (radio -> USB IN) – receive audio to PC
    // -----------------------------------------------------------------
    if (streaming_mic_active) {
        if (adc_audio_ready && trx_state == SEQ_RX) {
            /* Snapshot which buffer index is ready BEFORE clearing the flag.
               Core 1 DMA ISR may update adc_audio_idx at any time. */
            uint8_t idx = adc_audio_idx;

            /* Copy FIRST, then consume. If we cleared the flag first the ISR
               could refill the buffer before our memcpy completes. */
            static int32_t mono[AUDIO_BUF_SAMPLES];
            memcpy(mono, adc_audio_buf[idx], AUDIO_BUF_SAMPLES * sizeof(int32_t));

            /* Now safe to release the buffer back to the ISR. */
            adc_audio_consume();

            // Apply digital AGC if enabled
            if (vfo.agc_on) agc_process(mono, AUDIO_BUF_SAMPLES);

            // Pack into 32-bit container (24-bit left-justified) + stereo
            int32_t stereo[AUDIO_BUF_SAMPLES * 2];
            for (int i = 0; i < AUDIO_BUF_SAMPLES; i++) {
                int32_t packed = mono[i] << 8;
                stereo[i * 2]     = packed;
                stereo[i * 2 + 1] = packed;
            }

            tud_audio_write((uint8_t*)stereo, sizeof(stereo));
        } else {
            // No new ADC data – send silence to avoid USB underrun
            static int32_t silence[AUDIO_BUF_SAMPLES * 2] = {0};
            tud_audio_write((uint8_t*)silence, sizeof(silence));
        }
    }
}



static void apply_volume_to_codec(void) {
    // In transceiver mode, volume control is handled by the host (WSJT-X)
    // or can be implemented as digital gain in the ADC path.
    // For now, just log.
    UAC_LOG("Volume changed (ignored in transceiver mode)\r\n");
}

static void apply_mute_to_codec(void) {
    UAC_LOG("Mute changed (ignored in transceiver mode)\r\n");
}

//--------------------------------------------------------------------+
// CLOCK ENTITY CONTROL HANDLERS (kept unchanged from working version)
//--------------------------------------------------------------------+

static bool tud_audio_clock_get_request(uint8_t rhport, audio20_control_request_t const *request) {
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);   

    if (request->bControlSelector == AUDIO20_CS_CTRL_SAM_FREQ) {
        if (request->bRequest == AUDIO20_CS_REQ_CUR) {
            UAC_LOG("Clock GET CUR freq: %" PRIu32 "\r\n", current_sample_rate);
            audio20_control_cur_4_t curf = { .bCur = (int32_t) tu_htole32(current_sample_rate) };
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &curf, sizeof(curf));
        } else if (request->bRequest == AUDIO20_CS_REQ_RANGE) {
            audio20_control_range_4_n_t(N_SAMPLE_RATES) rangef;
            rangef.wNumSubRanges = tu_htole16(N_SAMPLE_RATES);
            UAC_LOG("Clock GET RANGE: %d rates\r\n", N_SAMPLE_RATES);
            for (uint8_t i = 0; i < N_SAMPLE_RATES; i++) {
                rangef.subrange[i].bMin = (int32_t) tu_htole32(sample_rates[i]);
                rangef.subrange[i].bMax = (int32_t) tu_htole32(sample_rates[i]);
                rangef.subrange[i].bRes = 0;
                UAC_LOG("  %u Hz\r\n", sample_rates[i]);
            }
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &rangef, sizeof(rangef));
        }
    } else if (request->bControlSelector == AUDIO20_CS_CTRL_CLK_VALID && request->bRequest == AUDIO20_CS_REQ_CUR) {
        audio20_control_cur_1_t cur_valid = { .bCur = 1 };
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_valid, sizeof(cur_valid));
    }

    UAC_LOG("Clock GET UNSUPPORTED: Ent=%u Sel=%u Req=%u\r\n",
               request->bEntityID, request->bControlSelector, request->bRequest);
    return false;
}

static bool tud_audio_clock_set_request(uint8_t rhport, audio20_control_request_t const *request, uint8_t const *buf) {
    (void)rhport;
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);
    TU_VERIFY(request->bRequest == AUDIO20_CS_REQ_CUR);

    if (request->bControlSelector == AUDIO20_CS_CTRL_SAM_FREQ) {
        TU_VERIFY(request->wLength == sizeof(audio20_control_cur_4_t));
        current_sample_rate = (uint32_t) tu_le32toh(((audio20_control_cur_4_t const *)buf)->bCur);
        UAC_LOG("Clock SET CUR freq: %" PRIu32 "\r\n", current_sample_rate);
        return true;
    }

    UAC_LOG("Clock SET UNSUPPORTED: Ent=%u Sel=%u\r\n",
               request->bEntityID, request->bControlSelector);
    return false;
}

//--------------------------------------------------------------------+
// FEATURE UNIT CONTROL HANDLERS (kept unchanged from working version)
//--------------------------------------------------------------------+

static bool tud_audio_feature_unit_get_request(uint8_t rhport, audio20_control_request_t const *request) {
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);

    if (request->bControlSelector == AUDIO20_FU_CTRL_MUTE && request->bRequest == AUDIO20_CS_REQ_CUR) {
        audio20_control_cur_1_t m = { .bCur = mute[request->bChannelNumber] };
        UAC_LOG("FU GET MUTE Ch=%u val=%d\r\n", request->bChannelNumber, m.bCur);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &m, sizeof(m));
    } else if (request->bControlSelector == AUDIO20_FU_CTRL_VOLUME) {
        if (request->bRequest == AUDIO20_CS_REQ_RANGE) {
            audio20_control_range_2_n_t(1) range_vol = { .wNumSubRanges = tu_htole16(1) };
            range_vol.subrange[0].bMin = tu_htole16(-VOLUME_CTRL_50_DB);
            range_vol.subrange[0].bMax = tu_htole16(VOLUME_CTRL_0_DB);
            range_vol.subrange[0].bRes = tu_htole16(256);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &range_vol, sizeof(range_vol));
        } else if (request->bRequest == AUDIO20_CS_REQ_CUR) {
            audio20_control_cur_2_t cur_vol = { .bCur = tu_htole16(volume[request->bChannelNumber]) };
            UAC_LOG("FU GET VOLUME Ch=%u val=%d dB\r\n",
                       request->bChannelNumber,
                       (int16_t)tu_le16toh(cur_vol.bCur) / 256);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_vol, sizeof(cur_vol));
        }
    }

    UAC_LOG("FU GET UNSUPPORTED: Ch=%u Sel=%u Req=%u\r\n",
               request->bChannelNumber, request->bControlSelector, request->bRequest);
    return false;
}

static bool tud_audio_feature_unit_set_request(uint8_t rhport,
                                               audio20_control_request_t const *request,
                                               uint8_t const *buf) {
    (void)rhport;
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);
    TU_VERIFY(request->bRequest == AUDIO20_CS_REQ_CUR);

    uint8_t ch = request->bChannelNumber;

    if (request->bControlSelector == AUDIO20_FU_CTRL_MUTE) {
        TU_VERIFY(request->wLength == sizeof(audio20_control_cur_1_t));
        
        if (ch <= CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX) {
            mute[ch] = ((audio20_control_cur_1_t const *)buf)->bCur;
            UAC_LOG("FU SET MUTE Ch=%d val=%d\r\n", ch, mute[ch]);
        }
        if (codec_initialized) apply_mute_to_codec();
        return true;
    } else if (request->bControlSelector == AUDIO20_FU_CTRL_VOLUME) {
        TU_VERIFY(request->wLength == sizeof(audio20_control_cur_2_t));
        
        if (ch <= CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX) {
            volume[ch] = tu_le16toh(((audio20_control_cur_2_t const *)buf)->bCur);
            UAC_LOG("FU SET VOLUME Ch=%d val=%d dB*256\r\n", ch, volume[ch]);
        }
        if (codec_initialized) apply_volume_to_codec();
        return true;
    }

    UAC_LOG("FU SET UNSUPPORTED: Ch=%u Sel=%u\r\n", ch, request->bControlSelector);
    return false;
}

//--------------------------------------------------------------------+
// Application Callback API (kept unchanged)
//--------------------------------------------------------------------+

bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    audio20_control_request_t const *request = (audio20_control_request_t const *)p_request;
    if (request->bEntityID == UAC2_ENTITY_CLOCK) {
        return tud_audio_clock_get_request(rhport, request);
    }
    if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT) {
        return tud_audio_feature_unit_get_request(rhport, request);
    }
    UAC_LOG("Audio GET unknown entity %d\r\n", request->bEntityID);
    return false;
}

bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf) {
    audio20_control_request_t const *request = (audio20_control_request_t const *)p_request;
    if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT) {
        return tud_audio_feature_unit_set_request(rhport, request, buf);
    }
    if (request->bEntityID == UAC2_ENTITY_CLOCK) {
        return tud_audio_clock_set_request(rhport, request, buf);
    }
    UAC_LOG("Audio SET unknown entity %d\r\n", request->bEntityID);
    return false;
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void)rhport;
    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

    if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt == 0) {
        streaming_spk_active = false;
        UAC_LOG("Audio streaming stopped\r\n");
        if (codec_initialized && !streaming_mic_active) {
            blink_interval_ms = BLINK_MOUNTED;
        }
    }
    
    if (ITF_NUM_AUDIO_STREAMING_MIC == itf && alt == 0) {
        streaming_mic_active = false;
        UAC_LOG("Mic streaming stopped\r\n");
        if (codec_initialized && !streaming_spk_active) {
            blink_interval_ms = BLINK_MOUNTED;
        }
    }
    
    return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void)rhport;
    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));
    
    if (ITF_NUM_AUDIO_STREAMING_SPK == itf) {
        if (alt != 0) {
            blink_interval_ms = BLINK_STREAMING;
            streaming_spk_active = true;
        } else {
            streaming_spk_active = false;
        }
        UAC_LOG("Speaker streaming %s\r\n", streaming_spk_active ? "ON" : "OFF");
    }

    if (ITF_NUM_AUDIO_STREAMING_MIC == itf) {
        if (alt != 0) {
            blink_interval_ms = BLINK_STREAMING;
            streaming_mic_active = true;
        } else {
            streaming_mic_active = false;
        }
        UAC_LOG("Mic streaming %s\r\n", streaming_mic_active ? "ON" : "OFF");
    }

    if (alt != 0) {
        current_resolution = resolutions_per_format[alt - 1];
    }

    return true;
}

//--------------------------------------------------------------------+
// LED Blinking Task (kept unchanged)
//--------------------------------------------------------------------+

void led_blinking_task(void) {
    static uint32_t start_ms = 0;
    static bool led_state = false;

    if (tusb_time_millis_api() - start_ms < blink_interval_ms) return;
    start_ms += blink_interval_ms;

    ws2812_led_write(led_state);
    led_state = 1 - led_state;
}
