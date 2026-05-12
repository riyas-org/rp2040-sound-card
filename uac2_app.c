/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Jerzy Kasenberg
 * Copyright (c) 2022 Angel Molina 
 * Copyright (c) 2023 Dhiru Kholia 
 * Copyright (c) 2026 Riyas Vettukattil  
 
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
 *
 */

#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "common.h"
#include "cdc_app.h"

//#if (CFG_TUSB_MCU == OPT_MCU_RP2040) && !defined(AUDIO_LOOPBACK)
#include "wm8731.h"
//#endif
#ifndef AUDIO_LOOPBACK
static uint16_t tx_buf_pos = 0;
static uint8_t  target_tx_buf = 1; // Start by filling the second buffer
static bool codec_initialized = false;
#endif

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTOTYPES
//--------------------------------------------------------------------+


#define N_SAMPLE_RATES  TU_ARRAY_SIZE(sample_rates)

uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

int8_t  mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];
int16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];

int32_t spk_buf[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4];

// Ensure the buffer is big enough for the DMA transfer
// We use the larger of the two possible constraints

#define MIC_BUF_SIZE (WM8731_DMA_BUFFER_SIZE > (CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ / 4) ? \
                      WM8731_DMA_BUFFER_SIZE : (CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ / 4))

int32_t mic_buf[MIC_BUF_SIZE];


uint16_t spk_data_size;

const uint8_t resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {
    CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX
};
uint8_t current_resolution;
static bool streaming_active = false;
uint32_t sample_rates[] = { 48000, 48000}; 
uint32_t current_sample_rate = 48000;


#include <string.h>

/**
 * HARDWARE LOOPBACK TASK
 * This function bypasses TinyUSB and directly mirrors the ADC input 
 * to the DAC output with a bitwise inversion.
 */
void run_bit_perfect_test() {
    // 1. Check if the RX DMA has finished filling a buffer
    if (wm8731_rx_ready) {
        
        // 2. Identify the buffer that was JUST filled by the ADC
        // Because the IRQ increments the index AFTER the transfer, 
        // the valid data is in (current - 1).
        uint8_t rx_idx = (wm8731_current_rx_buffer + WM8731_DMA_NUM_BUFFERS - 1) % WM8731_DMA_NUM_BUFFERS;
        
        // 3. Identify the TX buffer that is currently idle (ready to be filled)
        // Since TX and RX are phase-locked, the buffer the DAC just finished 
        // playing is the same index as the one the ADC just finished filling.
        uint8_t tx_idx = rx_idx;

        // 4. Process the data
        for (int i = 0; i < WM8731_DMA_BUFFER_SIZE; i++) {
            int32_t sample = wm8731_rx_buffer[rx_idx][i];

            /* --- THE INVERSION --- */
            // If the audio is bit-aligned, this is clean.
            // If the audio is shifted by 1 bit, this sounds like hell.
           //wm8731_tx_buffer[tx_idx][i] = ~sample;
           // wm8731_tx_buffer[tx_idx][i] = sample;
            wm8731_tx_buffer[tx_idx][i] = sample << 8;
        }

        // 5. Clear the ready flag so we wait for the next DMA cycle
        wm8731_rx_ready = false;
    }
}

// Call this from main loop, not from CDC handler directly —
// tud_disconnect() must not be called from an interrupt context
void audio_apply_pending_rate_change(void) {
    if (!sample_rate_change_pending) return;
    sample_rate_change_pending = false;

    uint32_t new_rate = pending_sample_rate;
    cdc_printf("Switching to %lu Hz — re-enumerating USB\r\n", new_rate);

    // Stop audio cleanly before touching the codec
    if (codec_initialized) {
        wm8731_stop_dma();
        streaming_active = false;
    }

    // Reconfigure codec and I2S clock
    current_sample_rate = new_rate;
    if (codec_initialized) {
        wm8731_configure(new_rate);
        wm8731_set_i2s_samplerate(new_rate);
    }

    // Force host to re-enumerate so it learns the new clock rate
    // and recalculates its isochronous packet size
    tud_disconnect();
    sleep_ms(200);
    tud_connect();
}


//--------------------------------------------------------------------+
// LOOPBACK FIFO
//
// USB OUT and IN endpoints are serviced at slightly different times in
// the main loop. Without a FIFO the mic write would either get stale
// data or nothing. This ring buffer decouples them cleanly.
//
// Size: 8 * max_endpoint_packet. At 48kHz 16-bit stereo the max
// isochronous packet is 192 bytes, so 8192 bytes is ample headroom.
//--------------------------------------------------------------------+

#ifdef AUDIO_LOOPBACK

#define LB_FIFO_SIZE  8192   // must be a power of two
#define LB_FIFO_MASK  (LB_FIFO_SIZE - 1)

static uint8_t  lb_fifo[LB_FIFO_SIZE];
static uint32_t lb_head = 0;
static uint32_t lb_tail = 0;

static inline uint32_t lb_used(void) { return (lb_head - lb_tail) & LB_FIFO_MASK; }
static inline uint32_t lb_free(void) { return LB_FIFO_SIZE - 1 - lb_used(); }

static void lb_write(const uint8_t *src, uint32_t len) {
    uint32_t f = lb_free();
    if (len > f) {
        // Drop oldest bytes so we always send fresh samples.
        lb_tail = (lb_tail + (len - f)) & LB_FIFO_MASK;
        cdc_printf("LB overrun: dropped %u bytes\r\n", len - f);
    }
    for (uint32_t i = 0; i < len; i++) {
        lb_fifo[lb_head & LB_FIFO_MASK] = src[i];
        lb_head++;
    }
}

static uint32_t lb_read(uint8_t *dst, uint32_t maxlen) {
    uint32_t n = lb_used();
    if (n > maxlen) n = maxlen;
    for (uint32_t i = 0; i < n; i++) {
        dst[i] = lb_fifo[lb_tail & LB_FIFO_MASK];
        lb_tail++;
    }
    return n;
}

#endif /* AUDIO_LOOPBACK */

//--------------------------------------------------------------------+
// Codec state (only used in normal mode)
//--------------------------------------------------------------------+





//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+

void audio_task(void) {
#ifndef AUDIO_LOOPBACK
    if (!codec_initialized || !streaming_active) {
        static uint32_t last_print = 0;
        if (tusb_time_millis_api() - last_print > 2000) {
            cdc_printf("Wait: CodecInit=%d, StreamActive=%d\n", codec_initialized, streaming_active);
            last_print = tusb_time_millis_api();
        }
        return;
    }

    // ----------------------------------------------------------------
    // 1. SPEAKER PATH (USB -> Codec)
    // ----------------------------------------------------------------
    
    // Check if the DMA IRQ just swapped buffers
    if (wm8731_tx_ready) {
        wm8731_tx_ready = false; 
        tx_buf_pos = 0;
        // The buffer we were filling (target_tx_buf) is now being played by DMA.
        // We must now start filling the OTHER buffer.
        target_tx_buf = (wm8731_current_tx_buffer + 1) % WM8731_DMA_NUM_BUFFERS;
    }

    // If our current target buffer isn't full yet, try to read from USB
    if (tx_buf_pos < WM8731_DMA_BUFFER_SIZE) {
        // Calculate how much space is left in the current DMA buffer (in bytes)
        uint32_t bytes_needed = (WM8731_DMA_BUFFER_SIZE - tx_buf_pos) * 4;
        uint32_t to_read = (bytes_needed < sizeof(spk_buf)) ? bytes_needed : sizeof(spk_buf);

        uint32_t n = tud_audio_read(spk_buf, to_read);
        if (n > 0) {
            int32_t *usb_samples = (int32_t *)spk_buf;
            uint32_t num_pairs = n / 8;
            
            for (uint32_t i = 0; i < num_pairs; i++) {
                int32_t left  = usb_samples[i * 2] ;
                int32_t right = usb_samples[i * 2 + 1] ;
                
                // Pack into the 32-bit DMA buffer word
                wm8731_tx_buffer[target_tx_buf][tx_buf_pos++] = left;
                wm8731_tx_buffer[target_tx_buf][tx_buf_pos++] = right;
					
            }
        }
    }
// 2. MICROPHONE PATH (Codec -> USB)
// Fix for uac2_app.c -> Microphone Path
if (wm8731_rx_ready) {
    uint8_t source_buf = (wm8731_current_rx_buffer + WM8731_DMA_NUM_BUFFERS - 1) % WM8731_DMA_NUM_BUFFERS;
    int32_t *codec_samples = (int32_t *)wm8731_rx_buffer[source_buf];
    int32_t *usb_mic_data = (int32_t *)mic_buf;

    // Correctly pass through both Left and Right channels for true Stereo
    for (uint16_t i = 0; i < WM8731_DMA_BUFFER_SIZE; i++) {
        usb_mic_data[i] = codec_samples[i]; 
    } 
    
    /*for (uint16_t i = 0; i < WM8731_DMA_BUFFER_SIZE; i += 2) {
    // Manually shift the noisy sample left by 1 bit
    // This often "fixes" the click if the PIO missed the first bit
    usb_mic_data[i]     = codec_samples[i] << 1; 
    usb_mic_data[i + 1] = codec_samples[i + 1]; 
	}   */
   
    /*for (uint16_t i = 0; i < WM8731_DMA_BUFFER_SIZE; i += 2) {
    int32_t left_sample = codec_samples[i+1];
    usb_mic_data[i]     = left_sample; // USB Left
    usb_mic_data[i + 1] = left_sample; // USB Right (Mirroring Left)
	}*/

    uint32_t bytes_total = WM8731_DMA_BUFFER_SIZE * 4;
    uint32_t chunk = (current_sample_rate / 1000) * 2 * 4; // 1 ms stereo (384 bytes @ 48k)
    
    for (uint32_t off = 0; off < bytes_total; off += chunk) {
        tud_audio_write((uint8_t *)mic_buf + off, TU_MIN(chunk, bytes_total - off));
    }
    
    wm8731_rx_ready = false;
}

#else
    // ----------------------------------------------------------------
    // Loopback: USB OUT -> FIFO -> USB IN, no codec.
    // ----------------------------------------------------------------
if (!streaming_active) return;

// 1. Read from Speaker (USB Out)
spk_data_size = tud_audio_read(spk_buf, sizeof(spk_buf));
if (spk_data_size > 0) {
    lb_write((const uint8_t *)spk_buf, spk_data_size);
    // Removed cdc_printf here to prevent timing jitter at higher bandwidths
}

// 2. Setup 24-bit/32-bit constraints
// In UAC2, 24-bit is typically 4 bytes per sample (left-justified).
// Stereo pair = 2 samples * 4 bytes = 8 bytes.
const uint32_t bytes_per_pair = 8; 

// 3. Forward to Microphone (USB In)
if (lb_used() >= bytes_per_pair) {
    // IMPORTANT: Do not read the entire mic_buf if it's very large.
    // Read only what TinyUSB can actually send in one go (e.g., 1ms of audio).
    // For 48kHz, 24-bit stereo in 32-bit slots: 48 * 8 = 384 bytes per ms.
    uint32_t max_transfer = 384; 
    uint32_t to_read = lb_used();
    
    if (to_read > max_transfer) to_read = max_transfer;
    if (to_read > sizeof(mic_buf)) to_read = sizeof(mic_buf);

    // Ensure we always read a whole number of L/R pairs
    to_read = (to_read / bytes_per_pair) * bytes_per_pair;

    if (to_read > 0) {
        uint32_t n = lb_read((uint8_t *)mic_buf, to_read);
        
        // Only write if the TinyUSB FIFO has space
        // This prevents 'fast' audio by syncing to the USB host's pull rate
        tud_audio_write((uint8_t *)mic_buf, n);
    }
}   
#endif
}

//--------------------------------------------------------------------+
// Codec Initialization
//--------------------------------------------------------------------+

void audio_init_codec(void) {
#ifdef AUDIO_LOOPBACK
    cdc_printf("*** AUDIO_LOOPBACK active — codec and I2S bypassed ***\r\n");
    cdc_printf("Play to speaker device, record from mic device to verify USB audio.\r\n");

#elif (CFG_TUSB_MCU == OPT_MCU_RP2040)
    if (codec_initialized) return;

    cdc_printf("audio_init_codec: setting up I2S PIO/DMA...\r\n");
    wm8731_i2s_init(current_sample_rate);
    cdc_printf("I2S PIO/DMA ready\r\n");

    if (wm8731_init()) {
        if (wm8731_configure(current_sample_rate)) {
            wm8731_set_volume(79);
            cdc_printf("WM8731 configured at %" PRIu32 " Hz\r\n", current_sample_rate);
        } else {
            cdc_printf("WARNING: wm8731_configure() failed\r\n");
        }
    codec_initialized = true;
    cdc_printf("audio_init_codec: done\r\n");
    } else {
        cdc_printf("WARNING: wm8731_init() failed - check I2C address/wiring\r\n");
    }

#endif
}



//--------------------------------------------------------------------+
// Volume / mute helpers — Windows compatibility
//
// UAC2 Feature Unit channels:
//   0 = master,  1 = left,  2 = right
//
// Linux mixer sends SET on ch 0 (master).
// Windows sends SET on ch 1 and ch 2 independently.
//
// Strategy: always keep the volume[] / mute[] arrays up to date, then
// derive a single codec setting from them after every update.
//--------------------------------------------------------------------+

#if (CFG_TUSB_MCU == OPT_MCU_RP2040) && !defined(AUDIO_LOOPBACK)

/**
 * Convert the UAC2 volume array to a single WM8731 headphone register
 * value (0-127) and write it.
 *
 * Priority: if master (ch0) != 0, use it; otherwise average ch1 & ch2.
 * UAC2 volume is Q8.8 signed, 0x0000 = 0 dB, negative = attenuation.
 * The codec range we advertise is [−VOLUME_CTRL_50_DB .. 0] mapped
 * onto WM8731 codes [48 .. 127].
 */
static void apply_volume_to_codec(void) {
    int16_t vol_db;   // Q8.8

    if (volume[0] != 0) {
        vol_db = volume[0];
    } else {
        // Average left and right; guard divide-by-zero if array too small
        int32_t sum = 0;
        int     cnt = 0;
        for (int ch = 1; ch <= CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX; ch++) {
            sum += volume[ch];
            cnt++;
        }
        vol_db = (cnt > 0) ? (int16_t)(sum / cnt) : 0;
    }

    uint8_t codec_vol;
    if      (vol_db <= -VOLUME_CTRL_50_DB) codec_vol = 48;
    else if (vol_db >= 0)                  codec_vol = 127;
    else    codec_vol = (uint8_t)(48 + ((int32_t)(vol_db + VOLUME_CTRL_50_DB) * 79)
                                        / VOLUME_CTRL_50_DB);
    wm8731_set_volume(codec_vol);
    cdc_printf("Codec vol: %u/127 (UAC %d dBx256)\r\n", codec_vol, (int)vol_db);
}

/**
 * Derive a single mute state for the codec:
 * muted if master (ch0) OR any per-channel flag is set.
 */
static void apply_mute_to_codec(void) {
    bool any_muted = (mute[0] != 0);
    for (int ch = 1; ch <= CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX && !any_muted; ch++) {
        if (mute[ch]) any_muted = true;
    }
    wm8731_set_mute(any_muted);
    cdc_printf("Codec mute: %s\r\n", any_muted ? "ON" : "off");
}

#endif  /* RP2040 && !AUDIO_LOOPBACK */




//--------------------------------------------------------------------+
// CLOCK ENTITY CONTROL HANDLERS
//--------------------------------------------------------------------+

static bool tud_audio_clock_get_request(uint8_t rhport, audio20_control_request_t const *request)
{
  TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);
  TU_LOG2("Clock GET: Sel=%u Req=%u\r\n", request->bControlSelector, request->bRequest);

  if (request->bControlSelector == AUDIO20_CS_CTRL_SAM_FREQ)
  {
    if (request->bRequest == AUDIO20_CS_REQ_CUR)
    {
      cdc_printf("Clock GET CUR freq: %" PRIu32 "\r\n", current_sample_rate);
      audio20_control_cur_4_t curf = { .bCur = (int32_t) tu_htole32(current_sample_rate) };
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &curf, sizeof(curf));
    }
    else if (request->bRequest == AUDIO20_CS_REQ_RANGE)
    {
      audio20_control_range_4_n_t(N_SAMPLE_RATES) rangef;
      rangef.wNumSubRanges = tu_htole16(N_SAMPLE_RATES);
      cdc_printf("Clock GET RANGE: %d rates\r\n", N_SAMPLE_RATES);
      for (uint8_t i = 0; i < N_SAMPLE_RATES; i++) {
        rangef.subrange[i].bMin = (int32_t) tu_htole32(sample_rates[i]);
        rangef.subrange[i].bMax = (int32_t) tu_htole32(sample_rates[i]);
        rangef.subrange[i].bRes = 0;
        cdc_printf("  %u Hz\r\n", sample_rates[i]);
      }
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &rangef, sizeof(rangef));
    }
  }
  else if (request->bControlSelector == AUDIO20_CS_CTRL_CLK_VALID &&
           request->bRequest == AUDIO20_CS_REQ_CUR)
  {
    audio20_control_cur_1_t cur_valid = { .bCur = 1 };
    return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_valid, sizeof(cur_valid));
  }

  cdc_printf("Clock GET UNSUPPORTED: Ent=%u Sel=%u Req=%u\r\n",
          request->bEntityID, request->bControlSelector, request->bRequest);
  return false;
}

static bool tud_audio_clock_set_request(uint8_t rhport, audio20_control_request_t const *request, uint8_t const *buf)
{
  (void)rhport;
  TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);
  TU_VERIFY(request->bRequest == AUDIO20_CS_REQ_CUR);

  if (request->bControlSelector == AUDIO20_CS_CTRL_SAM_FREQ)
  {
    TU_VERIFY(request->wLength == sizeof(audio20_control_cur_4_t));
    current_sample_rate = (uint32_t) tu_le32toh(((audio20_control_cur_4_t const *)buf)->bCur);
    cdc_printf("Clock SET CUR freq: %" PRIu32 "\r\n", current_sample_rate);
#if (CFG_TUSB_MCU == OPT_MCU_RP2040) && !defined(AUDIO_LOOPBACK)
    if (codec_initialized) {
        wm8731_configure(current_sample_rate);
        wm8731_set_i2s_samplerate(current_sample_rate); 
        cdc_printf("WM8731 reconfigured for %" PRIu32 " Hz\r\n", current_sample_rate);
    }
#endif
    return true;
  }

  cdc_printf("Clock SET UNSUPPORTED: Ent=%u Sel=%u\r\n",
          request->bEntityID, request->bControlSelector);
  return false;
}

//--------------------------------------------------------------------+
// FEATURE UNIT CONTROL HANDLERS
//--------------------------------------------------------------------+

static bool tud_audio_feature_unit_get_request(uint8_t rhport, audio20_control_request_t const *request)
{
  TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);

  if (request->bControlSelector == AUDIO20_FU_CTRL_MUTE &&
      request->bRequest == AUDIO20_CS_REQ_CUR)
  {
    audio20_control_cur_1_t m = { .bCur = mute[request->bChannelNumber] };
    cdc_printf("FU GET MUTE Ch=%u val=%d\r\n", request->bChannelNumber, m.bCur);
    return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &m, sizeof(m));
  }
  else if (request->bControlSelector == AUDIO20_FU_CTRL_VOLUME)
  {
    if (request->bRequest == AUDIO20_CS_REQ_RANGE)
    {
      audio20_control_range_2_n_t(1) range_vol = { .wNumSubRanges = tu_htole16(1) };
      range_vol.subrange[0].bMin = tu_htole16(-VOLUME_CTRL_50_DB);
      range_vol.subrange[0].bMax = tu_htole16(VOLUME_CTRL_0_DB);
      range_vol.subrange[0].bRes = tu_htole16(256);
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &range_vol, sizeof(range_vol));
    }
    else if (request->bRequest == AUDIO20_CS_REQ_CUR)
    {
      audio20_control_cur_2_t cur_vol = { .bCur = tu_htole16(volume[request->bChannelNumber]) };
      cdc_printf("FU GET VOLUME Ch=%u val=%d dB\r\n",
              request->bChannelNumber,
              (int16_t)tu_le16toh(cur_vol.bCur) / 256);
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_vol, sizeof(cur_vol));
    }
  }

  cdc_printf("FU GET UNSUPPORTED: Ch=%u Sel=%u Req=%u\r\n",
          request->bChannelNumber, request->bControlSelector, request->bRequest);
  return false;
}

static bool tud_audio_feature_unit_set_request(uint8_t rhport,
                                               audio20_control_request_t const *request,
                                               uint8_t const *buf)
{
    (void)rhport;
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);
    TU_VERIFY(request->bRequest == AUDIO20_CS_REQ_CUR);

    uint8_t ch = request->bChannelNumber;

    if (request->bControlSelector == AUDIO20_FU_CTRL_MUTE) {
        TU_VERIFY(request->wLength == sizeof(audio20_control_cur_1_t));
        /* Guard: don't write past the mute[] array */
        if (ch <= CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX) {
            mute[ch] = ((audio20_control_cur_1_t const *)buf)->bCur;
            cdc_printf("FU SET MUTE Ch=%d val=%d\r\n", ch, mute[ch]);
        }
#if (CFG_TUSB_MCU == OPT_MCU_RP2040) && !defined(AUDIO_LOOPBACK)
        /*
         * Windows fix: apply mute regardless of which channel changed.
         * apply_mute_to_codec() checks all channels.
         */
        if (codec_initialized) apply_mute_to_codec();
#endif
        return true;
    } else if (request->bControlSelector == AUDIO20_FU_CTRL_VOLUME) {
        TU_VERIFY(request->wLength == sizeof(audio20_control_cur_2_t));
        if (ch <= CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX) {
            volume[ch] = tu_le16toh(((audio20_control_cur_2_t const *)buf)->bCur);
            cdc_printf("FU SET VOLUME Ch=%d val=%d dB*256\r\n", ch, volume[ch]);
        }
#if (CFG_TUSB_MCU == OPT_MCU_RP2040) && !defined(AUDIO_LOOPBACK)
        /*
         * Windows fix: apply volume whenever ANY channel is updated.
         * apply_volume_to_codec() derives a single codec value from all
         * channels, so both the Linux master path and the Windows
         * per-channel path produce a codec update.
         */
        if (codec_initialized) apply_volume_to_codec();
#endif
        return true;
    }

    cdc_printf("FU SET UNSUPPORTED: Ch=%u Sel=%u\r\n", ch, request->bControlSelector);
    return false;
}


//--------------------------------------------------------------------+
// Application Callback API
//--------------------------------------------------------------------+

bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  audio20_control_request_t const *request = (audio20_control_request_t const *)p_request;
  if (request->bEntityID == UAC2_ENTITY_CLOCK)            return tud_audio_clock_get_request(rhport, request);
  if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT) return tud_audio_feature_unit_get_request(rhport, request);
  cdc_printf("Audio GET unknown entity %d\r\n", request->bEntityID);
  return false;
}

bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf) {
  audio20_control_request_t const *request = (audio20_control_request_t const *)p_request;
  if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT) return tud_audio_feature_unit_set_request(rhport, request, buf);
  if (request->bEntityID == UAC2_ENTITY_CLOCK)            return tud_audio_clock_set_request(rhport, request, buf);
  cdc_printf("Audio SET unknown entity %d\r\n", request->bEntityID);
  return false;
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
  (void)rhport;
  uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
  uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

  if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt == 0) {
    blink_interval_ms = BLINK_MOUNTED;
    streaming_active  = false;
    cdc_printf("Audio streaming stopped\r\n");
#if (CFG_TUSB_MCU == OPT_MCU_RP2040) && !defined(AUDIO_LOOPBACK)
    if (codec_initialized) wm8731_stop_dma();
#endif
  }
  return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
  (void)rhport;
  uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
  uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

  TU_LOG2("Audio SET Interface: itf=%u alt=%u\r\n", itf, alt);

  if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt != 0) {
    blink_interval_ms = BLINK_STREAMING;
    streaming_active  = true;

#ifdef AUDIO_LOOPBACK
    lb_head = lb_tail = 0;  // flush stale data on each stream open
    cdc_printf("Audio streaming started (LOOPBACK alt=%u)\r\n", alt);
#else
    cdc_printf("Audio streaming started (alt=%u)\r\n", alt);
  #if (CFG_TUSB_MCU == OPT_MCU_RP2040)
    if (codec_initialized) wm8731_start_dma();    
  #endif
#endif
  }

  if (alt != 0) {
    current_resolution = resolutions_per_format[alt - 1];
    TU_LOG2("Resolution: %u bits\r\n", current_resolution);
  }

  return true;
}

//--------------------------------------------------------------------+
// LED Blinking Task
//--------------------------------------------------------------------+

void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  if (tusb_time_millis_api() - start_ms < blink_interval_ms) return;
  start_ms += blink_interval_ms;

  ws2812_led_write(led_state);
  led_state = 1 - led_state;
}
