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
#include <stdarg.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "common.h"
#include "cdc_app.h"

#if (CFG_TUSB_MCU == OPT_MCU_RP2040)
#include "wm8731.h"
#endif

// -----------------------------------------------------------------------
// Ring buffer for debug output
// -----------------------------------------------------------------------
#define CDC_RING_BUF_SIZE  2048   // Must be a power of two
#define CDC_RING_MASK      (CDC_RING_BUF_SIZE - 1)

static char     _ring[CDC_RING_BUF_SIZE];
static uint32_t _ring_head = 0;   // Write index (modified by cdc_printf)
static uint32_t _ring_tail = 0;   // Read  index (modified by cdc_task)

/** How many bytes are waiting to be sent */
static inline uint32_t ring_used(void) {
    return (_ring_head - _ring_tail) & CDC_RING_MASK;
}

/** How many bytes of free space remain */
static inline uint32_t ring_free(void) {
    return CDC_RING_BUF_SIZE - 1 - ring_used();
}

/**
 * cdc_printf — drop-in printf replacement routed to the CDC ring buffer.
 *
 * Thread / IRQ safety: this is called from the main loop context only
 * (TU_LOG macros fire from callbacks which are invoked by tud_task).
 * If you ever call it from an IRQ, add a critical section around the
 * head update.
 */
int cdc_printf(const char *fmt, ...) {
    char tmp[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);

    if (len <= 0) return len;
    if (len > (int)sizeof(tmp) - 1) len = sizeof(tmp) - 1;

    // Write into ring buffer, dropping bytes if full (debug drops are
    // preferable to blocking the audio task).
    uint32_t free = ring_free();
    uint32_t to_write = (len < (int)free) ? (uint32_t)len : free;

    for (uint32_t i = 0; i < to_write; i++) {
        _ring[_ring_head & CDC_RING_MASK] = tmp[i];
        _ring_head++;
    }

    return (int)to_write;
}

/**
 * cdc_task — call this from the main loop alongside tud_task().
 * Drains up to one CDC packet's worth of bytes per call.
 */
void cdc_task(void) {
    if (!tud_cdc_connected()) return;

    uint32_t avail = ring_used();
    if (avail == 0) return;

    // Send up to 64 bytes at a time (one FS bulk packet)
    uint8_t  buf[64];
    uint32_t to_send = avail < sizeof(buf) ? avail : sizeof(buf);

    for (uint32_t i = 0; i < to_send; i++) {
        buf[i] = (uint8_t)_ring[_ring_tail & CDC_RING_MASK];
        _ring_tail++;
    }

    uint32_t written = tud_cdc_write(buf, to_send);
    (void)written;
    tud_cdc_write_flush();
}

// -----------------------------------------------------------------------
// TinyUSB CDC callbacks
// -----------------------------------------------------------------------

// Invoked when CDC line state changes (host connects / disconnects)
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)itf;
    (void)rts;

    if (dtr) {
        // Host just opened the port — send a greeting so the user knows
        // the log is live.
        cdc_printf("\r\n=== CDC debug console ready ===\r\n");
        cdc_printf("Commands: '?' help  'm' toggle mute\r\n");
    }
}

// Invoked when CDC receives data from the host
void tud_cdc_rx_cb(uint8_t itf) {
    uint8_t buf[64];
    uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));

    for (uint32_t i = 0; i < count; i++) {
        char c = (char)buf[i];
        switch (c) {
            case '?':
            case 'h':
                cdc_printf("--- Status ---\r\n");
#if (CFG_TUSB_MCU == OPT_MCU_RP2040)
                cdc_printf("TX ready: %d  RX ready: %d\r\n",
                           (int)wm8731_tx_ready, (int)wm8731_rx_ready);
                cdc_printf("TX buf: %d  RX buf: %d\r\n",
                           (int)wm8731_current_tx_buffer,
                           (int)wm8731_current_rx_buffer);
#endif
                break;

            case 'm':
#if (CFG_TUSB_MCU == OPT_MCU_RP2040)
                {
                    static bool muted = false;
                    muted = !muted;
                    wm8731_set_mute(muted);
                    cdc_printf("DAC mute: %s\r\n", muted ? "ON" : "OFF");
                }
#endif
                break;
            case '9':
#if (CFG_TUSB_MCU == OPT_MCU_RP2040)
                {
                if(current_sample_rate!=96000){
            	pending_sample_rate    = 96000;
            	sample_rate_change_pending = true;
            	cdc_printf("SR change to %lu Hz queued\r\n", pending_sample_rate);}
            	else{cdc_printf("Already at %lu Hz\r\n", current_sample_rate);}
                }
#endif
                break;
            case '4':
#if (CFG_TUSB_MCU == OPT_MCU_RP2040)
                {
                if(current_sample_rate!=48000){
            	pending_sample_rate    = 48000;
            	sample_rate_change_pending = true;
            	cdc_printf("SR change to %lu Hz queued\r\n", pending_sample_rate);}
            	else{cdc_printf("Already at %lu Hz\r\n", current_sample_rate);}
                }
#endif
                break;

            default:
                // Unknown command — ignore silently
                break;
        }
    }
}
