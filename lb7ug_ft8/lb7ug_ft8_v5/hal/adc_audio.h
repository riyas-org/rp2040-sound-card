#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* Initialise ADC + DMA, start continuous 192kHz capture.
   Call from Core 1 only (DMA IRQ handler is registered there). */
void adc_audio_init(void);

/* True when a fresh AUDIO_BUF_SAMPLES decimated buffer is ready.
   Read adc_audio_buf[adc_audio_idx] then call adc_audio_consume(). */
extern volatile bool    adc_audio_ready;
extern volatile uint8_t adc_audio_idx;   /* index of ready buffer: 0 or 1 */

/* 24-bit signed samples, AUDIO_BUF_SAMPLES per frame, ping-pong.
   Written by Core 1 DMA ISR, read by Core 0 audio_task().
   Always snapshot via memcpy into a local buffer before calling
   adc_audio_consume() — do NOT read after clearing the flag. */
extern int32_t adc_audio_buf[2][AUDIO_BUF_SAMPLES];

/* Clear the ready flag after you have FINISHED copying the buffer.
   The memory barrier ensures Core 1 sees the flag cleared only after
   Core 0's reads are complete. */
static inline void adc_audio_consume(void) {
    __dmb();               /* data memory barrier — ARM required for SMP */
    adc_audio_ready = false;
}
