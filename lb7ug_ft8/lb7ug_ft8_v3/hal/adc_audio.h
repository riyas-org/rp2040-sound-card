#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Initialise ADC + DMA, start continuous 192kHz capture.
   Call from Core 1 only.                                     */
void adc_audio_init(void);

/* True when a fresh 48-sample decimated buffer is ready.
   Read adc_audio_buf[] then call adc_audio_consume().        */
extern volatile bool    adc_audio_ready;
extern volatile uint8_t adc_audio_idx;   /* index of ready buffer 0 or 1  */

/* 24-bit signed samples, 48 per frame, ping-pong             */
extern int32_t adc_audio_buf[2][48];

/* Clear the ready flag after you have consumed the buffer     */
static inline void adc_audio_consume(void) { adc_audio_ready = false; }
