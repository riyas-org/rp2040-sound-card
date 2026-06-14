#pragma once
#include <stdint.h>
#include "../radio/modes.h"   /* radio_mode_t */

/* Feed one 48-sample ADC block (mono, 24-bit) from USB OUT.
   Accumulates GOERTZEL_ACCUM_FRAMES blocks then decides tone.
   Returns dominant tone index (0-7 for FT8, 0-3 for WSPR),
   or -1 if below noise floor / not enough frames yet.
   Also drives Si5351 CLK0 shift and PTT via trx_go_tx/rx.   */
int  goertzel_fsk_feed(const int32_t *samples, int n, radio_mode_t mode);

/* Reset accumulator (call on mode change or RX) */
void goertzel_reset(void);
