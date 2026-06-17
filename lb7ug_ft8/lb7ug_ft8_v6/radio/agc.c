/*
 * radio/agc.c  — software AGC for ADC audio before USB upload.
 * STAGE 9 — enable alongside adc_audio.
 */
#include <stdint.h>
#include "agc.h"

#define AGC_TARGET   838860L   /* -20 dBFS in 24-bit scale */
#define AGC_MAX      32L
#define AGC_MIN      1L

static int32_t gain = 8;

void agc_process(int32_t *buf, int n) {
    int32_t peak = 0;
    for (int i=0;i<n;i++) {
        int32_t a = buf[i] < 0 ? -buf[i] : buf[i];
        if (a > peak) peak = a;
    }
    if (peak > 0) {
        int32_t needed = AGC_TARGET / ((peak / gain) + 1);
        if (needed < gain) gain = needed;
    }
    if (++gain > AGC_MAX) gain = AGC_MAX;
    if (gain < AGC_MIN)   gain = AGC_MIN;

    for (int i=0;i<n;i++) {
        int32_t s = buf[i] * gain;
        if (s >  8388607) s =  8388607;
        if (s < -8388608) s = -8388608;
        buf[i] = s;
    }
}
