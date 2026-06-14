/*
 * dsp/goertzel.c
 * Tone detection for FT8 (8 tones, 6.25 Hz) and WSPR (4 tones, 1.4648 Hz).
 * Drives Si5351 CLK0 frequency directly for FSK TX.
 *
 * STAGE 10 — enable after USB audio receive path works.
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "../config.h"
#include "../hal/si5351.h"
#include "../radio/trx.h"
#include "../radio/vfo.h"
#include "goertzel.h"

static const float ft8_tones[8]  = {1575,1581.25f,1587.5f,1593.75f,1600,1606.25f,1612.5f,1618.75f};
static const float wspr_tones[4] = {1500,1501.4648f,1502.9297f,1504.3945f};

#define GBUF (AUDIO_BUF_SAMPLES * GOERTZEL_ACCUM_FRAMES)
static int32_t gbuf[GBUF];
static int     gfill = 0;

static float goertzel_mag(const int32_t *s, int N, float freq, float fs) {
    float k     = freq / fs * N;
    float omega = 2.0f * (float)M_PI * k / N;
    float coeff = 2.0f * cosf(omega);
    float s0=0,s1=0,s2=0;
    for (int i=0;i<N;i++) {
        float x = (float)s[i] / (float)(1<<23);
        s0 = x + coeff*s1 - s2; s2=s1; s1=s0;
    }
    return s1*s1 + s2*s2 - coeff*s1*s2;
}

static float block_rms(const int32_t *buf, int n) {
    double acc=0;
    for (int i=0;i<n;i++) acc+=(double)buf[i]*(double)buf[i];
    return sqrtf((float)(acc/n));
}

void goertzel_reset(void) { gfill=0; }

int goertzel_fsk_feed(const int32_t *samples, int n, radio_mode_t mode) {
    /* Silence check — end TX if audio drops */
    if (block_rms(samples, n) < 30000.0f) {
        if (trx_state == SEQ_TX) trx_go_rx();
        goertzel_reset();
        return -1;
    }

    /* Start TX on first audio burst */
    if (trx_state != SEQ_TX) trx_go_tx();

    /* Accumulate */
    int copy = n;
    if (gfill + copy > GBUF) copy = GBUF - gfill;
    memcpy(gbuf + gfill, samples, copy * sizeof(int32_t));
    gfill += copy;
    if (gfill < GBUF) return -1;   /* not enough yet */
    gfill = 0;

    const float *tones  = (mode==MODE_WSPR) ? wspr_tones : ft8_tones;
    int          ntones = (mode==MODE_WSPR) ? 4 : 8;
    float best=-1; int best_idx=0;
    for (int t=0;t<ntones;t++) {
        float m = goertzel_mag(gbuf, GBUF, tones[t], (float)AUDIO_SAMPLE_RATE);
        if (m>best) { best=m; best_idx=t; }
    }
    if (best < 1e-6f) return -1;

    /* Shift Si5351 CLK0 by tone offset */
    float offset = (mode==MODE_WSPR) ? best_idx*1.4648f : best_idx*6.25f;
    si5351_set_freq(0, (uint64_t)((float)vfo.freq_hz + offset));

    return best_idx;
}
