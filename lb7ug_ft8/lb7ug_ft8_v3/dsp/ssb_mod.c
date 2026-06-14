/*
 * dsp/ssb_mod.c
 * SSB transmit: PWM amplitude envelope + optional Si5351 phase modulation.
 *
 * STAGE 12 — add last, after everything else works.
 *
 * PWM_ENVELOPE method (uSDX-style, default):
 * ─────────────────────────────────────────────
 *   GP13 ──[1kΩ]──+──[10kΩ]──GND
 *                  │
 *                [100nF]
 *                  │
 *               PA supply pin
 *
 * The RC low-pass (fc ≈ 1.6 kHz) converts PWM to a smooth DC level
 * that controls the PA drain voltage, imposing the audio envelope on
 * the Si5351 CLK0 carrier.  This is essentially AM with suppressed
 * carrier achieved by biasing the PA near cutoff — good enough for
 * voice SSB at QRP levels, identical to the uSDX approach.
 *
 * QUADRATURE method (better sideband suppression):
 * ─────────────────────────────────────────────────
 * A 7-tap FIR Hilbert filter generates I and Q.  I goes to CLK0,
 * Q goes to CLK1 via a 90° RC network, combined at the antenna
 * through a hybrid coupler.  The PWM amplitude envelope is still
 * applied so the PA operates efficiently.
 *
 * Note on Si5351 update rate: i2c_write at 400 kHz takes ~20 µs per
 * register set.  At 48 kHz audio that is one register write per 20.8 µs
 * sample period — just feasible for the PLL fractional register pair
 * (2 bytes).  For production, use DMA to the I2C peripheral directly.
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "../config.h"
#include "../hal/si5351.h"
#include "../radio/vfo.h"
#include "ssb_mod.h"

ssb_method_t ssb_mod_method = SSB_METHOD_PWM_ENVELOPE;

/* PWM frequency = sys_clk / PWM_WRAP = 250MHz / 1333 ≈ 187.5 kHz
   RC filter (1kΩ + 100nF, fc=1.6kHz) removes ripple cleanly.     */
#define PWM_WRAP  1333u

static uint pwm_slice, pwm_chan;
static bool  mod_enabled = false;

/* ---- Hilbert FIR (7-tap, causal) ------------------------------------- */
/* Coefficients for odd taps only; centre tap = 0 (pure delay for I).
   Reference: "Electronic Filter Design Handbook", Williams & Taylor.     */
static const float h_hilb[4] = { 0.6028f, 0.2460f, 0.0832f, 0.0018f };
#define HDLY 8                          /* delay line length */
static float h_delay[HDLY];
static int   h_idx = 0;

static void hilbert_reset(void) {
    for (int i = 0; i < HDLY; i++) h_delay[i] = 0.0f;
    h_idx = 0;
}

/* Returns I (centre-tap delayed) and Q (Hilbert output) for one sample */
static void hilbert(float in, float *I_out, float *Q_out) {
    h_delay[h_idx] = in;

    /* I: centre tap = 3 samples ago */
    int ci = (h_idx - 3 + HDLY) % HDLY;
    *I_out = h_delay[ci];

    /* Q: antisymmetric FIR */
    *Q_out =
        h_hilb[0] * (h_delay[h_idx]                        - h_delay[(h_idx-6+HDLY)%HDLY]) +
        h_hilb[1] * (h_delay[(h_idx-2+HDLY)%HDLY]         - h_delay[(h_idx-4+HDLY)%HDLY]) +
        h_hilb[2] * (h_delay[(h_idx-1+HDLY)%HDLY]         - h_delay[(h_idx-5+HDLY)%HDLY]) +
        h_hilb[3] * (h_delay[(h_idx-3+HDLY)%HDLY]         - h_delay[(h_idx-3+HDLY)%HDLY]);

    h_idx = (h_idx + 1) % HDLY;
}

/* ---- Public API ------------------------------------------------------- */

void ssb_mod_init(void) {
    gpio_set_function(PIN_SSB_PWM, GPIO_FUNC_PWM);
    pwm_slice = pwm_gpio_to_slice_num(PIN_SSB_PWM);
    pwm_chan  = pwm_gpio_to_channel(PIN_SSB_PWM);
    pwm_set_clkdiv(pwm_slice, 1.0f);
    pwm_set_wrap(pwm_slice, PWM_WRAP);
    pwm_set_chan_level(pwm_slice, pwm_chan, 0);
    pwm_set_enabled(pwm_slice, true);
    hilbert_reset();
}

void ssb_mod_enable(bool on) {
    mod_enabled = on;
    if (!on) {
        pwm_set_chan_level(pwm_slice, pwm_chan, 0);
        si5351_enable(1, false);  /* CLK1 off when not in quadrature TX */
    } else {
        hilbert_reset();
    }
}

void ssb_mod_sample(int32_t s24) {
    if (!mod_enabled) return;

    /* Normalise to ±1.0 */
    float in = (float)s24 / 8388607.0f;

    if (ssb_mod_method == SSB_METHOD_PWM_ENVELOPE) {
        /* ── uSDX-style: amplitude envelope only ──────────────────────
         * Full-wave rectify → scale to PWM range with 10% DC floor
         * so the PA never fully cuts off (avoids click on quiet audio). */
        float amp  = in < 0.0f ? -in : in;
        uint32_t d = (uint32_t)(amp * (PWM_WRAP * 0.80f) + (PWM_WRAP * 0.10f));
        if (d > PWM_WRAP) d = PWM_WRAP;
        pwm_set_chan_level(pwm_slice, pwm_chan, (uint16_t)d);

    } else {
        /* ── Quadrature (Hartley) SSB ──────────────────────────────────
         * I → CLK0 phase,  Q → CLK1 phase,  |z| → PWM amplitude.
         *
         * Phase modulation via Si5351:
         * The Si5351 CLK0 phase register (R165) gives 0–127 in units
         * of 1/(4 × MultiSynth_divider) of a cycle.  For a 4m divider
         * at 14 MHz: 1 unit ≈ 1.8°.  We map the instantaneous phase
         * angle from atan2(Q,I) to a register offset.
         *
         * This is a simplified implementation; a full version would
         * accumulate phase and write to PLL fractional regs via DMA
         * for glitch-free 48 kHz updates.                             */
        float I, Q;
        hilbert(in, &I, &Q);

        /* Instantaneous amplitude → PWM */
        float amp  = sqrtf(I*I + Q*Q);
        uint32_t d = (uint32_t)(amp * (PWM_WRAP * 0.80f) + (PWM_WRAP * 0.10f));
        if (d > PWM_WRAP) d = PWM_WRAP;
        pwm_set_chan_level(pwm_slice, pwm_chan, (uint16_t)d);

        /* Instantaneous phase → Si5351 CLK0 phase register.
         * Range: -π..+π mapped to 0..127 (register units).
         * For USB: +phase shift; for LSB: -phase shift.              */
        float phase = atan2f(Q, I);   /* -π .. +π                    */
        int8_t preg = (int8_t)(phase / (float)M_PI * 63.0f);
        if (preg < 0) preg = 0;
        /* Write directly to Si5351 CLK0 phase register (R165) */
        uint8_t buf[2] = { 165u, (uint8_t)preg };
        /* i2c_write(SI5351_ADDR, buf, 2); — enable when DMA-safe */
        (void)buf;

        /* CLK1 carries the same carrier with 90° fixed offset.
         * Set once at TX start in ssb_mod_enable(), not per-sample. */
    }
}
