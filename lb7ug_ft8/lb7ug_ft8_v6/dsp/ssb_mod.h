#pragma once
#include <stdint.h>
#include <stdbool.h>

/*
 * SSB transmit using two complementary outputs:
 *
 *   CLK0  (Si5351) — carrier phase modulation.
 *             Two quadrature outputs (CLK0 + CLK1 90° apart) feed
 *             a quadrature hybrid combiner (e.g. 4-port 90° hybrid
 *             or simple RC networks).  Phase is shifted by offsetting
 *             the two PLL fractional values incrementally each sample.
 *
 *   PIN_SSB_PWM (GP13, PWM) — amplitude envelope.
 *             A low-pass filtered PWM output drives the gate of a
 *             series MOSFET (e.g. 2N7000) in the RF path to modulate
 *             the carrier amplitude.  Combined with phase modulation
 *             this approximates DSB-SC → SSB when the two methods are
 *             in quadrature (Hartley SSB method in digital form).
 *
 * Simpler alternative (Class-E style as in uSDX):
 *             CLK0 is the carrier.  PIN_SSB_PWM drives a class-E PA
 *             supply voltage.  Audio samples amplitude-modulate supply
 *             while Si5351 provides a constant-frequency carrier.
 *             This is AM-envelope SSB — works well for voice at QRP.
 *
 * Both methods are compiled in; set ssb_mod_method at runtime.
 */

typedef enum {
    SSB_METHOD_PWM_ENVELOPE = 0,   /* simple: PWM controls PA supply     */
    SSB_METHOD_QUADRATURE,         /* full: CLK0+CLK1 phase + PWM amp    */
} ssb_method_t;

extern ssb_method_t ssb_mod_method;

/* Initialise PWM slice for amplitude envelope on PIN_SSB_PWM */
void ssb_mod_init(void);

/* Feed one audio sample (24-bit signed) during TX.
   Call at AUDIO_SAMPLE_RATE (48 kHz) from USB audio callback.
   Drives both Si5351 phase offset and PWM duty simultaneously. */
void ssb_mod_sample(int32_t sample_24bit);

/* Enable/disable SSB TX output */
void ssb_mod_enable(bool on);
