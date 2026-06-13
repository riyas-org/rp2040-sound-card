#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Initialise Si5351, all outputs off */
void si5351_init(void);

/* Set output frequency on CLK0/1/2. freq_hz=0 disables that output.
   Calibration offset (ppb) is applied automatically. */
void si5351_set_freq(uint8_t clk, uint64_t freq_hz);

/* Crystal calibration offset in parts-per-billion.
   Positive = crystal runs fast. Saved/loaded by vfo.c. */
extern int32_t si5351_cal_ppb;

/* Directly disable/enable a clock output */
void si5351_enable(uint8_t clk, bool on);
