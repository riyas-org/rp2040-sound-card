// hal/si5351.h
#pragma once
#include <stdint.h>
#include <stdbool.h>

void si5351_init(void);
void si5351_enable(uint8_t clk, bool on);
void si5351_set_freq(uint8_t clk, uint64_t freq_hz, uint8_t drive_ma);
extern int32_t si5351_cal_ppb;   // calibration in ppb (optional)