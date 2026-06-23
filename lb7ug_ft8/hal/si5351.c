// hal/si5351.c
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../config.h"
#include "i2c_bus.h"
#include "si5351.h"

#define SI5351_XTAL_HZ    27000000ULL   // 27 MHz
#define SI5351_VCO_MIN    600000000ULL
#define SI5351_VCO_MAX    900000000ULL

// Register map
#define R_OE        3
#define R_CLK0      16
#define R_CLK1      17
#define R_CLK2      18
#define R_PLL_A     26
#define R_PLL_B     34
#define R_MS0       42
#define R_MS1       50
#define R_MS2       58
#define R_PLL_RST   177

static uint8_t oe_shadow = 0xFF;   // all outputs off initially

static void wr(uint8_t reg, uint8_t val) {
    uint8_t b[2] = {reg, val};
    i2c_write(SI5351_ADDR, b, 2);
}

/* Write PLL multisynth registers (8 bytes) */
static void wr_pll(uint8_t base, uint32_t p1, uint32_t p2, uint32_t p3) {
    uint8_t buf[9] = { base,
        (p3 >> 8) & 0xFF, p3 & 0xFF,
        (p1 >> 16) & 0x03, (p1 >> 8) & 0xFF, p1 & 0xFF,
        ((p3 >> 12) & 0xF0) | ((p2 >> 16) & 0x0F),
        (p2 >> 8) & 0xFF, p2 & 0xFF };
    i2c_write(SI5351_ADDR, buf, 9);
}

/* Write output multisynth */
static void wr_ms(uint8_t base, uint32_t p1, uint32_t p2, uint32_t p3, uint8_t r_div) {
    uint8_t buf[9] = { base,
        (p3 >> 8) & 0xFF, p3 & 0xFF,
        ((r_div & 7) << 4) | ((p1 >> 16) & 0x03),
        (p1 >> 8) & 0xFF, p1 & 0xFF,
        ((p3 >> 12) & 0xF0) | ((p2 >> 16) & 0x0F),
        (p2 >> 8) & 0xFF, p2 & 0xFF };
    i2c_write(SI5351_ADDR, buf, 9);
}

void si5351_init(void) {
    wr(R_OE, 0xFF);                 // all off
    for (int i = 16; i <= 18; i++)  // power down all CLK
        wr(i, 0x80);
    sleep_ms(10);
    oe_shadow = 0xFF;
    printf("[SI5351] init done\n");
}

void si5351_enable(uint8_t clk, bool on) {
    if (on)
        oe_shadow &= ~(1u << clk);
    else
        oe_shadow |= (1u << clk);
    wr(R_OE, oe_shadow);
}

void si5351_set_freq(uint8_t clk, uint64_t freq_hz, uint8_t drive_ma) {
    if (freq_hz == 0) {
        si5351_enable(clk, false);
        return;
    }
    if (freq_hz < 500) freq_hz = 500;
    if (freq_hz > 150000000) freq_hz = 150000000;

    // Apply calibration (if any)
    extern int32_t si5351_cal_ppb;
    if (si5351_cal_ppb != 0)
        freq_hz = (uint64_t)((double)freq_hz *
                  (1.0 + (double)si5351_cal_ppb * 1e-9));

    // Determine R divider (to keep VCO high enough)
    uint8_t r_div = 0;
    uint32_t f_eff = freq_hz;
    while (f_eff < 4000000 && r_div < 7) {
        f_eff <<= 1;
        r_div++;
    }

    // Integer multiplier (must be even for best output purity)
    uint32_t ms = SI5351_VCO_MAX / f_eff;
    if (ms < 6) ms = 6;
    if (ms > 900) ms = 900;
    if (ms % 2) ms++;

    uint64_t vco = (uint64_t)f_eff * ms;
    if (vco < SI5351_VCO_MIN) {
        vco = SI5351_VCO_MIN;
        ms = vco / f_eff;
        if (ms % 2) ms++;
        vco = (uint64_t)f_eff * ms;
    }
    if (vco > SI5351_VCO_MAX) {
        vco = SI5351_VCO_MAX;
        ms = vco / f_eff;
        if (ms % 2) ms++;
        vco = (uint64_t)f_eff * ms;
    }

    // PLL fractional multiplier
    uint32_t pll_a = vco / SI5351_XTAL_HZ;
    uint32_t rem   = vco % SI5351_XTAL_HZ;
    const uint32_t pll_c = 1048575U;
    uint32_t pll_b = (uint32_t)(((uint64_t)rem * pll_c) / SI5351_XTAL_HZ);

    uint32_t fbc = (uint32_t)((uint64_t)128 * pll_b / pll_c);
    uint32_t p1 = 128 * pll_a + fbc - 512;
    uint32_t p2 = 128 * pll_b - pll_c * fbc;
    uint32_t p3 = pll_c;

    // Output multisynth (integer mode)
    uint32_t ms_p1 = 128 * ms - 512;
    uint32_t ms_p2 = 0;
    uint32_t ms_p3 = 1;

    // Disable output while reprogramming
    si5351_enable(clk, false);

    // Select PLL: CLK0/1 use PLL A, CLK2 uses PLL B
    uint8_t pll_base = (clk < 2) ? R_PLL_A : R_PLL_B;
    uint8_t ms_base  = R_MS0 + clk * 8;
    uint8_t clk_ctrl = R_CLK0 + clk;

    wr_pll(pll_base, p1, p2, p3);
    wr_ms(ms_base, ms_p1, ms_p2, ms_p3, r_div);

    // Set drive level (0=2mA,1=4mA,2=6mA,3=8mA)
    uint8_t drv = (drive_ma >= 8) ? 3 : (drive_ma >= 6) ? 2 : (drive_ma >= 4) ? 1 : 0;
    uint8_t ctrl = 0x0F | drv;          // integer mode, CLKx source = MS, PLL source = 0 (PLL A)
    if (clk >= 2) ctrl |= 0x20;         // CLK2 uses PLL B
    wr(clk_ctrl, ctrl);

    // Reset PLL
    wr(R_PLL_RST, (clk < 2) ? 0x20 : 0x80);
    sleep_us(100);

    // Re‑enable output
    si5351_enable(clk, true);
    //printf("[SI5351] CLK%d set to %llu Hz, drive %d mA\n", clk, (unsigned long long)freq_hz, drive_ma);
}
