/*
 * hal/si5351.c
 * Minimal Si5351A driver.  CLK0=VFO/TX, CLK1=spare, CLK2=FSK/SSB.
 *
 * STAGE 4 — after OLED works, confirm CLK0 with a freq counter or SDR dongle.
 */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../config.h"
#include "i2c_bus.h"
#include "si5351.h"

/* Register map (AN619) */
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
#define R_XTAL_LOAD 183

int32_t si5351_cal_ppb = 0;

static void wr(uint8_t reg, uint8_t val) {
    uint8_t b[2] = {reg, val};
    i2c_write(SI5351_ADDR, b, 2);
}

static void wr_pll(uint8_t base, uint32_t a, uint32_t b, uint32_t c) {
    uint32_t p1 = 128*a + (uint32_t)floor(128.0*(double)b/(double)c) - 512;
    uint32_t p2 = 128*b - c*(uint32_t)floor(128.0*(double)b/(double)c);
    uint32_t p3 = c;
    uint8_t buf[9] = { base,
        (p3>>8)&0xFF, p3&0xFF,
        (p1>>16)&0x03, (p1>>8)&0xFF, p1&0xFF,
        ((p3>>12)&0xF0)|((p2>>16)&0x0F),
        (p2>>8)&0xFF, p2&0xFF };
    i2c_write(SI5351_ADDR, buf, 9);
}

static void wr_ms(uint8_t base, uint32_t a, uint32_t b, uint32_t c, uint8_t rdiv) {
    uint32_t p1, p2, p3;
    if (b == 0) { p1 = 128*a-512; p2 = 0; p3 = 1; }
    else {
        p1 = 128*a + (uint32_t)floor(128.0*(double)b/(double)c) - 512;
        p2 = 128*b - c*(uint32_t)floor(128.0*(double)b/(double)c);
        p3 = c;
    }
    uint8_t buf[9] = { base,
        (p3>>8)&0xFF, p3&0xFF,
        ((rdiv&7)<<4)|((p1>>16)&0x03), (p1>>8)&0xFF, p1&0xFF,
        ((p3>>12)&0xF0)|((p2>>16)&0x0F),
        (p2>>8)&0xFF, p2&0xFF };
    i2c_write(SI5351_ADDR, buf, 9);
}

void si5351_init(void) {
    wr(R_OE, 0xFF);        /* all outputs off                 */
    wr(R_XTAL_LOAD, 0xD2); /* 10 pF load capacitor            */
    wr(R_CLK0, 0x0F);      /* CLK0: PLL A, integer, 8 mA      */
    wr(R_CLK1, 0x2F);      /* CLK1: PLL B, integer, 8 mA      */
    wr(R_CLK2, 0x2F);      /* CLK2: PLL B, integer, 8 mA      */
}

void si5351_enable(uint8_t clk, bool on) {
    uint8_t oe;
    uint8_t r = R_OE;
    i2c_write(SI5351_ADDR, &r, 1);
    /* read back — Si5351 doesn't support direct read of OE, use shadow */
    static uint8_t oe_shadow = 0xFF;
    if (on)  oe_shadow &= ~(1u << clk);
    else     oe_shadow |=  (1u << clk);
    wr(R_OE, oe_shadow);
    (void)oe;
}

void si5351_set_freq(uint8_t clk, uint64_t freq_hz) {
    static uint8_t oe_shadow = 0xFF;

    if (freq_hz == 0) {
        oe_shadow |= (1u << clk);
        wr(R_OE, oe_shadow);
        return;
    }

    /* Apply crystal calibration */
    if (si5351_cal_ppb != 0)
        freq_hz = (uint64_t)((double)freq_hz *
                  (1.0 + (double)si5351_cal_ppb * 1e-9));

    /* Choose integer MS divider so VCO stays in 600–900 MHz */
    uint32_t ms = (uint32_t)(600000000ULL / freq_hz);
    if (ms < 4)   ms = 4;
    if (ms > 900) ms = 900;
    uint64_t vco = freq_hz * ms;

    /* PLL fractional multiplier */
    uint32_t pa = (uint32_t)(vco / SI5351_XTAL_HZ);
    uint32_t pr = (uint32_t)(vco % SI5351_XTAL_HZ);
    uint32_t pc = 1000000;
    uint32_t pb = (uint32_t)((uint64_t)pr * pc / SI5351_XTAL_HZ);

    /* CLK0/1 use PLL A; CLK2 uses PLL B */
    uint8_t pll_base = (clk < 2) ? R_PLL_A : R_PLL_B;
    uint8_t ms_base  = R_MS0 + clk * 8;
    uint8_t clk_ctrl = R_CLK0 + clk;
    uint8_t pll_rst  = (clk < 2) ? 0x20 : 0x80;

    /* Glitch-free: disable → reprogram → re-enable */
    oe_shadow |= (1u << clk);
    wr(R_OE, oe_shadow);

    wr_pll(pll_base, pa, pb, pc);
    wr_ms(ms_base, ms, 0, 1, 0);
    wr(clk_ctrl, 0x0F | (clk >= 2 ? 0x20 : 0x00));
    wr(R_PLL_RST, pll_rst);

    oe_shadow &= ~(1u << clk);
    wr(R_OE, oe_shadow);
}
