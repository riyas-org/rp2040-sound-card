/*
 * hal/si4732.c
 * Si4732 receiver driver — AM/SSB/FM + SSB patch.
 *
 * STAGE 3 — get AM receive working first (no patch).
 * STAGE 8 — enable SSB patch for LSB/USB.
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "../config.h"
#include "i2c_bus.h"
#include "si4732.h"

/* Commands */
#define CMD_POWER_UP       0x01
#define CMD_SET_PROPERTY   0x12
#define CMD_AM_TUNE        0x40
#define CMD_AM_STATUS      0x42
#define CMD_FM_TUNE        0x20
#define CMD_POWER_DOWN     0x11

/* Properties */
#define PROP_AM_CHANNEL_FILTER   0x3102
#define PROP_AM_AGC_MAX_GAIN     0x3103
#define PROP_AM_SOFT_MUTE_MAX    0x3302
#define PROP_AM_SOFT_MUTE_SNR    0x3303
#define PROP_SSB_BFO             0x0100
#define PROP_SSB_MODE            0x0101
#define PROP_SSB_BW              0x0102
#define PROP_SSB_AGC             0x0103
#define PROP_SSB_AFC             0x3104

bool si4732_ssb_patched = false;

/* ---- SSB patch data ---------------------------------------------------
 * Replace the placeholder bytes below with the full 2252-byte array from:
 * https://github.com/pu2clr/SI4735/blob/master/extras/patches/patch_full_ssb.h
 * Copy the body of SSBRX_PATCH_FULL_CONTENT[] here.
 * The code will work in AM-only mode without it; SSB needs the full patch.
 * ----------------------------------------------------------------------- */
static const uint8_t ssb_patch[] = {
    0x15,0x00,0x0F,0xE0,0xF2,0x17,0x17,0x17,
    /* <<<  paste full patch bytes here  >>> */
};
#define SSB_PATCH_LEN sizeof(ssb_patch)

/* ---- helpers ---------------------------------------------------------- */
static void cmd(const uint8_t *b, uint8_t n) {
    i2c_write(SI4732_ADDR, b, n);
    sleep_us(300);
}

static void wait_cts(void) {
    uint8_t s; int t = 2000;
    do { i2c_read(SI4732_ADDR, &s, 1); sleep_us(50); }
    while (!(s & 0x80) && --t);
}

static void reset_chip(void) {
    gpio_put(PIN_SI4732_RST, 0); sleep_ms(10);
    gpio_put(PIN_SI4732_RST, 1); sleep_ms(250);
}

/* ---- public API ------------------------------------------------------- */

void si4732_set_property(uint16_t prop, uint16_t val) {
    uint8_t b[] = { CMD_SET_PROPERTY, 0,
                    (uint8_t)(prop>>8), (uint8_t)(prop),
                    (uint8_t)(val>>8),  (uint8_t)(val) };
    cmd(b, 6); wait_cts();
}

void si4732_init_am(void) {
    reset_chip();
    uint8_t b[] = { CMD_POWER_UP, 0x01, 0x05 }; /* AM mode, analog out */
    cmd(b, 3); wait_cts();
}

void si4732_init_fm(void) {
    reset_chip();
    uint8_t b[] = { CMD_POWER_UP, 0x00, 0x05 }; /* FM mode, analog out */
    cmd(b, 3); wait_cts();
}

void si4732_apply_ssb_patch(void) {
    if (si4732_ssb_patched) return;
    const uint8_t *p = ssb_patch;
    uint32_t rem = SSB_PATCH_LEN;
    while (rem >= 8) {
        i2c_write(SI4732_ADDR, p, 8);
        sleep_us(500); wait_cts();
        p += 8; rem -= 8;
    }
    if (rem > 0) {
        i2c_write(SI4732_ADDR, p, (int)rem);
        sleep_ms(1); wait_cts();
    }
    si4732_ssb_patched = true;
}

void si4732_tune_am(uint32_t freq_hz) {
    uint16_t kHz = (uint16_t)(freq_hz / 1000);
    uint8_t b[] = { CMD_AM_TUNE, 0,
                    (uint8_t)(kHz>>8), (uint8_t)(kHz), 0, 0 };
    cmd(b, 6); wait_cts();
}

void si4732_tune_fm(uint32_t freq_hz) {
    uint16_t u = (uint16_t)(freq_hz / 10000);
    uint8_t b[] = { CMD_FM_TUNE, 0,
                    (uint8_t)(u>>8), (uint8_t)(u), 0 };
    cmd(b, 5); wait_cts();
}

void si4732_set_ssb_mode(radio_mode_t mode, uint8_t bw_code) {
    /* mode: 1=USB, 2=LSB */
    si4732_set_property(PROP_SSB_MODE,  (mode == MODE_LSB) ? 2 : 1);
    si4732_set_property(PROP_SSB_BW,    bw_code);
    si4732_set_property(PROP_SSB_AGC,   0x0000);
    si4732_set_property(PROP_SSB_AFC,   0x0000);
    si4732_set_property(PROP_AM_SOFT_MUTE_MAX, 0x0000);
    si4732_set_property(PROP_AM_SOFT_MUTE_SNR, 0x0008);
}

void si4732_set_bfo(int16_t hz) {
    if (hz >  16380) hz =  16380;
    if (hz < -16380) hz = -16380;
    si4732_set_property(PROP_SSB_BFO, (uint16_t)(int16_t)hz);
}

void si4732_set_am_bw(uint8_t bw) {
    si4732_set_property(PROP_AM_CHANNEL_FILTER, bw);
}

void si4732_agc_enable(bool on) {
    si4732_set_property(PROP_AM_AGC_MAX_GAIN, on ? 0x0001 : 0x0000);
}

void si4732_soft_mute_off(void) {
    si4732_set_property(PROP_AM_SOFT_MUTE_MAX, 0x0000);
}

int8_t si4732_get_rssi(void) {
    uint8_t b[] = { CMD_AM_STATUS, 0x01 };
    cmd(b, 2);
    uint8_t r[8] = {0};
    i2c_read(SI4732_ADDR, r, 8);
    return (int8_t)r[4];
}
