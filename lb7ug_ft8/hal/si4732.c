// hal/si4732.c - Exact match to working main.c logic
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "../config.h"
#include "si4732.h"
#include "patch_full.h"

/* ============================================================
   Commands and Properties - EXACT from main.c
   ============================================================ */
#define SI_POWER_UP     0x01
#define SI_GET_REV      0x10
#define SI_POWER_DOWN   0x11
#define SI_SET_PROPERTY 0x12
#define SI_FM_TUNE      0x20
#define SI_FM_SEEK      0x21
#define SI_FM_RSQ       0x23
#define SI_AM_TUNE      0x40
#define SI_AM_RSQ       0x43

#define P_RX_VOLUME     0x4000
#define P_RX_HARD_MUTE  0x4001
#define P_FM_DEEMPHASIS 0x1100
#define P_FM_CHAN_SPACE 0x1101
#define P_FM_MAX_TUNE   0x1108
#define P_FM_MIN_TUNE   0x1109
#define P_AM_CHAN_FILTER 0x3102
#define P_SSB_BFO       0x0100
#define P_SSB_MODE      0x0101
#define P_SSB_SOFT_MUTE 0x3302
#define P_SSB_AGC_SPEED 0x3700



#define SSB_PATCH_ROWS (sizeof(ssb_patch) / 8)

/* ============================================================
   State variables - EXACT from main.c
   ============================================================ */
static radio_mode_t g_mode = MODE_AM;
static uint32_t g_freq_khz = 7100;
static uint32_t g_fm_10khz = 10080;
static int8_t g_rssi = 0;
static uint8_t g_snr = 0;
static int16_t g_bfo_hz = 0;
static uint8_t g_volume = 50;
static bool g_ssb_ok = false;

/* ============================================================
   Low-level I2C functions - EXACT from main.c
   ============================================================ */
static bool si4732_cts(uint32_t ms) {
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    for (;;) {
        uint8_t st = 0;
        i2c_read_blocking(I2C_PORT, SI4732_ADDR, &st, 1, false);
        if (st & 0x80) return true;
        if (st & 0x40) {
            printf("[SI4732] ERR st=0x%02X\n", st);
            return false;
        }
        if (to_ms_since_boot(get_absolute_time()) - t0 > ms) {
            printf("[SI4732] CTS timeout\n");
            return false;
        }
        sleep_ms(2);
    }
}

static bool si4732_cmd(const uint8_t *c, int len, uint32_t cts_ms) {
    if (i2c_write_blocking(I2C_PORT, SI4732_ADDR, c, len, false) < 0) {
        printf("[SI4732] I2C write err\n");
        return false;
    }
    sleep_ms(1);
    return si4732_cts(cts_ms);
}

static void si4732_read(uint8_t *buf, int len) {
    i2c_read_blocking(I2C_PORT, SI4732_ADDR, buf, len, false);
}

static bool si4732_set_prop(uint16_t prop, uint16_t val) {
    uint8_t c[6] = {SI_SET_PROPERTY, 0x00,
                    (prop >> 8) & 0xFF, prop & 0xFF,
                    (val >> 8) & 0xFF, val & 0xFF};
    bool ok = si4732_cmd(c, 6, 200);
    printf("[SI4732] PROP 0x%04X=0x%04X %s\n", prop, val, ok ? "OK" : "FAIL");
    return ok;
}

static void si4732_hw_reset(void) {
    gpio_put(PIN_SI4732_RST, 0);
    sleep_ms(15);
    gpio_put(PIN_SI4732_RST, 1);
    sleep_ms(250);
    printf("[SI4732] HW reset\n");
}

/* ============================================================
   Power-up functions - EXACT from main.c
   ============================================================ */
static bool si4732_power_up_fm(void) {
    uint8_t c[3] = {SI_POWER_UP, 0x50, 0x05};
    bool ok = si4732_cmd(c, 3, 700);
    printf("[SI4732] POWER_UP FM %s\n", ok ? "OK" : "FAIL");
    if (!ok) return false;
    si4732_set_prop(P_FM_CHAN_SPACE, 0x0001);
    si4732_set_prop(P_FM_DEEMPHASIS, 0x0002);
    si4732_set_prop(P_FM_MIN_TUNE, 6400);
    si4732_set_prop(P_FM_MAX_TUNE, 10800);
    si4732_set_prop(P_RX_VOLUME, g_volume);
    si4732_set_prop(P_RX_HARD_MUTE, 0x0000);
    return true;
}

static bool si4732_power_up_am(void) {
    uint8_t c[3] = {SI_POWER_UP, 0x51, 0x05};
    bool ok = si4732_cmd(c, 3, 700);
    printf("[SI4732] POWER_UP AM %s\n", ok ? "OK" : "FAIL");
    if (!ok) return false;
    si4732_set_prop(P_AM_CHAN_FILTER, 3);
    si4732_set_prop(P_RX_VOLUME, g_volume);
    si4732_set_prop(P_RX_HARD_MUTE, 0x0000);
    return true;
}

static bool si4732_load_ssb(void) {
    printf("[SI4732] SSB patch load (%u rows)...\n", (unsigned)SSB_PATCH_ROWS);
    uint8_t pu[3] = {SI_POWER_UP, 0x71, 0x05};
    if (!si4732_cmd(pu, 3, 700)) {
        printf("[SI4732] POWER_UP+PATCH fail\n");
        return false;
    }
    for (size_t i = 0; i < SSB_PATCH_ROWS; i++) {
        const uint8_t *row = &ssb_patch[i * 8];
        bool zero = true;
        for (int j = 0; j < 8; j++) {
            if (row[j]) {
                zero = false;
                break;
            }
        }
        if (zero) break;
        if (i2c_write_blocking(I2C_PORT, SI4732_ADDR, row, 8, false) < 0) {
            printf("[SI4732] Patch row %u I2C err\n", (unsigned)i);
            return false;
        }
        sleep_ms(1);
        if (!si4732_cts(300)) {
            printf("[SI4732] Patch row %u CTS fail\n", (unsigned)i);
            return false;
        }
    }
    si4732_set_prop(P_SSB_MODE, 1);
    si4732_set_prop(P_SSB_BFO, 0);
    si4732_set_prop(P_AM_CHAN_FILTER, 4);
    si4732_set_prop(P_SSB_SOFT_MUTE, 0);
    si4732_set_prop(P_SSB_AGC_SPEED, 0);
    si4732_set_prop(P_RX_VOLUME, g_volume);
    si4732_set_prop(P_RX_HARD_MUTE, 0);
    g_ssb_ok = true;
    printf("[SI4732] SSB patch OK\n");
    return true;
}

/* ============================================================
   Tune functions - EXACT from main.c
   ============================================================ */
void si4732_tune_am(uint32_t khz) {
    if (khz < 150) khz = 150;
    if (khz > 30000) khz = 30000;
    g_freq_khz = khz;
    uint8_t c[5] = {SI_AM_TUNE, 0x00,
                    (khz >> 8) & 0xFF, khz & 0xFF, 0x00};
    si4732_cmd(c, 5, 400);
    printf("[SI4732] AM tune %u kHz\n", (unsigned)khz);
}

void si4732_tune_fm(uint32_t f10) {
    if (f10 < 6400) f10 = 6400;
    if (f10 > 10800) f10 = 10800;
    g_fm_10khz = f10;
    uint8_t c[5] = {SI_FM_TUNE, 0x00,
                    (f10 >> 8) & 0xFF, f10 & 0xFF, 0x00};
    si4732_cmd(c, 5, 400);
    printf("[SI4732] FM tune %.1f MHz\n", f10 / 100.0f);
}

void si4732_fm_seek(bool up) {
    uint8_t arg1 = 0x04;
    if (up) arg1 |= 0x08;
    uint8_t c[2] = {SI_FM_SEEK, arg1};
    si4732_cmd(c, 2, 800);
    
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    for (;;) {
        uint8_t st = 0;
        i2c_read_blocking(I2C_PORT, SI4732_ADDR, &st, 1, false);
        if (st & 0x01) break;
        if (to_ms_since_boot(get_absolute_time()) - t0 > 1200) break;
        sleep_ms(5);
    }
    
    uint8_t c2[2] = {0x22, 0x00};
    si4732_cmd(c2, 2, 200);
    uint8_t resp[8] = {0};
    si4732_read(resp, 8);
    uint16_t nf = (uint16_t)(((uint16_t)resp[2] << 8) | resp[3]);
    if (nf >= 6400 && nf <= 10800) g_fm_10khz = nf;
    printf("[SI4732] FM seek→ %.1f MHz\n", g_fm_10khz / 100.0f);
}

/* ============================================================
   Mode switch - EXACT from main.c
   ============================================================ */
void si4732_switch_mode(radio_mode_t nm) {
    printf("[SI4732] switch %d→%d\n", g_mode, nm);
    uint8_t pd = SI_POWER_DOWN;
    si4732_cmd(&pd, 1, 200);
    sleep_ms(50);
    si4732_hw_reset();
    g_ssb_ok = false;
    g_mode = nm;
    
    switch (nm) {
        case MODE_FM:
            si4732_power_up_fm();
            si4732_tune_fm(g_fm_10khz);
            break;
        case MODE_AM:
            si4732_power_up_am();
            si4732_tune_am(g_freq_khz);
            break;
        case MODE_USB:
        case MODE_LSB:
            if (si4732_load_ssb()) {
                si4732_tune_am(g_freq_khz);
                si4732_set_ssb_mode(nm, g_bfo_hz);
            } else {
                printf("[SI4732] SSB fail, fallback AM\n");
                g_mode = MODE_AM;
                si4732_power_up_am();
                si4732_tune_am(g_freq_khz);
            }
            break;
    }
}

/* ============================================================
   SSB functions - EXACT from main.c
   ============================================================ */
void si4732_set_ssb_mode(radio_mode_t mode, int16_t bfo) {
    g_bfo_hz = bfo;
    g_mode = mode;
    if (!g_ssb_ok) return;
    si4732_set_prop(P_SSB_MODE, (mode == MODE_USB) ? 1 : 0);
    si4732_set_prop(P_SSB_BFO, (uint16_t)(int16_t)bfo);
    printf("[SI4732] SSB %s BFO=%d Hz\n",
           (mode == MODE_USB) ? "USB" : "LSB", bfo);
}

void si4732_set_bfo(int16_t bfo_hz) {
    si4732_set_ssb_mode(g_mode, bfo_hz);
}

void si4732_set_am_bw(uint8_t bw_index) {
    static const uint16_t bw_val[] = {8, 7, 6, 5, 4, 3, 2, 1};
    if (bw_index < 8) {
        si4732_set_prop(P_AM_CHAN_FILTER, bw_val[bw_index]);
    }
}

/* ============================================================
   Volume and mute - EXACT from main.c
   ============================================================ */
void si4732_set_volume(uint8_t v) {
    if (v > 63) v = 63;
    g_volume = v;
    si4732_set_prop(P_RX_VOLUME, v);
}

void si4732_mute(bool m) {
    si4732_set_prop(P_RX_HARD_MUTE, m ? 0x0001 : 0x0000);
}

/* ============================================================
   RSQ status - EXACT from main.c
   ============================================================ */
void si4732_poll_rsq(void) {
    uint8_t resp[8] = {0};
    if (g_mode == MODE_FM) {
        uint8_t c[2] = {SI_FM_RSQ, 0x00};
        si4732_cmd(c, 2, 100);
        si4732_read(resp, 8);
    } else {
        uint8_t c[2] = {SI_AM_RSQ, 0x00};
        si4732_cmd(c, 2, 100);
        si4732_read(resp, 6);
    }
    g_rssi = (int8_t)resp[4];
    g_snr = resp[5];
}

int8_t si4732_get_rssi(void) {
    return g_rssi;
}

uint8_t si4732_get_snr(void) {
    return g_snr;
}

/* ============================================================
   Getters - EXACT from main.c
   ============================================================ */
uint32_t si4732_get_freq_khz(void) {
    return g_freq_khz;
}

uint32_t si4732_get_fm_10khz(void) {
    return g_fm_10khz;
}

radio_mode_t si4732_get_mode(void) {
    return g_mode;
}

bool si4732_is_ssb_loaded(void) {
    return g_ssb_ok;
}

/* ============================================================
   Init and power down - EXACT from main.c
   ============================================================ */
void si4732_init(void) {
    gpio_init(PIN_SI4732_RST);
    gpio_set_dir(PIN_SI4732_RST, GPIO_OUT);
    gpio_put(PIN_SI4732_RST, 1);
    si4732_hw_reset();
    si4732_power_up_am();
    
    uint8_t c = SI_GET_REV;
    si4732_cmd(&c, 1, 200);
    uint8_t r[9] = {0};
    si4732_read(r, 9);
    printf("[SI4732] PN=%02X FW=%02X.%02X PATCH=%02X%02X CHIPREV=%02X\n",
           r[1], r[2], r[3], r[4], r[5], r[8]);
    
    si4732_tune_am(g_freq_khz);
}

void si4732_power_down(void) {
    uint8_t pd = SI_POWER_DOWN;
    si4732_cmd(&pd, 1, 200);
    sleep_ms(50);
}
