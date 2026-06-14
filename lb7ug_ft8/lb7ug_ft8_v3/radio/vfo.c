/*
 * radio/vfo.c
 * VFO state, band/mode tables, apply_vfo, flash save/load.
 *
 * STAGE 5 — core of the radio. Get this right before UI.
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "../config.h"
#include "../hal/si5351.h"
#include "../hal/si4732.h"
#include "vfo.h"

const char *mode_names[MODE_COUNT] = {
    "LSB","USB","CW","AM","FM","FT8","WSPR"
};

const band_t bands[NUM_BANDS] = {
    {"160m",  1840000,  1800000,  2000000, MODE_LSB},
    { "80m",  3573000,  3500000,  4000000, MODE_LSB},
    { "60m",  5357000,  5330000,  5410000, MODE_USB},
    { "40m",  7074000,  7000000,  7300000, MODE_LSB},
    { "30m", 10136000, 10100000, 10150000, MODE_USB},
    { "20m", 14074000, 14000000, 14350000, MODE_USB},
    { "17m", 18100000, 18068000, 18168000, MODE_USB},
    { "15m", 21074000, 21000000, 21450000, MODE_USB},
    { "12m", 24915000, 24890000, 24990000, MODE_USB},
    { "10m", 28074000, 28000000, 29700000, MODE_USB},
    {  "6m", 50313000, 50000000, 54000000, MODE_USB},
};

const bw_entry_t ssb_bw[SSB_BW_COUNT] = {
    {"1.2k",0},{"2.2k",1},{"3.0k",2},{"4.0k",3},{"0.5k",4},{"1.0k",5}
};
const bw_entry_t am_bw[AM_BW_COUNT] = {
    {"6k",4},{"4k",3},{"3k",2},{"2k",1},{"1k",0},{"1.8k",5}
};
const uint32_t step_table[STEP_COUNT] = {
    1, 10, 100, 500, 1000, 5000, 10000, 100000
};

vfo_state_t vfo = {
    .freq_hz   = 14074000,
    .mode      = MODE_USB,
    .band_idx  = 5,
    .bw_idx    = 1,
    .rit_hz    = 0,
    .rit_on    = false,
    .split     = false,
    .vfob_hz   = 14074000,
    .step_hz   = 100,
    .step_idx  = 2,
    .agc_on    = true,
    .bfo_offset= 0,
    .cw_wpm    = CW_WPM_DEFAULT,
    .cal_ppb   = 0,
    .magic     = FLASH_SAVE_MAGIC,
};

/* Known FT8 and WSPR frequencies (dial) — within 500 Hz match → set mode */
static const uint32_t ft8_freqs[] = {
    1840000,3573000,5357000,7074000,10136000,
    14074000,18100000,21074000,24915000,28074000,50313000
};
static const uint32_t wspr_freqs[] = {
    1836600,3568600,5364700,7038600,10138700,
    14095600,18104600,21094600,24924600,28124600,50293000
};

void vfo_auto_mode(uint32_t freq_hz) {
    for (unsigned i=0;i<sizeof(ft8_freqs)/4;i++)
        if ((uint32_t)abs((int32_t)(freq_hz-ft8_freqs[i]))<500){ vfo.mode=MODE_FT8; return; }
    for (unsigned i=0;i<sizeof(wspr_freqs)/4;i++)
        if ((uint32_t)abs((int32_t)(freq_hz-wspr_freqs[i]))<500){ vfo.mode=MODE_WSPR; return; }
    /* Don't override digital/CW/AM/FM */
    if (vfo.mode==MODE_FT8||vfo.mode==MODE_WSPR||
        vfo.mode==MODE_CW ||vfo.mode==MODE_AM  ||vfo.mode==MODE_FM) return;
    vfo.mode = (freq_hz < 10000000) ? MODE_LSB : MODE_USB;
}

void vfo_auto_band(uint32_t freq_hz) {
    for (int i=0;i<NUM_BANDS;i++)
        if (freq_hz>=bands[i].freq_min && freq_hz<=bands[i].freq_max)
            { vfo.band_idx=i; return; }
}

void vfo_apply(void) {
    uint32_t rx = vfo.freq_hz;
    if (vfo.rit_on) {
        int64_t adj = (int64_t)rx + vfo.rit_hz;
        if (adj > 0) rx = (uint32_t)adj;
    }

    si5351_cal_ppb = vfo.cal_ppb;

    bool is_ssb = (vfo.mode==MODE_LSB||vfo.mode==MODE_USB||
                   vfo.mode==MODE_CW ||vfo.mode==MODE_FT8||vfo.mode==MODE_WSPR);

    if (vfo.mode == MODE_FM) {
        si5351_set_freq(0, 0);   /* CLK0 off during FM receive */
        si4732_tune_fm(rx);
    } else {
        si5351_set_freq(0, (uint64_t)vfo.freq_hz);  /* TX VFO */
        si4732_tune_am(rx);

        if (is_ssb && si4732_ssb_patched) {
            /* FT8/WSPR: force widest BW to capture all tones */
            uint8_t bw = (vfo.mode==MODE_FT8||vfo.mode==MODE_WSPR)
                         ? 3 : ssb_bw[vfo.bw_idx % SSB_BW_COUNT].code;
            radio_mode_t m = (vfo.mode==MODE_LSB) ? MODE_LSB : MODE_USB;
            si4732_set_ssb_mode(m, bw);
            /* Sub-kHz BFO correction */
            int16_t bfo = (int16_t)(rx % 1000);
            if (vfo.mode == MODE_LSB) bfo = -bfo;
            bfo += vfo.bfo_offset;
            si4732_set_bfo(bfo);
        } else if (vfo.mode == MODE_AM) {
            si4732_set_am_bw(am_bw[vfo.bw_idx % AM_BW_COUNT].code);
        }
    }
    si4732_agc_enable(vfo.agc_on);
}

void vfo_set_band(int idx) {
    if (idx < 0 || idx >= NUM_BANDS) return;
    vfo.band_idx = idx;
    vfo.freq_hz  = bands[idx].freq_hz;
    vfo.mode     = bands[idx].default_mode;
    vfo.bw_idx   = 1;
    vfo.rit_hz   = 0;
    vfo.rit_on   = false;
    vfo_apply();
}

#ifdef FEATURE_FLASH_SAVE
void vfo_save(void) {
    /* Pack vfo_state_t + CW memories into one flash page (256 bytes) */
    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));
    vfo.magic = FLASH_SAVE_MAGIC;
    memcpy(page, &vfo, sizeof(vfo));
    uint32_t irq = save_and_disable_interrupts();
    flash_range_erase(FLASH_SAVE_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_SAVE_OFFSET, page, FLASH_PAGE_SIZE);
    restore_interrupts(irq);
}

static void vfo_load(void) {
    const uint8_t *fl = (const uint8_t *)(XIP_BASE + FLASH_SAVE_OFFSET);
    vfo_state_t tmp;
    memcpy(&tmp, fl, sizeof(tmp));
    if (tmp.magic == FLASH_SAVE_MAGIC) vfo = tmp;
}
#else
void vfo_save(void) {}
static void vfo_load(void) {}
#endif

void vfo_init(void) {
    vfo_load();
    si5351_cal_ppb = vfo.cal_ppb;
}
