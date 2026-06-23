#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "modes.h"   /* radio_mode_t */

extern const char *mode_names[MODE_COUNT];

typedef struct {
    const char   *name;
    uint32_t      freq_hz;
    uint32_t      freq_min;
    uint32_t      freq_max;
    radio_mode_t  default_mode;
} band_t;

#define NUM_BANDS 11
extern const band_t bands[NUM_BANDS];

typedef struct { const char *name; uint8_t code; } bw_entry_t;
#define SSB_BW_COUNT 6
#define AM_BW_COUNT  6
extern const bw_entry_t ssb_bw[SSB_BW_COUNT];
extern const bw_entry_t am_bw[AM_BW_COUNT];

#define STEP_COUNT 8
extern const uint32_t step_table[STEP_COUNT];

typedef struct {
    uint32_t     freq_hz;
    radio_mode_t mode;
    int          band_idx;
    int          bw_idx;
    int32_t      rit_hz;
    bool         rit_on;
    bool         split;
    uint32_t     vfob_hz;
    uint32_t     step_hz;
    int          step_idx;
    bool         agc_on;
    int16_t      bfo_offset;
    uint32_t     cw_wpm;
    int32_t      cal_ppb;
    uint8_t      magic;
} vfo_state_t;

extern vfo_state_t vfo;

void vfo_apply(void);

/* Request a vfo_apply() to happen from the main loop (non-blocking from ISR
   or menu context). Call this instead of vfo_apply() from menu/encoder
   handlers to avoid blocking tud_task() during I2C CTS polling. */
void vfo_request_apply(void);
bool vfo_apply_pending(void);
void vfo_set_band(int idx);
void vfo_auto_band(uint32_t freq_hz);
void vfo_auto_mode(uint32_t freq_hz);
void vfo_init(void);
void vfo_save(void);
