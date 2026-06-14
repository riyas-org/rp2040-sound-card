
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "../radio/modes.h"

/* Boot and patch */
void si4732_init_am(void);
void si4732_init_fm(void);
void si4732_apply_ssb_patch(void);
void si4732_power_down(void);
extern bool si4732_ssb_patched;

/* Tune */
void si4732_tune_am(uint32_t freq_hz);
void si4732_tune_fm(uint32_t freq_10khz);

/* SSB properties */
void si4732_set_ssb_mode(radio_mode_t mode, uint8_t bw_code);
void si4732_set_bfo(int16_t offset_hz);

/* AM bandwidth code (0-5) */
void si4732_set_am_bw(uint8_t bw_code);

/* AGC */
void si4732_agc_enable(bool on);

/* Soft mute off — call after patch to prevent audio blanking */
void si4732_soft_mute_off(void);

/* Signal strength -127..0 dBm */
int8_t si4732_get_rssi(void);

/* Volume and mute */
void si4732_set_volume(uint8_t vol);
void si4732_mute(bool mute);

/* Low-level property write */
void si4732_set_property(uint16_t prop, uint16_t val);
