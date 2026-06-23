// hal/si4732.h
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "../radio/modes.h"

/* Boot and patch */
void si4732_init(void);
void si4732_power_down(void);
bool si4732_is_ssb_loaded(void);

/* Tune */
void si4732_tune_am(uint32_t freq_khz);
void si4732_tune_fm(uint32_t freq_10khz);
void si4732_fm_seek(bool up);

/* Mode switch */
void si4732_switch_mode(radio_mode_t new_mode);

/* SSB properties */
void si4732_set_ssb_mode(radio_mode_t mode, int16_t bfo_hz);
void si4732_set_bfo(int16_t bfo_hz);
void si4732_set_am_bw(uint8_t bw_index);  /* 0-7: All,6k,4k,3k,2.5,2k,1.8,1k */

/* Volume and mute */
void si4732_set_volume(uint8_t vol);
void si4732_mute(bool mute);

/* Signal strength */
void si4732_poll_rsq(void);
int8_t si4732_get_rssi(void);
uint8_t si4732_get_snr(void);

/* Frequency and mode getters */
uint32_t si4732_get_freq_khz(void);
uint32_t si4732_get_fm_10khz(void);
radio_mode_t si4732_get_mode(void);
