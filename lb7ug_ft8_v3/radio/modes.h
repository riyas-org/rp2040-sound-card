#pragma once
/*
 * radio/modes.h
 * radio_mode_t definition shared by vfo.h, si4732.h, goertzel.h etc.
 * Keep this file dependency-free (no other project headers).
 */
typedef enum {
    MODE_LSB = 0,
    MODE_USB,
    MODE_CW,
    MODE_AM,
    MODE_FM,
    MODE_FT8,
    MODE_WSPR,
    MODE_COUNT
} radio_mode_t;
