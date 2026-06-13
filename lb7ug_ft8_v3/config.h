#pragma once
/*
 * config.h — single place for all hardware pin assignments,
 * clock frequencies, and compile-time feature flags.
 */

/* ---- I2C bus ---------------------------------------------------------- */
#define I2C_PORT        i2c0
#define PIN_SDA         0
#define PIN_SCL         1
#define I2C_FREQ_HZ     400000

/* ---- Chip I2C addresses ----------------------------------------------- */
#define SI5351_ADDR     0x60
#define SI4732_ADDR     0x11
#define SSD1306_ADDR    0x3C

/* ---- GPIO assignments ------------------------------------------------- */
#define PIN_SI4732_RST  6
#define PIN_ENC_A       7
#define PIN_ENC_B       8
#define PIN_ENC_BTN     9
#define PIN_RXTX        10      /* HIGH = TX                                */
#define PIN_CW_KEY      11      /* active LOW                               */
#define PIN_SIDETONE    12      /* PWM 700 Hz sidetone                      */
#define PIN_SSB_PWM     13      /* SSB TX amplitude envelope PWM            */
#define PIN_AUDIO_ADC   26      /* ADC0 — Si4732 audio in                   */
#define AUDIO_ADC_CHAN   0      /* ADC channel number for GP26               */
#define PIN_LED         25

/* ---- Audio / ADC ------------------------------------------------------ */
#define AUDIO_SAMPLE_RATE    48000
#define AUDIO_OVERSAMPLE     4
#define AUDIO_BUF_SAMPLES    48
#define AUDIO_CHANNELS       2
#define AUDIO_BYTES_PER_SAMP 4
#define AUDIO_FRAME_BYTES    (AUDIO_BUF_SAMPLES*AUDIO_CHANNELS*AUDIO_BYTES_PER_SAMP)

/* ---- CW --------------------------------------------------------------- */
#define SIDETONE_FREQ_HZ     700
#define CW_WPM_DEFAULT       15
#define CW_WPM_MIN           5
#define CW_WPM_MAX           40
#define CW_MEM_SLOTS         5
#define CW_MEM_LEN           40

/* ---- Si5351 ----------------------------------------------------------- */
#define SI5351_XTAL_HZ       25000000ULL

/* ---- Flash ------------------------------------------------------------ */
#define FLASH_SAVE_OFFSET    (2*1024*1024 - FLASH_SECTOR_SIZE)
#define FLASH_SAVE_MAGIC     0xA7

/* ---- Feature flags  (comment out to disable for a build stage) -------- */
#define FEATURE_SSB_PATCH
#define FEATURE_SSB_TX
#define FEATURE_FT8_WSPR
#define FEATURE_CW_DECODER
#define FEATURE_FLASH_SAVE
#define FEATURE_DUAL_CORE

/* ---- Goertzel --------------------------------------------------------- */
#define GOERTZEL_ACCUM_FRAMES  8

/* ---- Timings ---------------------------------------------------------- */
#define MENU_TIMEOUT_MS      5000
#define LONG_PRESS_MS        600
#define AUTOSAVE_DELAY_MS    5000
