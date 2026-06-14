// config.h
#pragma once

#include "pico/stdlib.h"

// I2C0
#define I2C_PORT       i2c0
#define PIN_SDA        0
#define PIN_SCL        1
#define I2C_FREQ_HZ    400000

// Device addresses
#define SSD1306_ADDR   0x3C
#define SI5351_ADDR    0x60
#define SI4732_ADDR    0x11

// SI4732 reset pin
#define PIN_SI4732_RST 6  // adjust to your wiring (was GP12 in main.c)

// Optional: OLED dimensions
#define OLED_W         128
#define OLED_H         64
