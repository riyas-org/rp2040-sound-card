#pragma once
#include "hardware/i2c.h"

/* Initialise I2C0, run bus-recovery sequence, confirm chips respond.
   Returns bitmask of found devices: bit0=Si5351, bit1=Si4732, bit2=SSD1306 */
uint8_t i2c_bus_init(void);

/* Write n bytes to a device; returns true on ACK */
bool i2c_write(uint8_t addr, const uint8_t *buf, size_t n);

/* Read n bytes from a device */
bool i2c_read(uint8_t addr, uint8_t *buf, size_t n);

/* Write reg then read len bytes (combined write-restart-read) */
bool i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len);
