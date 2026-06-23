/*
 * hal/i2c_bus.c
 * Shared I2C0 init with bus-recovery and device scan.
 *
 * STAGE 1 — first thing to get working.
 * Confirm with logic analyser or by checking return value of i2c_bus_init().
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "../config.h"
#include "i2c_bus.h"

uint8_t i2c_bus_init(void) {
    /* Bus recovery: toggle SCL 9 times with SDA high to release any
       slave that got stuck mid-transaction from a previous power cycle. */
    gpio_init(PIN_SCL); gpio_set_dir(PIN_SCL, GPIO_OUT);
    gpio_init(PIN_SDA); gpio_set_dir(PIN_SDA, GPIO_IN);
    gpio_pull_up(PIN_SDA);
    for (int i = 0; i < 9; i++) {
        gpio_put(PIN_SCL, 1); sleep_us(5);
        gpio_put(PIN_SCL, 0); sleep_us(5);
    }
    /* STOP condition */
    gpio_set_dir(PIN_SDA, GPIO_OUT);
    gpio_put(PIN_SDA, 0); sleep_us(5);
    gpio_put(PIN_SCL, 1); sleep_us(5);
    gpio_put(PIN_SDA, 1); sleep_us(5);

    /* Now hand pins over to I2C peripheral */
    i2c_init(I2C_PORT, I2C_FREQ_HZ);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);
    sleep_ms(10);

    /* Scan for expected devices */
    uint8_t found = 0;
    uint8_t dummy;
    if (i2c_read_timeout_us(I2C_PORT, SI5351_ADDR, &dummy, 1, false, 2000) >= 0) found |= (1<<0);
    if (i2c_read_timeout_us(I2C_PORT, SI4732_ADDR, &dummy, 1, false, 2000) >= 0) found |= (1<<1);
    if (i2c_read_timeout_us(I2C_PORT, SSD1306_ADDR,&dummy, 1, false, 2000) >= 0) found |= (1<<2);
    return found;
}

bool i2c_write(uint8_t addr, const uint8_t *buf, size_t n) {
    return i2c_write_blocking(I2C_PORT, addr, buf, n, false) == (int)n;
}

bool i2c_read(uint8_t addr, uint8_t *buf, size_t n) {
    return i2c_read_blocking(I2C_PORT, addr, buf, n, false) == (int)n;
}

bool i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
    if (i2c_write_blocking(I2C_PORT, addr, &reg, 1, true) != 1) return false;
    return i2c_read_blocking(I2C_PORT, addr, buf, len, false) == (int)len;
}
