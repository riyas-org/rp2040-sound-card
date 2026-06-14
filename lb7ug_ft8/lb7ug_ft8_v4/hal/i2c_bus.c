// hal/i2c_bus.c
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "../config.h"
#include "i2c_bus.h"

uint8_t i2c_bus_init(void) {
    // IMPORTANT: First, ensure Si4732 is properly reset and released
    gpio_init(PIN_SI4732_RST);
    gpio_set_dir(PIN_SI4732_RST, GPIO_OUT);
    
    // Hold in reset
    gpio_put(PIN_SI4732_RST, 0);
    sleep_ms(50);
    
    // Now init I2C
    i2c_init(I2C_PORT, I2C_FREQ_HZ);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);
    sleep_ms(10);
    
    // Release Si4732 from reset
    gpio_put(PIN_SI4732_RST, 1);
    sleep_ms(300);  // Important: Wait for Si4732 to boot
    
    // Scan for devices
    uint8_t found = 0;
    uint8_t dummy;
    
    // Check Si5351
    if (i2c_read_timeout_us(I2C_PORT, SI5351_ADDR, &dummy, 1, false, 2000) >= 0) {
        found |= (1<<0);
        printf("[I2C] Si5351 found at 0x%02X\n", SI5351_ADDR);
    } else {
        printf("[I2C] Si5351 NOT found at 0x%02X\n", SI5351_ADDR);
    }
    
    // Check Si4732
    if (i2c_read_timeout_us(I2C_PORT, SI4732_ADDR, &dummy, 1, false, 2000) >= 0) {
        found |= (1<<1);
        printf("[I2C] Si4732 found at 0x%02X\n", SI4732_ADDR);
    } else {
        printf("[I2C] Si4732 NOT found at 0x%02X\n", SI4732_ADDR);
    }
    
    // Check OLED
    if (i2c_read_timeout_us(I2C_PORT, SSD1306_ADDR, &dummy, 1, false, 2000) >= 0) {
        found |= (1<<2);
        printf("[I2C] OLED found at 0x%02X\n", SSD1306_ADDR);
    } else {
        printf("[I2C] OLED NOT found at 0x%02X\n", SSD1306_ADDR);
    }
    
    printf("[I2C] Scan complete: 0x%02X (Si5351=%d, Si4732=%d, OLED=%d)\n",
           found, (found>>0)&1, (found>>1)&1, (found>>2)&1);
    
    return found;
}

bool i2c_write(uint8_t addr, const uint8_t *buf, size_t n) {
    int ret = i2c_write_blocking(I2C_PORT, addr, buf, n, false);
    if (ret != (int)n) {
        printf("[I2C] Write failed to 0x%02X, ret=%d\n", addr, ret);
    }
    return ret == (int)n;
}

bool i2c_read(uint8_t addr, uint8_t *buf, size_t n) {
    int ret = i2c_read_blocking(I2C_PORT, addr, buf, n, false);
    if (ret != (int)n) {
        printf("[I2C] Read failed from 0x%02X, ret=%d\n", addr, ret);
    }
    return ret == (int)n;
}

bool i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
    if (i2c_write_blocking(I2C_PORT, addr, &reg, 1, true) != 1)
        return false;
    return i2c_read_blocking(I2C_PORT, addr, buf, len, false) == (int)len;
}
