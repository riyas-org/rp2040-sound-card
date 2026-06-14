/*
 * stages/stage1_blink_i2c.c
 *
 * STAGE 1 — Minimal build. Blinks LED, scans I2C bus, prints results
 * over USB serial (stdio). Flash this first before touching any radio code.
 *
 * HOW TO USE:
 *   1. In CMakeLists.txt, temporarily replace main.c with this file:
 *        add_executable(trx stages/stage1_blink_i2c.c hal/i2c_bus.c)
 *      and remove all other sources.
 *   2. Enable USB stdio:
 *        pico_enable_stdio_usb(trx 1)
 *   3. Flash. Open a terminal on the CDC port.
 *   4. Expected output:
 *        I2C scan...
 *        Si5351  @ 0x60 : FOUND
 *        Si4732  @ 0x11 : FOUND
 *        SSD1306 @ 0x3C : FOUND
 *        All chips OK. Blink rate = 250ms.
 *
 * If a chip shows MISS:
 *   - Check solder joints
 *   - Confirm 4.7k pull-ups on SDA/SCL to 3.3V
 *   - Check Si4732 RST pin — must be HIGH (GP6 driven HIGH here)
 *   - Si4732 may show at 0x63 if SEN pin is high — check datasheet
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "../config.h"
#include "../hal/i2c_bus.h"

int main(void) {
    set_sys_clock_khz(250000, true);
    stdio_init_all();
    sleep_ms(2000);   /* wait for USB serial to connect */

    /* Si4732 reset must be HIGH before I2C scan */
    gpio_init(PIN_SI4732_RST);
    gpio_set_dir(PIN_SI4732_RST, GPIO_OUT);
    gpio_put(PIN_SI4732_RST, 1);
    sleep_ms(10);

    /* LED */
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    printf("\r\n=== Stage 1: I2C scan ===\r\n");

    uint8_t found = i2c_bus_init();

    printf("Si5351  @ 0x%02X : %s\r\n", SI5351_ADDR,  (found & 1) ? "FOUND" : "MISS");
    printf("Si4732  @ 0x%02X : %s\r\n", SI4732_ADDR,  (found & 2) ? "FOUND" : "MISS");
    printf("SSD1306 @ 0x%02X : %s\r\n", SSD1306_ADDR, (found & 4) ? "FOUND" : "MISS");

    if (found == 0x07) {
        printf("All chips OK. Blink rate = 250ms.\r\n");
    } else {
        printf("WARNING: %d chip(s) missing! Check wiring.\r\n", 3 - __builtin_popcount(found));
    }

    /* Blink: fast if all found, slow if something missing */
    uint32_t blink_ms = (found == 0x07) ? 250 : 1000;
    bool led = false;

    while (true) {
        gpio_put(PIN_LED, led = !led);
        sleep_ms(blink_ms);
    }
}
