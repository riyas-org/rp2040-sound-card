/*
 * stages/stage2_oled.c
 *
 * STAGE 2 — OLED display. Draws text and a progress bar.
 * Confirms I2C to SSD1306 works and font/flush pipeline is correct.
 *
 * CMakeLists.txt sources for this stage:
 *   stages/stage2_oled.c  hal/i2c_bus.c  hal/oled.c
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "../config.h"
#include "../hal/i2c_bus.h"
#include "../hal/oled.h"

int main(void) {
    set_sys_clock_khz(250000, true);
    stdio_init_all();

    gpio_init(PIN_SI4732_RST); gpio_set_dir(PIN_SI4732_RST, GPIO_OUT); gpio_put(PIN_SI4732_RST, 1);
    gpio_init(PIN_LED);        gpio_set_dir(PIN_LED, GPIO_OUT);

    i2c_bus_init();
    oled_init();

    /* Static text */
    oled_clear();
    oled_str(0, 0, "Stage 2: OLED OK",  false);
    oled_str(0, 1, "----------------",  false);
    oled_str(0, 2, "Hello, TRX!",       false);
    oled_str(0, 3, "14.074.000 MHz",    false);
    oled_str(0, 4, "Mode: FT8   USB",   false);
    oled_hline(0, 127, 5);
    oled_str(0, 6, "S-meter:",          false);
    oled_flush();
    sleep_ms(1000);

    /* Animated S-meter bar */
    int8_t fake_rssi = -127;
    bool rising = true;
    while (true) {
        oled_smeter(48, 126, 6, fake_rssi);
        oled_printf(0, 7, false, "RSSI: %4d dBm", (int)fake_rssi);
        oled_flush();
        if (rising) { fake_rssi += 3; if (fake_rssi >= 0)   rising = false; }
        else        { fake_rssi -= 3; if (fake_rssi <= -127) rising = true;  }
        gpio_put(PIN_LED, rising);
        sleep_ms(40);
    }
}
