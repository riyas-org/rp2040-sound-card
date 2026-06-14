/*
 * stages/stage3_rx.c
 *
 * STAGE 3 — Basic AM receive.
 * Si4732 powered up in AM mode, tuned to 1 MHz (AM broadcast).
 * OLED shows frequency. Encoder tunes +/- 1 kHz per step.
 * No SSB patch, no Si5351, no USB audio — just prove the receiver works.
 *
 * Connect headphones or a small speaker between Si4732 LOUT/ROUT
 * and ground (through a 100uF cap to block DC).
 *
 * CMakeLists.txt sources:
 *   stages/stage3_rx.c
 *   hal/i2c_bus.c  hal/oled.c  hal/si4732.c
 *   (si4732.c depends on radio/modes.h for radio_mode_t — include path needed)
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "../config.h"
#include "../hal/i2c_bus.h"
#include "../hal/oled.h"
#include "../radio/modes.h"   /* radio_mode_t for si4732.h */
#include "../hal/si4732.h"

static volatile int32_t enc_delta = 0;
static uint8_t enc_last = 0;
static const int8_t enc_lut[16] = {0,-1,+1,0,+1,0,0,-1,-1,0,0,+1,0,+1,-1,0};

static void gpio_irq(uint gpio, uint32_t events) {
    (void)events; (void)gpio;
    uint8_t cur = ((uint8_t)gpio_get(PIN_ENC_A)<<1)|(uint8_t)gpio_get(PIN_ENC_B);
    enc_delta += enc_lut[(enc_last<<2)|cur];
    enc_last = cur;
}

int main(void) {
    set_sys_clock_khz(250000, true);
    stdio_init_all();

    gpio_init(PIN_SI4732_RST); gpio_set_dir(PIN_SI4732_RST,GPIO_OUT); gpio_put(PIN_SI4732_RST,1);
    gpio_init(PIN_LED);        gpio_set_dir(PIN_LED,GPIO_OUT);
    gpio_init(PIN_ENC_A);      gpio_set_dir(PIN_ENC_A,GPIO_IN); gpio_pull_up(PIN_ENC_A);
    gpio_init(PIN_ENC_B);      gpio_set_dir(PIN_ENC_B,GPIO_IN); gpio_pull_up(PIN_ENC_B);
    gpio_set_irq_enabled_with_callback(PIN_ENC_A, GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL, true, gpio_irq);
    gpio_set_irq_enabled(PIN_ENC_B, GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL, true);
    enc_last = (uint8_t)((gpio_get(PIN_ENC_A)<<1)|gpio_get(PIN_ENC_B));

    i2c_bus_init();
    oled_init();

    oled_clear();
    oled_str(0, 0, "Stage 3: AM RX", false);
    oled_str(0, 1, "Si4732 init...", false);
    oled_flush();

    si4732_init_am();
    si4732_soft_mute_off();

    uint32_t freq = 1000000;   /* 1 MHz — AM broadcast */
    si4732_tune_am(freq);

    oled_str(0, 1, "              ", false);

    uint32_t last_draw = 0;
    bool led = false;

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        /* Read encoder */
        uint32_t s = save_and_disable_interrupts();
        int32_t d = enc_delta; enc_delta = 0;
        restore_interrupts(s);

        if (d != 0) {
            int64_t nf = (int64_t)freq + d * 1000;
            if (nf < 500000)    nf = 500000;
            if (nf > 30000000)  nf = 30000000;
            freq = (uint32_t)nf;
            si4732_tune_am(freq);
        }

        /* Refresh display every 200 ms */
        if (now - last_draw >= 200) {
            last_draw = now;
            led = !led; gpio_put(PIN_LED, led);

            int8_t rssi = si4732_get_rssi();

            oled_clear();
            oled_str(0, 0, "Stage 3: AM RX",  false);
            oled_hline(0, 127, 1);
            oled_printf(0, 2, false, "%lu.%03lu kHz",
                        (unsigned long)(freq/1000),
                        (unsigned long)(freq%1000));
            oled_str(0, 3, "Mode: AM",         false);
            oled_str(0, 4, "Enc: tune 1kHz",   false);
            oled_hline(0, 127, 5);
            oled_str(0, 6, "S:", false);
            oled_smeter(12, 120, 6, rssi);
            oled_printf(0, 7, false, "RSSI: %d dBm", (int)rssi);
            oled_flush();
        }
    }
}
