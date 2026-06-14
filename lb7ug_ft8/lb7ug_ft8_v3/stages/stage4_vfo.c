/*
 * stages/stage4_vfo.c
 *
 * STAGE 4 — Si5351 VFO output.
 * Encoder tunes CLK0. Verify with an SDR dongle or frequency counter.
 * Si4732 also tunes so you can listen while confirming Si5351 output.
 *
 * CMakeLists.txt sources:
 *   stages/stage4_vfo.c
 *   hal/i2c_bus.c  hal/oled.c  hal/si4732.c  hal/si5351.c
 *   radio/vfo.c    (brings in vfo state + auto_mode + auto_band)
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "../config.h"
#include "../hal/i2c_bus.h"
#include "../hal/oled.h"
#include "../hal/si4732.h"
#include "../hal/si5351.h"
#include "../radio/vfo.h"

static volatile int32_t enc_delta = 0;
static uint8_t enc_last = 0;
static const int8_t enc_lut[16] = {0,-1,+1,0,+1,0,0,-1,-1,0,0,+1,0,+1,-1,0};
static volatile bool btn_ev = false;

static void gpio_irq(uint gpio, uint32_t events) {
    (void)events;
    if (gpio==PIN_ENC_A||gpio==PIN_ENC_B) {
        uint8_t cur=((uint8_t)gpio_get(PIN_ENC_A)<<1)|(uint8_t)gpio_get(PIN_ENC_B);
        enc_delta+=enc_lut[(enc_last<<2)|cur]; enc_last=cur;
    }
    if (gpio==PIN_ENC_BTN) {
        static bool last=true; bool c=gpio_get(PIN_ENC_BTN);
        if(!c&&last) btn_ev=true; last=c;
    }
}

static const uint32_t steps[] = {100,1000,10000,100000};
static int step_i = 1;

int main(void) {
    set_sys_clock_khz(250000, true);
    stdio_init_all();

    gpio_init(PIN_SI4732_RST); gpio_set_dir(PIN_SI4732_RST,GPIO_OUT); gpio_put(PIN_SI4732_RST,1);
    gpio_init(PIN_LED);        gpio_set_dir(PIN_LED,GPIO_OUT);
    gpio_init(PIN_ENC_A);      gpio_set_dir(PIN_ENC_A,GPIO_IN);   gpio_pull_up(PIN_ENC_A);
    gpio_init(PIN_ENC_B);      gpio_set_dir(PIN_ENC_B,GPIO_IN);   gpio_pull_up(PIN_ENC_B);
    gpio_init(PIN_ENC_BTN);    gpio_set_dir(PIN_ENC_BTN,GPIO_IN); gpio_pull_up(PIN_ENC_BTN);
    gpio_set_irq_enabled_with_callback(PIN_ENC_A,  GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL, true, gpio_irq);
    gpio_set_irq_enabled(PIN_ENC_B,   GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_ENC_BTN, GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL, true);
    enc_last = (uint8_t)((gpio_get(PIN_ENC_A)<<1)|gpio_get(PIN_ENC_B));

    i2c_bus_init();
    oled_init();
    si5351_init();
    si4732_init_am();
    si4732_soft_mute_off();
    vfo_init();     /* loads from flash if saved, otherwise defaults to 20m FT8 */
    vfo_apply();    /* tunes Si4732 + sets Si5351 CLK0 */

    uint32_t last_draw = 0;
    bool led = false;

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        uint32_t sv = save_and_disable_interrupts();
        int32_t d = enc_delta; enc_delta = 0;
        bool btn = btn_ev; btn_ev = false;
        restore_interrupts(sv);

        if (btn) {
            step_i = (step_i + 1) % 4;
            vfo.step_hz = steps[step_i];
        }
        if (d) {
            int64_t nf = (int64_t)vfo.freq_hz + d*(int64_t)vfo.step_hz;
            if (nf < (int64_t)bands[vfo.band_idx].freq_min) nf = bands[vfo.band_idx].freq_min;
            if (nf > (int64_t)bands[vfo.band_idx].freq_max) nf = bands[vfo.band_idx].freq_max;
            vfo.freq_hz = (uint32_t)nf;
            vfo_auto_mode(vfo.freq_hz);
            vfo_apply();
        }

        if (now - last_draw >= 150) {
            last_draw = now; led=!led; gpio_put(PIN_LED,led);
            int8_t rssi = si4732_get_rssi();
            oled_clear();
            oled_str(0,0,"Stage 4: VFO",false);
            oled_hline(0,127,1);
            uint32_t f = vfo.freq_hz;
            oled_printf(0,2,false,"%3lu.%03lu.%03lu",
                        (unsigned long)(f/1000000),
                        (unsigned long)((f%1000000)/1000),
                        (unsigned long)(f%1000));
            oled_printf(0,3,false,"Mode:%-4s Band:%s",
                        mode_names[vfo.mode], bands[vfo.band_idx].name);
            oled_printf(0,4,false,"Step: %lu Hz",(unsigned long)steps[step_i]);
            oled_str(0,5,"Btn=cycle step",false);
            oled_hline(0,127,5);
            oled_str(0,6,"S:",false);
            oled_smeter(12,120,6,rssi);
            oled_printf(0,7,false,"CLK0 active RSSI:%d",(int)rssi);
            oled_flush();
        }
    }
}
