/*
 * radio/trx.c
 * RX/TX sequencer — controls antenna relay and coordinates PTT.
 *
 * STAGE 5 — needed before CW or FT8 TX.
 */
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "../config.h"
#include "../hal/si5351.h"
#include "trx.h"

trx_state_t trx_state   = SEQ_RX;
bool        usb_streaming = false;

void trx_init(void) {
    gpio_init(PIN_RXTX);
    gpio_set_dir(PIN_RXTX, GPIO_OUT);
    gpio_put(PIN_RXTX, 0);
}

void trx_go_tx(void) {
    if (trx_state == SEQ_TX) return;
    trx_state = SEQ_TX;
    gpio_put(PIN_RXTX, 1);
    sleep_ms(5);   /* relay settle */
}

void trx_go_rx(void) {
    if (trx_state == SEQ_RX) return;
    trx_state = SEQ_RX;
    gpio_put(PIN_RXTX, 0);
    sleep_ms(5);
}

void trx_toggle(void) {
    if (trx_state == SEQ_TX) trx_go_rx();
    else                      trx_go_tx();
}
