#pragma once
#include <stdbool.h>

typedef enum { SEQ_RX, SEQ_TX } trx_state_t;
extern trx_state_t trx_state;
extern bool        usb_streaming;

void trx_init(void);
void trx_go_tx(void);
void trx_go_rx(void);
void trx_toggle(void);
