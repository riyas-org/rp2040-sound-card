#pragma once
#include <stdint.h>
#include <stdbool.h>

extern uint32_t cw_wpm;
static inline uint32_t cw_dot_ms(void) { return 1200 / cw_wpm; }

/* CW memory slots */
extern char cw_mem[5][40];

/* Call every main-loop iteration — drives keyer state machine */
void cw_keyer_task(uint32_t now_ms);

/* Start playing a memory slot (0-4) */
void cw_mem_play(int slot);

/* True while a memory is playing */
extern bool cw_mem_playing;

/* Last decoded character (updated by decoder), 0 if none */
extern char cw_decoded_buf[32];
extern int  cw_decoded_len;

void cw_keyer_init(void);
