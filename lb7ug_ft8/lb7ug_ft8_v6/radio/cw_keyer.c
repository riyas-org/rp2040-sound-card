/*
 * radio/cw_keyer.c
 * Straight-key CW keyer, sidetone PWM, Morse decoder, memory playback.
 *
 * STAGE 11 — add after FT8/USB audio works.
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "../config.h"
#include "trx.h"
#include "cw_keyer.h"

uint32_t cw_wpm = CW_WPM_DEFAULT;

char cw_mem[CW_MEM_SLOTS][CW_MEM_LEN] = {
    "CQ CQ DE XX0YYY K",
    "599 73",
    "XX0YYY",
    "",""
};
bool cw_mem_playing = false;
char cw_decoded_buf[32];
int  cw_decoded_len = 0;

/* ---- Morse table ------------------------------------------------------ */
typedef struct { const char *code; char ch; } morse_t;
static const morse_t morse_tab[] = {
    {".-",'A'},{"-...",'B'},{"-.-.",'C'},{"-..",'D'},{".",'E'},
    {"..-.",'F'},{"--.",'G'},{"....",'H'},{"..",'I'},{".---",'J'},
    {"-.-",'K'},{".-..",'L'},{"--",'M'},{"-.",'N'},{"---",'O'},
    {".--.",'P'},{"--.-",'Q'},{".-.",'R'},{"...",'S'},{"-",'T'},
    {"..-",'U'},{"...-",'V'},{".--",'W'},{"-..-",'X'},{"-.--",'Y'},
    {"--..",'Z'},{"-----",'0'},{".----",'1'},{"..---",'2'},
    {"...--",'3'},{"....-",'4'},{".....",'5'},{"-....",'6'},
    {"--...",'7'},{"---..",'8'},{"----.",'9'},{NULL,0}
};

static const char *morse_for(char c) {
    if (c>='a'&&c<='z') c=c-32;
    for (int i=0; morse_tab[i].code; i++)
        if (morse_tab[i].ch==c) return morse_tab[i].code;
    return NULL;
}

/* ---- Sidetone --------------------------------------------------------- */
static uint st_slice, st_chan;

static void sidetone_on(void) {
    uint32_t wrap = clock_get_hz(clk_sys) / SIDETONE_FREQ_HZ - 1;
    pwm_set_wrap(st_slice, wrap);
    pwm_set_chan_level(st_slice, st_chan, wrap / 2);
}
static void sidetone_off(void) {
    pwm_set_chan_level(st_slice, st_chan, 0);
}

/* ---- Keyer state machine type ----------------------------------------- */
typedef enum { KS_IDLE, KS_DOWN, KS_UP_WAIT } ks_t;

/* ---- Memory playback state (file scope so cw_mem_play() can set slot) - */
static int      mp_slot  = 0;
static int      mp_pos   = 0;
static int      mp_sym   = 0;
static bool     mp_tone  = false;
static uint32_t mp_next  = 0;
static ks_t     ks       = KS_IDLE;
static uint32_t ks_time  = 0;

/* Decoder state */
static char morse_buf[9];
static int  morse_len = 0;

static void decode_push(char sym) {
    if (morse_len < 8) morse_buf[morse_len++] = sym;
}
static void decode_letter(void) {
    morse_buf[morse_len] = 0;
    for (int i=0; morse_tab[i].code; i++) {
        if (!strcmp(morse_tab[i].code, morse_buf)) {
            if (cw_decoded_len < 31) cw_decoded_buf[cw_decoded_len++] = morse_tab[i].ch;
            cw_decoded_buf[cw_decoded_len] = 0;
            break;
        }
    }
    morse_len = 0;
}
static void decode_space(void) {
    if (cw_decoded_len < 31) { cw_decoded_buf[cw_decoded_len++]=' '; cw_decoded_buf[cw_decoded_len]=0; }
}

void cw_keyer_task(uint32_t now_ms) {
    /* ---- Memory playback ---------------------------------------------- */
    if (cw_mem_playing) {
        if (now_ms >= mp_next) {
            const char *msg = cw_mem[mp_slot];

            if (!msg[mp_pos]) {
                /* End of message */
                cw_mem_playing=false; sidetone_off(); trx_go_rx();
                mp_pos=0; mp_sym=0; mp_tone=false;
                return;
            }
            char ch = msg[mp_pos];
            if (ch==' ') { mp_next=now_ms+cw_dot_ms()*7; mp_pos++; return; }
            const char *mc = morse_for(ch);
            if (!mc) { mp_pos++; return; }
            if (mp_sym < (int)strlen(mc)) {
                uint32_t dur = (mc[mp_sym]=='-') ? cw_dot_ms()*3 : cw_dot_ms();
                if (!mp_tone) {
                    trx_go_tx(); sidetone_on();
                    mp_tone=true; mp_next=now_ms+dur;
                } else {
                    sidetone_off(); mp_tone=false; mp_sym++;
                    if (mp_sym>=(int)strlen(mc)) {
                        mp_pos++; mp_sym=0;
                        mp_next=now_ms+cw_dot_ms()*3; /* inter-char gap */
                    } else {
                        mp_next=now_ms+cw_dot_ms();   /* inter-element gap */
                    }
                }
            }
        }
        return;  /* don't process straight key during playback */
    }

    /* ---- Straight key ------------------------------------------------- */
    bool key = !gpio_get(PIN_CW_KEY);

    switch (ks) {
        case KS_IDLE:
            if (key) { trx_go_tx(); sidetone_on(); ks=KS_DOWN; ks_time=now_ms; }
            break;
        case KS_DOWN:
            if (!key) {
                sidetone_off();
                uint32_t mark = now_ms - ks_time;
                decode_push(mark < cw_dot_ms()*2 ? '.' : '-');
                ks=KS_UP_WAIT; ks_time=now_ms;
            }
            break;
        case KS_UP_WAIT:
            if (key) { sidetone_on(); ks=KS_DOWN; ks_time=now_ms; }
            else {
                uint32_t sp = now_ms - ks_time;
                if (sp >= cw_dot_ms()*3 && sp < cw_dot_ms()*7) decode_letter();
                if (sp >= cw_dot_ms()*7) { decode_letter(); decode_space(); }
                if (sp >= cw_dot_ms()*4) { trx_go_rx(); ks=KS_IDLE; }
            }
            break;
    }
}

void cw_mem_play(int slot) {
    if (cw_mem_playing) return;
    mp_slot = slot;
    mp_pos  = 0;
    mp_sym  = 0;
    mp_tone = false;
    mp_next = 0;
    cw_mem_playing = true;
}

void cw_keyer_init(void) {
    gpio_init(PIN_CW_KEY);
    gpio_set_dir(PIN_CW_KEY, GPIO_IN);
    gpio_pull_up(PIN_CW_KEY);

    gpio_set_function(PIN_SIDETONE, GPIO_FUNC_PWM);
    st_slice = pwm_gpio_to_slice_num(PIN_SIDETONE);
    st_chan  = pwm_gpio_to_channel(PIN_SIDETONE);
    uint32_t wrap = clock_get_hz(clk_sys) / SIDETONE_FREQ_HZ - 1;
    pwm_set_wrap(st_slice, wrap);
    pwm_set_chan_level(st_slice, st_chan, 0);
    pwm_set_enabled(st_slice, true);
}
