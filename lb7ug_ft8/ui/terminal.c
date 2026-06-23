/*
 * ui/terminal.c
 * QMX-style serial terminal + Kenwood TS-480 CAT over USB CDC.
 *
 * Navigation model (matches QMX SerialManager UX):
 *   • At any menu level, press Q <CR> or just Q to go back one level.
 *   • At root, Q shows status (nothing to go back to).
 *   • In CAT passthrough mode, Q<CR> or MENU<CR> both return to the menu.
 *   • Arrow-key ESC sequences are ignored gracefully (no accidental navigation).
 *   • ENTER alone at any sub-menu re-prints that menu.
 *   • Shortcut commands work from the root prompt without entering menus.
 *
 * CAT routing (auto-detect):
 *   • A burst containing ';' and only uppercase + digits triggers CAT parsing.
 *   • Type "cat<CR>" at root to lock into CAT passthrough mode (e.g. for logging sw).
 *   • In CAT mode every byte goes to the CAT parser; Q<CR> or MENU<CR> exits.
 *
 * Navigation stack:
 *   • Up to NAV_DEPTH levels are remembered so Q always goes exactly one step back.
 *   • This replaces the fragile single ts_par slot in the original code.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "tusb.h"
#include "../config.h"
#include "../radio/vfo.h"
#include "../radio/trx.h"
#include "../radio/cw_keyer.h"
#include "../hal/si4732.h"
#include "../hal/si5351.h"
#include "menu.h"
#include "terminal.h"

#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

/* =========================================================================
 * CDC ring buffer (TX to host)
 * ========================================================================= */
#define RING_SZ   4096
#define RING_MASK (RING_SZ - 1)

static char     ring[RING_SZ];
static uint32_t rhead = 0, rtail = 0;

static void ring_push(const char *s, int n)
{
    uint32_t free_space = RING_SZ - 1 - ((rhead - rtail) & RING_MASK);
    uint32_t wr = (uint32_t)n < free_space ? (uint32_t)n : free_space;
    for (uint32_t i = 0; i < wr; i++) {
        ring[rhead & RING_MASK] = s[i];
        rhead++;
    }
}

int term_printf(const char *fmt, ...)
{
    char b[256];
    va_list a;
    va_start(a, fmt);
    int n = vsnprintf(b, sizeof(b), fmt, a);
    va_end(a);
    if (n > 0) ring_push(b, n);
    return n;
}

void terminal_task(void)
{
    if (!tud_cdc_connected()) return;
    uint32_t av = (rhead - rtail) & RING_MASK;
    if (!av) return;
    uint8_t b[64];
    uint32_t n = av < sizeof(b) ? av : (uint32_t)sizeof(b);
    for (uint32_t i = 0; i < n; i++) {
        b[i] = (uint8_t)ring[rtail & RING_MASK];
        rtail++;
    }
    tud_cdc_write(b, n);
    tud_cdc_write_flush();
}

/* =========================================================================
 * CAT parser (Kenwood TS-480 subset)
 * ========================================================================= */
#define CAT_BUF 64
static char cat_buf[CAT_BUF];
static int  cat_len = 0;

static int mode_to_cat(radio_mode_t m)
{
    switch (m) {
        case MODE_LSB: return 1;
        case MODE_USB: return 2;
        case MODE_CW:  return 3;
        case MODE_FM:  return 4;
        case MODE_AM:  return 5;
        default:       return 2;
    }
}

static radio_mode_t cat_to_mode(int n)
{
    switch (n) {
        case 1: return MODE_LSB;
        case 2: return MODE_USB;
        case 3: return MODE_CW;
        case 4: return MODE_FM;
        case 5: return MODE_AM;
        default: return MODE_USB;
    }
}

static void cat_send(const char *s) { ring_push(s, (int)strlen(s)); }

static void cat_exec(const char *cmd, int len)
{
    if (len < 2) return;
    char r[48];

    /* FA – VFO-A frequency */
    if (cmd[0] == 'F' && cmd[1] == 'A') {
        if (len == 2) {
            snprintf(r, sizeof(r), "FA%011lu;", (unsigned long)vfo.freq_hz);
            cat_send(r);
        } else if (len == 13) {
            uint32_t f = (uint32_t)atol(cmd + 2);
            vfo.freq_hz = f;
            vfo_auto_band(f); vfo_auto_mode(f); vfo_apply();
            screen_draw_main();
        }
        return;
    }
    /* FB – VFO-B frequency */
    if (cmd[0] == 'F' && cmd[1] == 'B') {
        if (len == 2) {
            snprintf(r, sizeof(r), "FB%011lu;", (unsigned long)vfo.vfob_hz);
            cat_send(r);
        } else if (len == 13) {
            vfo.vfob_hz = (uint32_t)atol(cmd + 2);
        }
        return;
    }
    /* MD – mode */
    if (cmd[0] == 'M' && cmd[1] == 'D') {
        if (len == 2) {
            snprintf(r, sizeof(r), "MD%d;", mode_to_cat(vfo.mode));
            cat_send(r);
        } else if (len == 3) {
            vfo.mode = cat_to_mode(cmd[2] - '0');
            vfo_apply(); screen_draw_main();
        }
        return;
    }
    /* IF – transceiver status */
    if (cmd[0] == 'I' && cmd[1] == 'F') {
        snprintf(r, sizeof(r), "IF%011lu     %+05ld0000%d000000000;",
                 (unsigned long)vfo.freq_hz,
                 (long)vfo.rit_hz,
                 mode_to_cat(vfo.mode));
        cat_send(r);
        return;
    }
    /* SM – S-meter */
    if (cmd[0] == 'S' && cmd[1] == 'M') {
        int8_t rssi = si4732_get_rssi();
        int sm = (rssi + 127) * 30 / 127;
        if (sm < 0) sm = 0;
        if (sm > 30) sm = 30;
        snprintf(r, sizeof(r), "SM0%04d;", sm);
        cat_send(r);
        return;
    }
    /* TX/RX */
    if (cmd[0] == 'R' && cmd[1] == 'X') { trx_go_rx(); cat_send("RX0;"); return; }
    if (cmd[0] == 'T' && cmd[1] == 'X') { trx_go_tx(); cat_send("TX0;"); return; }
    /* Standard identification / keep-alive responses */
    if (cmd[0] == 'I' && cmd[1] == 'D') { cat_send("ID019;");  return; }
    if (cmd[0] == 'P' && cmd[1] == 'S') { cat_send("PS1;");    return; }
    if (cmd[0] == 'A' && cmd[1] == 'I') { cat_send("AI0;");    return; }
    if (cmd[0] == 'R' && cmd[1] == 'A') { cat_send("RA00;");   return; }
    if (cmd[0] == 'S' && cmd[1] == 'H') { cat_send("SH0000;"); return; }
    if (cmd[0] == 'V' && cmd[1] == 'X') { cat_send("VX0;");    return; }
    if (cmd[0] == 'A' && cmd[1] == 'G') { cat_send("AG0000;"); return; }
    if (cmd[0] == 'F' && (cmd[1] == 'R' || cmd[1] == 'T')) {
        snprintf(r, sizeof(r), "%c%c0;", cmd[0], cmd[1]);
        cat_send(r);
        return;
    }
    /* Unknown command */
    cat_send("?;");
}

static void cat_feed_byte(char c)
{
    if (cat_len < CAT_BUF - 1) cat_buf[cat_len++] = c;
    if (c == ';') {
        cat_buf[cat_len - 1] = 0;
        cat_exec(cat_buf, cat_len - 1);
        cat_len = 0;
    }
}

/* =========================================================================
 * Terminal state machine
 * ========================================================================= */
typedef enum {
    TS_ROOT,
    TS_VFO, TS_MODE, TS_CW, TS_TX,
    TS_CAT,                          /* CAT passthrough lock */
    TS_EDIT_FREQ, TS_EDIT_BAND, TS_EDIT_STEP,
    TS_EDIT_MODE, TS_EDIT_BW, TS_EDIT_BFO,
    TS_EDIT_RIT, TS_EDIT_CW_WPM, TS_EDIT_CAL,
    TS_EDIT_SPLIT, TS_EDIT_VFOB,
    TS_EDIT_MEM_PLAY, TS_EDIT_MEM_EDIT, TS_EDIT_MEM_TXT,
    TS_COUNT
} ts_t;

/* Navigation stack – supports Q at any depth */
#define NAV_DEPTH 8
static ts_t nav_stack[NAV_DEPTH];
static int  nav_top = 0;           /* index of current state */
#define ts   nav_stack[nav_top]    /* current state alias */

static char inbuf[64];
static int  inlen = 0;
static int  mem_edit_slot = 0;

/* ---- output helpers ----------------------------------------------------- */
#define CRLF  "\r\n"
#define tp(s) ring_push(s, (int)strlen(s))
#define tpf(...) \
    do { char _b[160]; snprintf(_b, sizeof(_b), __VA_ARGS__); \
         ring_push(_b, (int)strlen(_b)); } while (0)

static void thr(void) { tp("----------------------------------------" CRLF); }

/* ---- navigation helpers ------------------------------------------------- */

/* Push a new state, remembering where we came from */
static void nav_push(ts_t next)
{
    if (nav_top < NAV_DEPTH - 1) nav_top++;
    nav_stack[nav_top] = next;
}

/* Pop one level (Q command); returns true if we actually moved */
static bool nav_pop(void)
{
    if (nav_top == 0) return false;
    nav_top--;
    return true;
}

/* Emit the right prompt for the current state */
static void show_prompt(void);   /* forward decl */

/* ---- status & menus ----------------------------------------------------- */
static void show_status(void)
{
    thr();
    tpf("  Freq  : %lu Hz  (%s)" CRLF,  (unsigned long)vfo.freq_hz, bands[vfo.band_idx].name);
    tpf("  Mode  : %-4s  BW: %s" CRLF,  mode_names[vfo.mode],
        vfo.mode == MODE_FM ? "FM" :
        vfo.mode == MODE_AM ? am_bw[vfo.bw_idx % AM_BW_COUNT].name :
                              ssb_bw[vfo.bw_idx % SSB_BW_COUNT].name);
    tpf("  Step  : %lu Hz" CRLF,         (unsigned long)vfo.step_hz);
    tpf("  RIT   : %-3s  %+ld Hz" CRLF, vfo.rit_on ? "ON" : "OFF", (long)vfo.rit_hz);
    tpf("  Split : %-3s  VFO-B %lu Hz" CRLF, vfo.split ? "ON" : "OFF", (unsigned long)vfo.vfob_hz);
    tpf("  AGC   : %-3s  BFO: %+d Hz" CRLF,  vfo.agc_on ? "ON" : "OFF", vfo.bfo_offset);
    tpf("  CW    : %lu WPM   Cal: %+ld ppb" CRLF, (unsigned long)cw_wpm, (long)si5351_cal_ppb);
    tpf("  TX/RX : %s   RSSI: %d dBm" CRLF, trx_state == SEQ_TX ? "TX" : "RX", (int)si4732_get_rssi());
    thr();
}

static void show_root(void)
{
    tp("\033[2J\033[H");
    tp("\033[1m  *** RP2040 HF TRX ***\033[0m" CRLF);
    thr();
    tp("  1 VFO/Freq    2 Mode     3 CW" CRLF);
    tp("  4 TX/RX       5 Status   6 CAT mode" CRLF);
    tp("  9 Save        ? Help / shortcuts" CRLF);
    thr();
}

static void show_help(void)
{
    thr();
    tp("  Shortcuts (work at root prompt):" CRLF);
    tp("  f <hz>      set freq        e.g.  f 14074000" CRLF);
    tp("  b <1-N>     set band        e.g.  b 6" CRLF);
    tp("  m <1-N>     set mode        e.g.  m 2" CRLF);
    tp("  s <hz>      set step        e.g.  s 1000" CRLF);
    tp("  w <wpm>     CW speed        e.g.  w 20" CRLF);
    tp("  r <hz>      RIT offset      e.g.  r +250" CRLF);
    tp("  cal <ppb>   XTAL cal        e.g.  cal -120" CRLF);
    tp("  agc         toggle AGC" CRLF);
    tp("  rit         toggle RIT" CRLF);
    tp("  spl/split   toggle Split" CRLF);
    tp("  tx / rx     PTT" CRLF);
    tp("  mem <n>     play CW mem     e.g.  mem 1" CRLF);
    tp("  save / stat" CRLF);
    tp("  cat         enter CAT passthrough" CRLF);
    thr();
    tp("  Navigation: number keys select items." CRLF);
    tp("  Q <Enter>   go back one level (any depth)" CRLF);
    tp("  <Enter>     re-print current menu" CRLF);
    thr();
}

static void show_vfo_menu(void)
{
    tpf("  VFO-A: %lu Hz   Band: %s" CRLF, (unsigned long)vfo.freq_hz, bands[vfo.band_idx].name);
    tp("  1 Frequency   2 Band   3 Step" CRLF);
    tp("  4 Split/B     5 RIT    Q Back" CRLF);
}

static void show_mode_menu(void)
{
    tpf("  Mode: %s   BW: %s" CRLF, mode_names[vfo.mode],
        vfo.mode == MODE_FM ? "FM" :
        vfo.mode == MODE_AM ? am_bw[vfo.bw_idx % AM_BW_COUNT].name :
                              ssb_bw[vfo.bw_idx % SSB_BW_COUNT].name);
    tp("  1 Mode   2 Bandwidth   3 AGC   4 BFO   Q Back" CRLF);
}

static void show_cw_menu(void)
{
    tpf("  CW speed: %lu WPM" CRLF, (unsigned long)cw_wpm);
    tp("  1 Speed   2 Play mem   3 Edit mem   Q Back" CRLF);
}

static void show_tx_menu(void)
{
    tpf("  %s" CRLF, trx_state == SEQ_TX ? "TX (transmitting)" : "RX (receiving)");
    tp("  1 TX   2 RX   3 Toggle   Q Back" CRLF);
}

/* Print the prompt line appropriate for the current state */
static void show_prompt(void)
{
    switch (ts) {
        case TS_ROOT:          tp("TRX> ");   break;
        case TS_VFO:           tp("VFO> ");   break;
        case TS_MODE:          tp("MODE> ");  break;
        case TS_CW:            tp("CW> ");    break;
        case TS_TX:            tp("TX> ");    break;
        case TS_CAT:           tp("CAT> ");   break;
        case TS_EDIT_FREQ:     tp("Freq Hz: ");   break;
        case TS_EDIT_BAND:     tp("Band #: ");    break;
        case TS_EDIT_STEP:     tp("Step #: ");    break;
        case TS_EDIT_MODE:     tp("Mode #: ");    break;
        case TS_EDIT_BW:       tp("BW #: ");      break;
        case TS_EDIT_BFO:      tp("BFO Hz: ");    break;
        case TS_EDIT_RIT:      tp("RIT> ");        break;
        case TS_EDIT_CW_WPM:   tp("WPM: ");       break;
        case TS_EDIT_CAL:      tp("Cal ppb: ");   break;
        case TS_EDIT_SPLIT:    tp("SPLIT> ");      break;
        case TS_EDIT_VFOB:     tp("VFO-B Hz: ");  break;
        case TS_EDIT_MEM_PLAY: tp("Slot #: ");    break;
        case TS_EDIT_MEM_EDIT: tp("Slot #: ");    break;
        case TS_EDIT_MEM_TXT:  tp("Text: ");      break;
        default:               tp("> ");           break;
    }
}

/* Re-display the current menu (ENTER with empty line) */
static void redraw_current(void)
{
    switch (ts) {
        case TS_ROOT:       show_root();     break;
        case TS_VFO:        show_vfo_menu(); break;
        case TS_MODE:       show_mode_menu(); break;
        case TS_CW:         show_cw_menu();  break;
        case TS_TX:         show_tx_menu();  break;
        default: break;
    }
    show_prompt();
}

/* =========================================================================
 * Shortcut parser (root-level one-liners)
 * ========================================================================= */
static void shortcut(const char *line)
{
    char cmd[24] = {0}, arg[24] = {0};
    sscanf(line, "%23s %23s", cmd, arg);
    /* lowercase the command token */
    for (int i = 0; cmd[i]; i++) cmd[i] = (char)tolower((unsigned char)cmd[i]);

    if (!strcmp(cmd, "f") && arg[0]) {
        uint32_t f = (uint32_t)atol(arg);
        if (f >= 100000 && f <= 60000000) {
            vfo.freq_hz = f;
            vfo_auto_band(f); vfo_auto_mode(f); vfo_apply(); screen_draw_main();
            tpf("  Freq: %lu Hz" CRLF, (unsigned long)f);
        } else {
            tp("  Range: 100 kHz – 60 MHz" CRLF);
        }
    }
    else if (!strcmp(cmd, "b") && arg[0]) {
        int i = atoi(arg) - 1;
        if (i >= 0 && i < NUM_BANDS) { vfo_set_band(i); screen_draw_main(); tpf("  Band: %s" CRLF, bands[i].name); }
        else tpf("  1-%d" CRLF, NUM_BANDS);
    }
    else if (!strcmp(cmd, "m") && arg[0]) {
        int i = atoi(arg) - 1;
        if (i >= 0 && i < (int)MODE_COUNT) { vfo.mode = (radio_mode_t)i; vfo_apply(); screen_draw_main(); tpf("  Mode: %s" CRLF, mode_names[i]); }
        else tpf("  1-%d" CRLF, (int)MODE_COUNT);
    }
    else if (!strcmp(cmd, "s") && arg[0]) {
        vfo.step_hz = (uint32_t)atol(arg);
        tpf("  Step: %lu Hz" CRLF, (unsigned long)vfo.step_hz);
    }
    else if (!strcmp(cmd, "w") && arg[0]) {
        uint32_t w = (uint32_t)atol(arg);
        if (w >= CW_WPM_MIN && w <= CW_WPM_MAX) { cw_wpm = w; vfo.cw_wpm = w; tpf("  %lu WPM" CRLF, (unsigned long)w); }
        else tp("  Range: 5-40 WPM" CRLF);
    }
    else if (!strcmp(cmd, "r") && arg[0]) {
        vfo.rit_hz = (int32_t)atol(arg); vfo_apply();
        tpf("  RIT: %+ld Hz" CRLF, (long)vfo.rit_hz);
    }
    else if (!strcmp(cmd, "cal") && arg[0]) {
        si5351_cal_ppb = (int32_t)atol(arg); vfo.cal_ppb = si5351_cal_ppb; vfo_apply();
        tpf("  Cal: %+ld ppb" CRLF, (long)si5351_cal_ppb);
    }
    else if (!strcmp(cmd, "agc"))  { vfo.agc_on = !vfo.agc_on; vfo_apply(); tpf("  AGC: %s" CRLF, vfo.agc_on ? "ON" : "OFF"); }
    else if (!strcmp(cmd, "rit"))  { vfo.rit_on = !vfo.rit_on; vfo_apply(); tpf("  RIT: %s" CRLF, vfo.rit_on ? "ON" : "OFF"); }
    else if (!strcmp(cmd, "spl") || !strcmp(cmd, "split")) {
        vfo.split = !vfo.split;
        if (vfo.split) vfo.vfob_hz = vfo.freq_hz;
        screen_draw_main(); tpf("  Split: %s" CRLF, vfo.split ? "ON" : "OFF");
    }
    else if (!strcmp(cmd, "tx"))  { trx_go_tx(); tp("  TX" CRLF); }
    else if (!strcmp(cmd, "rx"))  { trx_go_rx(); tp("  RX" CRLF); }
    else if (!strcmp(cmd, "mem") && arg[0]) {
        int slot = atoi(arg) - 1;
        if (slot >= 0 && slot < CW_MEM_SLOTS) { cw_mem_play(slot); tpf("  Playing: %s" CRLF, cw_mem[slot]); }
        else tpf("  1-%d" CRLF, CW_MEM_SLOTS);
    }
    else if (!strcmp(cmd, "save")) {
        vfo.cw_wpm = cw_wpm; vfo.cal_ppb = si5351_cal_ppb; vfo_save();
        tp("  Saved." CRLF);
    }
    else if (!strcmp(cmd, "stat") || !strcmp(cmd, "status")) { show_status(); }
    else if (!strcmp(cmd, "cat"))  {
        nav_push(TS_CAT);
        tp("  CAT passthrough.  Q<Enter> or MENU<Enter> to return." CRLF);
    }
    else if (!strcmp(cmd, "?") || !strcmp(cmd, "help")) { show_help(); }
    /* Debug commands – clearly labelled, no production side-effects */
    else if (!strcmp(cmd, "test_tx")) {
        tpf("  [DEBUG] Enabling CLK0 at %lu Hz" CRLF, (unsigned long)vfo.freq_hz);
        si5351_set_freq(0, vfo.freq_hz,4);
        si5351_enable(0, true);
        tp("  CLK0 on. Use  test_tx_off  to stop." CRLF);
    }
    else if (!strcmp(cmd, "test_tx_off")) {
        si5351_enable(0, false);
        tp("  [DEBUG] CLK0 disabled." CRLF);
    }
    else if (!strcmp(cmd, "test_rx")) {
        tpf("  [DEBUG] AM RX test at %lu kHz" CRLF, (unsigned long)(vfo.freq_hz / 1000));
        //si4732_tune_am(vfo.freq_hz);
        //si4732_soft_mute_off();
        tp("  dummy not working" CRLF);
    }
    else {
        tp("  Unknown command.  ? for help." CRLF);
    }

    show_prompt();
}

/* =========================================================================
 * Line processor (called when user hits Enter)
 * ========================================================================= */
static void line_clear(void) { inlen = 0; memset(inbuf, 0, sizeof(inbuf)); }

/* Go back one level and redraw; used by Q and nav_pop callers */
static void go_back(void)
{
    if (!nav_pop()) {
        /* Already at root – show status as a gentle nudge */
        show_status();
    }
    redraw_current();
}

static void process_line(void)
{
    char line[64];
    strncpy(line, inbuf, sizeof(line) - 1);
    line[sizeof(line) - 1] = 0;
    line_clear();

    /* Q at any level means "go back" – QMX-style */
    if ((line[0] == 'q' || line[0] == 'Q') && line[1] == 0) {
        go_back();
        return;
    }

    int ch = atoi(line);   /* numeric selection; 0 if non-numeric */

    switch (ts) {

        /* ---- ROOT ------------------------------------------------------ */
        case TS_ROOT:
            if (!line[0]) { show_root(); tp("TRX> "); break; }
            if (ch == 1) { nav_push(TS_VFO);  show_vfo_menu();  show_prompt(); break; }
            if (ch == 2) { nav_push(TS_MODE); show_mode_menu(); show_prompt(); break; }
            if (ch == 3) { nav_push(TS_CW);   show_cw_menu();  show_prompt(); break; }
            if (ch == 4) { nav_push(TS_TX);   show_tx_menu();  show_prompt(); break; }
            if (ch == 5) { show_status(); tp("TRX> "); break; }
            if (ch == 6) {
                nav_push(TS_CAT);
                tp("  CAT passthrough.  Q<Enter> or MENU<Enter> to return." CRLF "CAT> ");
                break;
            }
            if (ch == 9) {
                vfo.cw_wpm = cw_wpm; vfo.cal_ppb = si5351_cal_ppb; vfo_save();
                tp("  Saved." CRLF "TRX> ");
                break;
            }
            shortcut(line);   /* try one-liner shortcut */
            break;

        /* ---- VFO ------------------------------------------------------- */
        case TS_VFO:
            if (!line[0]) { show_vfo_menu(); show_prompt(); break; }
            if (ch == 1) { nav_push(TS_EDIT_FREQ); tpf("  Freq (%lu): ", (unsigned long)vfo.freq_hz); break; }
            if (ch == 2) {
                nav_push(TS_EDIT_BAND);
                for (int i = 0; i < NUM_BANDS; i++) tpf("  %2d  %s" CRLF, i+1, bands[i].name);
                tp("  Band #: "); break;
            }
            if (ch == 3) {
                nav_push(TS_EDIT_STEP);
                for (int i = 0; i < STEP_COUNT; i++) tpf("  %d  %lu Hz" CRLF, i+1, (unsigned long)step_table[i]);
                tp("  Step #: "); break;
            }
            if (ch == 4) {
                nav_push(TS_EDIT_SPLIT);
                tpf("  Split: %s   VFO-B: %lu Hz" CRLF, vfo.split?"ON":"OFF", (unsigned long)vfo.vfob_hz);
                tp("  1 Toggle   2 Set VFO-B   Q Back" CRLF "SPLIT> "); break;
            }
            if (ch == 5) {
                nav_push(TS_EDIT_RIT);
                tpf("  RIT: %s   Offset: %+ld Hz" CRLF, vfo.rit_on?"ON":"OFF", (long)vfo.rit_hz);
                tp("  1 Toggle   2 Set offset   3 Clear   Q Back" CRLF "RIT> "); break;
            }
            tp("  1-5 or Q" CRLF); show_prompt(); break;

        /* ---- MODE ------------------------------------------------------ */
        case TS_MODE:
            if (!line[0]) { show_mode_menu(); show_prompt(); break; }
            if (ch == 1) {
                nav_push(TS_EDIT_MODE);
                for (int i = 0; i < (int)MODE_COUNT; i++)
                    tpf("  %d  %s%s" CRLF, i+1, mode_names[i], (radio_mode_t)i == vfo.mode ? " <--" : "");
                tp("  Mode #: "); break;
            }
            if (ch == 2) {
                nav_push(TS_EDIT_BW);
                bool am = (vfo.mode == MODE_AM);
                int cnt = am ? AM_BW_COUNT : SSB_BW_COUNT;
                for (int i = 0; i < cnt; i++)
                    tpf("  %d  %s%s" CRLF, i+1, am?am_bw[i].name:ssb_bw[i].name, i==vfo.bw_idx?" <--":"");
                tp("  BW #: "); break;
            }
            if (ch == 3) { vfo.agc_on = !vfo.agc_on; vfo_apply(); tpf("  AGC: %s" CRLF, vfo.agc_on?"ON":"OFF"); show_prompt(); break; }
            if (ch == 4) { nav_push(TS_EDIT_BFO); tpf("  BFO (%+d): ", vfo.bfo_offset); break; }
            tp("  1-4 or Q" CRLF); show_prompt(); break;

        /* ---- CW -------------------------------------------------------- */
        case TS_CW:
            if (!line[0]) { show_cw_menu(); show_prompt(); break; }
            if (ch == 1) { nav_push(TS_EDIT_CW_WPM); tpf("  Speed (%lu WPM): ", (unsigned long)cw_wpm); break; }
            if (ch == 2) {
                nav_push(TS_EDIT_MEM_PLAY);
                for (int i = 0; i < CW_MEM_SLOTS; i++) tpf("  %d  %s" CRLF, i+1, cw_mem[i]);
                tp("  Play slot #: "); break;
            }
            if (ch == 3) {
                nav_push(TS_EDIT_MEM_EDIT);
                for (int i = 0; i < CW_MEM_SLOTS; i++) tpf("  %d  %s" CRLF, i+1, cw_mem[i]);
                tp("  Edit slot #: "); break;
            }
            tp("  1-3 or Q" CRLF); show_prompt(); break;

        /* ---- TX -------------------------------------------------------- */
        case TS_TX:
            if (!line[0]) { show_tx_menu(); show_prompt(); break; }
            if (ch == 1) { trx_go_tx(); tp("  TX" CRLF); }
            else if (ch == 2) { trx_go_rx(); tp("  RX" CRLF); }
            else if (ch == 3) { trx_toggle(); tpf("  %s" CRLF, trx_state==SEQ_TX?"TX":"RX"); }
            show_prompt(); break;

        /* ---- EDIT: frequency ------------------------------------------- */
        case TS_EDIT_FREQ:
            if (line[0]) {
                uint32_t f = (uint32_t)atol(line);
                if (f >= 100000 && f <= 60000000) {
                    vfo.freq_hz = f; vfo_auto_band(f); vfo_auto_mode(f); vfo_apply(); screen_draw_main();
                    tpf("  Set: %lu Hz" CRLF, (unsigned long)f);
                } else tp("  Range: 100 kHz – 60 MHz" CRLF);
            }
            nav_pop(); show_prompt(); break;

        /* ---- EDIT: band ------------------------------------------------ */
        case TS_EDIT_BAND:
            if (line[0]) {
                int i = ch - 1;
                if (i >= 0 && i < NUM_BANDS) { vfo_set_band(i); screen_draw_main(); tpf("  Band: %s" CRLF, bands[i].name); }
                else tpf("  1-%d" CRLF, NUM_BANDS);
            }
            nav_pop(); show_prompt(); break;

        /* ---- EDIT: step ------------------------------------------------ */
        case TS_EDIT_STEP:
            if (line[0]) {
                int i = ch - 1;
                if (i >= 0 && i < STEP_COUNT) { vfo.step_idx = i; vfo.step_hz = step_table[i]; tpf("  Step: %lu Hz" CRLF, (unsigned long)vfo.step_hz); }
                else tp("  1-8" CRLF);
            }
            nav_pop(); show_prompt(); break;

        /* ---- EDIT: split ----------------------------------------------- */
        case TS_EDIT_SPLIT:
            if (!line[0]) { show_prompt(); break; }
            if (ch == 1) {
                vfo.split = !vfo.split;
                if (vfo.split) vfo.vfob_hz = vfo.freq_hz;
                screen_draw_main(); tpf("  Split: %s" CRLF, vfo.split?"ON":"OFF");
                show_prompt(); break;
            }
            if (ch == 2) { nav_push(TS_EDIT_VFOB); tpf("  VFO-B (%lu): ", (unsigned long)vfo.vfob_hz); break; }
            show_prompt(); break;

        /* ---- EDIT: VFO-B ----------------------------------------------- */
        case TS_EDIT_VFOB:
            if (line[0]) {
                uint32_t f = (uint32_t)atol(line);
                if (f >= 100000 && f <= 60000000) { vfo.vfob_hz = f; screen_draw_main(); tpf("  VFO-B: %lu Hz" CRLF, (unsigned long)f); }
                else tp("  Range error" CRLF);
            }
            nav_pop(); nav_pop(); show_prompt(); break;   /* back past EDIT_SPLIT to VFO */

        /* ---- EDIT: RIT ------------------------------------------------- */
        case TS_EDIT_RIT:
            if (!line[0]) { show_prompt(); break; }
            if (ch == 1) { vfo.rit_on = !vfo.rit_on; vfo_apply(); tpf("  RIT: %s" CRLF, vfo.rit_on?"ON":"OFF"); show_prompt(); break; }
            if (ch == 2) { tp("  Offset (Hz, e.g. +250): "); break; }   /* stay in state, next line is offset */
            if (ch == 3) { vfo.rit_hz = 0; vfo.rit_on = false; vfo_apply(); tp("  RIT cleared." CRLF); show_prompt(); break; }
            /* any other non-empty: treat as offset value */
            if (line[0]) { vfo.rit_hz = (int32_t)atol(line); vfo_apply(); tpf("  RIT: %+ld Hz" CRLF, (long)vfo.rit_hz); show_prompt(); break; }
            show_prompt(); break;

        /* ---- EDIT: mode ------------------------------------------------ */
        case TS_EDIT_MODE:
            if (line[0]) {
                int i = ch - 1;
                if (i >= 0 && i < (int)MODE_COUNT) { vfo.mode = (radio_mode_t)i; vfo_apply(); screen_draw_main(); tpf("  Mode: %s" CRLF, mode_names[i]); }
                else tpf("  1-%d" CRLF, (int)MODE_COUNT);
            }
            nav_pop(); show_prompt(); break;

        /* ---- EDIT: bandwidth ------------------------------------------- */
        case TS_EDIT_BW: {
            bool am = (vfo.mode == MODE_AM);
            int cnt = am ? AM_BW_COUNT : SSB_BW_COUNT;
            if (line[0]) {
                int i = ch - 1;
                if (i >= 0 && i < cnt) { vfo.bw_idx = i; vfo_apply(); tpf("  BW: %s" CRLF, am?am_bw[i].name:ssb_bw[i].name); }
                else tpf("  1-%d" CRLF, cnt);
            }
            nav_pop(); show_prompt(); break;
        }

        /* ---- EDIT: BFO ------------------------------------------------- */
        case TS_EDIT_BFO:
            if (line[0]) { vfo.bfo_offset = (int16_t)atoi(line); vfo_apply(); tpf("  BFO: %+d Hz" CRLF, vfo.bfo_offset); }
            nav_pop(); show_prompt(); break;

        /* ---- EDIT: CW WPM ---------------------------------------------- */
        case TS_EDIT_CW_WPM:
            if (line[0]) {
                uint32_t w = (uint32_t)atol(line);
                if (w >= CW_WPM_MIN && w <= CW_WPM_MAX) { cw_wpm = w; vfo.cw_wpm = w; tpf("  %lu WPM" CRLF, (unsigned long)w); }
                else tp("  Range: 5-40" CRLF);
            }
            nav_pop(); show_prompt(); break;

        /* ---- EDIT: calibration ----------------------------------------- */
        case TS_EDIT_CAL:
            if (line[0]) { si5351_cal_ppb = (int32_t)atol(line); vfo.cal_ppb = si5351_cal_ppb; vfo_apply(); tpf("  Cal: %+ld ppb" CRLF, (long)si5351_cal_ppb); }
            nav_pop(); show_prompt(); break;

        /* ---- EDIT: CW mem play ----------------------------------------- */
        case TS_EDIT_MEM_PLAY:
            if (line[0]) {
                int i = ch - 1;
                if (i >= 0 && i < CW_MEM_SLOTS) { cw_mem_play(i); tpf("  Playing: %s" CRLF, cw_mem[i]); }
                else tpf("  1-%d" CRLF, CW_MEM_SLOTS);
            }
            nav_pop(); show_prompt(); break;

        /* ---- EDIT: CW mem slot selection for editing ------------------- */
        case TS_EDIT_MEM_EDIT:
            if (line[0]) {
                int i = ch - 1;
                if (i >= 0 && i < CW_MEM_SLOTS) {
                    mem_edit_slot = i;
                    nav_push(TS_EDIT_MEM_TXT);
                    tpf("  Mem %d: [%s]" CRLF "  New text: ", i+1, cw_mem[i]);
                    break;
                }
                tpf("  1-%d" CRLF, CW_MEM_SLOTS);
            }
            nav_pop(); show_prompt(); break;

        /* ---- EDIT: CW mem text entry ----------------------------------- */
        case TS_EDIT_MEM_TXT:
            if (line[0]) {
                strncpy(cw_mem[mem_edit_slot], line, CW_MEM_LEN - 1);
                cw_mem[mem_edit_slot][CW_MEM_LEN - 1] = 0;
                tpf("  Saved to mem %d." CRLF, mem_edit_slot + 1);
            }
            nav_pop(); nav_pop(); show_prompt(); break;   /* back to CW menu */

        /* ---- CAT (should not reach here; handled in terminal_rx) ------- */
        case TS_CAT:
            /* CAT exit via Q is handled in terminal_rx before reaching here */
            show_prompt(); break;

        default:
            nav_top = 0; nav_stack[0] = TS_ROOT;
            show_root(); tp("TRX> ");
            break;
    }
}

/* =========================================================================
 * CAT auto-detection
 * A burst qualifies as CAT only if it:
 *   (a) contains a semicolon, AND
 *   (b) consists *entirely* of uppercase, digits, '+', '-', and ';'
 * This avoids misfiring on menu input that happens to contain a capital
 * letter (e.g. "MENU" command in CAT mode is handled separately).
 * ========================================================================= */
static bool looks_like_cat(const uint8_t *d, uint32_t n)
{
    bool has_semi  = false;
    bool has_upper = false;
    for (uint32_t i = 0; i < n; i++) {
        uint8_t c = d[i];
        if (c == ';')                                  { has_semi = true; continue; }
        if (c >= 'A' && c <= 'Z')                      { has_upper = true; continue; }
        if ((c >= '0' && c <= '9') || c == '+' || c == '-') continue;
        if (c == '\r' || c == '\n')                    continue;
        /* Any other character (lowercase, space, etc.) → not a CAT burst */
        return false;
    }
    return has_semi && has_upper;
}

/* =========================================================================
 * Public RX dispatcher
 * ========================================================================= */
void terminal_rx(const uint8_t *data, uint32_t len)
{
    /* ---- CAT passthrough mode ------------------------------------------ */
    if (ts == TS_CAT) {
        /* Accumulate into a small line buffer to check for exit commands */
        static char exit_buf[8];
        static int  exit_len = 0;
        for (uint32_t i = 0; i < len; i++) {
            char c = (char)data[i];
            if (c == '\r' || c == '\n') {
                exit_buf[exit_len] = 0;
                /* Accept both "Q" (QMX style) and "MENU" (legacy) to exit */
                if (!strcmp(exit_buf, "Q") || !strcmp(exit_buf, "q") ||
                    !strcmp(exit_buf, "MENU") || !strcmp(exit_buf, "menu")) {
                    exit_len = 0;
                    nav_pop();
                    tp(CRLF);
                    show_root();
                    show_status();
                    tp("TRX> ");
                    return;
                }
                exit_len = 0;
            } else {
                if (exit_len < 7) exit_buf[exit_len++] = c;
                else exit_len = 0;   /* too long – not an exit command */
            }
            cat_feed_byte(c);
        }
        return;
    }

    /* ---- Auto-detect CAT burst (not in passthrough mode) --------------- */
    if (looks_like_cat(data, len)) {
        for (uint32_t i = 0; i < len; i++) cat_feed_byte((char)data[i]);
        return;
    }

    /* ---- Interactive terminal ------------------------------------------ */
    for (uint32_t i = 0; i < len; i++) {
        char c = (char)data[i];

        /* Swallow ESC sequences (arrow keys, function keys, etc.) without
         * acting on them.  Previously Up/Down arrow triggered navigation;
         * now Q<Enter> is used instead, which works on all terminals. */
        static uint8_t esc_st = 0;
        if (c == '\033') { esc_st = 1; continue; }
        if (esc_st == 1) { esc_st = (c == '[') ? 2 : 0; continue; }
        if (esc_st == 2) { esc_st = 0; continue; }   /* consume final byte */

        /* Backspace */
        if (c == '\b' || c == 0x7F) {
            if (inlen > 0) { inlen--; inbuf[inlen] = 0; tp("\b \b"); }
            continue;
        }
        /* Printable character */
        if (c >= 0x20 && c != 0x7F) {
            if (inlen < (int)(sizeof(inbuf) - 1)) {
                inbuf[inlen++] = c;
                inbuf[inlen]   = 0;
                char s[2] = {c, 0};
                tp(s);          /* local echo */
            }
            continue;
        }
        /* Enter */
        if (c == '\r' || c == '\n') {
            tp(CRLF);
            process_line();
            continue;
        }
    }
}

/* =========================================================================
 * Called when USB CDC connects
 * ========================================================================= */
void terminal_connected(void)
{
    nav_top = 0;
    nav_stack[0] = TS_ROOT;
    line_clear();
    sleep_ms(50);
    show_root();
    show_status();
    tp("TRX> ");
}
