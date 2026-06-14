/*
 * ui/terminal.c
 * QMX-style serial terminal + Kenwood TS-480 CAT over USB CDC.
 *
 * STAGE 7 — after basic radio works, add terminal for headless control.
 *
 * Routing logic:
 *   • Data with uppercase letters + ';' → CAT parser
 *   • Anything else → interactive terminal menu
 *   • In CAT mode: all data → CAT. Type "MENU<CR>" to return.
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
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

/* ---- CDC ring buffer -------------------------------------------------- */
#define RING_SZ   4096
#define RING_MASK (RING_SZ-1)
static char     ring[RING_SZ];
static uint32_t rhead=0, rtail=0;

static void ring_push(const char *s, int n) {
    uint32_t fr = RING_SZ-1-((rhead-rtail)&RING_MASK);
    uint32_t wr = (uint32_t)n < fr ? (uint32_t)n : fr;
    for (uint32_t i=0;i<wr;i++) { ring[rhead&RING_MASK]=s[i]; rhead++; }
}

int term_printf(const char *fmt, ...) {
    char b[256]; va_list a; va_start(a,fmt);
    int n=vsnprintf(b,sizeof(b),fmt,a); va_end(a);
    if (n>0) ring_push(b,n); return n;
}

void terminal_task(void) {
    if (!tud_cdc_connected()) return;
    uint32_t av=(rhead-rtail)&RING_MASK; if(!av) return;
    uint8_t b[64];
    uint32_t n=av<sizeof(b)?av:(uint32_t)sizeof(b);
    for(uint32_t i=0;i<n;i++){b[i]=(uint8_t)ring[rtail&RING_MASK];rtail++;}
    tud_cdc_write(b,n); tud_cdc_write_flush();
}

/* ---- CAT parser ------------------------------------------------------- */
#define CAT_BUF 64
static char cat_buf[CAT_BUF]; static int cat_len=0;

static int mode_to_cat(radio_mode_t m) {
    switch(m){case MODE_LSB:return 1;case MODE_USB:return 2;
              case MODE_CW:return 3;case MODE_FM:return 4;
              case MODE_AM:return 5;default:return 2;}
}
static radio_mode_t cat_to_mode(int n) {
    switch(n){case 1:return MODE_LSB;case 2:return MODE_USB;
              case 3:return MODE_CW;case 4:return MODE_FM;
              case 5:return MODE_AM;default:return MODE_USB;}
}

static void cat_send(const char *s) { ring_push(s,(int)strlen(s)); }

static void cat_exec(const char *cmd, int len) {
    if (len<2) return;
    char r[40];
    if (cmd[0]=='F'&&cmd[1]=='A') {
        if (len==2){ snprintf(r,sizeof(r),"FA%011lu;",(unsigned long)vfo.freq_hz);cat_send(r);}
        else if(len==13){ uint32_t f=(uint32_t)atol(cmd+2); vfo.freq_hz=f;
            vfo_auto_band(f);vfo_auto_mode(f);vfo_apply();screen_draw_main(); }
        return;
    }
    if (cmd[0]=='F'&&cmd[1]=='B') {
        if (len==2){ snprintf(r,sizeof(r),"FB%011lu;",(unsigned long)vfo.vfob_hz);cat_send(r);}
        else if(len==13) vfo.vfob_hz=(uint32_t)atol(cmd+2);
        return;
    }
    if (cmd[0]=='M'&&cmd[1]=='D') {
        if (len==2){ snprintf(r,sizeof(r),"MD%d;",mode_to_cat(vfo.mode));cat_send(r);}
        else if(len==3){ vfo.mode=cat_to_mode(cmd[2]-'0');vfo_apply();screen_draw_main(); }
        return;
    }
    if (cmd[0]=='I'&&cmd[1]=='F') {
        snprintf(r,sizeof(r),"IF%011lu     %+05ld0000%d000000000;",
                 (unsigned long)vfo.freq_hz,(long)vfo.rit_hz,mode_to_cat(vfo.mode));
        cat_send(r); return;
    }
    if (cmd[0]=='S'&&cmd[1]=='M') {
        int8_t rssi=si4732_get_rssi();
        int sm=(rssi+127)*30/127; if(sm<0)sm=0;if(sm>30)sm=30;
        snprintf(r,sizeof(r),"SM0%04d;",sm); cat_send(r); return;
    }
    if (cmd[0]=='R'&&cmd[1]=='X'){ trx_go_rx(); cat_send("RX0;"); return; }
    if (cmd[0]=='T'&&cmd[1]=='X'){ trx_go_tx(); cat_send("TX0;"); return; }
    if (cmd[0]=='I'&&cmd[1]=='D'){ cat_send("ID019;"); return; }
    if (cmd[0]=='P'&&cmd[1]=='S'){ cat_send("PS1;");   return; }
    if (cmd[0]=='A'&&cmd[1]=='I'){ cat_send("AI0;");   return; }
    if (cmd[0]=='F'&&(cmd[1]=='R'||cmd[1]=='T')){snprintf(r,sizeof(r),"%c%c0;",cmd[0],cmd[1]);cat_send(r);return;}
    if (cmd[0]=='R'&&cmd[1]=='A'){ cat_send("RA00;");   return; }
    if (cmd[0]=='S'&&cmd[1]=='H'){ cat_send("SH0000;"); return; }
    if (cmd[0]=='V'&&cmd[1]=='X'){ cat_send("VX0;");    return; }
    if (cmd[0]=='A'&&cmd[1]=='G'){ cat_send("AG0000;"); return; }
    cat_send("?;");
}

static void cat_feed_byte(char c) {
    if (cat_len<CAT_BUF-1) cat_buf[cat_len++]=c;
    if (c==';'){ cat_buf[cat_len-1]=0; cat_exec(cat_buf,cat_len-1); cat_len=0; }
}

/* ---- Terminal state machine ------------------------------------------- */
typedef enum {
    TS_ROOT,TS_VFO,TS_MODE,TS_CW,TS_TX,TS_CAT,
    TS_EDIT_FREQ,TS_EDIT_BAND,TS_EDIT_STEP,
    TS_EDIT_MODE,TS_EDIT_BW,TS_EDIT_BFO,
    TS_EDIT_RIT,TS_EDIT_CW_WPM,TS_EDIT_CAL,
    TS_EDIT_SPLIT,TS_EDIT_VFOB,TS_EDIT_MEM,TS_EDIT_MEM_TXT,
} ts_t;

static ts_t ts=TS_ROOT, ts_par=TS_ROOT;
static char inbuf[48]; static int inlen=0;
static int  mem_edit_slot=0;

#define CRLF "\r\n"
#define tp(s) ring_push(s,(int)strlen(s))
#define tpf(...) do{ char _b[160];snprintf(_b,sizeof(_b),__VA_ARGS__);ring_push(_b,(int)strlen(_b));}while(0)

static void thr(void) { tp("----------------------------------------"CRLF); }

static void show_status(void) {
    thr();
    tpf("  Freq  : %lu Hz  (%s)"CRLF,(unsigned long)vfo.freq_hz,bands[vfo.band_idx].name);
    tpf("  Mode  : %s   BW: %s"CRLF, mode_names[vfo.mode],
        vfo.mode==MODE_FM?"FM":vfo.mode==MODE_AM?am_bw[vfo.bw_idx%AM_BW_COUNT].name:
        ssb_bw[vfo.bw_idx%SSB_BW_COUNT].name);
    tpf("  Step  : %lu Hz"CRLF,(unsigned long)vfo.step_hz);
    tpf("  RIT   : %s  %+ld Hz"CRLF,vfo.rit_on?"ON":"OFF",(long)vfo.rit_hz);
    tpf("  Split : %s  VFO-B %lu Hz"CRLF,vfo.split?"ON":"OFF",(unsigned long)vfo.vfob_hz);
    tpf("  AGC   : %s   BFO: %+d Hz"CRLF,vfo.agc_on?"ON":"OFF",vfo.bfo_offset);
    tpf("  CW    : %lu WPM   Cal: %+ld ppb"CRLF,(unsigned long)cw_wpm,(long)si5351_cal_ppb);
    tpf("  TX/RX : %s   RSSI: %d dBm"CRLF,trx_state==SEQ_TX?"TX":"RX",(int)si4732_get_rssi());
    thr();
}

static void show_root(void) {
    tp("\033[2J\033[H");
    tp("\033[1m  *** RP2040 HF TRX ***\033[0m"CRLF);
    thr();
    tp("  1 VFO/Freq  2 Mode  3 CW  4 TX/RX"CRLF);
    tp("  5 Status    6 CAT mode"CRLF);
    tp("  9 Save to flash"CRLF);
    tp("  ? Help/shortcuts"CRLF);
    thr();
    tp("TRX> ");
}

static void show_help(void) {
    thr();
    tp("  f <hz>      set freq      e.g. f 14074000"CRLF);
    tp("  b <1-11>    set band      e.g. b 6"CRLF);
    tp("  m <1-7>     set mode      e.g. m 2"CRLF);
    tp("  s <hz>      set step      e.g. s 1000"CRLF);
    tp("  w <wpm>     CW speed      e.g. w 20"CRLF);
    tp("  r <hz>      RIT offset    e.g. r +250"CRLF);
    tp("  cal <ppb>   xtal cal      e.g. cal -120"CRLF);
    tp("  agc         toggle AGC"CRLF);
    tp("  rit         toggle RIT"CRLF);
    tp("  spl         toggle Split"CRLF);
    tp("  tx / rx     PTT"CRLF);
    tp("  mem <n>     play CW mem   e.g. mem 1"CRLF);
    tp("  save        save to flash"CRLF);
    tp("  stat        status"CRLF);
    tp("  cat         CAT mode (MENU<CR> to return)"CRLF);
    thr();
}

/* One-line shortcut parser (root level) */
static void shortcut(const char *line) {
    char cmd[24]={0},arg[24]={0};
    sscanf(line,"%23s %23s",cmd,arg);
    for(int i=0;cmd[i];i++) if(cmd[i]>='A'&&cmd[i]<='Z') cmd[i]+=32;

    if(!strcmp(cmd,"f")&&arg[0]){ uint32_t f=(uint32_t)atol(arg);
        if(f>=100000&&f<=60000000){vfo.freq_hz=f;vfo_auto_band(f);vfo_auto_mode(f);vfo_apply();screen_draw_main();tpf("  %lu Hz"CRLF,(unsigned long)f);}
        else tp("  Range 100kHz-60MHz"CRLF); }
    else if(!strcmp(cmd,"b")&&arg[0]){ int i=atoi(arg)-1;
        if(i>=0&&i<NUM_BANDS){vfo_set_band(i);screen_draw_main();tpf("  %s"CRLF,bands[i].name);}
        else tpf("  1-%d"CRLF,NUM_BANDS); }
    else if(!strcmp(cmd,"m")&&arg[0]){ int i=atoi(arg)-1;
        if(i>=0&&i<(int)MODE_COUNT){vfo.mode=(radio_mode_t)i;vfo_apply();screen_draw_main();tpf("  %s"CRLF,mode_names[i]);}
        else tpf("  1-%d"CRLF,(int)MODE_COUNT); }
    else if(!strcmp(cmd,"s")&&arg[0]){vfo.step_hz=(uint32_t)atol(arg);tpf("  %lu Hz"CRLF,(unsigned long)vfo.step_hz);}
    else if(!strcmp(cmd,"w")&&arg[0]){ uint32_t w=(uint32_t)atol(arg);
        if(w>=CW_WPM_MIN&&w<=CW_WPM_MAX){cw_wpm=w;vfo.cw_wpm=w;tpf("  %lu WPM"CRLF,(unsigned long)w);}
        else tp("  5-40 WPM"CRLF); }
    else if(!strcmp(cmd,"r")&&arg[0]){vfo.rit_hz=(int32_t)atol(arg);vfo_apply();tpf("  RIT %+ld Hz"CRLF,(long)vfo.rit_hz);}
    else if(!strcmp(cmd,"cal")&&arg[0]){si5351_cal_ppb=(int32_t)atol(arg);vfo.cal_ppb=si5351_cal_ppb;vfo_apply();tpf("  Cal %+ld ppb"CRLF,(long)si5351_cal_ppb);}
    else if(!strcmp(cmd,"agc")){vfo.agc_on=!vfo.agc_on;tpf("  AGC: %s"CRLF,vfo.agc_on?"ON":"OFF");}
    else if(!strcmp(cmd,"rit")){vfo.rit_on=!vfo.rit_on;vfo_apply();tpf("  RIT: %s"CRLF,vfo.rit_on?"ON":"OFF");}
    else if(!strcmp(cmd,"spl")||!strcmp(cmd,"split")){vfo.split=!vfo.split;if(vfo.split)vfo.vfob_hz=vfo.freq_hz;screen_draw_main();tpf("  Split: %s"CRLF,vfo.split?"ON":"OFF");}
    else if(!strcmp(cmd,"tx")){trx_go_tx();tp("  TX"CRLF);}
    else if(!strcmp(cmd,"rx")){trx_go_rx();tp("  RX"CRLF);}
    else if(!strcmp(cmd,"mem")&&arg[0]){ int slot=atoi(arg)-1;
        if(slot>=0&&slot<CW_MEM_SLOTS){cw_mem_play(slot);tpf("  Playing: %s"CRLF,cw_mem[slot]);} }
    else if(!strcmp(cmd,"save")){vfo.cw_wpm=cw_wpm;vfo.cal_ppb=si5351_cal_ppb;vfo_save();tp("  Saved."CRLF);}
    else if(!strcmp(cmd,"stat")||!strcmp(cmd,"status")) show_status();
    else if(!strcmp(cmd,"cat")){ts=TS_CAT;tp("  CAT mode. MENU<CR> to return."CRLF"CAT> ");}
    else if(!strcmp(cmd,"?")||!strcmp(cmd,"help")) show_help();
    else if (!strcmp(cmd, "test_tx")) {
    tpf("Enabling CLK0 at %lu Hz\n", vfo.freq_hz);
    si5351_set_freq(0, vfo.freq_hz);
    si5351_enable(0, true);
    tpf("CLK0 enabled. Use 'test_tx_off' to disable.\n");
}
else if (!strcmp(cmd, "test_tx_off")) {
    si5351_enable(0, false);
    tpf("CLK0 disabled\n");
}
else if (!strcmp(cmd, "test_rx")) {
    tpf("Testing AM receive at %lu kHz\n", vfo.freq_hz / 1000);
    si4732_tune_am(vfo.freq_hz);
    si4732_soft_mute_off();
    tpf("Volume set to 50. Do you hear audio?\n");
}
else if (!strcmp(cmd, "fm105_simple")) {
    tpf("Tuning to 105.0 MHz FM (simplified)\n");
    
    // Power up FM
    uint8_t pu[] = {0x01, 0x50, 0x05};
    i2c_write_blocking(I2C_PORT, SI4732_ADDR, pu, 3, false);
    sleep_ms(200);
    
    // Tune to 105.0 MHz
    uint8_t tune[] = {0x20, 0x00, 0x29, 0x04, 0x00};  // 10500 = 0x2904
    i2c_write_blocking(I2C_PORT, SI4732_ADDR, tune, 5, false);
    sleep_ms(200);
    
    // Unmute and set volume
    uint8_t unmute_vol[] = {0x12, 0x00, 0x40, 0x00, 0x00, 0x3F};  // Volume 63
    i2c_write_blocking(I2C_PORT, SI4732_ADDR, unmute_vol, 6, false);
    
    tpf("Done. Volume at max.\n");
}
    else tp("  Unknown. ? for help."CRLF);
    tp("TRX> ");
}

static void line_clear(void){ inlen=0; memset(inbuf,0,sizeof(inbuf)); }

static void process_line(void) {
    char line[48]; strncpy(line,inbuf,47); line[47]=0; line_clear();
    int ch=atoi(line);

    switch(ts){
        case TS_ROOT:
            if(!line[0]||line[0]=='?') show_root();
            else if(ch==1){ts_par=TS_ROOT;ts=TS_VFO;tpf("  VFO: %lu Hz"CRLF,(unsigned long)vfo.freq_hz);tp("  1 Freq  2 Band  3 Step  4 Split  5 RIT  ESC Back"CRLF"> ");}
            else if(ch==2){ts_par=TS_ROOT;ts=TS_MODE;tpf("  Mode: %s"CRLF,mode_names[vfo.mode]);tp("  1 Mode  2 BW  3 AGC  4 BFO  ESC Back"CRLF"> ");}
            else if(ch==3){ts_par=TS_ROOT;ts=TS_CW;tpf("  CW: %lu WPM"CRLF,(unsigned long)cw_wpm);tp("  1 Speed  2 Play mem  3 Edit mem  ESC Back"CRLF"> ");}
            else if(ch==4){ts_par=TS_ROOT;ts=TS_TX;tpf("  %s"CRLF,trx_state==SEQ_TX?"TX":"RX");tp("  1 TX  2 RX  3 Toggle  ESC Back"CRLF"> ");}
            else if(ch==5){ show_status(); tp("TRX> "); }
            else if(ch==6){ts=TS_CAT;tp("  CAT mode. MENU<CR> to return."CRLF"CAT> ");}
            else if(ch==9){vfo.cw_wpm=cw_wpm;vfo.cal_ppb=si5351_cal_ppb;vfo_save();tp("  Saved."CRLF"TRX> ");}
            else shortcut(line);
            break;

        case TS_VFO:
            if(ch==1){ts_par=TS_VFO;ts=TS_EDIT_FREQ;tpf("  Freq (%lu): ",(unsigned long)vfo.freq_hz);}
            else if(ch==2){ts_par=TS_VFO;ts=TS_EDIT_BAND;for(int i=0;i<NUM_BANDS;i++)tpf("  %2d %s"CRLF,i+1,bands[i].name);tp("  Band: ");}
            else if(ch==3){ts_par=TS_VFO;ts=TS_EDIT_STEP;for(int i=0;i<STEP_COUNT;i++)tpf("  %d %lu Hz"CRLF,i+1,(unsigned long)step_table[i]);tp("  Step: ");}
            else if(ch==4){ts_par=TS_VFO;ts=TS_EDIT_SPLIT;tpf("  Split %s  VFO-B %lu"CRLF,vfo.split?"ON":"OFF",(unsigned long)vfo.vfob_hz);tp("  1 Toggle  2 Set B"CRLF"> ");}
            else if(ch==5){ts_par=TS_VFO;ts=TS_EDIT_RIT;tpf("  RIT %s  %+ld"CRLF,vfo.rit_on?"ON":"OFF",(long)vfo.rit_hz);tp("  1 Toggle  2 Set  3 Clear"CRLF"> ");}
            else tp("> ");
            break;

        case TS_MODE:
            if(ch==1){ts_par=TS_MODE;ts=TS_EDIT_MODE;for(int i=0;i<(int)MODE_COUNT;i++)tpf("  %d %s%s"CRLF,i+1,mode_names[i],(radio_mode_t)i==vfo.mode?" <--":"");tp("  Mode: ");}
            else if(ch==2){ts_par=TS_MODE;ts=TS_EDIT_BW;bool am=(vfo.mode==MODE_AM);int cnt=am?AM_BW_COUNT:SSB_BW_COUNT;for(int i=0;i<cnt;i++)tpf("  %d %s%s"CRLF,i+1,am?am_bw[i].name:ssb_bw[i].name,i==vfo.bw_idx?" <--":"");tp("  BW: ");}
            else if(ch==3){vfo.agc_on=!vfo.agc_on;tpf("  AGC: %s"CRLF,vfo.agc_on?"ON":"OFF");tp("> ");}
            else if(ch==4){ts_par=TS_MODE;ts=TS_EDIT_BFO;tpf("  BFO: %+d Hz"CRLF,vfo.bfo_offset);tp("  New BFO (Hz): ");}
            else tp("> ");
            break;

        case TS_CW:
            if(ch==1){ts_par=TS_CW;ts=TS_EDIT_CW_WPM;tpf("  Speed %lu WPM: ",(unsigned long)cw_wpm);}
            else if(ch==2){ts_par=TS_CW;ts=TS_EDIT_MEM;for(int i=0;i<CW_MEM_SLOTS;i++)tpf("  %d: %s"CRLF,i+1,cw_mem[i]);tp("  Play slot: ");}
            else if(ch==3){ts_par=TS_CW;ts=TS_EDIT_MEM;for(int i=0;i<CW_MEM_SLOTS;i++)tpf("  %d: %s"CRLF,i+1,cw_mem[i]);tp("  Edit slot: ");}
            else tp("> ");
            break;

        case TS_TX:
            if(ch==1){trx_go_tx();tp("  TX"CRLF);}
            else if(ch==2){trx_go_rx();tp("  RX"CRLF);}
            else if(ch==3){trx_toggle();tpf("  %s"CRLF,trx_state==SEQ_TX?"TX":"RX");}
            tp("> "); break;

        case TS_EDIT_FREQ:
            if(line[0]){uint32_t f=(uint32_t)atol(line);if(f>=100000&&f<=60000000){vfo.freq_hz=f;vfo_auto_band(f);vfo_auto_mode(f);vfo_apply();screen_draw_main();tpf("  %lu Hz"CRLF,(unsigned long)f);}else tp("  Out of range"CRLF);}
            ts=TS_VFO; tp("> "); break;

        case TS_EDIT_BAND:
            if(line[0]){int i=ch-1;if(i>=0&&i<NUM_BANDS){vfo_set_band(i);screen_draw_main();tpf("  %s"CRLF,bands[i].name);}else tpf("  1-%d"CRLF,NUM_BANDS);}
            ts=TS_VFO; tp("> "); break;

        case TS_EDIT_STEP:
            if(line[0]){int i=ch-1;if(i>=0&&i<STEP_COUNT){vfo.step_idx=i;vfo.step_hz=step_table[i];tpf("  %lu Hz"CRLF,(unsigned long)vfo.step_hz);}else tp("  1-8"CRLF);}
            ts=TS_VFO; tp("> "); break;

        case TS_EDIT_SPLIT:
            if(ch==1){vfo.split=!vfo.split;if(vfo.split)vfo.vfob_hz=vfo.freq_hz;screen_draw_main();tpf("  Split %s"CRLF,vfo.split?"ON":"OFF");tp("> ");}
            else if(ch==2){ts=TS_EDIT_VFOB;tpf("  VFO-B (%lu): ",(unsigned long)vfo.vfob_hz);}
            else tp("> ");
            break;

        case TS_EDIT_VFOB:
            if(line[0]){uint32_t f=(uint32_t)atol(line);if(f>=100000&&f<=60000000){vfo.vfob_hz=f;screen_draw_main();tpf("  VFO-B %lu"CRLF,(unsigned long)f);}else tp("  Range error"CRLF);}
            ts=TS_EDIT_SPLIT; tp("> "); break;

        case TS_EDIT_RIT:
            if(ch==1){vfo.rit_on=!vfo.rit_on;vfo_apply();tpf("  RIT %s"CRLF,vfo.rit_on?"ON":"OFF");tp("> ");}
            else if(ch==2){tp("  Offset (Hz): ");}
            else if(ch==3){vfo.rit_hz=0;vfo.rit_on=false;vfo_apply();tp("  Cleared"CRLF"> ");}
            else if(line[0]){vfo.rit_hz=(int32_t)atol(line);vfo_apply();tpf("  RIT %+ld Hz"CRLF,(long)vfo.rit_hz);tp("> ");}
            else tp("> ");
            break;

        case TS_EDIT_MODE:
            if(line[0]){int i=ch-1;if(i>=0&&i<(int)MODE_COUNT){vfo.mode=(radio_mode_t)i;vfo_apply();screen_draw_main();tpf("  %s"CRLF,mode_names[i]);}else tpf("  1-%d"CRLF,(int)MODE_COUNT);}
            ts=TS_MODE; tp("> "); break;

        case TS_EDIT_BW:{
            bool am=(vfo.mode==MODE_AM);int cnt=am?AM_BW_COUNT:SSB_BW_COUNT;
            if(line[0]){int i=ch-1;if(i>=0&&i<cnt){vfo.bw_idx=i;vfo_apply();tpf("  %s"CRLF,am?am_bw[i].name:ssb_bw[i].name);}else tpf("  1-%d"CRLF,cnt);}
            ts=TS_MODE; tp("> "); break;}

        case TS_EDIT_BFO:
            if(line[0]){vfo.bfo_offset=(int16_t)atoi(line);vfo_apply();tpf("  BFO %+d Hz"CRLF,vfo.bfo_offset);}
            ts=TS_MODE; tp("> "); break;

        case TS_EDIT_CW_WPM:
            if(line[0]){uint32_t w=(uint32_t)atol(line);if(w>=CW_WPM_MIN&&w<=CW_WPM_MAX){cw_wpm=w;vfo.cw_wpm=w;tpf("  %lu WPM"CRLF,(unsigned long)w);}else tp("  Range 5-40"CRLF);}
            ts=TS_CW; tp("> "); break;

        case TS_EDIT_CAL:
            if(line[0]){si5351_cal_ppb=(int32_t)atol(line);vfo.cal_ppb=si5351_cal_ppb;vfo_apply();tpf("  Cal %+ld ppb"CRLF,(long)si5351_cal_ppb);}
            ts=TS_ROOT; tp("TRX> "); break;

        case TS_EDIT_MEM:
            if(line[0]){int i=ch-1;if(i>=0&&i<CW_MEM_SLOTS){
                if(ts_par==TS_CW){mem_edit_slot=i;ts=TS_EDIT_MEM_TXT;tpf("  Mem %d: %s"CRLF,i+1,cw_mem[i]);tp("  New text: ");}
                else{cw_mem_play(i);tpf("  Playing: %s"CRLF,cw_mem[i]);ts=ts_par;tp("> ");}
            }else{tpf("  1-%d"CRLF,CW_MEM_SLOTS);ts=ts_par;tp("> ");}}
            else{ts=ts_par;tp("> ");}
            break;

        case TS_EDIT_MEM_TXT:
            if(line[0]){strncpy(cw_mem[mem_edit_slot],line,CW_MEM_LEN-1);cw_mem[mem_edit_slot][CW_MEM_LEN-1]=0;tpf("  Saved mem %d"CRLF,mem_edit_slot+1);}
            ts=TS_CW; tp("> "); break;

        default: ts=TS_ROOT; show_root(); break;
    }
}

/* ---- Public RX dispatcher --------------------------------------------- */
static bool looks_like_cat(const uint8_t *d, uint32_t n) {
    bool semi=false,upper=false;
    for(uint32_t i=0;i<n;i++){if(d[i]==';')semi=true;if(d[i]>='A'&&d[i]<='Z')upper=true;}
    return semi&&upper;
}

void terminal_rx(const uint8_t *data, uint32_t len) {
    /* CAT passthrough mode */
    if (ts==TS_CAT) {
        static char magic[5]={0}; static int mlen=0;
        for (uint32_t i=0;i<len;i++) {
            char c=(char)data[i];
            if(c=='\r'||c=='\n'){magic[mlen]=0;if(!strcmp(magic,"MENU")){mlen=0;ts=TS_ROOT;show_root();return;}mlen=0;}
            else if(mlen<4) magic[mlen++]=c; else mlen=0;
            cat_feed_byte(c);
        }
        return;
    }
    /* Auto-detect CAT burst */
    if (looks_like_cat(data,len)) {
        for (uint32_t i=0;i<len;i++) cat_feed_byte((char)data[i]);
        return;
    }
    /* Terminal byte-by-byte */
    static int esc=0;
    for (uint32_t i=0;i<len;i++) {
        char c=(char)data[i];
        if(c=='\033'){esc=1;continue;}
        if(esc==1){esc=(c=='[')?2:0;continue;}
        if(esc==2){esc=0;if(c=='A'||c=='B'){ts=ts_par;if(ts==TS_ROOT)show_root();}continue;}
        if(c=='\b'||c==0x7F){if(inlen>0){inlen--;inbuf[inlen]=0;tp("\b \b");}continue;}
        if(c>=0x20&&c!=0x7F){if(inlen<47){inbuf[inlen++]=c;inbuf[inlen]=0;char s[2]={c,0};tp(s);}continue;}
        if(c=='\r'||c=='\n'){tp(CRLF);process_line();continue;}
    }
}

void terminal_connected(void) {
    ts=TS_ROOT; line_clear();
    sleep_ms(50);
    show_root();
    show_status();
    tp("TRX> ");
}
