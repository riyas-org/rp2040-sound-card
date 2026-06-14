/*
 * ui/menu.c
 * OLED menu system + main VFO screen.
 * Encoder = tune/navigate. Short press = cycle step / confirm.
 * Long press = open next menu item.
 *
 * STAGE 6 — after VFO works.
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "../config.h"
#include "../hal/oled.h"
#include "../hal/si4732.h"
#include "../radio/vfo.h"
#include "../radio/trx.h"
#include "../radio/cw_keyer.h"
#include "menu.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

static menu_id_t active  = MENU_NONE;
static int       sel     = 0;
static uint32_t  timeout = 0;

static const char *menu_names[] = {
    "","Band","Mode","BW","Step","RIT","AGC","CW WPM","BFO","Split","Mem"
};

/* Rolling S-meter average */
static int8_t smeter = -100;

/* ---- Main VFO screen -------------------------------------------------- */
void screen_draw_main(void) {
    oled_clear();

    /* Row 0: band + mode + TX/RX indicator */
    oled_printf(0, 0, false, "%-4s %-3s",
                bands[vfo.band_idx].name, mode_names[vfo.mode]);
    if (trx_state == SEQ_TX) oled_str(14, 0, "[TX]", true);
    else                      oled_str(14, 0, " RX ", false);

    /* Rows 1+2: frequency (printed twice for visual weight) */
    char fstr[18];
    uint32_t f = vfo.freq_hz;
    snprintf(fstr, sizeof(fstr), "%3lu.%03lu.%03lu",
             (unsigned long)(f/1000000),
             (unsigned long)((f%1000000)/1000),
             (unsigned long)(f%1000));
    oled_str(0, 1, fstr, false);
    oled_str(0, 2, fstr, false);

    /* Row 3: step + optional RIT */
    const char *sname = "?";
    switch (vfo.step_hz) {
        case 1:      sname="1Hz";  break; case 10:    sname="10";   break;
        case 100:    sname="100";  break; case 500:   sname="500";  break;
        case 1000:   sname="1k";   break; case 5000:  sname="5k";   break;
        case 10000:  sname="10k";  break; case 100000:sname="100k"; break;
    }
    if (vfo.rit_on)
        oled_printf(0, 3, false, "Stp:%-4s R:%+ld", sname, (long)vfo.rit_hz);
    else
        oled_printf(0, 3, false, "Step: %-5s", sname);

    oled_hline(0, 127, 4);

    /* Row 5: S-meter */
    int8_t r = si4732_get_rssi();
    smeter = (int8_t)((smeter * 3 + r) / 4);
    oled_str(0, 5, "S", false);
    oled_smeter(8, 120, 5, smeter);

    /* Row 6: AGC + BW */
    const char *bwname;
    if      (vfo.mode == MODE_FM) bwname = "FM";
    else if (vfo.mode == MODE_AM) bwname = am_bw[vfo.bw_idx % AM_BW_COUNT].name;
    else                          bwname = ssb_bw[vfo.bw_idx % SSB_BW_COUNT].name;
    oled_printf(0, 6, false, "AGC:%s BW:%-4s",
                vfo.agc_on ? "ON" : "OF", bwname);

    /* Row 7: split / CW decode / blank */
    if (vfo.split) {
        oled_printf(0, 7, false, "SPL:%lu.%03lu",
                    (unsigned long)(vfo.vfob_hz/1000000),
                    (unsigned long)((vfo.vfob_hz%1000000)/1000));
    } else if (vfo.mode == MODE_CW && cw_decoded_len > 0) {
        int start = cw_decoded_len > 20 ? cw_decoded_len - 20 : 0;
        oled_str(0, 7, cw_decoded_buf + start, false);
    } else {
        oled_str(0, 7, "                    ", false);
    }
    oled_flush();
}

/* ---- Menu screens ----------------------------------------------------- */
static void draw_menu(void) {
    oled_clear();
    oled_str(0, 0, menu_names[active], true);
    oled_hline(0, 127, 1);
    char buf[22];
    switch (active) {
        case MENU_BAND:
            for (int i=0;i<NUM_BANDS&&i<6;i++) {
                int idx = (sel>2) ? (sel-2+i)%NUM_BANDS : i;
                oled_str(0, 2+i, bands[idx].name, idx==sel);
            }
            break;
        case MENU_MODE:
            for (int i=0;i<(int)MODE_COUNT&&i<6;i++)
                oled_str(0, 2+i, mode_names[i], i==sel);
            break;
        case MENU_BW: {
            bool am = (vfo.mode==MODE_AM);
            int  cnt = am ? AM_BW_COUNT : SSB_BW_COUNT;
            for (int i=0;i<cnt&&i<6;i++)
                oled_str(0, 2+i, am?am_bw[i].name:ssb_bw[i].name, i==sel);
            break;
        }
        case MENU_STEP:
            for (int i=0;i<STEP_COUNT&&i<6;i++) {
                snprintf(buf,sizeof(buf),"%lu",(unsigned long)step_table[i]);
                oled_str(0,2+i,buf,i==sel);
            }
            break;
        case MENU_RIT:
            oled_printf(0,2,false,"RIT: %s", vfo.rit_on?"ON":"OFF");
            oled_printf(0,3,false,"Ofs:%+4ld Hz",(long)vfo.rit_hz);
            oled_str(0,5,"Enc=adj Btn=toggle",false);
            break;
        case MENU_AGC:
            oled_printf(0,2,true,"AGC: %s",vfo.agc_on?"ON":"OFF");
            oled_str(0,4,"Btn=toggle",false);
            break;
        case MENU_CW_SPEED:
            oled_printf(0,2,false,"CW: %lu WPM",(unsigned long)cw_wpm);
            oled_printf(0,3,false,"Dot: %lu ms",(unsigned long)cw_dot_ms());
            oled_str(0,5,"Enc=chg Btn=ok",false);
            break;
        case MENU_BFO:
            oled_printf(0,2,false,"BFO: %+d Hz",vfo.bfo_offset);
            oled_str(0,4,"Enc=trim Btn=ok",false);
            break;
        case MENU_SPLIT:
            oled_printf(0,2,true,"Split: %s",vfo.split?"ON":"OFF");
            oled_printf(0,3,false,"B:%lu.%03lu",
                        (unsigned long)(vfo.vfob_hz/1000000),
                        (unsigned long)((vfo.vfob_hz%1000000)/1000));
            break;
        case MENU_MEM:
            oled_str(0,2,"CW Memories:",false);
            for (int i=0;i<CW_MEM_SLOTS&&i<5;i++) {
                snprintf(buf,sizeof(buf),"%d:%-16s",i+1,cw_mem[i]);
                oled_str(0,3+i,buf,i==sel);
            }
            break;
        default: break;
    }
    oled_flush();
}

/* ---- Encoder in menu -------------------------------------------------- */
static void menu_enc(int32_t d) {
    timeout = to_ms_since_boot(get_absolute_time()) + MENU_TIMEOUT_MS;
    switch (active) {
        case MENU_BAND:  sel=(sel+(int)d+NUM_BANDS)%NUM_BANDS; break;
        case MENU_MODE:  sel=(sel+(int)d+(int)MODE_COUNT)%(int)MODE_COUNT; break;
        case MENU_BW: {
            int cnt=(vfo.mode==MODE_AM)?AM_BW_COUNT:SSB_BW_COUNT;
            sel=(sel+(int)d+cnt)%cnt; break;
        }
        case MENU_STEP:  sel=(sel+(int)d+STEP_COUNT)%STEP_COUNT; break;
        case MENU_RIT:
            vfo.rit_hz+=d*10;
            if(vfo.rit_hz>9990) vfo.rit_hz=9990;
            if(vfo.rit_hz<-9990) vfo.rit_hz=-9990;
            vfo_request_apply(); break;
        case MENU_CW_SPEED:
            cw_wpm=(uint32_t)((int32_t)cw_wpm+d);
            if(cw_wpm<CW_WPM_MIN) cw_wpm=CW_WPM_MIN;
            if(cw_wpm>CW_WPM_MAX) cw_wpm=CW_WPM_MAX; break;
        case MENU_BFO:
            vfo.bfo_offset+=(int16_t)(d*10); vfo_request_apply(); break;
        case MENU_MEM:
            sel=(sel+(int)d+CW_MEM_SLOTS)%CW_MEM_SLOTS; break;
        default: break;
    }
    draw_menu();
}

/* ---- Button confirm in menu ------------------------------------------- */
static void menu_btn(void) {
    timeout = to_ms_since_boot(get_absolute_time()) + MENU_TIMEOUT_MS;
    switch (active) {
        case MENU_BAND:  vfo_set_band(sel); active=MENU_NONE; break;
        case MENU_MODE:  vfo.mode=(radio_mode_t)sel; vfo_request_apply(); active=MENU_NONE; break;
        case MENU_BW:    vfo.bw_idx=sel; vfo_request_apply(); active=MENU_NONE; break;
        case MENU_STEP:  vfo.step_idx=sel; vfo.step_hz=step_table[sel]; active=MENU_NONE; break;
        case MENU_RIT:   vfo.rit_on=!vfo.rit_on; vfo_request_apply(); break;
        case MENU_AGC:   vfo.agc_on=!vfo.agc_on; break;
        case MENU_CW_SPEED: vfo.cw_wpm=cw_wpm; active=MENU_NONE; break;
        case MENU_BFO:   active=MENU_NONE; break;
        case MENU_SPLIT:
            vfo.split=!vfo.split;
            if(vfo.split) vfo.vfob_hz=vfo.freq_hz; break;
        case MENU_MEM:
            cw_mem_play(sel); active=MENU_NONE; break;
        default: active=MENU_NONE; break;
    }
    if (active==MENU_NONE) screen_draw_main();
    else draw_menu();
}

/* ---- Public task ------------------------------------------------------ */
void menu_init(void) { /* nothing needed */ }

bool menu_is_open(void) { return active != MENU_NONE; }

void menu_task(int32_t enc_delta, bool btn_short, bool btn_long, uint32_t now_ms) {
    /* Long press: cycle to next menu */
    if (btn_long) {
        active = (menu_id_t)(((int)active + 1) % (int)MENU_COUNT);
        if (active == MENU_NONE) active = MENU_BAND;
        switch (active) {
            case MENU_BAND: sel=vfo.band_idx; break;
            case MENU_MODE: sel=(int)vfo.mode; break;
            case MENU_BW:   sel=vfo.bw_idx;   break;
            case MENU_STEP: sel=vfo.step_idx;  break;
            default:        sel=0; break;
        }
        timeout = now_ms + MENU_TIMEOUT_MS;
        draw_menu();
        return;
    }

    if (active != MENU_NONE) {
        if (enc_delta) menu_enc(enc_delta);
        if (btn_short) menu_btn();
        if (now_ms >= timeout) { active=MENU_NONE; screen_draw_main(); }
        return;
    }

    /* Normal VFO tuning */
    if (enc_delta) {
        int64_t nf = (int64_t)vfo.freq_hz + enc_delta*(int64_t)vfo.step_hz;
        if (nf < (int64_t)bands[vfo.band_idx].freq_min) nf=bands[vfo.band_idx].freq_min;
        if (nf > (int64_t)bands[vfo.band_idx].freq_max) nf=bands[vfo.band_idx].freq_max;
        vfo.freq_hz = (uint32_t)nf;
        vfo_auto_mode(vfo.freq_hz);
        vfo_request_apply();
    }
    if (btn_short) {
        vfo.step_idx = (vfo.step_idx + 1) % STEP_COUNT;
        vfo.step_hz  = step_table[vfo.step_idx];
    }
}
