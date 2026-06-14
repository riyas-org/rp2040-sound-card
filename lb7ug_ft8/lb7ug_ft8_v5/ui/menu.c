/*
 * ui/menu.c
 * OLED menu system + main VFO screen.
 * 
 * MENU NAVIGATION:
 * - Long press: Open main menu (shows list of menu categories)
 * - Rotate: Navigate up/down through menu categories
 * - Short press: Enter selected sub-menu
 * - In sub-menu: Rotate to adjust value, Short press to confirm and return
 * - Long press (in any menu): Exit menu completely
 *
 * FIXES:
 * - Added encoder debouncing to prevent touchy selection
 * - Fixed menu freezing with better state machine
 * - Added mutex for OLED access
 * - Prevented re-entrant menu operations
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
#include "terminal.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

/* Debug macro - set to 1 to enable debug output */
#define MENU_DEBUG 1

#if MENU_DEBUG
#define DEBUG_PRINT(fmt, ...) term_printf("[MENU] " fmt "\r\n", ##__VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...) ((void)0)
#endif

/* Menu state */
typedef enum {
    MENU_STATE_MAIN,      // Showing main menu categories
    MENU_STATE_SUB,       // Inside a sub-menu (adjusting value)
    MENU_STATE_NONE       // No menu active
} menu_state_t;

static menu_state_t state = MENU_STATE_NONE;
static menu_id_t    current_menu = MENU_NONE;  // Which sub-menu we're in
static int          selected_index = 0;         // Current selection (main menu or sub-menu value)
static uint32_t     timeout = 0;
static bool         needs_redraw = true;

/* Encoder debouncing */
static int32_t      last_enc_value = 0;
static uint32_t     last_enc_time = 0;
static const uint32_t ENC_DEBOUNCE_MS = 50;  // Debounce time in milliseconds

/* Rolling S-meter average */
static int8_t smeter = -100;

/* Main menu items - these match menu_id_t order */
static const char *main_menu_items[] = {
    "Band",
    "Mode", 
    "Bandwidth",
    "Step",
    "RIT",
    "AGC",
    "CW Speed",
    "BFO",
    "Split",
    "CW Memories"
};

#define MAIN_MENU_COUNT (sizeof(main_menu_items) / sizeof(main_menu_items[0]))

/* Flag to prevent re-entrant menu operations */
static bool menu_busy = false;

/* ---- Helper functions for values --------------------------------------- */
static void apply_setting(void) {
    DEBUG_PRINT("apply_setting(): current_menu=%d, selected_index=%d", current_menu, selected_index);
    
    switch (current_menu) {
        case MENU_BAND:
            DEBUG_PRINT("  Setting band to %d (%s)", selected_index, bands[selected_index].name);
            vfo_set_band(selected_index);
            break;
            
        case MENU_MODE:
            DEBUG_PRINT("  Setting mode to %d (%s)", selected_index, mode_names[selected_index]);
            vfo.mode = (radio_mode_t)selected_index;
            vfo_request_apply();
            break;
            
        case MENU_BW:
            DEBUG_PRINT("  Setting BW index to %d", selected_index);
            vfo.bw_idx = selected_index;
            vfo_request_apply();
            break;
            
        case MENU_STEP:
            DEBUG_PRINT("  Setting step to %lu Hz", (unsigned long)step_table[selected_index]);
            vfo.step_idx = selected_index;
            vfo.step_hz = step_table[selected_index];
            break;
            
        case MENU_RIT:
            /* RIT is toggled, not a selection - handled elsewhere */
            break;
            
        case MENU_AGC:
            /* AGC is toggled */
            break;
            
        case MENU_CW_SPEED:
            DEBUG_PRINT("  Setting CW speed to %d WPM", selected_index + CW_WPM_MIN);
            cw_wpm = selected_index + CW_WPM_MIN;
            vfo.cw_wpm = cw_wpm;
            break;
            
        case MENU_BFO:
            DEBUG_PRINT("  Setting BFO offset to %d Hz", selected_index * 10 - 500);
            vfo.bfo_offset = (int16_t)(selected_index * 10 - 500);
            vfo_request_apply();
            break;
            
        case MENU_SPLIT:
            /* Split is toggled */
            break;
            
        case MENU_MEM:
            DEBUG_PRINT("  Playing CW memory slot %d", selected_index);
            cw_mem_play(selected_index);
            break;
            
        default:
            break;
    }
}

/* Get current value for display in sub-menu */
static int get_current_value(menu_id_t menu) {
    switch (menu) {
        case MENU_BAND:     return vfo.band_idx;
        case MENU_MODE:     return (int)vfo.mode;
        case MENU_BW:       return vfo.bw_idx;
        case MENU_STEP:     return vfo.step_idx;
        case MENU_RIT:      return vfo.rit_on ? 1 : 0;
        case MENU_AGC:      return vfo.agc_on ? 1 : 0;
        case MENU_CW_SPEED: return cw_wpm - CW_WPM_MIN;
        case MENU_BFO:      return (vfo.bfo_offset + 500) / 10;
        case MENU_SPLIT:    return vfo.split ? 1 : 0;
        case MENU_MEM:      return 0;
        default:            return 0;
    }
}

/* Get number of options for a sub-menu */
static int get_menu_max(menu_id_t menu) {
    switch (menu) {
        case MENU_BAND:     return NUM_BANDS;
        case MENU_MODE:     return (int)MODE_COUNT;
        case MENU_BW:       return (vfo.mode == MODE_AM) ? AM_BW_COUNT : SSB_BW_COUNT;
        case MENU_STEP:     return STEP_COUNT;
        case MENU_RIT:      return 2;  // On/Off
        case MENU_AGC:      return 2;  // On/Off
        case MENU_CW_SPEED: return CW_WPM_MAX - CW_WPM_MIN + 1;
        case MENU_BFO:      return 101;  // -500 to +500 in 10Hz steps
        case MENU_SPLIT:    return 2;   // On/Off
        case MENU_MEM:      return CW_MEM_SLOTS;
        default:            return 0;
    }
}

/* Get display string for current value */
static void get_value_string(menu_id_t menu, int value, char *buf, int buflen) {
    switch (menu) {
        case MENU_BAND:
            snprintf(buf, buflen, "%s", bands[value].name);
            break;
        case MENU_MODE:
            snprintf(buf, buflen, "%s", mode_names[value]);
            break;
        case MENU_BW: {
            bool am = (vfo.mode == MODE_AM);
            const char *name = am ? am_bw[value].name : ssb_bw[value].name;
            snprintf(buf, buflen, "%s", name);
            break;
        }
        case MENU_STEP:
            if (step_table[value] >= 1000000)
                snprintf(buf, buflen, "%lu.%03lu MHz", 
                         step_table[value]/1000000, (step_table[value]%1000000)/1000);
            else if (step_table[value] >= 1000)
                snprintf(buf, buflen, "%lu.%03lu kHz", 
                         step_table[value]/1000, step_table[value]%1000);
            else
                snprintf(buf, buflen, "%lu Hz", (unsigned long)step_table[value]);
            break;
        case MENU_RIT:
            snprintf(buf, buflen, "%s (%+ld Hz)", value ? "ON" : "OFF", (long)vfo.rit_hz);
            break;
        case MENU_AGC:
            snprintf(buf, buflen, "%s", value ? "ON" : "OFF");
            break;
        case MENU_CW_SPEED:
            snprintf(buf, buflen, "%d WPM", value + CW_WPM_MIN);
            break;
        case MENU_BFO:
            snprintf(buf, buflen, "%+d Hz", value * 10 - 500);
            break;
        case MENU_SPLIT:
            if (value)
                snprintf(buf, buflen, "ON");
            else
                snprintf(buf, buflen, "OFF");
            break;
        case MENU_MEM:
            snprintf(buf, buflen, "%d: %s", value + 1, cw_mem[value]);
            break;
        default:
            snprintf(buf, buflen, "?");
            break;
    }
}

/* ---- Drawing functions ------------------------------------------------ */
static void draw_main_menu(void) {
    if (menu_busy) return;
    menu_busy = true;
    
    oled_clear();
    oled_str(0, 0, "== MAIN MENU ==", true);
    oled_hline(0, 127, 1);
    
    int start_idx = 0;
    int visible = 6;
    
    /* Scrolling logic */
    if (selected_index >= visible) {
        start_idx = selected_index - visible + 1;
        if (start_idx + visible > MAIN_MENU_COUNT) {
            start_idx = MAIN_MENU_COUNT - visible;
        }
    }
    
    for (int i = 0; i < visible && (start_idx + i) < MAIN_MENU_COUNT; i++) {
        int item_idx = start_idx + i;
        if (item_idx == selected_index) {
            oled_printf(0, 2 + i, true, "> %s", main_menu_items[item_idx]);
        } else {
            oled_printf(0, 2 + i, false, "  %s", main_menu_items[item_idx]);
        }
    }
    
    /* Scroll indicators */
    if (start_idx > 0) {
        oled_str(120, 1, "^", false);
    }
    if (start_idx + visible < MAIN_MENU_COUNT) {
        oled_str(120, 7, "v", false);
    }
    
    oled_flush();
    menu_busy = false;
}

static void draw_sub_menu(void) {
    if (menu_busy) return;
    menu_busy = true;
    
    oled_clear();
    oled_printf(0, 0, true, "%s", main_menu_items[current_menu - 1]);
    oled_hline(0, 127, 1);
    
    char value_str[32];
    get_value_string(current_menu, selected_index, value_str, sizeof(value_str));
    
    /* Show current value */
    oled_printf(0, 2, false, "Value:");
    oled_printf(0, 3, true, "%s", value_str);
    
    /* Show range indicator */
    int max_val = get_menu_max(current_menu);
    oled_printf(0, 5, false, "%d / %d", selected_index + 1, max_val);
    
    /* Instructions */
    oled_str(0, 7, "Rotate=Change Short=OK", false);
    
    oled_flush();
    menu_busy = false;
}

static void draw_rit_menu(void) {
    if (menu_busy) return;
    menu_busy = true;
    
    oled_clear();
    oled_str(0, 0, "RIT (Receiver IT)", true);
    oled_hline(0, 127, 1);
    
    oled_printf(0, 2, false, "Status: %s", vfo.rit_on ? "ON" : "OFF");
    oled_printf(0, 3, false, "Offset: %+ld Hz", (long)vfo.rit_hz);
    oled_str(0, 5, "Short=Toggle RIT", false);
    oled_str(0, 6, "Rotate=Adjust Offset", false);
    oled_str(0, 7, "Long=Exit Menu", false);
    
    oled_flush();
    menu_busy = false;
}

static void draw_agc_menu(void) {
    if (menu_busy) return;
    menu_busy = true;
    
    oled_clear();
    oled_str(0, 0, "AGC (Auto Gain)", true);
    oled_hline(0, 127, 1);
    
    oled_printf(0, 3, true, "AGC: %s", vfo.agc_on ? "ON" : "OFF");
    oled_str(0, 5, "Short=Toggle AGC", false);
    oled_str(0, 7, "Long=Exit Menu", false);
    
    oled_flush();
    menu_busy = false;
}

static void draw_split_menu(void) {
    if (menu_busy) return;
    menu_busy = true;
    
    oled_clear();
    oled_str(0, 0, "Split Operation", true);
    oled_hline(0, 127, 1);
    
    oled_printf(0, 2, false, "Split: %s", vfo.split ? "ON" : "OFF");
    if (vfo.split) {
        oled_printf(0, 3, false, "VFO-B: %lu.%03lu MHz",
                    (unsigned long)(vfo.vfob_hz/1000000),
                    (unsigned long)((vfo.vfob_hz%1000000)/1000));
    }
    oled_str(0, 5, "Short=Toggle Split", false);
    oled_str(0, 7, "Long=Exit Menu", false);
    
    oled_flush();
    menu_busy = false;
}

/* ---- Main VFO screen -------------------------------------------------- */
void screen_draw_main(void) {
    if (menu_busy) return;
    menu_busy = true;
    
    oled_clear();

    /* Row 0: band + mode + TX/RX indicator */
    oled_printf(0, 0, false, "%-4s %-3s",
                bands[vfo.band_idx].name, mode_names[vfo.mode]);
    if (trx_state == SEQ_TX) oled_str(14, 0, "[TX]", true);
    else                      oled_str(14, 0, " RX ", false);

    /* Rows 1+2: frequency */
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
    menu_busy = false;
}

/* ---- Public functions ------------------------------------------------ */
void menu_init(void) { 
    DEBUG_PRINT("menu_init(): Starting");
    state = MENU_STATE_NONE;
    menu_busy = false;
}

bool menu_is_open(void) { 
    return state != MENU_STATE_NONE; 
}

void menu_task(int32_t enc_delta, bool btn_short, bool btn_long, uint32_t now_ms) {
    static uint32_t last_action_time = 0;
    
    /* Debounce encoder - ignore rapid changes */
    int32_t filtered_delta = 0;
    if (enc_delta != 0) {
        if (now_ms - last_enc_time >= ENC_DEBOUNCE_MS) {
            /* Reduce sensitivity - only use sign, not magnitude */
            filtered_delta = (enc_delta > 0) ? 1 : -1;
            last_enc_time = now_ms;
            DEBUG_PRINT("Encoder: raw=%ld, filtered=%ld", enc_delta, filtered_delta);
        } else {
            /* Ignore this encoder event, it's too soon */
            filtered_delta = 0;
        }
    }
    
    /* Long press: toggle menu on/off */
    if (btn_long) {
        DEBUG_PRINT("LONG PRESS - toggling menu");
        
        /* Prevent double-action */
        if (now_ms - last_action_time < 200) {
            DEBUG_PRINT("  Ignoring long press - too soon");
            return;
        }
        last_action_time = now_ms;
        
        if (state == MENU_STATE_NONE) {
            /* Open main menu */
            state = MENU_STATE_MAIN;
            selected_index = 0;
            timeout = now_ms + MENU_TIMEOUT_MS;
            draw_main_menu();
            DEBUG_PRINT("  Main menu opened");
        } else {
            /* Exit menu completely */
            state = MENU_STATE_NONE;
            screen_draw_main();
            DEBUG_PRINT("  Menu closed");
        }
        return;
    }
    
    /* Handle menu navigation */
    if (state != MENU_STATE_NONE) {
        /* Update timeout */
        timeout = now_ms + MENU_TIMEOUT_MS;
        
        /* Handle encoder rotation with debouncing */
        if (filtered_delta != 0) {
            DEBUG_PRINT("Processing encoder: delta=%ld, state=%d", filtered_delta, state);
            
            if (state == MENU_STATE_MAIN) {
                /* Navigate main menu items - single step only */
                int new_index = selected_index + filtered_delta;
                if (new_index < 0) new_index = MAIN_MENU_COUNT - 1;
                if (new_index >= MAIN_MENU_COUNT) new_index = 0;
                
                if (new_index != selected_index) {
                    selected_index = new_index;
                    draw_main_menu();
                    DEBUG_PRINT("  Main menu selection: %d (%s)", 
                               selected_index, main_menu_items[selected_index]);
                }
            } 
            else if (state == MENU_STATE_SUB) {
                /* Adjust value in sub-menu - single step only */
                int max_val = get_menu_max(current_menu);
                int new_index = selected_index + filtered_delta;
                
                if (new_index < 0) new_index = max_val - 1;
                if (new_index >= max_val) new_index = 0;
                
                if (new_index != selected_index) {
                    selected_index = new_index;
                    
                    /* For menus that need real-time updates */
                    switch (current_menu) {
                        case MENU_CW_SPEED:
                            cw_wpm = selected_index + CW_WPM_MIN;
                            break;
                        case MENU_BFO:
                            vfo.bfo_offset = (int16_t)(selected_index * 10 - 500);
                            vfo_request_apply();
                            break;
                        default:
                            /* Other menus only apply on short press */
                            break;
                    }
                    
                    draw_sub_menu();
                    DEBUG_PRINT("  Sub-menu value: %d", selected_index);
                }
            }
        }
        
        /* Handle short press (select/confirm) */
        if (btn_short) {
            /* Prevent double-action */
            if (now_ms - last_action_time < 200) {
                DEBUG_PRINT("  Ignoring short press - too soon");
                return;
            }
            last_action_time = now_ms;
            
            DEBUG_PRINT("Short press, state=%d", state);
            
            if (state == MENU_STATE_MAIN) {
                /* Enter selected sub-menu */
                current_menu = (menu_id_t)(selected_index + 1);
                selected_index = get_current_value(current_menu);
                state = MENU_STATE_SUB;
                
                /* Special handling for simple toggle menus */
                if (current_menu == MENU_RIT) {
                    draw_rit_menu();
                } else if (current_menu == MENU_AGC) {
                    draw_agc_menu();
                } else if (current_menu == MENU_SPLIT) {
                    draw_split_menu();
                } else {
                    draw_sub_menu();
                }
                
                DEBUG_PRINT("  Entered sub-menu: %s", main_menu_items[selected_index]);
            } 
            else if (state == MENU_STATE_SUB) {
                /* Apply the setting and return to main menu */
                if (current_menu == MENU_RIT) {
                    /* RIT is toggled, not a selection */
                    vfo.rit_on = !vfo.rit_on;
                    vfo_request_apply();
                    draw_rit_menu();  /* Refresh display */
                } else if (current_menu == MENU_AGC) {
                    vfo.agc_on = !vfo.agc_on;
                    draw_agc_menu();  /* Refresh display */
                } else if (current_menu == MENU_SPLIT) {
                    vfo.split = !vfo.split;
                    if (vfo.split) vfo.vfob_hz = vfo.freq_hz;
                    draw_split_menu();  /* Refresh display */
                } else {
                    apply_setting();
                    /* Return to main menu */
                    state = MENU_STATE_MAIN;
                    selected_index = current_menu - 1;
                    draw_main_menu();
                }
                
                DEBUG_PRINT("  Applied setting");
                
                /* For non-toggle menus, we already returned to main menu */
                if (current_menu == MENU_RIT || current_menu == MENU_AGC || current_menu == MENU_SPLIT) {
                    /* Stay in sub-menu for toggles - don't return to main menu */
                } else {
                    state = MENU_STATE_MAIN;
                    selected_index = current_menu - 1;
                    draw_main_menu();
                }
            }
        }
        
        /* Check timeout */
        if (now_ms >= timeout) {
            DEBUG_PRINT("Menu timeout");
            state = MENU_STATE_NONE;
            screen_draw_main();
        }
        
        return;
    }
    
    /* Normal VFO tuning (no menu) */
    if (filtered_delta != 0) {
        int64_t nf = (int64_t)vfo.freq_hz + filtered_delta * (int64_t)vfo.step_hz;
        if (nf < (int64_t)bands[vfo.band_idx].freq_min) nf = bands[vfo.band_idx].freq_min;
        if (nf > (int64_t)bands[vfo.band_idx].freq_max) nf = bands[vfo.band_idx].freq_max;
        vfo.freq_hz = (uint32_t)nf;
        //vfo_auto_mode(vfo.freq_hz);
        vfo_request_apply();
        screen_draw_main();  /* Update display while tuning */
    }
    if (btn_short) {
        vfo.step_idx = (vfo.step_idx + 1) % STEP_COUNT;
        vfo.step_hz  = step_table[vfo.step_idx];
        screen_draw_main();
    }
}
