#pragma once
#include <stdint.h>
#include <stdbool.h>
/*
typedef enum {
    MENU_NONE=0, MENU_BAND, MENU_MODE, MENU_BW,
    MENU_STEP,   MENU_RIT,  MENU_AGC,  MENU_CW_SPEED,
    MENU_BFO,    MENU_SPLIT,MENU_MEM,  MENU_COUNT
} menu_id_t;
*/

typedef enum {
    MENU_NONE     = 0,
    MENU_BAND     = 1,
    MENU_MODE     = 2,
    MENU_BW       = 3,
    MENU_STEP     = 4,
    MENU_AGC      = 5,
    MENU_RIT      = 6,
    MENU_RIT_OFFS = 7,   /* NEW — RIT offset as separate item              */
    MENU_SPLIT    = 8,
    MENU_CW_SPEED = 9,
    MENU_BFO      = 10,
    MENU_MEM      = 11,
    MENU_SAVE     = 12,  /* NEW — save to flash from OLED menu             */
} menu_id_t;

/* Call once at startup */
void menu_init(void);

/* Call every main loop — handles encoder delta and button */
void menu_task(int32_t enc_delta, bool btn_short, bool btn_long, uint32_t now_ms);

/* Redraw the main VFO screen (also called by menu after closing) */
void screen_draw_main(void);

/* True when a menu is open (main loop uses this to skip VFO screen redraw) */
bool menu_is_open(void);
