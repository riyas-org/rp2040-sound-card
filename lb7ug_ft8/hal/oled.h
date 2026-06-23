#pragma once
#include <stdbool.h>
#include <stdint.h>

void oled_init(void);
void oled_clear(void);
void oled_flush(void);                          /* push framebuffer to display  */

/* Draw character at pixel column x, page row p (0-7), optional inverse */
void oled_char(int x, int p, char c, bool inv);

/* Draw string at char column cx (0=leftmost), page row p */
void oled_str(int cx, int p, const char *s, bool inv);

/* Printf-style string at char column cx, page p */
void oled_printf(int cx, int p, bool inv, const char *fmt, ...);

/* Horizontal line (pixel coords) on page p */
void oled_hline(int x0, int x1, int p);

/* S-meter bar: maps rssi (-127..0) to a pixel bar on page p, cols x0..x1 */
void oled_smeter(int x0, int x1, int p, int8_t rssi);
