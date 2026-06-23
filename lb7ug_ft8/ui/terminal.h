#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Feed bytes from USB CDC RX into the terminal / CAT dispatcher */
void terminal_rx(const uint8_t *data, uint32_t len);

/* Call from main loop alongside tud_task() — drains TX ring buffer */
void terminal_task(void);

/* Print to the terminal (safe to call from any context on Core 0) */
int  term_printf(const char *fmt, ...);

/* Called by TinyUSB when host connects DTR */
void terminal_connected(void);
