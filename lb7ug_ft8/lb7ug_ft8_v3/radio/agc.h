#pragma once
#include <stdint.h>
/* Apply digital AGC in-place to buf[n] (24-bit signed samples).
   Fast attack 1ms, slow decay 500ms, target -20 dBFS.          */
void agc_process(int32_t *buf, int n);
