/*
 * usb/uac2_app.h
 * USB Audio Class 2.0 application layer — public API.
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Called every main-loop iteration to service USB audio I/O.
   Does nothing until uac2_set_codec_ready(true) has been called. */
void audio_task(void);

/* Call once after ALL hardware (Si4732, ADC, VFO) is fully initialised.
   Gating USB audio on this flag prevents audio_task() from touching
   uninitialised peripherals during the boot sequence. */
void uac2_set_codec_ready(bool ready);

/* LED blink helper, called from main loop. */
void led_blinking_task(void);

/* Sample rate change: set by the TinyUSB callback, consumed by main loop. */

extern uint32_t current_sample_rate;
extern uint32_t blink_interval_ms;
