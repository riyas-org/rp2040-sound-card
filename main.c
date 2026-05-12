/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Jerzy Kasenberg
 * Copyright (c) 2022 Angel Molina 
 * Copyright (c) 2023 Dhiru Kholia 
 * Copyright (c) 2026 Riyas Vettukattil  
 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "common.h"

extern uint32_t blink_interval_ms;
extern void audio_init_codec(void);

#if (CFG_TUSB_MCU == OPT_MCU_RP2040)
#include "pico/stdlib.h"
#endif

#include "hardware/pio.h"
#include "ws2812.pio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "cdc_app.h"
#include "hardware/pll.h"
#include "hardware/xosc.h"
#include "hardware/clocks.h"

#define WS2812_PIN 16

// INIT BOARD LED
void init_ws2812_led(void) {
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, false);
} 


void ws2812_led_write(bool state) {
    // Green when "on", Red when "off" (or 0 for dark)
    uint32_t color = state ? 0x00FF0000 : 0xFF000000; 
    pio_sm_put_blocking(pio0, 0, color);
}


void set_audio_clocks_safe() {
    // 1. Park clk_sys on the 12MHz Crystal (REF) 
    // This keeps the CPU running while we stop and restart the PLLs
    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF,
                    0,          // No auxiliary mux needed for REF
                    12 * MHZ,
                    12 * MHZ);

    // 2. Initialize PLLs to Audio frequencies
    // PLL_SYS = 122.88 MHz (12 * 102.4)
    pll_init(pll_sys, 1, 1228800000, 5, 2);
    // PLL_USB = 48 MHz (Standard for TinyUSB)
    pll_init(pll_usb, 1, 480000000, 5, 2);

    // 3. Switch clk_sys to the new 122.88 MHz PLL
    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                    122880000,
                    122880000);

    // 4. Switch clk_usb to the 48 MHz PLL
    clock_configure(clk_usb,
                    0, 
                    CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    48 * MHZ);

    // 5. IMPORTANT: Re-init stdio because the UART baud rate is calculated from clk_sys
    stdio_init_all();
}



/*------------- MAIN -------------*/
int main(void)
{

set_audio_clocks_safe();


#if (CFG_TUSB_MCU == OPT_MCU_RP2040)
  stdio_init_all();
#endif

  
board_init();
init_ws2812_led();



  TU_LOG1("CDC UAC2 Stereo SDR example running\r\n");
  TU_LOG1("Stereo audio: 2 channels TX (mic), 2 channels RX (speaker)\r\n");


// init device stack on configured roothub port
tusb_rhport_init_t dev_init = {
.role = TUSB_ROLE_DEVICE,
.speed = TUSB_SPEED_AUTO
};
tusb_init(BOARD_TUD_RHPORT, &dev_init);

audio_init_codec();

//wm8731_start_dma();    


  while (1)
  {
    tud_task(); // TinyUSB device task
    led_blinking_task();
    audio_task();
    cdc_task();
    audio_apply_pending_rate_change();
    //run_bit_perfect_test();    
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
  TU_LOG1("Device mounted\r\n");
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
  TU_LOG1("Device unmounted\r\n");
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void)remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
  TU_LOG1("Device suspended\r\n");
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
  TU_LOG1("Device resumed\r\n");
}
