/*
 * main.c — RP2040 HF Transceiver
 *
 * Core 0: UI (OLED, encoder, menu), CAT/terminal (USB CDC),
 *         TinyUSB task, VFO control, CW keyer, main loop.
 * Core 1: ADC DMA capture (48 kHz oversampled from 192 kHz).
 *         Communicates with Core 0 via inter-core FIFO.
 *
 * Build stages — comment out FEATURE_* flags in config.h to
 * disable sections not yet needed:
 *
 *  Stage 1  LED blinks, I2C scan prints to serial
 *  Stage 2  OLED shows boot message
 *  Stage 3  Si4732 AM receive, audio on speaker pin
 *  Stage 4  Si5351 outputs VFO freq (check with SDR dongle)
 *  Stage 5  VFO tuning on encoder, frequency on OLED
 *  Stage 6  Full OLED menu
 *  Stage 7  USB CDC terminal + CAT (test with WSJT-X)
 *  Stage 8  SSB patch, LSB/USB receive
 *  Stage 9  ADC → USB audio IN (WSJT-X hears the radio)
 *  Stage 10 FT8/WSPR TX via Goertzel + Si5351 FSK
 *  Stage 11 CW keyer, sidetone, decoder, memories
 *  Stage 12 SSB TX via PWM envelope + Si5351
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "bsp/board_api.h"
#include "tusb.h"

#include "config.h"
#include "hal/i2c_bus.h"
#include "hal/si5351.h"
#include "hal/si4732.h"
#include "hal/oled.h"
#include "hal/adc_audio.h"
#include "radio/vfo.h"
#include "radio/trx.h"
#include "radio/cw_keyer.h"
#include "radio/agc.h"
#include "dsp/goertzel.h"
#include "dsp/ssb_mod.h"
#include "ui/menu.h"
#include "ui/terminal.h"
#include "hardware/pll.h"
#include "hardware/xosc.h"
#include "hardware/clocks.h"
#include "usb/uac2_app.h"

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

    // 5. Explicitly route clk_adc from PLL_USB (48 MHz).
    //    Without this the ADC clock source is undefined after PLL_SYS changes.
    clock_configure(clk_adc,
                    0,
                    CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    48 * MHZ);

    // 6. IMPORTANT: Re-init stdio because the UART baud rate is calculated from clk_sys
    stdio_init_all();
}


/* =========================================================================
   Encoder (GPIO IRQ, Core 0)
   ========================================================================= */

static volatile int32_t enc_delta = 0;
static volatile bool    btn_event = false;
static uint8_t          enc_last  = 0;
static const int8_t     enc_lut[16] = {0,-1,+1,0,+1,0,0,-1,-1,0,0,+1,0,+1,-1,0};

static void gpio_irq_cb(uint gpio, uint32_t events) {
    (void)events;
    if (gpio == PIN_ENC_A || gpio == PIN_ENC_B) {
        uint8_t cur = ((uint8_t)gpio_get(PIN_ENC_A)<<1)|(uint8_t)gpio_get(PIN_ENC_B);
        enc_delta  += enc_lut[(enc_last<<2)|cur];
        enc_last    = cur;
    }
    if (gpio == PIN_ENC_BTN) {
        static bool last = true;
        bool c = gpio_get(PIN_ENC_BTN);
        if (!c && last) btn_event = true;
        last = c;
    }
}

static int32_t enc_read(void) {
    uint32_t s = save_and_disable_interrupts();
    int32_t v  = enc_delta; enc_delta = 0;
    restore_interrupts(s); return v;
}
static bool btn_read(void) {
    uint32_t s = save_and_disable_interrupts();
    bool v = btn_event; btn_event = false;
    restore_interrupts(s); return v;
}

/* =========================================================================
   Long-press detection
   ========================================================================= */

static uint32_t btn_down_at       = 0;
static bool     btn_was_down      = false;
static bool     long_fired        = false;

static void btn_update(uint32_t now, bool *short_press, bool *long_press) {
    *short_press = false;
    *long_press  = false;
    bool raw = !gpio_get(PIN_ENC_BTN);
    if (raw && !btn_was_down) {
        btn_down_at = now;
        btn_was_down = true;
        long_fired = false;
        /* Clear any stale IRQ-sourced btn_event so the press that opened
           long-press detection doesn't also register as a short press. */
        uint32_t s = save_and_disable_interrupts();
        btn_event = false;
        restore_interrupts(s);
    }
    if (!raw && btn_was_down) {
        /* Only emit short press if long press never fired this press cycle. */
        if (!long_fired) *short_press = true;
        btn_was_down = false;
        /* Clear IRQ event again — release edge may have set it. */
        uint32_t s = save_and_disable_interrupts();
        btn_event = false;
        restore_interrupts(s);
    }
    if (raw && btn_was_down && !long_fired && (now - btn_down_at >= LONG_PRESS_MS)) {
        *long_press = true;
        long_fired = true;
        /* Immediately clear event so the release doesn't trigger short press. */
        uint32_t s = save_and_disable_interrupts();
        btn_event = false;
        restore_interrupts(s);
    }
}

/* =========================================================================
   Core 1 — ADC audio capture
   ========================================================================= */

static void core1_entry(void) {
#ifdef FEATURE_DUAL_CORE
    adc_audio_init();
#endif
    /* Core 1 spins here; all real work is done in the DMA IRQ handler
       (adc_audio.c) which fires at 1 ms intervals.                    */
    while (true) tight_loop_contents();
}

/* =========================================================================
   Boot splash
   ========================================================================= */

static void boot_splash(uint8_t i2c_found) {
    oled_clear();
    oled_str(0, 0, " RP2040 HF TRX v2", false);
    oled_str(0, 1, "------------------", false);
    oled_printf(0, 2, false, " Si5351: %s", (i2c_found&1) ? "OK" : "MISS");
    oled_printf(0, 3, false, " Si4732: %s", (i2c_found&2) ? "OK" : "MISS");
    oled_printf(0, 4, false, " OLED  : %s", (i2c_found&4) ? "OK" : "MISS");
    oled_str(0, 6, " SSB patch...", false);
    oled_flush();
}

/* =========================================================================
   Autosave state
   ========================================================================= */

static bool     dirty_save    = false;
static uint32_t dirty_save_at = 0;

static void mark_dirty(uint32_t now) {
    dirty_save    = true;
    dirty_save_at = now + AUTOSAVE_DELAY_MS;
}

/* =========================================================================
   main()
   ========================================================================= */

int main(void) {
    /* ---- System clock: 250 MHz for USB PLL stability ------------------ */
    //set_sys_clock_khz(250000, true);
    set_audio_clocks_safe();
    stdio_init_all();
    
    board_init();

    /* ---- GPIO setup --------------------------------------------------- */
    gpio_init(PIN_LED);     gpio_set_dir(PIN_LED,     GPIO_OUT);
    gpio_init(PIN_ENC_A);   gpio_set_dir(PIN_ENC_A,   GPIO_IN); gpio_pull_up(PIN_ENC_A);
    gpio_init(PIN_ENC_B);   gpio_set_dir(PIN_ENC_B,   GPIO_IN); gpio_pull_up(PIN_ENC_B);
    gpio_init(PIN_ENC_BTN); gpio_set_dir(PIN_ENC_BTN, GPIO_IN); gpio_pull_up(PIN_ENC_BTN);
    gpio_init(PIN_SI4732_RST); gpio_set_dir(PIN_SI4732_RST, GPIO_OUT); gpio_put(PIN_SI4732_RST, 1);

    /* ---- Encoder IRQ -------------------------------------------------- */
    gpio_set_irq_enabled_with_callback(PIN_ENC_A,
        GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL, true, gpio_irq_cb);
    gpio_set_irq_enabled(PIN_ENC_B,
        GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_ENC_BTN,
        GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL, true);
    enc_last = (uint8_t)((gpio_get(PIN_ENC_A)<<1)|gpio_get(PIN_ENC_B));

    /* ---- Si4732 hardware reset FIRST, before any I2C scan ------------- */
    /* The Si4732 must be reset before it will respond correctly to I2C.
       If i2c_bus_init() scans the bus while the chip is in an undefined
       state it corrupts the power-up sequence → hissing audio output.
       Reset here, then scan, then si4732_init_am() resets again (harmless,
       ensures clean state after anything the scan may have disturbed). */
    gpio_put(PIN_SI4732_RST, 0);
    sleep_ms(15);
    gpio_put(PIN_SI4732_RST, 1);
    sleep_ms(250);

    /* ---- I2C + bus scan ----------------------------------------------- */
    uint8_t i2c_found = i2c_bus_init();

    /* ---- OLED --------------------------------------------------------- */
    /* Stage 2: should see splash screen */
    oled_init();
    boot_splash(i2c_found);

    /* ---- Si5351 ------------------------------------------------------- */
    si5351_init();

    /* ---- Si4732 ------------------------------------------------------- */
    /* Stage 3: AM receive; Stage 8: SSB patch */
    si4732_init_am();

#ifdef FEATURE_SSB_PATCH
    si4732_apply_ssb_patch();
    oled_str(0, 6, si4732_ssb_patched ? " SSB patch OK  " :
                                         " SSB patch SKIP", false);
    oled_flush();
#endif
    si4732_soft_mute_off();

    /* ---- VFO: load from flash, then apply to Si4732/Si5351 ----------- */
    /* Stage 5: frequency appears on OLED, Si5351 CLK0 active */
    vfo_init();

    /* ---- CW keyer + sidetone ----------------------------------------- */
    cw_keyer_init();

    /* ---- SSB TX PWM -------------------------------------------------- */
#ifdef FEATURE_SSB_TX
    ssb_mod_init();
#endif

    /* ---- TX/RX sequencer --------------------------------------------- */
    trx_init();

    /* ---- TinyUSB ----------------------------------------------------- */
    /* Stage 7: CDC terminal and CAT appear */
    board_init();
    /* tud_init(rhport) works on TinyUSB <0.16; newer versions prefer
       tusb_init(). Both take rhport=0 for RP2040 USB.
       Using tud_init(0) directly — suppress the deprecation warning
       with a pragma if your toolchain shows it.                         */
    // init device stack on configured roothub port
	tusb_rhport_init_t dev_init = {
	.role = TUSB_ROLE_DEVICE,
	.speed = TUSB_SPEED_AUTO
	};
	tusb_init(BOARD_TUD_RHPORT, &dev_init);

    /* ---- Launch Core 1 (ADC audio) ----------------------------------- */
    /* Stage 9: enable FEATURE_DUAL_CORE in config.h */
#ifdef FEATURE_DUAL_CORE
    multicore_launch_core1(core1_entry);
#endif

    /* ---- Apply VFO to radio ------------------------------------------ */
    /* Stage 5: after this call Si4732 should be tuned */
    vfo_apply();
    sleep_ms(200);

    /* ---- Signal that hardware is ready for USB audio ----------------- */
    /* codec_initialized gates every path in audio_task(). It must be set
       after ALL hardware (Si4732, ADC, VFO) is fully configured, not before,
       or audio_task() will try to use uninitialised peripherals. */
    uac2_set_codec_ready(true);

    /* ---- Initial display --------------------------------------------- */
    screen_draw_main();

    /* ===================================================================
       MAIN LOOP
       =================================================================== */

    uint32_t last_display = 0;
    uint32_t last_led     = 0;
    bool     led          = false;

    while (true) {

        /* ---- TinyUSB + CDC terminal + USB audio ---------------------- */
        tud_task();
        terminal_task();
        audio_task();

        /* ---- Deferred VFO apply (Si4732 I2C, may take ~300ms) -------- */
        /* Must run AFTER tud_task() so USB is never starved by I2C CTS. */
        vfo_apply_pending();

        uint32_t now = to_ms_since_boot(get_absolute_time());

        /* ---- LED heartbeat ------------------------------------------- */
        if (now - last_led >= 500) {
            last_led = now; led = !led;
            gpio_put(PIN_LED, led);
        }

        /* ---- Encoder + button ---------------------------------------- */
        int32_t delta = enc_read();
        bool    btn   = btn_read();
        bool    short_p, long_p;
        btn_update(now, &short_p, &long_p);
        if (btn) short_p = true;   /* edge from IRQ also counts */

        /* Delegate to menu system */
        if (delta || short_p || long_p) {
            menu_task(delta, short_p, long_p, now);
            mark_dirty(now);
        }

        /* ---- CW keyer ------------------------------------------------ */
        cw_keyer_task(now);

        /* ---- Display refresh every 100 ms ---------------------------- */
        if (now - last_display >= 100) {
            last_display = now;
            if (!menu_is_open()) screen_draw_main();
        }

        /* ---- Autosave ------------------------------------------------ */
        if (dirty_save && now >= dirty_save_at) {
            dirty_save = false;
            vfo.cw_wpm  = cw_wpm;
            vfo.cal_ppb = si5351_cal_ppb;
            vfo_save();
        }

        /* ---- USB audio sample-rate change (guard: only in RX) -------- */
        /*if (sample_rate_change_pending && trx_state == SEQ_RX) {
            sample_rate_change_pending = false;
            current_sample_rate = pending_sample_rate;
            tud_disconnect();
            sleep_ms(200);
            tud_connect();
        }*/
    }
    return 0;
}
