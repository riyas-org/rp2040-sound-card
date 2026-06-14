// main.c - Test program for modular SI4732 with SSB
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "config.h"
#include "hal/i2c_bus.h"
#include "hal/oled.h"
#include "hal/si5351.h"
#include "hal/si4732.h"

// Test frequency: 7.050 MHz LSB
#define RX_FREQ_KHZ     7050
#define TX_CENTER_HZ    7050000ULL
#define WOBBLE_RANGE_HZ 5000      // ±5 kHz sweep
#define WOBBLE_PERIOD_MS 2000     // 2 seconds up, 2 seconds down

int32_t si5351_cal_ppb = 0;   // no calibration for test

// Simple delay function for OLED updates
static void show_message(const char *line1, const char *line2, const char *line3) {
    oled_clear();
    if (line1) oled_str(0, 0, line1, false);
    if (line2) oled_str(0, 2, line2, false);
    if (line3) oled_str(0, 4, line3, false);
    oled_flush();
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("\n=== SI4732 + SI5351 Test ===\n");
    
    // 1. I2C bus
    uint8_t found = i2c_bus_init();
    printf("I2C devices found: 0x%02X (expected 0x07)\n", found);
    
    if ((found & 0x06) != 0x06) {
        printf("ERROR: Missing I2C devices! Si5351 and Si4732 required.\n");
        show_message("I2C ERROR!", "Check wiring", NULL);
        while (1) tight_loop_contents();
    }
    
    // 2. OLED init
    oled_init();
    show_message("OLED OK", "Initializing...", NULL);
    sleep_ms(1000);
    
    // 3. Si5351 init
    show_message("Init Si5351...", NULL, NULL);
    si5351_init();
    si5351_set_freq(0, TX_CENTER_HZ, 4);
    si5351_enable(0, false);
    printf("Si5351 initialized\n");
    sleep_ms(500);
    
    // 4. Si4732 init (this does hardware reset, power up, and gets revision)
    show_message("Init Si4732...", "Please wait", NULL);
    si4732_init();
    printf("Si4732 initialized\n");
    sleep_ms(500);
    
    // 5. Switch to LSB mode (this loads SSB patch)
    show_message("Loading SSB...", "This takes ~2 sec", NULL);
    printf("Switching to LSB mode...\n");
    si4732_switch_mode(MODE_LSB);
    sleep_ms(500);
    
    // 6. Check if SSB loaded successfully
    if (!si4732_is_ssb_loaded()) {
        show_message("SSB PATCH FAILED!", "Check patch array", "Using AM mode");
        printf("WARNING: SSB patch not loaded, using AM mode\n");
    } else {
        show_message("SSB Patch OK!", "LSB mode active", NULL);
        printf("SSB patch loaded successfully!\n");
        sleep_ms(1000);
    }
    
    // 7. Tune to frequency
    show_message("Tuning...", NULL, NULL);
    si4732_tune_am(RX_FREQ_KHZ);
    printf("Tuned to %u kHz\n", RX_FREQ_KHZ);
    sleep_ms(500);
    
    // 8. Set volume and unmute
    si4732_set_volume(50);
    si4732_mute(false);
    printf("Volume set, audio enabled\n");
    
    // 9. Main display
    show_message("RX: 7.050 MHz LSB", "TX: wobbling", "Listening...");
    sleep_ms(1000);
    
    // 10. Start transmitting (wobble)
    si5351_enable(0, true);
    printf("TX wobble started\n");
    
    // Wobble variables
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    uint32_t last_rssi_update = 0;
    uint32_t last_wobble_update = 0;
    uint32_t last_display_update = 0;
    int bfo_direction = 1;
    int16_t current_bfo = 0;
    int cycle_count = 0;
    
    printf("\n=== Starting main loop ===\n");
    printf("Listen with signal generator at 7.050 MHz USB\n");
    printf("BFO will sweep to help find the signal\n\n");
    
    while (1) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        // Update wobble every 50 ms
        if (now - last_wobble_update >= 50) {
            last_wobble_update = now;
            
            float t = (float)((now - t0) % WOBBLE_PERIOD_MS) / WOBBLE_PERIOD_MS;
            float phase = t * 2.0f * 3.14159f;
            float tri = (phase < 3.14159f) ? (phase / 3.14159f) : (2.0f - phase / 3.14159f);
            int32_t offset = (int32_t)((tri * 2.0f - 1.0f) * WOBBLE_RANGE_HZ);
            uint64_t tx_hz = TX_CENTER_HZ + offset;
            si5351_set_freq(0, tx_hz, 4);
        }
        
        // Sweep BFO every 3 seconds to help find SSB signal
        if (now - last_display_update >= 3000) {
            last_display_update = now;
            
            // Cycle BFO from -2000 to +2000 Hz
            current_bfo += 250 * bfo_direction;
            if (current_bfo >= 2000) {
                current_bfo = 2000;
                bfo_direction = -1;
                cycle_count++;
            } else if (current_bfo <= -2000) {
                current_bfo = -2000;
                bfo_direction = 1;
                cycle_count++;
            }
            
            si4732_set_bfo(current_bfo);
            printf("BFO: %d Hz (cycle %d)\n", current_bfo, cycle_count);
            
            // Update OLED with current BFO
            char bfo_str[20];
            snprintf(bfo_str, sizeof(bfo_str), "BFO: %d Hz", current_bfo);
            oled_clear();
            oled_str(0, 0, "LSB MODE", false);
            oled_printf(0, 2, false, "Freq: %u kHz", RX_FREQ_KHZ);
            oled_str(0, 4, bfo_str, false);
            char cycle_str[20];
            snprintf(cycle_str, sizeof(cycle_str), "Cycle: %d", cycle_count);
            oled_str(0, 6, cycle_str, false);
            oled_flush();
        }
        
        // Update RSSI every second
        if (now - last_rssi_update >= 1000) {
            last_rssi_update = now;
            si4732_poll_rsq();
            int8_t rssi = si4732_get_rssi();
            uint8_t snr = si4732_get_snr();
            printf("RSSI: %d dBuV, SNR: %d dB\n", rssi, snr);
            
            // Update RSSI on OLED (line 7)
            char rssi_str[20];
            snprintf(rssi_str, sizeof(rssi_str), "RSSI:%d SNR:%d", rssi, snr);
            oled_clear();
            oled_str(0, 0, "LSB MODE", false);
            oled_printf(0, 2, false, "Freq: %u kHz", RX_FREQ_KHZ);
            char bfo_str[20];
            snprintf(bfo_str, sizeof(bfo_str), "BFO: %d Hz", current_bfo);
            oled_str(0, 4, bfo_str, false);
            oled_str(0, 6, rssi_str, false);
            oled_flush();
        }
        
        sleep_ms(10);
    }
    
    return 0;
}
