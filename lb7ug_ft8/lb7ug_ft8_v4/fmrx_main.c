// main.c - FM Receiver test on 105 MHz using your existing library
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "config.h"
#include "hal/i2c_bus.h"
#include "hal/oled.h"
#include "hal/si5351.h"
#include "hal/si4732.h"

int32_t si5351_cal_ppb = 0;

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
    printf("\n=== SI4732 FM Receiver Test ===\n");
    
    // 1. I2C bus init
    uint8_t found = i2c_bus_init();
    printf("I2C devices found: 0x%02X\n", found);
    
    if ((found & 0x06) != 0x06) {
        printf("ERROR: Missing I2C devices!\n");
        show_message("I2C ERROR!", "Check wiring", NULL);
        while (1) tight_loop_contents();
    }
    
    // 2. OLED init
    oled_init();
    show_message("FM Receiver", "Initializing...", NULL);
    sleep_ms(1000);
    
    // 3. Si5351 init (optional, for completeness)
    si5351_init();
    si5351_enable(0, false);  // Not needed for reception
    printf("Si5351 initialized\n");
    
    // 4. Initialize Si4732 (starts in AM mode per your init function)
    show_message("Init Si4732...", NULL, NULL);
    si4732_init();
    printf("Si4732 initialized in AM mode\n");
    sleep_ms(500);
    
    // 5. Switch to FM mode with frequency 105.0 MHz
    // FM frequency in your library is in units of 10 kHz
    // 105.0 MHz = 10500 (1050 * 10)
    uint32_t fm_freq_10khz = 10500;  // 105.0 MHz
    printf("Switching to FM mode at %.1f MHz\n", fm_freq_10khz / 100.0f);
    show_message("Switching to FM...", "105.0 MHz", NULL);
    
    // Switch mode - this will power down, reset, and power up in FM mode
    si4732_switch_mode(MODE_FM);
    sleep_ms(500);
    
    // 6. Tune to 105.0 MHz using the library's FM tune function
    si4732_tune_fm(fm_freq_10khz);
    sleep_ms(500);
    
    // 7. Set volume (0-63)
    si4732_set_volume(45);
    si4732_mute(false);
    printf("Volume set to 45, audio enabled\n");
    
    // 8. Display initial info
    char freq_str[20];
    snprintf(freq_str, sizeof(freq_str), "FM: %.1f MHz", fm_freq_10khz / 100.0f);
    show_message(freq_str, "Volume: 45%", "Listening...");
    sleep_ms(1000);
    
    printf("\n=== Tuned to 105.0 MHz FM ===\n");
    printf("You should hear FM broadcast if there's a station at 105.0 MHz\n");
    printf("Try different frequencies by modifying fm_freq_10khz in code\n\n");
    
    // Variables for display updates
    uint32_t last_update = 0;
    
    while (1) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        // Update RSSI and display every second
        if (now - last_update >= 1000) {
            last_update = now;
            
            // Poll RSSI (works for FM too)
            si4732_poll_rsq();
            int8_t rssi = si4732_get_rssi();
            uint8_t snr = si4732_get_snr();
            uint32_t current_freq = si4732_get_fm_10khz();
            float freq_mhz = current_freq / 100.0f;
            
            printf("FM %.1f MHz | RSSI: %d dBuV | SNR: %d dB\n", 
                   freq_mhz, rssi, snr);
            
            // Update OLED
            char line1[20], line2[20], line3[20];
            snprintf(line1, sizeof(line1), "FM: %.1f MHz", freq_mhz);
            snprintf(line2, sizeof(line2), "RSSI:%d SNR:%d", rssi, snr);
            
            // Signal strength bar
            int bar_len = 0;
            if (rssi > 0) {
                bar_len = rssi / 6;  // Scale RSSI 0-60 to 0-10 bars
                if (bar_len > 10) bar_len = 10;
            }
            
            char bar[12];
            for (int i = 0; i < bar_len; i++) bar[i] = '=';
            bar[bar_len] = '\0';
            if (bar_len > 0) {
                snprintf(line3, sizeof(line3), "[%s]", bar);
            } else {
                snprintf(line3, sizeof(line3), "[no signal]");
            }
            
            oled_clear();
            oled_str(0, 0, line1, false);
            oled_str(0, 2, line2, false);
            oled_str(0, 4, line3, false);
            
            // Add tuning hint
            if (rssi < 15) {
                oled_str(0, 6, "Weak signal", false);
            } else if (rssi > 40) {
                oled_str(0, 6, "Strong signal", false);
            }
            
            oled_flush();
        }
        
        sleep_ms(100);
    }
    
    return 0;
}
