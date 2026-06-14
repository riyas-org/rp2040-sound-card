// main.c - 40m CW Transceiver with Proper Sideband
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "config.h"
#include "hal/i2c_bus.h"
#include "hal/oled.h"
#include "hal/si5351.h"
#include "hal/si4732.h"

int32_t si5351_cal_ppb = 0;

// Pin definitions
#define PIN_ENC_A       7
#define PIN_ENC_B       8
#define PIN_ENC_BTN     9
#define PIN_TX_EN       12

// Frequency
#define FREQ_MIN        7000
#define FREQ_MAX        7200
#define FREQ_STEP       1  // 1kHz steps

// Global variables
static uint32_t frequency = 7020;  // kHz (CW portion of 40m)
static bool tx_mode = false;
static volatile int32_t encoder_count = 0;
static volatile bool button_pressed = false;

// Interrupt handler
void gpio_callback(uint gpio, uint32_t events) {
    static uint32_t last_interrupt = 0;
    uint32_t now = time_us_32();
    
    if (now - last_interrupt < 5000) return;
    last_interrupt = now;
    
    // Handle encoder
    if (gpio == PIN_ENC_A || gpio == PIN_ENC_B) {
        static int last_a = 0, last_b = 0;
        int a = gpio_get(PIN_ENC_A);
        int b = gpio_get(PIN_ENC_B);
        
        if (a != last_a || b != last_b) {
            if (a == b) encoder_count++;
            else encoder_count--;
            last_a = a;
            last_b = b;
        }
    }
    
    // Handle button
    if (gpio == PIN_ENC_BTN) {
        if (!gpio_get(PIN_ENC_BTN)) {
            button_pressed = true;
        }
    }
}

// Toggle TX/RX
void toggle_tx() {
    tx_mode = !tx_mode;
    
    if (tx_mode) {
        // TRANSMIT MODE
        printf("TX ON at %u.%03u MHz\n", frequency/1000, frequency%1000);
        
        // Set Si5351 to transmit frequency
        si5351_set_freq(0, frequency * 1000, 6);
        si5351_enable(0, true);
        gpio_put(PIN_TX_EN, 1);
        
        // For CW monitoring: Switch to USB and tune ABOVE your signal
        // This gives a 1kHz beat note
        si4732_switch_mode(MODE_USB);  // USB for monitoring CW
        si4732_tune_am(frequency + 1); // Tune 1kHz above
        si4732_set_volume(50);
        si4732_mute(false);
        
    } else {
        // RECEIVE MODE
        printf("RX OFF\n");
        
        // Turn off transmitter
        si5351_enable(0, false);
        gpio_put(PIN_TX_EN, 0);
        
        // Return to LSB for normal 40m reception
        si4732_switch_mode(MODE_LSB);
        si4732_tune_am(frequency);
        si4732_set_volume(40);
        si4732_mute(false);
    }
    
    // Update display
    char buf[24];
    oled_clear();
    oled_str(0, 0, "40m CW Transceiver", false);
    snprintf(buf, sizeof(buf), "%u.%03u MHz", frequency/1000, frequency%1000);
    oled_str(0, 2, buf, true);
    
    if (tx_mode) {
        oled_str(0, 4, ">> TRANSMITTING <<", false);
        oled_str(0, 5, "Monitor: USB +1kHz", false);
        oled_str(0, 6, "You'll hear 1kHz tone", false);
    } else {
        oled_str(0, 4, "RX - LSB Mode", false);
        oled_str(0, 5, "Button = TX", false);
    }
    
    oled_flush();
}

void set_frequency(uint32_t freq) {
    if (freq < FREQ_MIN) freq = FREQ_MIN;
    if (freq > FREQ_MAX) freq = FREQ_MAX;
    frequency = freq;
    
    // Update receiver based on current mode
    if (tx_mode) {
        // In TX: monitor in USB mode, 1kHz above
        si4732_switch_mode(MODE_USB);
        si4732_tune_am(frequency + 1);
    } else {
        // In RX: normal LSB
        si4732_switch_mode(MODE_LSB);
        si4732_tune_am(frequency);
    }
    
    // Update display
    char buf[24];
    oled_clear();
    oled_str(0, 0, "40m CW Transceiver", false);
    snprintf(buf, sizeof(buf), "%u.%03u MHz", frequency/1000, frequency%1000);
    oled_str(0, 2, buf, true);
    
    if (tx_mode) {
        oled_str(0, 4, ">> TRANSMITTING <<", false);
        oled_str(0, 5, "Monitor: USB +1kHz", false);
        oled_str(0, 6, "You'll hear 1kHz tone", false);
    } else {
        oled_str(0, 4, "RX - LSB Mode", false);
        oled_str(0, 5, "Button = TX", false);
    }
    
    oled_flush();
    printf("Freq: %u.%03u MHz\n", frequency/1000, frequency%1000);
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n=== 40m CW Transceiver ===\n");
    printf("RX: LSB mode for 40m CW\n");
    printf("TX: USB mode +1kHz offset for monitoring\n");
    printf("You will hear your CW as a 1kHz tone\n\n");
    
    // Init hardware
    i2c_bus_init();
    oled_init();
    
    // Init encoder pins
    gpio_init(PIN_ENC_A);
    gpio_set_dir(PIN_ENC_A, GPIO_IN);
    gpio_pull_up(PIN_ENC_A);
    
    gpio_init(PIN_ENC_B);
    gpio_set_dir(PIN_ENC_B, GPIO_IN);
    gpio_pull_up(PIN_ENC_B);
    
    gpio_init(PIN_ENC_BTN);
    gpio_set_dir(PIN_ENC_BTN, GPIO_IN);
    gpio_pull_up(PIN_ENC_BTN);
    
    gpio_init(PIN_TX_EN);
    gpio_set_dir(PIN_TX_EN, GPIO_OUT);
    gpio_put(PIN_TX_EN, 0);
    
    // Setup interrupts
    gpio_set_irq_enabled_with_callback(PIN_ENC_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled(PIN_ENC_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_ENC_BTN, GPIO_IRQ_EDGE_FALL, true);
    
    // Init Si5351
    si5351_init();
    si5351_enable(0, false);
    
    // Init Si4732 - Start in LSB for 40m
    si4732_init();
    si4732_switch_mode(MODE_LSB);
    si4732_set_volume(40);
    si4732_set_bfo(0);
    si4732_mute(false);
    
    // Set initial frequency
    set_frequency(frequency);
    
    printf("Ready!\n");
    printf("- Encoder: Change frequency (1kHz steps)\n");
    printf("- Button: Toggle TX/RX\n");
    printf("- In TX mode, you'll hear your signal as a 1kHz tone\n\n");
    
    // Main loop
    uint32_t last_rssi = 0;
    uint32_t last_button_check = 0;
    
    while (1) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        // Handle encoder
        if (encoder_count != 0) {
            int32_t delta = encoder_count;
            encoder_count = 0;
            
            int32_t new_freq = frequency + (delta * FREQ_STEP);
            if (new_freq >= FREQ_MIN && new_freq <= FREQ_MAX) {
                set_frequency(new_freq);
            }
        }
        
        // Handle button
        if (now - last_button_check > 50) {
            last_button_check = now;
            if (button_pressed) {
                button_pressed = false;
                toggle_tx();
            }
        }
        
        // Update RSSI (only in RX mode)
        if (now - last_rssi > 2000 && !tx_mode) {
            last_rssi = now;
            si4732_poll_rsq();
            int8_t rssi = si4732_get_rssi();
            
            char buf[24];
            snprintf(buf, sizeof(buf), "RSSI:%3d dBuV", rssi);
            oled_str(0, 7, buf, false);
            oled_flush();
        }
        
        sleep_ms(10);
    }
    
    return 0;
}
