/*
 * hal/adc_audio.c
 * Continuous ADC capture at 192 kHz, decimated 4:1 to 48 kHz.
 * DMA ping-pong; IRQ runs on Core 1.
 *
 * STAGE 9 — enable once USB audio works on Core 0.
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "../config.h"
#include "adc_audio.h"

#define RAW_BUF (AUDIO_BUF_SAMPLES * AUDIO_OVERSAMPLE)   /* 192 raw samples */

static uint16_t raw[2][RAW_BUF];

int32_t adc_audio_buf[2][AUDIO_BUF_SAMPLES];
volatile bool    adc_audio_ready = false;
volatile uint8_t adc_audio_idx   = 0;

static int      dma_chan = -1;
static volatile uint8_t dma_idx = 0;   /* which raw[] DMA is filling       */

static void __isr dma_handler(void) {
    dma_hw->ints0 = (1u << dma_chan);

    /* Decimate: average AUDIO_OVERSAMPLE raw samples → 1 PCM sample */
    uint16_t *src = raw[dma_idx];
    int32_t  *dst = adc_audio_buf[dma_idx];

    for (int i = 0; i < AUDIO_BUF_SAMPLES; i++) {
        uint32_t acc = 0;
        for (int k = 0; k < AUDIO_OVERSAMPLE; k++)
            acc += src[i * AUDIO_OVERSAMPLE + k];
        /* acc is 14-bit (12-bit × 4). Centre on zero, scale to 24-bit. */
        int32_t s = (int32_t)(acc >> 2) - 8192;  /* 14-bit centred          */
        dst[i]    = s << 10;                       /* → 24-bit range          */
    }

    adc_audio_idx   = dma_idx;
    adc_audio_ready = true;
    dma_idx ^= 1;

    /* Re-arm into alternate buffer immediately */
    dma_channel_set_write_addr(dma_chan, raw[dma_idx], true);
}

void adc_audio_init(void) {
    adc_init();
    adc_gpio_init(PIN_AUDIO_ADC);
    adc_select_input(AUDIO_ADC_CHAN);
    adc_fifo_setup(true, true, 1, false, false);

    /* ADC clock: 48 MHz USB PLL.
       Divider = 48e6 / (48000 * AUDIO_OVERSAMPLE) - 1 = 249 */
    adc_set_clkdiv((float)(48000000 / (AUDIO_SAMPLE_RATE * AUDIO_OVERSAMPLE)) - 1.0f);

    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, DREQ_ADC);

    dma_channel_configure(dma_chan, &cfg,
        raw[0], &adc_hw->fifo, RAW_BUF, false);

    dma_channel_set_irq0_enabled(dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    adc_run(true);
    dma_channel_start(dma_chan);
}
