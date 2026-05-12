/*
 * WM8731 Audio Codec Driver Implementation for RP2040
 *
 * Key changes from the previous version:
 *
 *  1. Pin layout changed: SDI moved to GP2, MCLK moved to GP6.
 *     GP2=SDI, GP3=BCLK, GP4=LRCK must be consecutive for i2s_in_slave.
 *
 *  2. PIO programs replaced:
 *     - TX: i2s_out_master  – RP2040 generates BCLK and LRCK (master mode).
 *     - RX: i2s_in_slave    – samples SDI by polling BCLK/LRCK at 4x BCK.
 *     Both SMs are started together with pio_enable_sm_mask_in_sync() so
 *     they are phase-locked and can never drift relative to each other.
 *
 *  3. DMA changed from 2 channels (simple reload-in-IRQ) to 4 channels
 *     (chained control-block double-buffering, after Collins):
 *       dma_tx_ctrl + dma_tx_data  – TX path (app buffer -> PIO TX FIFO)
 *       dma_rx_ctrl + dma_rx_data  – RX path (PIO RX FIFO -> app buffer)
 *     The ctrl channels reload the data channel addresses automatically so
 *     the IRQ handler only needs to update the tracking variables and flags.
 *
 *  4. The IRQ handler is now interrupt-minimal: no DMA register writes,
 *     just flag + index updates.
 */

#include "wm8731.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "i2s.pio.h"
#include "cdc_app.h"

/* -------------------------------------------------------------------------
 * PIO configuration
 *
 * Both state machines run on pio1 and are started in sync.
 *   sm_tx (i2s_out_master) – clocked at BCK * 2
 *   sm_rx (i2s_in_slave)   – clocked at SCK * 2 = BCK * 8  (4x oversampling)
 * ------------------------------------------------------------------------- */
#define I2S_PIO         pio1
#define I2S_BIT_DEPTH   32          /* must match WM8731_IWL_32BIT */

static uint sm_tx;
static uint sm_rx;
static uint sm_mask;                /* bitmask of both SMs for sync-start */
static uint offset_tx;
static uint offset_rx;

/* -------------------------------------------------------------------------
 * DMA channels (claimed in wm8731_i2s_init)
 * ------------------------------------------------------------------------- */
static int dma_tx_ctrl = -1;
static int dma_tx_data = -1;
static int dma_rx_ctrl = -1;
static int dma_rx_data = -1;

/*
 * Control blocks for double-buffering.
 * Each array holds pointers to the two buffer halves.
 * MUST be 8-byte aligned so the DMA ring-wrap (ring=3 -> 2^3=8 bytes) works.
 */
static int32_t* tx_ctrl_blocks[2] __attribute__((aligned(8)));
static int32_t* rx_ctrl_blocks[2] __attribute__((aligned(8)));

/* -------------------------------------------------------------------------
 * Application-visible DMA buffers and state
 * ------------------------------------------------------------------------- */
int32_t wm8731_tx_buffer[WM8731_DMA_NUM_BUFFERS][WM8731_DMA_BUFFER_SIZE];
int32_t wm8731_rx_buffer[WM8731_DMA_NUM_BUFFERS][WM8731_DMA_BUFFER_SIZE];

volatile uint8_t wm8731_current_tx_buffer = 0;
volatile uint8_t wm8731_current_rx_buffer = 0;
volatile bool    wm8731_tx_ready          = false;
volatile bool    wm8731_rx_ready          = false;

/* I2C instance */
#define WM8731_I2C  i2c0

/* =========================================================================
 * WM8731 register access
 * ========================================================================= */

bool wm8731_write_reg(uint8_t reg, uint16_t val) {
    uint8_t data[2];
    data[0] = (reg << 1) | ((val >> 8) & 0x01);
    data[1] = val & 0xFF;
    int ret = i2c_write_timeout_us(WM8731_I2C, WM8731_I2C_ADDR, data, 2, false, 2000);
    return (ret == 2);
}

/* =========================================================================
 * I2C initialisation (with bus-recovery)
 * ========================================================================= */

static bool wm8731_i2c_init(void) {
    /* Bus recovery: clock SCL 9 times to unstick any hung slave */
    gpio_init(WM8731_I2C_SCL_PIN);
    gpio_set_dir(WM8731_I2C_SCL_PIN, GPIO_OUT);
    gpio_init(WM8731_I2C_SDA_PIN);
    gpio_set_dir(WM8731_I2C_SDA_PIN, GPIO_IN);

    for (int i = 0; i < 9; i++) {
        gpio_put(WM8731_I2C_SCL_PIN, 1); sleep_us(5);
        gpio_put(WM8731_I2C_SCL_PIN, 0); sleep_us(5);
        if (gpio_get(WM8731_I2C_SDA_PIN)) break;
    }
    /* Issue STOP condition */
    gpio_set_dir(WM8731_I2C_SDA_PIN, GPIO_OUT);
    gpio_put(WM8731_I2C_SDA_PIN, 0); sleep_us(5);
    gpio_put(WM8731_I2C_SCL_PIN, 1); sleep_us(5);
    gpio_put(WM8731_I2C_SDA_PIN, 1); sleep_us(5);

    /* Hand pins over to hardware I2C */
    i2c_init(WM8731_I2C, 400 * 1000);
    gpio_set_function(WM8731_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(WM8731_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(WM8731_I2C_SDA_PIN);
    gpio_pull_up(WM8731_I2C_SCL_PIN);
    return true;
}

/* =========================================================================
 * MCLK via PWM on GP6
 *
 * At 120 MHz system clock, wrap=9 gives:
 *   120 MHz / 10 = 12 MHz  (close enough for USB-mode 48 kHz / 96 kHz)
 *
 * For a true 12.288 MHz you would need an external oscillator or use the
 * USB-mode setting in the WM8731 sampling register (which we do).
 * ========================================================================= */

static void wm8731_setup_mclk120(void) {
    gpio_set_function(WM8731_MCLK_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(WM8731_MCLK_PIN);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 1.0f);
    pwm_config_set_wrap(&cfg, 9);           /* 120 MHz / 10 = 12 MHz         */
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(WM8731_MCLK_PIN, 5); /* 50 % duty cycle               */
}

static void wm8731_setup_mclk(void) {
    gpio_set_function(WM8731_MCLK_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(WM8731_MCLK_PIN);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 1.0f);
    
    // CHANGE: 147.6 MHz / (11 + 1) = 12.3 MHz
    pwm_config_set_wrap(&cfg, 10); 
    
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(WM8731_MCLK_PIN, 6); // 50% duty cycle
}

/* =========================================================================
 * PIO setup
 *
 * Clock dividers:
 *   TX (i2s_out_master) runs at 2 PIO clocks per BCK:
 *     div_tx = sys_clk / (fs * 64 * 2)
 *
 *   RX (i2s_in_slave) runs at 2 PIO clocks per SCK (= 4x BCK):
 *     div_rx = sys_clk / (fs * 256 * 2)
 *
 *   At 120 MHz / 48 kHz:  div_tx = 19.53125,  div_rx = 4.8828125
 *   At 120 MHz / 96 kHz:  div_tx = 9.765625,  div_rx = 2.44140625
 *   Both have exact 1/256 representations -> zero clock-ratio error.
 * ========================================================================= */

static void wm8731_setup_i2s_pio(uint32_t sample_rate) {
    cdc_printf("DEBUG: wm8731_setup_i2s_pio, fs=%lu\n", sample_rate);

    /* Load PIO programs */
    offset_tx = pio_add_program(I2S_PIO, &i2s_out_master_program);
    offset_rx = pio_add_program(I2S_PIO, &i2s_in_slave_program);

    /* Claim state machines */
    sm_tx = pio_claim_unused_sm(I2S_PIO, true);
    sm_rx = pio_claim_unused_sm(I2S_PIO, true);
    sm_mask = (1u << sm_tx) | (1u << sm_rx);

    /* Initialise out_master FIRST (it sets BCLK and LRCK as outputs).
     * init function does NOT start the SM. */
    i2s_out_master_program_init(I2S_PIO, sm_tx, offset_tx,
                                I2S_BIT_DEPTH,
                                WM8731_SDO_PIN,
                                WM8731_BCLK_PIN);   /* clock_pin_base */

    /* Initialise in_slave SECOND (it only sets SDI as input; BCLK and LRCK
     * directions are already correct from out_master init above). */
    i2s_in_slave_program_init(I2S_PIO, sm_rx, offset_rx,
                              WM8731_SDI_PIN);       /* din_pin_base   */

    /* Set clock dividers */
    wm8731_set_i2s_samplerate(sample_rate);
}

/* =========================================================================
 * Clock divider update (can be called at runtime to change sample rate)
 * ========================================================================= */

void wm8731_set_i2s_samplerate(uint32_t sample_rate) {
    float sys_hz = (float)clock_get_hz(clk_sys);

    /*
     * The i2s_out_master loop body takes 3 PIO clocks per BCK period:
     *   cycle 0: out pins, 1  side BCK=0  (BCK low, data presented)
     *   cycle 1: irq  4       side BCK=1  (BCK rises, RX samples)
     *   cycle 2: jmp  x--     side BCK=1  (BCK high hold)
     *
     * BCK = fs * 64, so PIO must tick at fs * 64 * 3.
     *   div = sys_clk / (fs * 64 * 3)
     *
     * At 147.6 MHz / 48 kHz:  div = 147600000 / 9216000  = 16.015625 (exact)
     * At 147.6 MHz / 96 kHz:  div = 147600000 / 18432000 =  8.0078125 (exact)
     *
     * The old code used fs*256 which produced BCK at 4x the correct rate,
     * making recorded audio play back at 2x speed (chipmunk effect).
     *
     * RX SM runs at the same divider. It spends nearly all its time stalled
     * in `wait 1 irq 4`, so the divider only needs to guarantee that
     * `in pins, 1` completes before the next irq 4 fires. At the same
     * divider this is guaranteed with >1 PIO clock of margin.
     */
    float div = sys_hz / ((float)sample_rate * 256.0f );

    cdc_printf("DEBUG: samplerate=%lu sys_hz=%.0f div=%.6f\n",
               sample_rate, sys_hz, (double)div);

    pio_sm_set_clkdiv(I2S_PIO, sm_tx, div);
    pio_sm_set_clkdiv(I2S_PIO, sm_rx, div);

    /* Restart both dividers simultaneously so they are phase-aligned */
    pio_sm_clkdiv_restart(I2S_PIO, sm_mask);
}

/* =========================================================================
 * Codec initialisation
 * ========================================================================= */

bool wm8731_init(void) {
    wm8731_setup_mclk();
    sleep_ms(10);

    if (!wm8731_i2c_init()) return false;
    sleep_ms(10);

    if (!wm8731_write_reg(WM8731_REG_RESET, 0x00)) return false;
    sleep_ms(10);

    /* Power everything up */
    if (!wm8731_write_reg(WM8731_REG_POWER_DOWN, 0x00)) return false;

    return true;
}

bool wm8731_configure(uint32_t sample_rate) {
    bool ok = true;

    /* Line input: 0 dB */
    ok &= wm8731_write_reg(WM8731_REG_LLINE_IN, 0x017);
    ok &= wm8731_write_reg(WM8731_REG_RLINE_IN, 0x017);

    /* Headphone output volume */
    ok &= wm8731_write_reg(WM8731_REG_LHPHONE_OUT, 0x5F);
    ok &= wm8731_write_reg(WM8731_REG_RHPHONE_OUT, 0x5F);

    /* Analog path: DAC selected, line in selected, mic boost off */
    ok &= wm8731_write_reg(WM8731_REG_ANALOG_PATH,
                           WM8731_DACSEL | WM8731_INSEL | WM8731_MICBOOST);

    /* Digital path: no soft-mute, no de-emphasis */
    ok &= wm8731_write_reg(WM8731_REG_DIGITAL_PATH, 0x00);

    /*
     * Digital interface format:
     *   FORMAT = I2S (bit 1:0 = 10)
     *   IWL    = 32-bit (bit 3:2 = 11)
     *   MS     = 0 (slave mode – RP2040 is master)
     */
    ok &= wm8731_write_reg(WM8731_REG_DIGITAL_IF,
                           WM8731_FORMAT_I2S | WM8731_IWL_32BIT);

    
	/* Sampling control – Normal mode (better for 147.6MHz) */
    uint16_t sampling = 0x00; // Normal Mode (bit 0 = 0), 48kHz (SR bits = 0000)
    
    // If you need 96kHz, set SR bits to 0111 (0x1C)
    if (sample_rate == 96000) {
        sampling = (0x07 << 2); 
    }
    ok &= wm8731_write_reg(WM8731_REG_SAMPLING, sampling);

    /* Activate */
    ok &= wm8731_write_reg(WM8731_REG_ACTIVE, WM8731_ACTIVE);
    sleep_ms(10);

    return ok;
}

bool wm8731_set_volume(uint8_t volume) {
    if (volume > 127) volume = 127;
    uint16_t val = (volume & WM8731_HPVOL_MASK) | WM8731_HPBOTH | WM8731_HPZCEN;
    bool ok = true;
    ok &= wm8731_write_reg(WM8731_REG_LHPHONE_OUT, val);
    ok &= wm8731_write_reg(WM8731_REG_RHPHONE_OUT, val);
    return ok;
}

bool wm8731_set_mute(bool mute) {
    return wm8731_write_reg(WM8731_REG_DIGITAL_PATH,
                            mute ? WM8731_DACMU : 0x00);
}

/* =========================================================================
 * DMA IRQ handler
 *
 * With chained control-block DMA the hardware reloads the data channel
 * address automatically – we do NOT touch any DMA registers here.
 * We only update the tracking index (toggle 0<->1) and raise the ready flag.
 *
 * TX: IRQ fires when the data channel finishes consuming buffer[x].
 *     DMA is now consuming buffer[x^1].  Tell app to fill buffer[x^1^1]=x.
 *
 * RX: IRQ fires when the data channel finishes filling buffer[x].
 *     DMA is now filling buffer[x^1].  Tell app to read buffer[x].
 *     We set current_rx_buffer = x^1 (the one now being filled) so that
 *     uac2_app's "source_buf = (current_rx_buffer + 1) % 2" gives x. ✓
 * ========================================================================= */

static void wm8731_dma_irq_handler(void) {
    if (dma_channel_get_irq0_status(dma_tx_data)) {
        dma_channel_acknowledge_irq0(dma_tx_data);
        wm8731_current_tx_buffer ^= 1;
        wm8731_tx_ready = true;
    }

    if (dma_channel_get_irq0_status(dma_rx_data)) {
        dma_channel_acknowledge_irq0(dma_rx_data);
        wm8731_current_rx_buffer ^= 1;
        wm8731_rx_ready = true;
    }
}

/* =========================================================================
 * I2S + DMA initialisation
 *
 * Sets up PIO programs and the 4-channel chained-DMA topology.
 * Does NOT start the SMs or DMA transfers – call wm8731_start_dma() for that.
 * ========================================================================= */

void wm8731_i2s_init(uint32_t sample_rate) {
    cdc_printf("DEBUG: wm8731_i2s_init\n");

    wm8731_setup_i2s_pio(sample_rate);

    /* ---- Claim four DMA channels ---- */
    dma_tx_ctrl = dma_claim_unused_channel(true);
    dma_tx_data = dma_claim_unused_channel(true);
    dma_rx_ctrl = dma_claim_unused_channel(true);
    dma_rx_data = dma_claim_unused_channel(true);

    /* ---------------------------------------------------------------
     * TX ctrl channel
     *
     * Reads a pointer from tx_ctrl_blocks[] (8-byte ring) and writes
     * it to the TX data channel's al3_read_addr_trig register, which
     * simultaneously sets the read address and re-triggers the data
     * channel.  This happens automatically whenever the data channel
     * chains back here.
     * --------------------------------------------------------------- */
    {
        dma_channel_config c = dma_channel_get_default_config(dma_tx_ctrl);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        channel_config_set_ring(&c, false, 3);  /* read wraps every 8 bytes */
        dma_channel_configure(
            dma_tx_ctrl, &c,
            &dma_hw->ch[dma_tx_data].al3_read_addr_trig, /* dst: tx_data read+trig */
            tx_ctrl_blocks,                               /* src: ctrl block array  */
            1,                                            /* 1 word per trigger     */
            false);                                       /* don't start yet        */
    }

    /* ---------------------------------------------------------------
     * TX data channel
     *
     * Reads from the application TX buffer (incrementing), writes to
     * the PIO TX FIFO (fixed address), paced by the PIO TX DREQ.
     * Chains back to tx_ctrl when a full buffer has been transferred.
     * --------------------------------------------------------------- */
    {
        dma_channel_config c = dma_channel_get_default_config(dma_tx_data);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        channel_config_set_chain_to(&c, dma_tx_ctrl);
        channel_config_set_dreq(&c, pio_get_dreq(I2S_PIO, sm_tx, true));
        dma_channel_configure(
            dma_tx_data, &c,
            &I2S_PIO->txf[sm_tx],  /* dst: PIO TX FIFO (fixed)  */
            NULL,                   /* src: set by ctrl channel  */
            WM8731_DMA_BUFFER_SIZE,
            false);
    }

    /* ---------------------------------------------------------------
     * RX ctrl channel
     *
     * Reads a pointer from rx_ctrl_blocks[] (8-byte ring) and writes
     * it to the RX data channel's al2_write_addr_trig register.
     * --------------------------------------------------------------- */
    {
        dma_channel_config c = dma_channel_get_default_config(dma_rx_ctrl);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        channel_config_set_ring(&c, false, 3);
        dma_channel_configure(
            dma_rx_ctrl, &c,
            &dma_hw->ch[dma_rx_data].al2_write_addr_trig, /* dst: rx_data write+trig */
            rx_ctrl_blocks,
            1,
            false);
    }

    /* ---------------------------------------------------------------
     * RX data channel
     *
     * Reads from the PIO RX FIFO (fixed), writes to the application
     * RX buffer (incrementing), paced by the PIO RX DREQ.
     * Chains back to rx_ctrl when a full buffer has been filled.
     * --------------------------------------------------------------- */
    {
        dma_channel_config c = dma_channel_get_default_config(dma_rx_data);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, false);
        channel_config_set_write_increment(&c, true);
        channel_config_set_chain_to(&c, dma_rx_ctrl);
        channel_config_set_dreq(&c, pio_get_dreq(I2S_PIO, sm_rx, false));
        dma_channel_configure(
            dma_rx_data, &c,
            NULL,                   /* dst: set by ctrl channel  */
            &I2S_PIO->rxf[sm_rx],  /* src: PIO RX FIFO (fixed)  */
            WM8731_DMA_BUFFER_SIZE,
            false);
    }

    /* ---- IRQ setup ---- */
    dma_channel_set_irq0_enabled(dma_tx_data, true);
    dma_channel_set_irq0_enabled(dma_rx_data, true);
    irq_set_exclusive_handler(DMA_IRQ_0, wm8731_dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
}

/* =========================================================================
 * Start DMA transfers and PIO state machines
 * ========================================================================= */

void wm8731_start_dma(void) {
    /* Reset tracking state */
    wm8731_current_tx_buffer = 0;
    wm8731_current_rx_buffer = 0;
    wm8731_tx_ready           = false;
    wm8731_rx_ready           = false;

    /* Zero both TX and RX buffers */
    for (int b = 0; b < WM8731_DMA_NUM_BUFFERS; b++) {
        for (int s = 0; s < WM8731_DMA_BUFFER_SIZE; s++) {
            wm8731_tx_buffer[b][s] = 0;
            wm8731_rx_buffer[b][s] = 0;
        }
    }

    /*
     * Point control blocks at the two buffer rows.
     * tx_ctrl_blocks[0] -> buffer being played first, [1] -> second.
     * rx_ctrl_blocks[0] -> buffer being filled first, [1] -> second.
     *
     * The DMA ctrl channel reads [0], triggers data channel with that
     * address, then (ring wrap) reads [1] after the next chain-back, etc.
     */
    tx_ctrl_blocks[0] = wm8731_tx_buffer[0];
    tx_ctrl_blocks[1] = wm8731_tx_buffer[1];
    rx_ctrl_blocks[0] = wm8731_rx_buffer[0];
    rx_ctrl_blocks[1] = wm8731_rx_buffer[1];

    /* Reload ctrl channel read addresses to the start of the arrays.
     * (In case wm8731_start_dma is called a second time.) */
    dma_channel_set_read_addr(dma_tx_ctrl, tx_ctrl_blocks, false);
    dma_channel_set_read_addr(dma_rx_ctrl, rx_ctrl_blocks, false);

    /* Make sure SMs are stopped before touching FIFOs */
    pio_sm_set_enabled(I2S_PIO, sm_tx, false);
    pio_sm_set_enabled(I2S_PIO, sm_rx, false);
    pio_sm_clear_fifos(I2S_PIO, sm_tx);
    pio_sm_clear_fifos(I2S_PIO, sm_rx);

    /*
     * Pre-fill the TX FIFO with silence.
     * The out_master program uses "pull noblock" which substitutes OSR=0
     * when the FIFO is empty, but pre-filling guarantees clean startup
     * without any glitch on the very first BCK edges.
     */
    for (int i = 0; i < 8; i++) {
        pio_sm_put(I2S_PIO, sm_tx, 0);
    }

    /*
     * Start ctrl channels first.  Each ctrl channel immediately reads its
     * first control block, writes the buffer address into the corresponding
     * data channel's addr+trig register, which starts the data channel.
     * The data channels are then self-sustaining via the chain-back.
     */
    dma_channel_start(dma_tx_ctrl);
    dma_channel_start(dma_rx_ctrl);

    /*
     * Enable both SMs simultaneously and in-phase.
     * pio_enable_sm_mask_in_sync holds the SMs in reset until the moment
     * all selected SMs can be released together on the same PIO clock edge.
     */
    pio_enable_sm_mask_in_sync(I2S_PIO, sm_mask);
    
    //sleep_us(200);
    
    //dma_channel_start(dma_tx_ctrl);
    //dma_channel_start(dma_rx_ctrl);
}

/* =========================================================================
 * Stop DMA transfers (SMs are left running; call wm8731_disable_i2s if needed)
 * ========================================================================= */

void wm8731_stop_dma(void) {
    if (dma_tx_data >= 0) dma_channel_abort(dma_tx_data);
    if (dma_tx_ctrl >= 0) dma_channel_abort(dma_tx_ctrl);
    if (dma_rx_data >= 0) dma_channel_abort(dma_rx_data);
    if (dma_rx_ctrl >= 0) dma_channel_abort(dma_rx_ctrl);
}

/* =========================================================================
 * Enable / disable PIO state machines
 * ========================================================================= */

void wm8731_enable_i2s(void) {
    pio_enable_sm_mask_in_sync(I2S_PIO, sm_mask);
}

void wm8731_disable_i2s(void) {
    pio_sm_set_enabled(I2S_PIO, sm_tx, false);
    pio_sm_set_enabled(I2S_PIO, sm_rx, false);
}
