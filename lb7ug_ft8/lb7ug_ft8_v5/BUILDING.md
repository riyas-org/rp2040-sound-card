# Building the RP2040 HF Transceiver

## Quick start

```bash
export PICO_SDK_PATH=~/pico-sdk
mkdir build && cd build
cmake .. -G Ninja
ninja
# Flash: hold BOOTSEL, plug USB, copy build/trx.uf2 to RPI-RP2 drive
```

## Stage-by-stage build guide

Work through these stages in order. At each stage, comment/uncomment
the relevant `FEATURE_*` flags in `config.h` and verify before moving on.

### Stage 1 — I2C scan (no OLED needed yet)
- All features disabled
- Connect serial terminal (UART0 on GP0/GP1 at 115200 if enabled,
  or use a logic analyser on I2C)
- Expected: `i2c_bus_init()` returns 0x07 (all three chips found)
- If a chip is missing: check wiring, pullups (4.7kΩ to 3.3V on SDA/SCL)

### Stage 2 — OLED
- `oled_init()` + `boot_splash()` should show chip status
- If blank: check SSD1306 address (0x3C vs 0x3D jumper)

### Stage 3 — AM receive
- Si4732 powered up, tuned to a known AM broadcast station (e.g. 1 MHz)
- Audio should come out of Si4732 line-out pin
- Verify with headphones directly on Si4732 LOUT before connecting ADC

### Stage 4 — Si5351 frequency output
- CLK0 should appear on CLK0 pin at the tuned frequency
- Verify with an SDR dongle or frequency counter
- If wrong frequency: check crystal (25 MHz), use `cal <ppb>` to trim

### Stage 5 — VFO + encoder
- Encoder rotation changes frequency on OLED and re-tunes Si4732
- Short press cycles step; long press opens band menu
- Flash save/load: power cycle should restore last frequency

### Stage 6 — Full OLED menu
- Long press cycles through: Band → Mode → BW → Step → RIT → AGC →
  CW WPM → BFO → Split → Mem
- Encoder navigates, short press confirms, 5s timeout auto-closes

### Stage 7 — USB CDC terminal + CAT
```
screen /dev/ttyACM0 115200
```
- TRX terminal menu should appear on connect
- Test CAT with WSJT-X: Rig=TS-480, port=ttyACM0
- `FA;` should return current frequency

### Stage 8 — SSB receive (needs full SSB patch)
1. Download patch:
   ```
   git clone --depth=1 https://github.com/pu2clr/SI4735.git /tmp/si
   ```
2. Copy body of `SSBRX_PATCH_FULL_CONTENT[]` from
   `/tmp/si/extras/patches/patch_full_ssb.h`
   into `hal/si4732.c` `ssb_patch[]` array.
3. Enable `FEATURE_SSB_PATCH` in config.h
4. Set mode to LSB/USB, tune to SSB station, verify audio

### Stage 9 — ADC audio → USB
- Enable `FEATURE_DUAL_CORE` in config.h
- GP26 circuit: Si4732 LOUT → 100nF → 1kΩ → GP26
                                               ├── 10kΩ → 3.3V
                                               └── 10kΩ → GND
- Open WSJT-X, set audio input to "HF Transceiver"
- Waterfall should show received signals

### Stage 10 — FT8/WSPR TX
- Enable `FEATURE_FT8_WSPR`
- Set mode to FT8, tune to 14.074 MHz
- WSJT-X transmit: audio flows USB OUT → Goertzel → Si5351 CLK0 FSK
- Monitor CLK0 with SDR dongle; should see 8 tones shifting at 160ms intervals

### Stage 11 — CW
- Straight key on GP11 (active LOW)
- Sidetone on GP12 (700 Hz, connect small speaker or headphone)
- Enable `FEATURE_CW_DECODER` to see decoded text on OLED bottom row
- Test CW memories from terminal: `mem 1`

### Stage 12 — SSB TX (voice)
- Enable `FEATURE_SSB_TX`
- GP13: PWM envelope → RC filter (1kΩ + 100nF) → PA supply modulation
- Si5351 CLK0: carrier to PA input
- Method: `ssb_mod_method = SSB_METHOD_PWM_ENVELOPE` (default, uSDX-style)
- For quadrature method: also connect CLK1 through 90° RC network,
  combine with CLK0 at antenna through a 90° hybrid coupler

## Common errors

| Error | Fix |
|-------|-----|
| `extern menu_id_t active` warning | Move `active` to a getter function in menu.c |
| `si4732.h` includes `vfo.h` before it exists | radio_mode_t is defined in vfo.h; si4732.h needs it. Compile order is fine with CMake. |
| Audio glitches | Increase `CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ` to 384*8 |
| Si5351 wrong frequency | Run `cal <ppb>` from terminal; typical TCXO offset ±200 ppb |
| WSJT-X `?;` responses | Check CAT command is uppercase + ends with `;` |
| Core 1 DMA IRQ conflict | Ensure no other DMA_IRQ_0 handler; use DMA_IRQ_1 if needed |

## File map — what to edit for each change

| Task | File |
|------|------|
| Change pins | `config.h` |
| Add a band | `radio/vfo.c` → `bands[]` |
| Add a menu item | `ui/menu.c` → `menu_id_t`, `draw_menu()`, `menu_btn()` |
| Add a CAT command | `ui/terminal.c` → `cat_exec()` |
| Add a terminal shortcut | `ui/terminal.c` → `shortcut()` |
| Change SSB TX method | `dsp/ssb_mod.c` → `ssb_mod_method` |
| Tune AGC | `radio/agc.c` → `AGC_TARGET`, `AGC_MAX` |
| Change Goertzel threshold | `dsp/goertzel.c` → noise floor constant |
