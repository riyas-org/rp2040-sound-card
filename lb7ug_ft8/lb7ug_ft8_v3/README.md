# RP2040 HF Transceiver

A full-featured HF transceiver firmware for the Raspberry Pi RP2040.

## Hardware

| Pin  | Function                          |
|------|-----------------------------------|
| GP0  | I2C0 SDA (Si5351, Si4732, OLED)  |
| GP1  | I2C0 SCL                         |
| GP6  | Si4732 RESET (active LOW)        |
| GP7  | Rotary encoder A                 |
| GP8  | Rotary encoder B                 |
| GP9  | Encoder push-button (active LOW) |
| GP10 | RX/TX relay (HIGH = TX)          |
| GP11 | CW straight key (active LOW)     |
| GP12 | Sidetone PWM output (700 Hz)     |
| GP13 | SSB TX amplitude envelope PWM   |
| GP25 | Onboard LED                      |
| GP26 | ADC audio in from Si4732         |

**GP26 input circuit:**
```
Si4732 LOUT ─── 100nF ─── 1kΩ ─── GP26
                                    ├── 10kΩ ── 3.3V
                                    └── 10kΩ ── GND
```
The two 10kΩ resistors bias the ADC midpoint to 1.65V.

## Features

- **Receive:** Si4732 AM/SSB/FM with SSB patch (USB/LSB/CW)
- **VFO:** Si5351 CLK0, 10 bands 160m–6m, full frequency display
- **FT8/WSPR TX:** Goertzel tone detection → Si5351 direct FSK
- **SSB TX:** PWM amplitude envelope on GP13 + Si5351 carrier (uSDX-style)
- **CW:** Straight key, sidetone, Morse decoder, 5 message memories
- **OLED menu:** Band/Mode/BW/Step/RIT/AGC/BFO/Split/CW Mem
- **Terminal:** QMX-style interactive menu over USB CDC
- **CAT:** Kenwood TS-480 subset (FA/FB/MD/IF/SM/TX/RX/ID/RA/SH/VX)
- **USB Audio:** UAC2 2ch 24-bit 48kHz bidirectional
- **Dual core:** Core 1 handles ADC DMA; Core 0 handles UI/USB
- **Flash:** Settings auto-saved 5s after last encoder activity

## Quick build

```bash
git clone https://github.com/raspberrypi/pico-sdk ~/pico-sdk
cd ~/pico-sdk && git submodule update --init
export PICO_SDK_PATH=~/pico-sdk

cd trx
mkdir build && cd build
cmake .. -G Ninja
ninja          # builds full firmware: trx.uf2
ninja stage1   # builds minimal I2C scan: stage1.uf2
ninja stage2   # OLED test
ninja stage3   # AM receive test
ninja stage4   # VFO test
```

Flash: hold BOOTSEL, plug USB, copy `.uf2` to the RPI-RP2 drive.

## Staged bring-up

Work through stages in order. Each stage is a self-contained `.c` file
in the `stages/` directory and compiles independently.

| Stage | File | What to verify |
|-------|------|----------------|
| 1 | `stages/stage1_blink_i2c.c` | LED blinks, serial shows `FOUND` for all 3 chips |
| 2 | `stages/stage2_oled.c` | OLED shows text and animated S-meter bar |
| 3 | `stages/stage3_rx.c` | AM receive, encoder tunes, audio from Si4732 |
| 4 | `stages/stage4_vfo.c` | Si5351 CLK0 on air, verify with SDR dongle |
| 5–12 | `main.c` + `config.h` feature flags | Enable one flag at a time |

For stages 5–12, edit `config.h` and comment out features not yet needed:
```c
// #define FEATURE_SSB_PATCH   // enable at stage 8
// #define FEATURE_SSB_TX      // enable at stage 12
// #define FEATURE_FT8_WSPR    // enable at stage 10
// #define FEATURE_CW_DECODER  // enable at stage 11
// #define FEATURE_FLASH_SAVE  // enable at stage 5
// #define FEATURE_DUAL_CORE   // enable at stage 9
```

## SSB patch

The Si4732 SSB patch (enables USB/LSB/CW receive) must be copied from the
PU2CLR Si4735 library into `hal/si4732.c`:

```bash
git clone --depth=1 https://github.com/pu2clr/SI4735.git /tmp/si
```
Open `/tmp/si/extras/patches/patch_full_ssb.h`, copy the body of
`SSBRX_PATCH_FULL_CONTENT[]` into the `ssb_patch[]` array in `hal/si4732.c`.

## WSJT-X / FT8CN setup

| Setting | Value |
|---------|-------|
| Rig | Kenwood TS-480 |
| Serial port | USB CDC (ttyACM0 / COMx) |
| Baud rate | Any (CDC ignores baud) |
| PTT method | CAT |
| Audio output | HF Transceiver (USB Audio) |
| Audio input | HF Transceiver (USB Audio) |

## Terminal usage

Connect any serial terminal to the USB CDC port:
```
screen /dev/ttyACM0 115200
# or: minicom -D /dev/ttyACM0
# or Windows: PuTTY → Serial → COMx → 115200
```

Type `?` for help. Common shortcuts:
```
f 14074000    set frequency
b 6           set band (20m)
m 2           set mode (USB)
w 20          CW speed 20 WPM
cal -120      crystal calibration in ppb
stat          show all settings
save          write to flash
cat           switch to CAT passthrough mode
              (type MENU<Enter> to return)
```

## File map

```
config.h              All pin/rate/feature defines — edit here first
main.c                Init + main loop only
hal/                  Hardware abstraction layer
  i2c_bus.c/h         I2C init + bus recovery + scan
  si5351.c/h          VFO/CLK driver
  si4732.c/h          Receiver + SSB patch
  oled.c/h            SSD1306 framebuffer
  adc_audio.c/h       DMA audio capture (Core 1)
radio/                Radio-layer logic
  vfo.c/h             Frequency/band/mode state + flash
  trx.c/h             RX/TX sequencer
  cw_keyer.c/h        Keyer + decoder + memories
  agc.c/h             Digital AGC
dsp/                  Signal processing
  goertzel.c/h        FT8/WSPR tone detection
  ssb_mod.c/h         SSB TX: PWM + Si5351 phase
ui/                   User interface
  menu.c/h            OLED menu system
  terminal.c/h        CDC terminal + CAT
usb/                  TinyUSB callbacks
  cdc_app.c/h         CDC RX/TX routing
  uac2_app.c/h        UAC2 audio callbacks
  usb_descriptors.c/h UAC2 + CDC composite descriptors
stages/               Standalone stage test programs
  stage1_blink_i2c.c
  stage2_oled.c
  stage3_rx.c
  stage4_vfo.c
```

## SSB TX method (Stage 12)

Two methods are available, selectable at runtime:

**PWM_ENVELOPE (default, uSDX-style):**
Audio amplitude modulates the PA supply voltage via a PWM-filtered signal
on GP13. Si5351 CLK0 provides a constant carrier. Simple, works well for
voice at QRP power levels.

**QUADRATURE:**
A 7-tap FIR Hilbert transform generates I and Q components. I drives CLK0
phase, Q drives CLK1 phase through a 90° RC network. Combined at the antenna
via a hybrid coupler → SSB. Better sideband suppression but requires more
hardware.

Set method in `dsp/ssb_mod.c`:
```c
ssb_mod_method = SSB_METHOD_PWM_ENVELOPE;   // default
ssb_mod_method = SSB_METHOD_QUADRATURE;     // experimental
```
