# Arel OS on esp32c3

## Initial Rust environment setup (macOS)

Install Apple's command-line tools and the native build prerequisites
([Homebrew](https://brew.sh/) is required for the second command):

```shell
xcode-select --install
brew install git ninja pkg-config
```

If Apple's command-line tools are already installed, skip the first command.
Install Rust using [rustup](https://rustup.rs/), then open a new terminal or load
the Cargo environment:

```shell
source "$HOME/.cargo/env"
rustup update stable
```

From this project's root directory, select stable Rust and install the ESP32-C3
target and command-line tools:

```shell
rustup override set stable
rustup target add riscv32imc-unknown-none-elf
cargo install laze --locked
cargo install espflash --locked
```

Ariel OS v0.6.0 requires Rust **1.95 or newer**. Keep Cargo updated with Rust:
this project's `.cargo/config.toml` includes Ariel's dependency patches and build
profiles from `build/imports/ariel-os/ariel-os-cargo.toml`. The `include` key is
[stable since Cargo 1.93](https://doc.rust-lang.org/cargo/reference/unstable.html#config-include),
so a current stable toolchain works without copying patches into `Cargo.toml`
or using nightly. The legacy `[unstable] config-include` entry in the config is
not needed by current Cargo. Laze downloads the Ariel import on the first build;
run laze before invoking Cargo directly on a fresh checkout.

ESP32-C3 is a **RISC-V** chip: this project does not require the Xtensa toolchain
installed by `espup`, or an ESP-IDF installation.

Check the setup and compile without flashing:

```shell
rustc --version
cargo --version
rustup target list --installed
ninja --version
laze --version
espflash --version
laze build -b espressif-esp32-c3-lcdkit
```

Connect the Waveshare using a USB data cable, then build, flash, and monitor logs:

```shell
laze build -b espressif-esp32-c3-lcdkit run
```

If espflash asks, select the board's `/dev/cu.usbmodem...` port. Close other serial
monitors before flashing. If the board will not enter download mode, hold BOOT,
press and release RESET, then release BOOT and retry. Press Ctrl+C to exit the
log monitor.

Common setup errors:

| Error | Fix |
| --- | --- |
| `launching ninja ... No such file or directory` | Install Ninja with `brew install ninja`. |
| `can't find crate for core` | Install `riscv32imc-unknown-none-elf` for the active Rust toolchain. |
| `embassy-nrf ... does not have feature optfield` | Check Rust/Cargo versions and that the Ariel config include exists and is loaded; run `rustup update stable`. |
| `espflash: command not found` | Install espflash and ensure `$HOME/.cargo/bin` is in `PATH`. |
| Serial port missing or busy | Check the USB data cable, port selection, and other serial monitors. |

## Hardware

- [ESP32-C3-DEV-KIT-M-1](https://manuals.plus/asin/B0CKR4R5VM)
- [BME280](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bme280/)
- [Waveshare ESP32-C3-LCD-1.47](https://docs.waveshare.com/ESP32-C3-LCD-1.47), with the onboard LCD
- [Sensirion SCD40](https://sensirion.com/products/catalog/SCD40), for CO₂, temperature, and humidity

## SCD40 sensor wiring and readings

Connect the sensor module to the board as follows. `03` and `04` mean GPIO3 and
GPIO4, rather than physical header positions.

| SCD40 connection | ESP32-C3 board connection |
| --- | --- |
| GND / GRND | GND |
| SCL | GPIO3 (`03`) |
| SDA | GPIO4 (`04`) |
| VDD / VCC | 3.3 V, if supported by your sensor breakout |

The SCD40 chip supports 3.3 V operation, with peak current up to 205 mA. Check
the breakout's input-voltage requirements and the available 3.3 V supply capacity.
Keep SDA/SCL pull-ups at 3.3 V for the ESP32-C3; use external pull-ups if the board
and sensor module do not already provide them. Supply and I²C requirements are
specified in the [Sensirion datasheet](https://sensirion.com/media/documents/48C4B7FB/67FE0194/CD_DS_SCD4x_Datasheet_D1.pdf).

GPIO3/GPIO4 already connect to the LCD I²C expander. The application initializes
**one I2C0 bus at 100 kHz** and uses it for both devices: the LCD expander at
`0x24` and the SCD40 at `0x62`. It initializes the LCD control lines and backlight
first, then gives the same bus to the `scd4x` async driver for sensor reads.

At startup, the application stops any previous periodic measurement before
starting a new one, so an ESP32 reset also works when the sensor remains powered.
The sensor produces a new measurement about every five seconds. The application
checks data-ready status before reading and uses the driver's CRC validation.
Valid nonzero CO₂ samples appear in the logs and in the lower part of the LCD:

```text
SCD40: CO2=500 ppm, temperature=25 C, humidity=37 %RH
```

The LCD shows waiting, disconnected, or read-failed status instead of stale data
when appropriate. Connection/start failures retry after five seconds while the
LCD demo continues. Build/flash with the existing command:

```shell
laze build -b espressif-esp32-c3-lcdkit run
```

## Rust LCD graphics

The application draws text, red/green/blue test bars, and a blinking cyan square
using `embedded-graphics` on a 172 × 320 RGB565 framebuffer. Drawing happens in
RAM; `flush_lcd_framebuffer()` sends the result using Ariel OS's async SPI bus.
The buffer occupies 110,080 bytes of static RAM, outside the executor stack.
The USB Type-C port supplies power and supports flashing/logging; the LCD itself
uses SPI, with control signals on an I²C expander.

This integration assumes the **Waveshare ESP32-C3-LCD-1.47** board. Although its
specification table lists **ST7798**, its [official panel driver](https://github.com/waveshareteam/ESP32-C3-LCD-1.47/blob/6e16df57c20b3b1651c1b6150a1c5b3c418a87f6/example/ESP-IDF/components/LCD_Driver/Vernon_ST7789T/Vernon_ST7789T.c)
uses an ST7789-compatible initialization sequence. The application follows that
sequence, including inversion, BGR color order, and little-endian RGB565 data.
A different board with the same advertised resolution may need different wiring
or initialization.

| LCD connection | ESP32-C3 pin / expander signal |
| --- | --- |
| SPI clock | GPIO7 |
| SPI MOSI | GPIO5 |
| SPI MISO (shared with SD; not required for LCD writes) | GPIO6 |
| Data/command | GPIO8 |
| Expander I²C SDA | GPIO4 |
| Expander I²C SCL | GPIO3 |
| LCD chip select | Expander EXIO0, active low |
| LCD reset | Expander EXIO1, active low |
| SD chip select | Expander EXIO2, held high |
| Backlight | Expander PWM register `0x05` |

The expander address is `0x24`. Its register map and wiring come from the
[official board library](https://github.com/waveshareteam/ESP32-C3-LCD-1.47/blob/6e16df57c20b3b1651c1b6150a1c5b3c418a87f6/example/Arduino/libraries/ESP32_C3_LCD_1in47/ESP32_C3_LCD_1in47.cpp)
and [expander definitions](https://github.com/waveshareteam/ESP32-C3-LCD-1.47/blob/6e16df57c20b3b1651c1b6150a1c5b3c418a87f6/example/Arduino/libraries/ESP32_C3_LCD_1in47/io_extension.h).
The application reserves SPI2 for the LCD and leaves the SD card deselected.
Supporting SD access later requires coordinating chip select through the expander.

The manufacturer demo uses **180 × 320 at column offset 30**, despite the
advertised width of 172. This application centers the requested 172 columns in
that window with `LCD_COLUMN_OFFSET = 34` in `src/lcd_display.rs`. The physical
panel width and alignment must be checked on hardware. For a confirmed 180-column
panel, change `LCD_WIDTH` to 180 in `src/lcd_framebuffer.rs` and the offset to 30.
SPI runs in mode 0 at 20 MHz for initial bring-up.

Build without flashing:

```shell
laze build -b espressif-esp32-c3-lcdkit
```

Build, flash, and view logs:

```shell
laze build -b espressif-esp32-c3-lcdkit run
```

The existing `espressif-esp32-c3-lcdkit` builder supplies the ESP32-C3 target and
USB serial logging; the Waveshare LCD pin mapping is defined in the application.
On boot, expect `LCD: display initialized; showing text and RGB test bars` in the
logs. A blank display with an expander error suggests a board/pinout mismatch.
Use the RGB bars and blinking square to check colors and refresh, and check the
left/right alignment before changing the column offset.

Display transport and panel setup live in `src/lcd_display.rs`; the graphics draw
target and its clipping/byte-order tests live in `src/lcd_framebuffer.rs`.

## Flashing

```shell
TODO: add instructions for flashing with espflash
like flash --chip esp32c3 --port /dev/ttyACM0 --baud 115200 --no-stub build/bin/espressif-esp32-c3-devkitc-1/cargo/riscv32imac-unknown-none-elf/release/hello-fosdem
```

## Build, flash and run
```shell
laze build -b espressif-esp32-c3-lcdkit run
```

### Erase flash

```shell
espflash erase-flash -p /dev/cu.usbmodem212401                                                                        
[2026-02-19T07:51:52Z INFO ] Serial port: '/dev/cu.usbmodem212401'
[2026-02-19T07:51:52Z INFO ] Connecting...
[2026-02-19T07:51:52Z INFO ] Using flash stub
[2026-02-19T07:51:52Z INFO ] Erasing Flash...
[2026-02-19T07:51:55Z INFO ] Flash has been erased!
```

### Device

Specifications

Parameter	Value
Interface	USB Type-C
Main Controller	ESP32-C3FH4
Screen Type	ISP
Screen Resolution	172 × 320
Display Chip	ST7798


```shell
Chip type:         esp32c3 (revision v1.1)
Crystal frequency: 40 MHz
Flash size:        4MB
Features:          WiFi, BLE
MAC address:       9c:cc:01:d9:6a:34
App/part. size:    91,120/4,128,768 bytes, 2.21%
[00:00:00] [========================================]      13/13      0x0      Verifying... OK!                                                                                                                                      [00:00:00] [========================================]       1/1       0x8000   Verifying... OK!                                                                                                                                      [00:00:00] [========================================]      24/24      0x10000  Verifying... OK!                                                                                                                                      [2026-10-14T16:28:17Z INFO ] Flashing has completed!
Commands:
    CTRL+R    Reset chip
    CTRL+C    Exit

ESP-ROM:esp32c3-eco7-20230720
Build:Jul 20 2023
rst:0x15 (USB_UART_CHIP_RESET),boot:0xf (SPI_FAST_FLASH_BOOT)
Saved PC:0x40380862
SPIWP:0xee
mode:DIO, clock div:2
load:0x3fcd5820,len:0x15c4
load:0x403cbf10,len:0xc84
load:0x403ce710,len:0x2fd0
entry 0x403cbf1a
```
