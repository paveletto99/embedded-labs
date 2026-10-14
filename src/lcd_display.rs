//! Waveshare ESP32-C3-LCD-1.47 transport, using Ariel OS async SPI and I2C.

use ariel_os::{gpio, time::Timer};
use embedded_hal_async::{i2c::I2c, spi::SpiBus};

use crate::lcd_framebuffer::{LCD_HEIGHT, LCD_WIDTH, LcdFramebuffer};

const LCD_EXPANDER_ADDRESS: u8 = 0x24;
const EXPANDER_MODE_REGISTER: u8 = 0x02;
const EXPANDER_OUTPUT_REGISTER: u8 = 0x03;
const EXPANDER_PWM_REGISTER: u8 = 0x05;
// EXIO0 = LCD CS, EXIO1 = LCD reset, EXIO2 = SD CS. Preserve the other output levels.
const EXPANDER_OUTPUT_INITIAL: u8 = 0xF7;
const EXPANDER_LCD_SELECTED: u8 = EXPANDER_OUTPUT_INITIAL & !0x01;
// Center the advertised 172 columns in the vendor demo's 180-column window at column 30.
const LCD_COLUMN_OFFSET: u16 = 34;

/// Selects and resets the LCD through its I2C expander, leaving the SD card deselected.
pub(crate) async fn prepare_lcd_expander<I: I2c>(i2c: &mut I) -> Result<(), I::Error> {
    for (register, value) in [
        (EXPANDER_MODE_REGISTER, 0xFF),
        (EXPANDER_OUTPUT_REGISTER, EXPANDER_OUTPUT_INITIAL),
        (EXPANDER_PWM_REGISTER, 0),
        (EXPANDER_OUTPUT_REGISTER, EXPANDER_LCD_SELECTED),
    ] {
        i2c.write(LCD_EXPANDER_ADDRESS, &[register, value]).await?;
    }
    Timer::after_millis(20).await;
    i2c.write(
        LCD_EXPANDER_ADDRESS,
        &[EXPANDER_OUTPUT_REGISTER, EXPANDER_LCD_SELECTED & !0x02],
    )
    .await?;
    Timer::after_millis(20).await;
    i2c.write(
        LCD_EXPANDER_ADDRESS,
        &[EXPANDER_OUTPUT_REGISTER, EXPANDER_LCD_SELECTED],
    )
    .await?;
    Timer::after_millis(120).await;
    Ok(())
}

/// Enables the LCD backlight after the first frame has been transferred successfully.
pub(crate) async fn enable_lcd_backlight<I: I2c>(i2c: &mut I) -> Result<(), I::Error> {
    i2c.write(LCD_EXPANDER_ADDRESS, &[EXPANDER_PWM_REGISTER, 0xFF])
        .await
}

async fn send_lcd_command<S: SpiBus<u8>>(
    spi: &mut S,
    dc: &mut gpio::Output<'_>,
    command: u8,
    parameters: &[u8],
) -> Result<(), S::Error> {
    dc.set_low();
    spi.write(&[command]).await?;
    // Ensure the command has left the bus before changing the data/command pin.
    spi.flush().await?;
    dc.set_high();
    if !parameters.is_empty() {
        spi.write(parameters).await?;
        spi.flush().await?;
    }
    Ok(())
}

/// Applies the manufacturer's ST7789-compatible panel setup, including little-endian RGB565.
pub(crate) async fn initialize_lcd_display<S: SpiBus<u8>>(
    spi: &mut S,
    dc: &mut gpio::Output<'_>,
) -> Result<(), S::Error> {
    // Register values from waveshareteam/ESP32-C3-LCD-1.47, commit 6e16df57.
    send_lcd_command(spi, dc, 0x11, &[]).await?; // Sleep out
    Timer::after_millis(120).await;
    let panel_registers: &[(u8, &[u8])] = &[
        (0x36, &[0x48]),       // Mirror X, BGR color order, portrait
        (0x3A, &[0x55]),       // RGB565
        (0xB0, &[0x00, 0xE8]), // Little-endian pixel data
        (0xB2, &[0x0C, 0x0C, 0x00, 0x33, 0x33]),
        (0xB7, &[0x75]),
        (0xBB, &[0x1A]),
        (0xC0, &[0x80]),
        (0xC2, &[0x01, 0xFF]),
        (0xC3, &[0x13]),
        (0xC4, &[0x20]),
        (0xC6, &[0x0F]),
        (0xD0, &[0xA4, 0xA1]),
        (
            0xE0,
            &[
                0xD0, 0x0D, 0x14, 0x0D, 0x0D, 0x09, 0x38, 0x44, 0x4E, 0x3A, 0x17, 0x18, 0x2F, 0x30,
            ],
        ),
        (
            0xE1,
            &[
                0xD0, 0x09, 0x0F, 0x08, 0x07, 0x14, 0x37, 0x44, 0x4D, 0x38, 0x15, 0x16, 0x2C, 0x2E,
            ],
        ),
        (0x21, &[]), // Color inversion on
        (0x29, &[]), // Display on
    ];
    for &(command, parameters) in panel_registers {
        send_lcd_command(spi, dc, command, parameters).await?;
    }
    Timer::after_millis(20).await;
    Ok(())
}

/// Transfers a full LCD frame asynchronously; the SPI bus is exclusively reserved for the LCD.
pub(crate) async fn flush_lcd_framebuffer<S: SpiBus<u8>>(
    spi: &mut S,
    dc: &mut gpio::Output<'_>,
    framebuffer: &LcdFramebuffer<'_>,
) -> Result<(), S::Error> {
    let start = LCD_COLUMN_OFFSET.to_be_bytes();
    let end = (LCD_COLUMN_OFFSET + LCD_WIDTH - 1).to_be_bytes();
    send_lcd_command(spi, dc, 0x2A, &[start[0], start[1], end[0], end[1]]).await?;
    let last_row = (LCD_HEIGHT - 1).to_be_bytes();
    send_lcd_command(spi, dc, 0x2B, &[0, 0, last_row[0], last_row[1]]).await?;
    send_lcd_command(spi, dc, 0x2C, &[]).await?;
    // Bounded writes let Ariel OS yield between chunks without a blocking graphics adapter.
    for chunk in framebuffer.bytes().chunks(4096) {
        spi.write(chunk).await?;
    }
    spi.flush().await
}
