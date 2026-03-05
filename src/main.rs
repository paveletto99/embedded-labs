#![no_main]
#![no_std]

use ariel_os::debug::log::info;
use ariel_os::hal::peripherals;
use ariel_os::time::Timer;
use ariel_os::{
    gpio, hal,
    spi::{
        Mode,
        main::{Kilohertz, highest_freq_in},
    },
};
use embedded_hal_async::spi::SpiBus as SpiBusTrait;

ariel_os::hal::define_peripherals!(Peripherals {
    spi_sck: GPIO6,
    spi_mosi: GPIO7,
    spi_miso: GPIO2,
    lcd_cs: GPIO18,
    lcd_dc: GPIO11,
    lcd_rst: GPIO10,
});

const LCD_WIDTH: u16 = 320;
const LCD_HEIGHT: u16 = 240;

// The esp-hal SpiBus::write() returns before the last FIFO chunk is fully
// clocked out, and flush() is a no-op through the BlockingAsync adapter.
// We use write() followed by a short Timer delay to guarantee all bits are
// on the wire before toggling CS/DC.  At 1 MHz the worst case for 64 bytes
// (FIFO_SIZE) is 512 µs; we wait 1 ms for generous margin.

/// Write bytes over SPI and wait long enough for the transfer to finish.
async fn spi_write_wait<S: SpiBusTrait>(spi: &mut S, data: &[u8]) -> Result<(), S::Error> {
    spi.write(data).await?;
    // At ≤1 MHz, 64 bytes (one FIFO chunk) takes ≤512 µs.
    // Wait 1 ms to be safe regardless of data length.
    Timer::after_millis(1).await;
    Ok(())
}

/// Send a command byte (DC=LOW), optionally followed by parameter bytes (DC=HIGH).
/// CS stays LOW for the entire command+data sequence.
async fn lcd_write_cmd<S: SpiBusTrait>(
    spi: &mut S,
    cs: &mut gpio::Output,
    dc: &mut gpio::Output,
    cmd: u8,
    data: &[u8],
) -> Result<(), S::Error> {
    cs.set_low();
    dc.set_low();
    spi_write_wait(spi, &[cmd]).await?;
    if !data.is_empty() {
        dc.set_high();
        spi_write_wait(spi, data).await?;
    }
    cs.set_high();
    Ok(())
}

async fn lcd_fill_color<S: SpiBusTrait>(
    spi: &mut S,
    cs: &mut gpio::Output,
    dc: &mut gpio::Output,
    color: u16,
) -> Result<(), S::Error> {
    // Keep CS LOW for the entire CASET+RASET+RAMWR+data sequence.
    // Some ILI9341 modules lose internal state when CS cycles between commands.
    cs.set_low();

    // CASET: Column Address Set
    dc.set_low();
    spi_write_wait(spi, &[0x2A]).await?;
    dc.set_high();
    spi_write_wait(
        spi,
        &[
            0,
            0,
            ((LCD_WIDTH - 1) >> 8) as u8,
            ((LCD_WIDTH - 1) & 0xFF) as u8,
        ],
    )
    .await?;

    // RASET: Row Address Set
    dc.set_low();
    spi_write_wait(spi, &[0x2B]).await?;
    dc.set_high();
    spi_write_wait(
        spi,
        &[
            0,
            0,
            ((LCD_HEIGHT - 1) >> 8) as u8,
            ((LCD_HEIGHT - 1) & 0xFF) as u8,
        ],
    )
    .await?;

    // RAMWR: Memory Write + pixel stream
    dc.set_low();
    spi_write_wait(spi, &[0x2C]).await?;
    dc.set_high();

    let hi = (color >> 8) as u8;
    let lo = (color & 0xFF) as u8;
    // 64-byte chunk = exactly 1 SPI FIFO → guaranteed flush per write()
    let mut chunk = [0u8; 64];
    for px in chunk.chunks_exact_mut(2) {
        px[0] = hi;
        px[1] = lo;
    }

    let total_bytes = (LCD_WIDTH as u32) * (LCD_HEIGHT as u32) * 2;
    let mut sent = 0u32;
    while sent < total_bytes {
        let remaining = (total_bytes - sent) as usize;
        let n = core::cmp::min(remaining, chunk.len());
        spi.write(&chunk[..n]).await?;
        sent += n as u32;
    }

    // Wait for last FIFO chunk to finish, then release CS
    Timer::after_millis(1).await;
    cs.set_high();

    Ok(())
}

async fn lcd_init_ili9341<S: SpiBusTrait>(
    spi: &mut S,
    cs: &mut gpio::Output,
    dc: &mut gpio::Output,
    rst: &mut gpio::Output,
) -> Result<(), S::Error> {
    info!("ILI9341: hardware reset");
    rst.set_low();
    Timer::after_millis(50).await;
    rst.set_high();
    Timer::after_millis(200).await;

    info!("ILI9341: software reset");
    lcd_write_cmd(spi, cs, dc, 0x01, &[]).await?;
    Timer::after_millis(200).await;

    info!("ILI9341: power-on sequence");
    lcd_write_cmd(spi, cs, dc, 0xCF, &[0x00, 0xC1, 0x30]).await?;
    lcd_write_cmd(spi, cs, dc, 0xED, &[0x64, 0x03, 0x12, 0x81]).await?;
    lcd_write_cmd(spi, cs, dc, 0xE8, &[0x85, 0x00, 0x78]).await?;
    lcd_write_cmd(spi, cs, dc, 0xCB, &[0x39, 0x2C, 0x00, 0x34, 0x02]).await?;
    lcd_write_cmd(spi, cs, dc, 0xF7, &[0x20]).await?;
    lcd_write_cmd(spi, cs, dc, 0xEA, &[0x00, 0x00]).await?;

    info!("ILI9341: power/VCOM");
    lcd_write_cmd(spi, cs, dc, 0xC0, &[0x23]).await?;
    lcd_write_cmd(spi, cs, dc, 0xC1, &[0x10]).await?;
    lcd_write_cmd(spi, cs, dc, 0xC5, &[0x3E, 0x28]).await?;
    lcd_write_cmd(spi, cs, dc, 0xC7, &[0x86]).await?;

    info!("ILI9341: display config");
    // MADCTL: MX | BGR → portrait 240x320
    lcd_write_cmd(spi, cs, dc, 0x36, &[0x48]).await?;
    // Pixel format: 16-bit/pixel (RGB565)
    lcd_write_cmd(spi, cs, dc, 0x3A, &[0x55]).await?;
    // Frame rate: 79 Hz
    lcd_write_cmd(spi, cs, dc, 0xB1, &[0x00, 0x18]).await?;
    // Display function control
    lcd_write_cmd(spi, cs, dc, 0xB6, &[0x08, 0x82, 0x27]).await?;
    // 3-Gamma off
    lcd_write_cmd(spi, cs, dc, 0xF2, &[0x00]).await?;
    // Gamma curve 1
    lcd_write_cmd(spi, cs, dc, 0x26, &[0x01]).await?;

    // Positive gamma correction
    lcd_write_cmd(
        spi,
        cs,
        dc,
        0xE0,
        &[
            0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09,
            0x00,
        ],
    )
    .await?;
    // Negative gamma correction
    lcd_write_cmd(
        spi,
        cs,
        dc,
        0xE1,
        &[
            0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36,
            0x0F,
        ],
    )
    .await?;

    info!("ILI9341: sleep out");
    lcd_write_cmd(spi, cs, dc, 0x11, &[]).await?;
    Timer::after_millis(200).await;

    info!("ILI9341: display ON");
    lcd_write_cmd(spi, cs, dc, 0x29, &[]).await?;
    Timer::after_millis(100).await;

    Ok(())
}

#[ariel_os::task(autostart, peripherals)]
async fn main(peripherals: Peripherals) {
    info!("=== LCD ILI9341 test ===");
    info!("Pins: SCK=GPIO6 MOSI=GPIO7 MISO=GPIO2 CS=GPIO18 DC=GPIO11 RST=GPIO10");

    let mut spi_config = hal::spi::main::Config::default();
    spi_config.frequency = const { highest_freq_in(Kilohertz::kHz(100)..=Kilohertz::kHz(500)) };
    spi_config.mode = Mode::Mode0;
    info!("SPI Mode0, freq 100-500 kHz");

    let mut spi = hal::spi::main::SPI2::new(
        peripherals.spi_sck,
        peripherals.spi_miso,
        peripherals.spi_mosi,
        spi_config,
    );

    let mut lcd_cs = gpio::Output::new(peripherals.lcd_cs, gpio::Level::High);
    let mut lcd_dc = gpio::Output::new(peripherals.lcd_dc, gpio::Level::Low);
    let mut lcd_rst = gpio::Output::new(peripherals.lcd_rst, gpio::Level::High);
    info!("GPIOs configured");

    info!("Starting ILI9341 init...");
    match lcd_init_ili9341(&mut spi, &mut lcd_cs, &mut lcd_dc, &mut lcd_rst).await {
        Ok(_) => info!("ILI9341 init complete OK"),
        Err(_) => info!("ILI9341 init FAILED"),
    }

    let colors: [(u16, &str); 4] = [
        (0xF800, "RED"),
        (0x07E0, "GREEN"),
        (0x001F, "BLUE"),
        (0x0000, "BLACK"),
    ];
    let mut idx = 0usize;

    loop {
        let (color, name) = colors[idx % colors.len()];
        info!("Filling screen: {} (0x{:04x})", name, color);
        match lcd_fill_color(&mut spi, &mut lcd_cs, &mut lcd_dc, color).await {
            Ok(_) => info!("Fill {} done", name),
            Err(_) => info!("Fill {} FAILED", name),
        }

        idx = idx.wrapping_add(1);
        Timer::after_secs(3).await;
    }
}
