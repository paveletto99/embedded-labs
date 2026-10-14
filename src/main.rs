#![no_main]
#![no_std]

mod lcd_display;
mod lcd_framebuffer;
mod prometheus_metrics;
mod prometheus_server;
mod scd40_display;
mod scd40_sensor;
mod sensor_state;

use ariel_os::{
    gpio, hal,
    log::*,
    reexports::static_cell::StaticCell,
    time::{Instant, Timer},
};
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
};
use lcd_display::{
    enable_lcd_backlight, flush_lcd_framebuffer, initialize_lcd_display, prepare_lcd_expander,
};
use lcd_framebuffer::{LCD_FRAME_BYTES, LcdFramebuffer};
use scd40_display::{draw_scd40_measurement, draw_scd40_status};
use scd40_sensor::scd40_sensor_task;
use sensor_state::{SCD40_STATE, Scd40Status};

hal::define_peripherals!(Peripherals {
    lcd_sck: GPIO7,
    lcd_mosi: GPIO5,
    lcd_miso: GPIO6,
    lcd_dc: GPIO8,
    i2c_sda: GPIO4,
    i2c_scl: GPIO3,
});

// 110,080 bytes in static RAM, outside the async executor stack.
static LCD_FRAMEBUFFER: StaticCell<[u8; LCD_FRAME_BYTES]> = StaticCell::new();

#[ariel_os::task(autostart, peripherals)]
async fn main(peripherals: Peripherals) {
    info!("LCD: starting Waveshare ESP32-C3-LCD-1.47");
    let mut i2c = hal::i2c::controller::I2C0::new(
        peripherals.i2c_sda,
        peripherals.i2c_scl,
        hal::i2c::controller::Config::default(),
    );
    if prepare_lcd_expander(&mut i2c).await.is_err() {
        error!("LCD: I2C expander initialization failed; check board model and address 0x24");
        return;
    }

    let mut spi_config = hal::spi::main::Config::default();
    spi_config.frequency = const {
        ariel_os::spi::main::highest_freq_in(
            ariel_os::spi::main::Kilohertz::MHz(10)..=ariel_os::spi::main::Kilohertz::MHz(20),
        )
    };
    spi_config.mode = ariel_os::spi::Mode::Mode0;
    let mut spi = hal::spi::main::SPI2::new(
        peripherals.lcd_sck,
        peripherals.lcd_miso,
        peripherals.lcd_mosi,
        spi_config,
    );
    let mut dc = gpio::Output::new(peripherals.lcd_dc, gpio::Level::High);
    if initialize_lcd_display(&mut spi, &mut dc).await.is_err() {
        error!("LCD: SPI panel initialization failed");
        return;
    }

    let mut framebuffer = LcdFramebuffer::new(LCD_FRAMEBUFFER.init_with(|| [0; LCD_FRAME_BYTES]));
    framebuffer.clear(Rgb565::BLACK).unwrap();
    let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
    Text::new("Rust + Ariel OS", Point::new(10, 24), text_style)
        .draw(&mut framebuffer)
        .unwrap();
    Text::new("embedded-graphics", Point::new(10, 44), text_style)
        .draw(&mut framebuffer)
        .unwrap();
    Text::new("172 x 320 / RGB565", Point::new(10, 64), text_style)
        .draw(&mut framebuffer)
        .unwrap();
    for (index, color) in [Rgb565::RED, Rgb565::GREEN, Rgb565::BLUE]
        .into_iter()
        .enumerate()
    {
        Rectangle::new(Point::new(10 + index as i32 * 52, 84), Size::new(48, 48))
            .into_styled(PrimitiveStyle::with_fill(color))
            .draw(&mut framebuffer)
            .unwrap();
    }

    draw_scd40_status(&mut framebuffer, "SCD40: starting...");
    if flush_lcd_framebuffer(&mut spi, &mut dc, &framebuffer)
        .await
        .is_err()
    {
        error!("LCD: first frame transfer failed");
        return;
    }
    if enable_lcd_backlight(&mut i2c).await.is_err() {
        error!("LCD: backlight enable failed");
        return;
    }
    info!("LCD: display initialized; showing text and RGB test bars");

    // Hand I2C0 to the sensor task after all LCD expander operations are complete.
    ariel_os::asynch::spawner()
        .spawn(scd40_sensor_task(i2c))
        .unwrap();
    let mut indicator_on = false;
    loop {
        Timer::after_secs(1).await;
        // Copy a consistent snapshot, then release the mutex before drawing or flushing SPI.
        let snapshot = *SCD40_STATE.lock().await;
        if let Some(data) = snapshot.fresh_measurement(Instant::now().as_millis()) {
            draw_scd40_measurement(&mut framebuffer, data);
        } else {
            let status = match snapshot.status {
                Scd40Status::Starting => "SCD40: starting...",
                Scd40Status::Waiting => "SCD40: waiting...",
                Scd40Status::Ready { .. } => "SCD40: stale data",
                Scd40Status::Disconnected => "SCD40: disconnected",
                Scd40Status::ReadFailed => "SCD40: read failed",
            };
            draw_scd40_status(&mut framebuffer, status);
        }
        indicator_on = !indicator_on;
        let color = if indicator_on {
            Rgb565::CYAN
        } else {
            Rgb565::BLACK
        };
        Rectangle::new(Point::new(10, 152), Size::new(20, 20))
            .into_styled(PrimitiveStyle::with_fill(color))
            .draw(&mut framebuffer)
            .unwrap();
        if flush_lcd_framebuffer(&mut spi, &mut dc, &framebuffer)
            .await
            .is_err()
        {
            error!("LCD: frame transfer failed");
            return;
        }
    }
}
