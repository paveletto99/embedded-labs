#![no_main]
#![no_std]

mod lcd_display;
mod lcd_framebuffer;
mod scd40_display;

use ariel_os::{
    gpio, hal,
    log::*,
    reexports::{embassy_time::Delay, static_cell::StaticCell},
    time::{Duration, Instant, Timer},
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
use scd4x::Scd4xAsync;
use scd40_display::{draw_scd40_measurement, draw_scd40_status};

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

    // Reuse I2C0 after LCD control setup: SCD40 is 0x62, the LCD expander is 0x24.
    // Allow power-up to settle before the first sensor command.
    Timer::after_millis(30).await;
    let mut sensor = Scd4xAsync::new(i2c, Delay);
    let mut sensor_running = false;
    let mut next_sensor_start = Instant::now();
    let mut indicator_on = false;
    loop {
        if !sensor_running && Instant::now() >= next_sensor_start {
            // The sensor may still be measuring after an ESP32-only reset.
            match sensor.stop_periodic_measurement().await {
                Ok(()) => match sensor.start_periodic_measurement().await {
                    Ok(()) => {
                        sensor_running = true;
                        info!("SCD40: periodic measurement started at I2C address 0x62");
                        draw_scd40_status(&mut framebuffer, "SCD40: waiting...");
                    }
                    Err(_) => {
                        error!("SCD40: start failed; retrying in 5 seconds");
                        draw_scd40_status(&mut framebuffer, "SCD40: start failed");
                    }
                },
                Err(_) => {
                    error!("SCD40: connection failed; check power, GND, SCL=GPIO3, SDA=GPIO4");
                    draw_scd40_status(&mut framebuffer, "SCD40: disconnected");
                }
            }
            next_sensor_start = Instant::now() + Duration::from_secs(5);
        }
        if sensor_running {
            match sensor.data_ready_status().await {
                Ok(true) => match sensor.measurement().await {
                    Ok(data) if data.co2 != 0 => {
                        info!(
                            "SCD40: CO2={} ppm, temperature={} C, humidity={} %RH",
                            data.co2, data.temperature, data.humidity,
                        );
                        draw_scd40_measurement(&mut framebuffer, data);
                    }
                    Ok(_) => {
                        warn!("SCD40: zero CO2 sample skipped");
                        draw_scd40_status(&mut framebuffer, "SCD40: waiting...");
                    }
                    Err(_) => {
                        warn!("SCD40: measurement read or CRC check failed; retrying");
                        draw_scd40_status(&mut framebuffer, "SCD40: read failed");
                    }
                },
                Ok(false) => {}
                Err(_) => {
                    warn!("SCD40: data-ready check failed; reconnecting in 5 seconds");
                    draw_scd40_status(&mut framebuffer, "SCD40: disconnected");
                    sensor_running = false;
                    next_sensor_start = Instant::now() + Duration::from_secs(5);
                }
            }
        }
        Timer::after_secs(1).await;
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
