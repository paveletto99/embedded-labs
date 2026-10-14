//! SCD40 readings and connection status in the lower part of the LCD.

use core::fmt::Write;
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
};
use heapless::String;
use scd4x::types::SensorData;

use crate::lcd_framebuffer::{LCD_HEIGHT, LCD_WIDTH, LcdFramebuffer};

/// Replaces the LCD sensor area so shorter readings or errors never leave stale text.
pub(crate) fn draw_scd40_status(framebuffer: &mut LcdFramebuffer<'_>, status: &str) {
    Rectangle::new(
        Point::new(0, 184),
        Size::new(LCD_WIDTH.into(), u32::from(LCD_HEIGHT) - 184),
    )
    .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
    .draw(framebuffer)
    .unwrap();
    Text::new(
        status,
        Point::new(10, 200),
        MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE),
    )
    .draw(framebuffer)
    .unwrap();
}

/// Draws SCD40 CO2 in ppm, temperature in Celsius, and relative humidity in percent.
pub(crate) fn draw_scd40_measurement(framebuffer: &mut LcdFramebuffer<'_>, data: SensorData) {
    let mut text = String::<96>::new();
    write!(
        text,
        "SCD40\nCO2: {} ppm\nTemp: {:.1} C\nRH: {:.1} %",
        data.co2, data.temperature, data.humidity,
    )
    .unwrap();
    draw_scd40_status(framebuffer, &text);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lcd_framebuffer::LCD_FRAME_BYTES;

    #[test]
    fn shorter_measurement_replaces_previous_text() {
        let mut previous_bytes = [0; LCD_FRAME_BYTES];
        let mut fresh_bytes = [0; LCD_FRAME_BYTES];
        let mut previous = LcdFramebuffer::new(&mut previous_bytes);
        let mut fresh = LcdFramebuffer::new(&mut fresh_bytes);
        draw_scd40_measurement(
            &mut previous,
            SensorData {
                co2: 40000,
                temperature: -10.0,
                humidity: 100.0,
            },
        );
        let next = SensorData {
            co2: 500,
            temperature: 20.0,
            humidity: 50.0,
        };
        draw_scd40_measurement(&mut previous, next);
        draw_scd40_measurement(&mut fresh, next);
        assert_eq!(previous.bytes(), fresh.bytes());
    }

    #[test]
    fn sensor_status_preserves_lcd_demo_above_sensor_area() {
        let mut bytes = [0; LCD_FRAME_BYTES];
        let mut framebuffer = LcdFramebuffer::new(&mut bytes);
        framebuffer.clear(Rgb565::BLUE).unwrap();
        draw_scd40_status(&mut framebuffer, "SCD40: waiting...");
        let sensor_start = LCD_WIDTH as usize * 184 * 2;
        assert!(
            framebuffer.bytes()[..sensor_start]
                .as_chunks::<2>()
                .0
                .iter()
                .all(|pixel| *pixel == [0x1F, 0x00])
        );
        assert!(framebuffer.bytes()[sensor_start..].contains(&0));
    }
}
