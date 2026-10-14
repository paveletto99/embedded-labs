//! RGB565 drawing buffer in the little-endian format used by the vendor panel setup.

use core::convert::Infallible;
use embedded_graphics::{pixelcolor::Rgb565, prelude::*};

pub(crate) const LCD_WIDTH: u16 = 172;
pub(crate) const LCD_HEIGHT: u16 = 320;
pub(crate) const LCD_FRAME_BYTES: usize = LCD_WIDTH as usize * LCD_HEIGHT as usize * 2;

/// An embedded graphics draw target backed by caller-owned memory, without heap allocation.
pub(crate) struct LcdFramebuffer<'a> {
    bytes: &'a mut [u8; LCD_FRAME_BYTES],
}

impl<'a> LcdFramebuffer<'a> {
    /// Uses an existing LCD buffer; drawing becomes visible only after an SPI flush.
    pub(crate) fn new(bytes: &'a mut [u8; LCD_FRAME_BYTES]) -> Self {
        Self { bytes }
    }

    /// Returns little-endian RGB565 pixels for the panel's RAMCTRL = 0x00, 0xE8 setting.
    pub(crate) fn bytes(&self) -> &[u8; LCD_FRAME_BYTES] {
        self.bytes
    }
}

impl OriginDimensions for LcdFramebuffer<'_> {
    fn size(&self) -> Size {
        Size::new(LCD_WIDTH.into(), LCD_HEIGHT.into())
    }
}

impl DrawTarget for LcdFramebuffer<'_> {
    type Color = Rgb565;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels {
            if point.x < 0
                || point.y < 0
                || point.x >= i32::from(LCD_WIDTH)
                || point.y >= i32::from(LCD_HEIGHT)
            {
                continue;
            }
            let offset = (point.y as usize * LCD_WIDTH as usize + point.x as usize) * 2;
            self.bytes[offset..offset + 2].copy_from_slice(&color.into_storage().to_le_bytes());
        }
        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        let pixel = color.into_storage().to_le_bytes();
        for bytes in self.bytes.as_chunks_mut::<2>().0 {
            *bytes = pixel;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_graphics::primitives::Rectangle;

    #[test]
    fn rgb565_pixels_use_panel_byte_order_and_row_stride() {
        let mut bytes = [0; LCD_FRAME_BYTES];
        let mut framebuffer = LcdFramebuffer::new(&mut bytes);
        framebuffer
            .draw_iter([
                Pixel(Point::new(0, 0), Rgb565::RED),
                Pixel(Point::new(1, 0), Rgb565::GREEN),
                Pixel(Point::new(0, 1), Rgb565::BLUE),
                Pixel(Point::new(171, 319), Rgb565::WHITE),
            ])
            .unwrap();
        assert_eq!(&framebuffer.bytes()[..4], &[0x00, 0xF8, 0xE0, 0x07]);
        let second_row = LCD_WIDTH as usize * 2;
        assert_eq!(
            &framebuffer.bytes()[second_row..second_row + 2],
            &[0x1F, 0x00]
        );
        assert_eq!(&framebuffer.bytes()[LCD_FRAME_BYTES - 2..], &[0xFF, 0xFF]);
    }

    #[test]
    fn pixels_outside_lcd_bounds_are_ignored() {
        let mut bytes = [0; LCD_FRAME_BYTES];
        let mut framebuffer = LcdFramebuffer::new(&mut bytes);
        framebuffer
            .draw_iter([
                Pixel(Point::new(-1, 0), Rgb565::WHITE),
                Pixel(Point::new(0, -1), Rgb565::WHITE),
                Pixel(Point::new(172, 0), Rgb565::WHITE),
                Pixel(Point::new(0, 320), Rgb565::WHITE),
            ])
            .unwrap();
        assert!(framebuffer.bytes().iter().all(|byte| *byte == 0));
    }

    #[test]
    fn clipped_contiguous_draw_preserves_source_pixel_positions() {
        let mut bytes = [0; LCD_FRAME_BYTES];
        let mut framebuffer = LcdFramebuffer::new(&mut bytes);
        framebuffer.clear(Rgb565::BLUE).unwrap();
        framebuffer
            .fill_contiguous(
                &Rectangle::new(Point::new(-1, 0), Size::new(2, 2)),
                [Rgb565::RED, Rgb565::GREEN, Rgb565::WHITE, Rgb565::BLACK],
            )
            .unwrap();
        assert_eq!(&framebuffer.bytes()[..4], &[0xE0, 0x07, 0x1F, 0x00]);
        let second_row = LCD_WIDTH as usize * 2;
        assert_eq!(
            &framebuffer.bytes()[second_row..second_row + 4],
            &[0, 0, 0x1F, 0x00]
        );
    }
}
