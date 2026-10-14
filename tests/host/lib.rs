//! Host regression tests for sensor state, exposition/HTTP framing, and LCD drawing.
#![allow(dead_code)]

#[path = "../../src/lcd_framebuffer.rs"]
mod lcd_framebuffer;
#[path = "../../src/prometheus_metrics.rs"]
mod prometheus_metrics;
#[path = "../../src/scd40_display.rs"]
mod scd40_display;
#[path = "../../src/sensor_state.rs"]
mod sensor_state;
