#![no_main]
#![no_std]

use ariel_os::debug::log::info;
use ariel_os::hal::i2c::controller::{Config, Frequency, I2C0};
use ariel_os::hal::peripherals;
use ariel_os::reexports::static_cell::StaticCell;
use ariel_os::time::Timer;
use ariel_os::{
    gpio, hal,
    spi::{
        Mode,
        main::{Kilohertz, SpiDevice, highest_freq_in},
    },
};
use embassy_sync::mutex::Mutex;
use embedded_hal_async::spi::SpiDevice as SpiDeviceTrait;

// Import the async I2c trait to use write_read
ariel_os::hal::define_peripherals!(Peripherals {
    i2c_scl: GPIO7,
    i2c_sda: GPIO6,
    // SPI2 is used by the LCD and exposes the following pins on the devkit:
    spi_sck: GPIO2,
    spi_mosi: GPIO3,
    spi_miso: GPIO4,
    spi_cs: GPIO10,
    spi_dc: GPIO11,
    spi_rst: GPIO12,
    spi_bl: GPIO5,
});

// SCK   → GPIO2
// MOSI  → GPIO3
// MISO  → GPIO4
// CS    → GPIO10
// DC    → GPIO11
// RST   → GPIO12
// LED   → GPIO5 (PWM)
// VCC   → 3V3
// GND   → GND

static SPI_BUS: StaticCell<
    Mutex<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, hal::spi::main::Spi>,
> = StaticCell::new();

const LCD_WIDTH: u16 = 240;
const LCD_HEIGHT: u16 = 320;

// BME280 I2C addresses (depends on SDO pin wiring)
const BME280_ADDRS: [u8; 2] = [0x76, 0x77];
const BME280_CHIP_ID_REG: u8 = 0xD0;
const BME280_RESET_REG: u8 = 0xE0;
const BME280_CTRL_HUM_REG: u8 = 0xF2;
const BME280_CTRL_MEAS_REG: u8 = 0xF4;
const BME280_DATA_START_REG: u8 = 0xF7;
const BME280_CALIB1_START_REG: u8 = 0x88;
const BME280_CALIB2_START_REG: u8 = 0xE1;

async fn lcd_cmd<S>(spi: &mut S, dc: &mut gpio::Output, cmd: u8) -> Result<(), S::Error>
where
    S: SpiDeviceTrait,
{
    dc.set_low();
    spi.write(&[cmd]).await
}

async fn lcd_data<S>(spi: &mut S, dc: &mut gpio::Output, data: &[u8]) -> Result<(), S::Error>
where
    S: SpiDeviceTrait,
{
    dc.set_high();
    spi.write(data).await
}

async fn lcd_set_window<S>(
    spi: &mut S,
    dc: &mut gpio::Output,
    x0: u16,
    y0: u16,
    x1: u16,
    y1: u16,
) -> Result<(), S::Error>
where
    S: SpiDeviceTrait,
{
    lcd_cmd(spi, dc, 0x2A).await?;
    lcd_data(
        spi,
        dc,
        &[
            (x0 >> 8) as u8,
            (x0 & 0xff) as u8,
            (x1 >> 8) as u8,
            (x1 & 0xff) as u8,
        ],
    )
    .await?;

    lcd_cmd(spi, dc, 0x2B).await?;
    lcd_data(
        spi,
        dc,
        &[
            (y0 >> 8) as u8,
            (y0 & 0xff) as u8,
            (y1 >> 8) as u8,
            (y1 & 0xff) as u8,
        ],
    )
    .await?;

    lcd_cmd(spi, dc, 0x2C).await
}

async fn lcd_fill_color<S>(
    spi: &mut S,
    dc: &mut gpio::Output,
    width: u16,
    height: u16,
    color: u16,
) -> Result<(), S::Error>
where
    S: SpiDeviceTrait,
{
    lcd_set_window(spi, dc, 0, 0, width - 1, height - 1).await?;

    let hi = (color >> 8) as u8;
    let lo = (color & 0xff) as u8;
    let mut chunk = [0u8; 128];
    for px in chunk.chunks_exact_mut(2) {
        px[0] = hi;
        px[1] = lo;
    }

    dc.set_high();
    let mut remaining = width as u32 * height as u32;
    while remaining > 0 {
        let chunk_pixels = core::cmp::min(remaining, (chunk.len() / 2) as u32);
        spi.write(&chunk[..(chunk_pixels as usize) * 2]).await?;
        remaining -= chunk_pixels;
    }

    Ok(())
}

async fn lcd_init<S>(
    spi: &mut S,
    dc: &mut gpio::Output,
    rst: &mut gpio::Output,
) -> Result<(), S::Error>
where
    S: SpiDeviceTrait,
{
    async fn cmd_data<S>(spi: &mut S, dc: &mut gpio::Output, cmd: u8, data: &[u8]) -> Result<(), S::Error>
    where
        S: SpiDeviceTrait,
    {
        lcd_cmd(spi, dc, cmd).await?;
        if !data.is_empty() {
            lcd_data(spi, dc, data).await?;
        }
        Ok(())
    }

    // Hardware reset.
    rst.set_low();
    Timer::after_millis(20).await;
    rst.set_high();
    Timer::after_millis(120).await;

    // Full ILI9341 init sequence used by common modules.
    lcd_cmd(spi, dc, 0x01).await?; // SWRESET
    Timer::after_millis(5).await;

    cmd_data(spi, dc, 0xCF, &[0x00, 0xC1, 0x30]).await?;
    cmd_data(spi, dc, 0xED, &[0x64, 0x03, 0x12, 0x81]).await?;
    cmd_data(spi, dc, 0xE8, &[0x85, 0x00, 0x78]).await?;
    cmd_data(spi, dc, 0xCB, &[0x39, 0x2C, 0x00, 0x34, 0x02]).await?;
    cmd_data(spi, dc, 0xF7, &[0x20]).await?;
    cmd_data(spi, dc, 0xEA, &[0x00, 0x00]).await?;

    cmd_data(spi, dc, 0xC0, &[0x23]).await?; // PWCTR1
    cmd_data(spi, dc, 0xC1, &[0x10]).await?; // PWCTR2
    cmd_data(spi, dc, 0xC5, &[0x3E, 0x28]).await?; // VMCTR1
    cmd_data(spi, dc, 0xC7, &[0x86]).await?; // VMCTR2

    cmd_data(spi, dc, 0x36, &[0x28]).await?; // MADCTL landscape
    cmd_data(spi, dc, 0x3A, &[0x55]).await?; // COLMOD 16-bit
    cmd_data(spi, dc, 0xB1, &[0x00, 0x18]).await?; // FRMCTR1
    cmd_data(spi, dc, 0xB6, &[0x08, 0x82, 0x27]).await?; // DFUNCTR
    cmd_data(spi, dc, 0xF2, &[0x00]).await?; // 3Gamma off
    cmd_data(spi, dc, 0x26, &[0x01]).await?; // Gamma curve
    cmd_data(
        spi,
        dc,
        0xE0,
        &[
            0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E,
            0x09, 0x00,
        ],
    )
    .await?;
    cmd_data(
        spi,
        dc,
        0xE1,
        &[
            0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31,
            0x36, 0x0F,
        ],
    )
    .await?;

    lcd_cmd(spi, dc, 0x11).await?; // SLPOUT
    Timer::after_millis(120).await;
    lcd_cmd(spi, dc, 0x29).await?; // DISPON
    Timer::after_millis(20).await;

    Ok(())
}

#[derive(Clone, Copy)]
struct Calibration {
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,
    dig_h1: u8,
    dig_h2: i16,
    dig_h3: u8,
    dig_h4: i16,
    dig_h5: i16,
    dig_h6: i8,
}

fn le_u16(bytes: &[u8], idx: usize) -> u16 {
    u16::from_le_bytes([bytes[idx], bytes[idx + 1]])
}

fn le_i16(bytes: &[u8], idx: usize) -> i16 {
    i16::from_le_bytes([bytes[idx], bytes[idx + 1]])
}

fn sign_extend_12(v: u16) -> i16 {
    let v = v & 0x0fff;
    if (v & 0x0800) != 0 {
        (v | 0xf000) as i16
    } else {
        v as i16
    }
}

async fn bme280_write_reg<I>(i2c: &mut I, addr: u8, reg: u8, value: u8) -> Result<(), I::Error>
where
    I: embedded_hal_async::i2c::I2c,
{
    i2c.write(addr, &[reg, value]).await
}

async fn bme280_read_calibration<I>(i2c: &mut I, addr: u8) -> Result<Calibration, I::Error>
where
    I: embedded_hal_async::i2c::I2c,
{
    let mut c1 = [0u8; 26];
    let mut c2 = [0u8; 7];
    i2c.write_read(addr, &[BME280_CALIB1_START_REG], &mut c1)
        .await?;
    i2c.write_read(addr, &[BME280_CALIB2_START_REG], &mut c2)
        .await?;

    let dig_h4_raw = ((c2[3] as u16) << 4) | ((c2[4] as u16) & 0x0f);
    let dig_h5_raw = ((c2[5] as u16) << 4) | (((c2[4] as u16) >> 4) & 0x0f);

    Ok(Calibration {
        dig_t1: le_u16(&c1, 0),
        dig_t2: le_i16(&c1, 2),
        dig_t3: le_i16(&c1, 4),
        dig_p1: le_u16(&c1, 6),
        dig_p2: le_i16(&c1, 8),
        dig_p3: le_i16(&c1, 10),
        dig_p4: le_i16(&c1, 12),
        dig_p5: le_i16(&c1, 14),
        dig_p6: le_i16(&c1, 16),
        dig_p7: le_i16(&c1, 18),
        dig_p8: le_i16(&c1, 20),
        dig_p9: le_i16(&c1, 22),
        dig_h1: c1[25],
        dig_h2: le_i16(&c2, 0),
        dig_h3: c2[2],
        dig_h4: sign_extend_12(dig_h4_raw),
        dig_h5: sign_extend_12(dig_h5_raw),
        dig_h6: c2[6] as i8,
    })
}

async fn bme280_init<I>(i2c: &mut I, addr: u8) -> Result<Calibration, I::Error>
where
    I: embedded_hal_async::i2c::I2c,
{
    // Reset and configure oversampling for humidity, pressure and temperature.
    bme280_write_reg(i2c, addr, BME280_RESET_REG, 0xB6).await?;
    Timer::after_millis(10).await;
    bme280_write_reg(i2c, addr, BME280_CTRL_HUM_REG, 0x01).await?; // x1 humidity
    bme280_write_reg(i2c, addr, BME280_CTRL_MEAS_REG, 0x27).await?; // x1 temp, x1 pressure, normal mode
    bme280_read_calibration(i2c, addr).await
}

async fn bme280_read_raw<I>(i2c: &mut I, addr: u8) -> Result<(i32, i32, i32), I::Error>
where
    I: embedded_hal_async::i2c::I2c,
{
    let mut data = [0u8; 8];
    i2c.write_read(addr, &[BME280_DATA_START_REG], &mut data)
        .await?;

    let adc_p = ((data[0] as i32) << 12) | ((data[1] as i32) << 4) | ((data[2] as i32) >> 4);
    let adc_t = ((data[3] as i32) << 12) | ((data[4] as i32) << 4) | ((data[5] as i32) >> 4);
    let adc_h = ((data[6] as i32) << 8) | (data[7] as i32);
    Ok((adc_t, adc_p, adc_h))
}

fn bme280_compensate(
    adc_t: i32,
    adc_p: i32,
    adc_h: i32,
    c: &Calibration,
) -> Option<(i32, u32, u32)> {
    let var1 = ((((adc_t >> 3) - ((c.dig_t1 as i32) << 1)) * (c.dig_t2 as i32)) >> 11) as i32;
    let var2 = (((((adc_t >> 4) - (c.dig_t1 as i32)) * ((adc_t >> 4) - (c.dig_t1 as i32))) >> 12)
        * (c.dig_t3 as i32))
        >> 14;
    let t_fine = var1 + var2;
    let temp_centi_c = (t_fine * 5 + 128) >> 8;

    let mut var1_p = (t_fine as i64) - 128000;
    let mut var2_p = var1_p * var1_p * (c.dig_p6 as i64);
    var2_p += (var1_p * (c.dig_p5 as i64)) << 17;
    var2_p += (c.dig_p4 as i64) << 35;
    var1_p = ((var1_p * var1_p * (c.dig_p3 as i64)) >> 8) + ((var1_p * (c.dig_p2 as i64)) << 12);
    var1_p = ((((1i64) << 47) + var1_p) * (c.dig_p1 as i64)) >> 33;
    if var1_p == 0 {
        return None;
    }

    let mut p = 1048576 - (adc_p as i64);
    p = (((p << 31) - var2_p) * 3125) / var1_p;
    var1_p = ((c.dig_p9 as i64) * (p >> 13) * (p >> 13)) >> 25;
    var2_p = ((c.dig_p8 as i64) * p) >> 19;
    p = ((p + var1_p + var2_p) >> 8) + ((c.dig_p7 as i64) << 4);
    let pressure_pa = ((p + 128) >> 8) as u32;

    let mut h = t_fine - 76800;
    h = ((((adc_h << 14) - ((c.dig_h4 as i32) << 20) - ((c.dig_h5 as i32) * h)) + 16384) >> 15)
        * (((((((h * (c.dig_h6 as i32)) >> 10) * (((h * (c.dig_h3 as i32)) >> 11) + 32768))
            >> 10)
            + 2097152)
            * (c.dig_h2 as i32)
            + 8192)
            >> 14);
    h = h - (((((h >> 15) * (h >> 15)) >> 7) * (c.dig_h1 as i32)) >> 4);
    h = h.clamp(0, 419430400);
    let humidity_milli_pct = (((h >> 12) * 1000 + 512) / 1024) as u32;

    Some((temp_centi_c, pressure_pa, humidity_milli_pct))
}

async fn detect_bme280_addr<I>(i2c: &mut I) -> Option<u8>
where
    I: embedded_hal_async::i2c::I2c,
{
    for addr in BME280_ADDRS {
        let mut chip_id = [0u8; 1];
        if i2c
            .write_read(addr, &[BME280_CHIP_ID_REG], &mut chip_id)
            .await
            .is_ok()
            && chip_id[0] == 0x60
        {
            info!("BME280 detected at 0x{:02x}", addr);
            return Some(addr);
        }
    }
    None
}

#[ariel_os::task(autostart, peripherals)]
async fn main(peripherals: Peripherals) {
    info!("Starting I2C demo for ESP32-C6");

    // Configure I2C
    let mut i2c_config = Config::default();
    i2c_config.frequency = Frequency::_400k;

    // Initialize I2C bus
    let mut i2c = I2C0::new(peripherals.i2c_sda, peripherals.i2c_scl, i2c_config);

    info!("I2C initialized at 400 kHz");

    let mut sensor: Option<(u8, Calibration)> = None;

    let mut spi_config = hal::spi::main::Config::default();
    spi_config.frequency = const { highest_freq_in(Kilohertz::kHz(1000)..=Kilohertz::kHz(2000)) };
    spi_config.mode = Mode::Mode0;

    let spi_bus = hal::spi::main::SPI2::new(
        peripherals.spi_sck,
        peripherals.spi_miso,
        peripherals.spi_mosi,
        spi_config,
    );
    let spi_bus = SPI_BUS.init(Mutex::new(spi_bus));
    let cs_output = gpio::Output::new(peripherals.spi_cs, gpio::Level::High);
    let mut spi_device = SpiDevice::new(spi_bus, cs_output);

    // Dummy transaction: proves the SPI device is declared and operational in Ariel.
    let mut spi_probe = [0x9f, 0x00, 0x00, 0x00];
    let _ = SpiDeviceTrait::transfer_in_place(&mut spi_device, &mut spi_probe).await;
    info!("SPI device declared on SPI2 (SCK=GPIO2, MOSI=GPIO3, MISO=GPIO4, CS=GPIO10)");

    // let _lcd_bl = gpio::Output::new(peripherals.spi_bl, gpio::Level::High);
    // let mut lcd_dc = gpio::Output::new(peripherals.spi_dc, gpio::Level::Low);
    // let mut lcd_rst = gpio::Output::new(peripherals.spi_rst, gpio::Level::High);
    // match lcd_init(&mut spi_device, &mut lcd_dc, &mut lcd_rst).await {
    //     Ok(_) => {
    //         info!("ILI9341 initialized");
    //         let _ = lcd_fill_color(&mut spi_device, &mut lcd_dc, LCD_WIDTH, LCD_HEIGHT, 0xF800).await;
    //         Timer::after_millis(300).await;
    //         let _ = lcd_fill_color(&mut spi_device, &mut lcd_dc, LCD_WIDTH, LCD_HEIGHT, 0x07E0).await;
    //         Timer::after_millis(300).await;
    //         let _ = lcd_fill_color(&mut spi_device, &mut lcd_dc, LCD_WIDTH, LCD_HEIGHT, 0x001F).await;
    //         Timer::after_millis(300).await;
    //         let _ = lcd_fill_color(&mut spi_device, &mut lcd_dc, LCD_WIDTH, LCD_HEIGHT, 0x0000).await;
    //         info!("ILI9341 test fill done");
    //     }
    //     Err(_) => info!("ILI9341 init failed"),
    // }

    // Poll BME280 data. Re-detect/re-init if it disappears.
    loop {
        if sensor.is_none() {
            if let Some(addr) = detect_bme280_addr(&mut i2c).await {
                match bme280_init(&mut i2c, addr).await {
                    Ok(calib) => {
                        info!("BME280 initialized at 0x{:02x}", addr);
                        sensor = Some((addr, calib));
                    }
                    Err(_) => info!("Failed to initialize BME280 at 0x{:02x}", addr),
                }
            } else {
                info!("No BME280 found at 0x76/0x77");
                info!("Check wiring: SDA=GPIO6, SCL=GPIO7, plus pull-ups and sensor power");
            }
        }

        if let Some((addr, calib)) = sensor {
            match bme280_read_raw(&mut i2c, addr).await {
                Ok((adc_t, adc_p, adc_h)) => {
                    if let Some((temp_centi_c, pressure_pa, humidity_milli_pct)) =
                        bme280_compensate(adc_t, adc_p, adc_h, &calib)
                    {
                        let temp_abs = temp_centi_c.abs();
                        let temp_sign = if temp_centi_c < 0 { "-" } else { "" };
                        let temp_whole = temp_abs / 100;
                        let temp_frac = temp_abs % 100;

                        let pressure_hpa_whole = pressure_pa / 100;
                        let pressure_hpa_frac = pressure_pa % 100;

                        let humidity_whole = humidity_milli_pct / 1000;
                        let humidity_frac = (humidity_milli_pct % 1000) / 10;

                        info!(
                            "BME280: temp={}{}.{:02} C pressure={}.{:02} hPa humidity={}.{:02} %RH",
                            temp_sign,
                            temp_whole,
                            temp_frac,
                            pressure_hpa_whole,
                            pressure_hpa_frac,
                            humidity_whole,
                            humidity_frac
                        );
                    } else {
                        info!("BME280 compensation failed at 0x{:02x}", addr);
                    }
                }
                Err(_) => {
                    info!("Lost BME280 at 0x{:02x}, re-scanning", addr);
                    sensor = None;
                }
            }
        } else {
            info!("Waiting for BME280...");
        }

        Timer::after_secs(2).await;
    }
}
