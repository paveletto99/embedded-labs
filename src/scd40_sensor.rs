//! SCD40 acquisition task; owns I2C0 after the LCD expander has been configured.

use ariel_os::{
    hal,
    log::*,
    reexports::embassy_time::Delay,
    time::{Duration, Instant, Timer},
};
use scd4x::Scd4xAsync;

use crate::sensor_state::{SCD40_STATE, Scd40Status};

/// Reads the SCD40 and publishes snapshots without holding the state mutex across sensor I/O.
#[ariel_os::task]
pub(crate) async fn scd40_sensor_task(i2c: hal::i2c::controller::I2c) {
    Timer::after_millis(30).await;
    let mut sensor = Scd4xAsync::new(i2c, Delay);
    let mut sensor_running = false;
    let mut next_sensor_start = Instant::now();
    loop {
        if !sensor_running && Instant::now() >= next_sensor_start {
            // The sensor may still be measuring after an ESP32-only reset.
            let result = async {
                sensor.stop_periodic_measurement().await?;
                sensor.start_periodic_measurement().await
            }
            .await;
            match result {
                Ok(()) => {
                    sensor_running = true;
                    info!("SCD40: periodic measurement started at I2C address 0x62");
                    SCD40_STATE.lock().await.status = Scd40Status::Waiting;
                }
                Err(_) => {
                    error!(
                        "SCD40: connection/start failed; check power, GND, SCL=GPIO3, SDA=GPIO4"
                    );
                    SCD40_STATE
                        .lock()
                        .await
                        .record_error(Scd40Status::Disconnected);
                }
            }
            next_sensor_start = Instant::now() + Duration::from_secs(5);
        }
        if sensor_running {
            match sensor.data_ready_status().await {
                Ok(true) => match sensor.measurement().await {
                    Ok(data) => {
                        SCD40_STATE
                            .lock()
                            .await
                            .record_measurement(data, Instant::now().as_millis());
                        if data.co2 != 0 {
                            info!(
                                "SCD40: CO2={} ppm, temperature={} C, humidity={} %RH",
                                data.co2, data.temperature, data.humidity
                            );
                        } else {
                            warn!("SCD40: zero CO2 sample skipped");
                        }
                    }
                    Err(_) => {
                        warn!("SCD40: measurement read or CRC check failed; retrying");
                        SCD40_STATE
                            .lock()
                            .await
                            .record_error(Scd40Status::ReadFailed);
                    }
                },
                Ok(false) => {}
                Err(_) => {
                    warn!("SCD40: data-ready check failed; reconnecting in 5 seconds");
                    SCD40_STATE
                        .lock()
                        .await
                        .record_error(Scd40Status::Disconnected);
                    sensor_running = false;
                    next_sensor_start = Instant::now() + Duration::from_secs(5);
                }
            }
        }
        Timer::after_secs(1).await;
    }
}
