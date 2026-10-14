//! Mutex-protected SCD40 snapshots shared by the sensor, LCD, and metrics tasks.

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use scd4x::types::SensorData;

/// Maximum sample age in milliseconds before a reading is considered stale.
pub(crate) const SCD40_MAX_SAMPLE_AGE_MS: u64 = 15_000;

/// SCD40 acquisition status; only Ready carries a validated measurement.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum Scd40Status {
    Starting,
    Waiting,
    Ready {
        data: SensorData,
        measured_at_ms: u64,
    },
    Disconnected,
    ReadFailed,
}

/// A copyable sensor snapshot; counters survive reconnects and reset on device reboot.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Scd40State {
    pub(crate) status: Scd40Status,
    pub(crate) measurements_total: u64,
    pub(crate) read_errors_total: u64,
}

impl Scd40State {
    /// Creates an initial sensor state with no sample and zero counters.
    pub(crate) const fn new() -> Self {
        Self {
            status: Scd40Status::Starting,
            measurements_total: 0,
            read_errors_total: 0,
        }
    }

    /// Publishes finite sensor data and its monotonic acquisition time in milliseconds.
    pub(crate) fn record_measurement(&mut self, data: SensorData, measured_at_ms: u64) {
        if data.co2 == 0
            || !data.temperature.is_finite()
            || !data.humidity.is_finite()
            || !(0.0..=100.0).contains(&data.humidity)
        {
            self.record_error(Scd40Status::ReadFailed);
            return;
        }
        self.status = Scd40Status::Ready {
            data,
            measured_at_ms,
        };
        self.measurements_total = self.measurements_total.saturating_add(1);
    }

    /// Invalidates the current reading and counts a failed sensor operation.
    pub(crate) fn record_error(&mut self, status: Scd40Status) {
        self.status = status;
        self.read_errors_total = self.read_errors_total.saturating_add(1);
    }

    /// Returns a valid sensor reading only while its age is below 15 seconds.
    pub(crate) fn fresh_measurement(&self, now_ms: u64) -> Option<SensorData> {
        match self.status {
            Scd40Status::Ready {
                data,
                measured_at_ms,
            } if now_ms.saturating_sub(measured_at_ms) < SCD40_MAX_SAMPLE_AGE_MS => Some(data),
            _ => None,
        }
    }
}

/// Shared sensor state; copy the value and release this async mutex before performing I/O.
pub(crate) static SCD40_STATE: Mutex<CriticalSectionRawMutex, Scd40State> =
    Mutex::new(Scd40State::new());

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stale_or_failed_samples_are_not_exposed_and_counters_survive_recovery() {
        let data = SensorData {
            co2: 500,
            temperature: 25.0,
            humidity: 37.0,
        };
        let mut state = Scd40State::new();
        assert_eq!(state.fresh_measurement(0), None);
        state.record_measurement(data, 1000);
        assert_eq!(state.fresh_measurement(15_999), Some(data));
        assert_eq!(state.fresh_measurement(16_000), None);
        state.record_error(Scd40Status::Disconnected);
        assert_eq!(state.fresh_measurement(1001), None);
        state.record_measurement(data, 20_000);
        assert_eq!(state.measurements_total, 2);
        assert_eq!(state.read_errors_total, 1);
    }

    #[test]
    fn invalid_sensor_values_are_rejected() {
        for data in [
            SensorData {
                co2: 0,
                temperature: 25.0,
                humidity: 37.0,
            },
            SensorData {
                co2: 500,
                temperature: f32::NAN,
                humidity: 37.0,
            },
            SensorData {
                co2: 500,
                temperature: 25.0,
                humidity: f32::INFINITY,
            },
            SensorData {
                co2: 500,
                temperature: 25.0,
                humidity: 101.0,
            },
        ] {
            let mut state = Scd40State::new();
            state.record_measurement(data, 0);
            assert_eq!(state.fresh_measurement(0), None);
            assert_eq!(state.measurements_total, 0);
            assert_eq!(state.read_errors_total, 1);
        }
    }
}
