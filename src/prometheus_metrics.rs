//! Prometheus text exposition and bounded HTTP response framing, independent of network I/O.

use core::fmt::Write;
use heapless::String;

use crate::sensor_state::{Scd40State, Scd40Status};

/// Formats Prometheus text 0.0.4; invalid/stale readings are omitted and health is zero.
pub(crate) fn format_prometheus_metrics(
    state: Scd40State,
    now_ms: u64,
) -> Result<String<1536>, core::fmt::Error> {
    let mut body = String::new();
    let data = state.fresh_measurement(now_ms);
    write!(
        body,
        "# HELP scd40_up Whether a valid SCD40 sample is less than 15 seconds old.\n\
         # TYPE scd40_up gauge\nscd40_up {}\n\
         # HELP scd40_measurements_total Total valid SCD40 measurements since boot.\n\
         # TYPE scd40_measurements_total counter\nscd40_measurements_total {}\n\
         # HELP scd40_read_errors_total Total failed SCD40 sensor operations since boot.\n\
         # TYPE scd40_read_errors_total counter\nscd40_read_errors_total {}\n",
        u8::from(data.is_some()),
        state.measurements_total,
        state.read_errors_total
    )?;
    if let Scd40Status::Ready { measured_at_ms, .. } = state.status {
        let age_ms = now_ms.saturating_sub(measured_at_ms);
        write!(
            body,
            "# HELP scd40_sample_age_seconds Age of the last validated SCD40 sample.\n\
             # TYPE scd40_sample_age_seconds gauge\nscd40_sample_age_seconds {}.{:03}\n",
            age_ms / 1000,
            age_ms % 1000
        )?;
    }
    if let Some(data) = data {
        write!(
            body,
            "# HELP scd40_co2_ppm Carbon dioxide concentration in parts per million.\n\
             # TYPE scd40_co2_ppm gauge\nscd40_co2_ppm {}\n\
             # HELP scd40_temperature_celsius Sensor temperature in degrees Celsius.\n\
             # TYPE scd40_temperature_celsius gauge\nscd40_temperature_celsius {:.3}\n\
             # HELP scd40_relative_humidity_ratio Relative humidity as a ratio from zero to one.\n\
             # TYPE scd40_relative_humidity_ratio gauge\nscd40_relative_humidity_ratio {:.5}\n",
            data.co2,
            data.temperature,
            data.humidity / 100.0
        )?;
    }
    Ok(body)
}

/// HTTP result codes supported by the single-request metrics endpoint.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum MetricsHttpStatus {
    Ok,
    BadRequest,
    NotFound,
    MethodNotAllowed,
    RequestTimeout,
    HeadersTooLarge,
    InternalError,
}

impl MetricsHttpStatus {
    fn status_line(self) -> &'static str {
        match self {
            Self::Ok => "200 OK",
            Self::BadRequest => "400 Bad Request",
            Self::NotFound => "404 Not Found",
            Self::MethodNotAllowed => "405 Method Not Allowed",
            Self::RequestTimeout => "408 Request Timeout",
            Self::HeadersTooLarge => "431 Request Header Fields Too Large",
            Self::InternalError => "500 Internal Server Error",
        }
    }
}

/// Accepts complete HTTP/1.0 or HTTP/1.1 GET requests for /metrics and rejects other routes/methods.
pub(crate) fn classify_metrics_request(request: &[u8]) -> MetricsHttpStatus {
    let Some(end) = request.windows(4).position(|bytes| bytes == b"\r\n\r\n") else {
        return MetricsHttpStatus::BadRequest;
    };
    let Some(line_end) = request[..end + 2]
        .windows(2)
        .position(|bytes| bytes == b"\r\n")
    else {
        return MetricsHttpStatus::BadRequest;
    };
    let Ok(line) = core::str::from_utf8(&request[..line_end]) else {
        return MetricsHttpStatus::BadRequest;
    };
    let mut fields = line.split(' ');
    let (Some(method), Some(path), Some(version)) = (fields.next(), fields.next(), fields.next())
    else {
        return MetricsHttpStatus::BadRequest;
    };
    if fields.next().is_some()
        || method.is_empty()
        || path.is_empty()
        || !matches!(version, "HTTP/1.0" | "HTTP/1.1")
    {
        return MetricsHttpStatus::BadRequest;
    }
    if method != "GET" {
        return MetricsHttpStatus::MethodNotAllowed;
    }
    if path != "/metrics" {
        return MetricsHttpStatus::NotFound;
    }
    MetricsHttpStatus::Ok
}

/// Builds HTTP headers with an exact byte count, explicit media type, and no persistent connection.
pub(crate) fn format_metrics_http_headers(
    status: MetricsHttpStatus,
    body_len: usize,
) -> Result<String<256>, core::fmt::Error> {
    let mut headers = String::new();
    let content_type = if status == MetricsHttpStatus::Ok {
        "text/plain; version=0.0.4; charset=utf-8"
    } else {
        "text/plain; charset=utf-8"
    };
    write!(
        headers,
        "HTTP/1.1 {}\r\nContent-Type: {}\r\nContent-Length: {}\r\nConnection: close\r\n",
        status.status_line(),
        content_type,
        body_len
    )?;
    if status == MetricsHttpStatus::MethodNotAllowed {
        headers
            .push_str("Allow: GET\r\n")
            .map_err(|_| core::fmt::Error)?;
    }
    headers.push_str("\r\n").map_err(|_| core::fmt::Error)?;
    Ok(headers)
}

#[cfg(test)]
mod tests {
    use super::*;
    use scd4x::types::SensorData;

    #[test]
    fn fresh_readings_have_types_units_and_final_newline() {
        let mut state = Scd40State::new();
        state.record_measurement(
            SensorData {
                co2: 500,
                temperature: -10.0,
                humidity: 37.0,
            },
            1000,
        );
        let body = format_prometheus_metrics(state, 2250).unwrap();
        assert!(body.contains("# TYPE scd40_co2_ppm gauge\nscd40_co2_ppm 500\n"));
        assert!(body.contains("scd40_temperature_celsius -10.000\n"));
        assert!(body.contains("scd40_relative_humidity_ratio 0.37000\n"));
        assert!(body.contains("scd40_sample_age_seconds 1.250\n"));
        assert!(body.contains("scd40_up 1\n"));
        assert!(body.ends_with('\n'));
        let headers = format_metrics_http_headers(MetricsHttpStatus::Ok, body.len()).unwrap();
        assert!(headers.contains("Content-Type: text/plain; version=0.0.4; charset=utf-8\r\n"));
        assert!(headers.contains(&format!("Content-Length: {}\r\n", body.len())));
        assert!(headers.ends_with("\r\n\r\n"));
    }

    #[test]
    fn no_initial_invalid_or_stale_measurements_are_reported() {
        let mut state = Scd40State::new();
        for now in [0, 20_000] {
            let body = format_prometheus_metrics(state, now).unwrap();
            assert!(body.contains("scd40_up 0\n"));
            assert!(!body.contains("scd40_co2_ppm"));
        }
        state.record_measurement(
            SensorData {
                co2: 500,
                temperature: 25.0,
                humidity: 37.0,
            },
            0,
        );
        let body = format_prometheus_metrics(state, 15_000).unwrap();
        assert!(body.contains("scd40_up 0\n"));
        assert!(!body.contains("scd40_co2_ppm"));
        state.record_error(Scd40Status::ReadFailed);
        let body = format_prometheus_metrics(state, 1000).unwrap();
        assert!(body.contains("scd40_read_errors_total 1\n"));
        assert!(!body.contains("scd40_temperature_celsius"));
    }

    #[test]
    fn endpoint_checks_complete_exact_routes_methods_and_versions() {
        for (request, expected) in [
            (
                "GET /metrics HTTP/1.1\r\nHost: board\r\n\r\n",
                MetricsHttpStatus::Ok,
            ),
            ("GET /metrics HTTP/1.0\r\n\r\n", MetricsHttpStatus::Ok),
            (
                "GET /metrics-extra HTTP/1.1\r\n\r\n",
                MetricsHttpStatus::NotFound,
            ),
            (
                "POST /metrics HTTP/1.1\r\n\r\n",
                MetricsHttpStatus::MethodNotAllowed,
            ),
            (
                "GET /metrics HTTP/2.0\r\n\r\n",
                MetricsHttpStatus::BadRequest,
            ),
            ("GET /metrics HTTP/1.1\r\n", MetricsHttpStatus::BadRequest),
            (
                "GET  /metrics HTTP/1.1\r\n\r\n",
                MetricsHttpStatus::BadRequest,
            ),
        ] {
            assert_eq!(classify_metrics_request(request.as_bytes()), expected);
        }
        let headers = format_metrics_http_headers(MetricsHttpStatus::MethodNotAllowed, 0).unwrap();
        assert!(headers.contains("Allow: GET\r\n"));
    }
}
