//! Wi-Fi HTTP/TCP exporter task using the network stack provided by Ariel OS.

use ariel_os::{
    log::*,
    net,
    reexports::{
        embassy_net::tcp::TcpSocket,
        embassy_time::{Duration, Instant, with_timeout},
    },
};
use embedded_io_async::Write;

use crate::{
    prometheus_metrics::{
        MetricsHttpStatus, classify_metrics_request, format_metrics_http_headers,
        format_prometheus_metrics,
    },
    sensor_state::SCD40_STATE,
};

const METRICS_PORT: u16 = 9100;

async fn read_metrics_request(
    socket: &mut TcpSocket<'_>,
    request: &mut [u8; 1024],
) -> Result<MetricsHttpStatus, ()> {
    let mut used = 0;
    loop {
        if used == request.len() {
            return Ok(MetricsHttpStatus::HeadersTooLarge);
        }
        let count = socket.read(&mut request[used..]).await.map_err(|_| ())?;
        if count == 0 {
            return Err(());
        }
        used += count;
        if request[..used].windows(4).any(|bytes| bytes == b"\r\n\r\n") {
            return Ok(classify_metrics_request(&request[..used]));
        }
    }
}

async fn serve_metrics_connection(
    socket: &mut TcpSocket<'_>,
    request: &mut [u8; 1024],
) -> Result<(), ()> {
    let mut status = match with_timeout(
        Duration::from_secs(5),
        read_metrics_request(socket, request),
    )
    .await
    {
        Ok(result) => result?,
        Err(_) => MetricsHttpStatus::RequestTimeout,
    };
    // The guard is dropped at this semicolon, before formatting and all TCP writes.
    let snapshot = *SCD40_STATE.lock().await;
    let body = if status == MetricsHttpStatus::Ok {
        match format_prometheus_metrics(snapshot, Instant::now().as_millis()) {
            Ok(body) => body,
            Err(_) => {
                status = MetricsHttpStatus::InternalError;
                heapless::String::new()
            }
        }
    } else {
        heapless::String::new()
    };
    let headers = format_metrics_http_headers(status, body.len()).map_err(|_| ())?;
    socket.write_all(headers.as_bytes()).await.map_err(|_| ())?;
    socket.write_all(body.as_bytes()).await.map_err(|_| ())?;
    socket.close();
    socket.flush().await.map_err(|_| ())
}

/// Serves one bounded GET /metrics request at a time; Wi-Fi setup/reconnection belongs to Ariel OS.
#[ariel_os::task(autostart)]
pub(crate) async fn prometheus_metrics_task() {
    let Some(stack) = net::network_stack().await else {
        error!("Metrics: no network stack on the current executor");
        return;
    };
    let mut rx_buffer = [0; 1024];
    let mut tx_buffer = [0; 2048];
    let mut request = [0; 1024];
    let mut advertised_address = None;
    loop {
        stack.wait_config_up().await;
        if let Some(config) = stack.config_v4()
            && advertised_address != Some(config.address)
        {
            info!(
                "Metrics: listening at http://{}:9100/metrics",
                config.address.address()
            );
            advertised_address = Some(config.address);
        }
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));
        if socket.accept(METRICS_PORT).await.is_err() {
            warn!("Metrics: TCP accept failed; retrying");
            ariel_os::time::Timer::after_millis(100).await;
            continue;
        }
        // Bound the whole connection as well as header reading, including slow clients/failed Wi-Fi.
        if !matches!(
            with_timeout(
                Duration::from_secs(10),
                serve_metrics_connection(&mut socket, &mut request)
            )
            .await,
            Ok(Ok(()))
        ) {
            warn!("Metrics: HTTP connection failed or timed out");
            socket.abort();
            let _ = with_timeout(Duration::from_secs(1), socket.flush()).await;
        }
    }
}
