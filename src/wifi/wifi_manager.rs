use crate::sensor::measurement::Measurement;
use crate::storage::session_config::{SessionConfig, SessionType};
use crate::{LoopEvent, SendingError};
use embedded_svc::http::Method;
use embedded_svc::io::Write;
use embedded_svc::{
    http::client::Client as HttpClient,
    wifi::{AuthMethod, ClientConfiguration, Configuration},
};
use esp32_nimble::utilities::mutex::Mutex;
use esp_idf_svc::http::client::EspHttpConnection;
use esp_idf_svc::wifi::{BlockingWifi, EspWifi};
use log::{error, info};
use std::sync::mpsc::Sender;

const MAGIC: &[u8; 2] = &[0xAB, 0xBA];

pub struct WifiManager {
    wifi: Mutex<BlockingWifi<EspWifi<'static>>>,
}

impl WifiManager {
    pub fn new(wifi: BlockingWifi<EspWifi<'static>>) -> Self {
        Self {
            wifi: Mutex::new(wifi),
        }
    }

    pub fn connect(&self, wifi_ssid: &str, wifi_password: &str) -> anyhow::Result<()> {
        if let Some(mut wifi) = self.wifi.try_lock() {
            let wifi_config = Configuration::Client(ClientConfiguration {
                ssid: wifi_ssid.try_into()?,
                bssid: None,
                auth_method: AuthMethod::WPA2Personal,
                password: wifi_password.try_into()?,
                channel: None,
                ..Default::default()
            });
            info!(
                "calling start, connect, wait_netif_up on wifi manager: {:?} ",
                wifi_config
            );
            wifi.set_configuration(&wifi_config)?;
            wifi.start()?;
            wifi.connect()?;
            wifi.wait_netif_up()?;
        }
        Ok(())
    }
    pub fn send_measurements(
        &self,
        measurements: &[Measurement],
        domain: &str,
        config: SessionConfig,
        event_tx: Sender<LoopEvent>,
    ) -> Result<(), SendingError> {
        if measurements.is_empty() {
            return Ok(());
        }
        if !self.is_connected() {
            return Err(SendingError::ConnectionError);
        }
        let SessionType::FIXED {
            pm1_index,
            pm2_5_index,
            token,
            wifi_ssid: _,
            wifi_password: _,
        } = config.session_type.clone()
        else {
            panic!("Config error, expected fixed session")
        };
        let payload = self.encode_measurements(measurements, pm1_index, pm2_5_index)?;
        use esp_idf_svc::http::client::Configuration as HttpConfiguration;

        let http_config = &HttpConfiguration {
            crt_bundle_attach: Some(esp_idf_svc::sys::esp_crt_bundle_attach),
            ..Default::default()
        };
        let mut client = HttpClient::wrap(
            EspHttpConnection::new(http_config).map_err(|_| SendingError::ConfigError)?,
        );
        let content_len_header = format!("{}", payload.len());

        let headers = [
            ("Content-Type", "application/octet-stream"),
            ("Content-Length", &content_len_header),
            ("Authorization", &format!("Bearer {:032x}", token)),
        ];

        let url = format!(
            "https://{}/api/v3/fixed_sessions/{}/measurements",
            domain, config.session_uuid
        );
        let mut request = client
            .request(Method::Post, &url, &headers)
            .map_err(|_| SendingError::Retry)?;
        request
            .write_all(&payload)
            .map_err(|_| SendingError::Overflow)?;
        request.flush().map_err(|_| SendingError::Retry)?;
        let mut response = request.submit().map_err(|_| SendingError::Retry)?;
        let status = response.status();

        if let Some(epoch) = response.header("X-Server-Time") {
            if let Ok(epoch) = epoch.parse::<i64>() {
                event_tx.send(LoopEvent::TimeUpdate(epoch)).unwrap();
            }
        }

        info!(
            "POST measurements → {status}, sent {} records ({} bytes)",
            measurements.len(),
            payload.len()
        );

        if !(200..300).contains(&(status as i32)) {
            return Err(SendingError::ConfigError);
        }

        Ok(())
    }

    pub fn disconnect(&self) -> anyhow::Result<(), WifiError> {
        if let Some(mut wifi) = self.wifi.try_lock() {
            wifi.disconnect().map_err(|_| WifiError::Other)?;
        } else {
            return Err(WifiError::LockError);
        }
        Ok(())
    }

    pub fn is_connected(&self) -> bool {
        if let Some(mut wifi) = self.wifi.try_lock() {
            wifi.is_connected().unwrap_or(false)
        } else {
            false
        }
    }

    fn encode_measurements(
        &self,
        measurements: &[Measurement],
        pm1_index: u8,
        pm2_5_index: u8,
    ) -> Result<Vec<u8>, SendingError> {
        let count = measurements.len();

        if count > u16::MAX as usize {
            return Err(SendingError::Overflow);
        }
        let count = (count * 2) as u16;

        // 0xAB + 0xBA + u16 + N * 2 * (u32 + u8 + float) + u8
        let capacity = 2 + 2 + count * 18 + 1;
        let mut buffer = Vec::with_capacity(capacity as usize);
        buffer.extend_from_slice(MAGIC);
        buffer.extend_from_slice(&count.to_be_bytes());
        for m in measurements {
            buffer.extend_from_slice(&m.timestamp.to_be_bytes());
            buffer.push(pm1_index);
            buffer.extend_from_slice(&f32::from(m.pm1_0_avg).to_be_bytes());
            buffer.extend_from_slice(&m.timestamp.to_be_bytes());
            buffer.push(pm2_5_index);
            buffer.extend_from_slice(&f32::from(m.pm2_5_avg).to_be_bytes());
        }
        let checksum = buffer.iter().fold(0u8, |acc, &b| acc ^ b);
        buffer.push(checksum);
        Ok(buffer)
    }
}

#[derive(Debug)]
pub enum WifiError {
    NotConnected,
    Config,
    NotStarted,
    LockError,
    Other,
}
