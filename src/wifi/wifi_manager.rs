use esp32_nimble::utilities::mutex::Mutex;
use esp_idf_svc::wifi::{BlockingWifi, EspWifi};
use embedded_svc::{
    http::client::Client as HttpClient,
    wifi::{AuthMethod, Configuration, ClientConfiguration}
};
use embedded_svc::http::Method;
use embedded_svc::io::Write;
use esp_idf_svc::http::client::EspHttpConnection;
use log::info;
use crate::SendingError;
use crate::sensor::sensor_thread::Measurement;
use crate::storage::session_config::{SessionConfig, SessionType};


const MAGIC: &[u8; 2] = &[0xAB, 0xBA];

pub struct WifiManager {
    wifi: Mutex<BlockingWifi<EspWifi<'static>>>,
}

impl WifiManager {
    pub fn new(
        wifi: BlockingWifi<EspWifi<'static>>,
    ) -> Self {
        Self {
            wifi: Mutex::new(wifi),
        }
    }

    pub fn connect(&self, wifi_ssid: &str, wifi_password: &str) -> anyhow::Result<()> {
        if let Some(mut wifi) = self.wifi.try_lock() {
            let auth = if wifi_password.is_empty() { AuthMethod::None } else { AuthMethod::WPA2Personal };
            let wifi_config = Configuration::Client(ClientConfiguration {
                ssid: wifi_ssid.try_into()?,
                bssid: None,
                auth_method: auth,
                password: wifi_password.try_into()?,
                channel: None,
                ..Default::default()
            });
            info!("calling start, connect, wait_netif_up on wifi manager: {:?} ", wifi_config);
            wifi.set_configuration(&wifi_config)?;
            wifi.start()?;
            wifi.connect()?;
            wifi.wait_netif_up()?;
        }
        Ok(())
    }
    pub fn send_measurements(&self, measurements: &[Measurement], domain: &str, config: SessionConfig) -> Result<(), SendingError> {
        if measurements.is_empty() { return Ok(()) }
        if !self.is_connected().map_err(|_| SendingError::ConnectionError)? {return Err(SendingError::ConnectionError)}
        let SessionType::FIXED { pm1_index, pm2_5_index, token, wifi_ssid: _, wifi_password: _ } = config.session_type.clone() else { panic!("Config error, expected fixed session") };
        let payload = self.encode_measurements(measurements, pm1_index, pm2_5_index)?;
        use esp_idf_svc::http::client::{Configuration as HttpConfiguration};

        let http_config = &HttpConfiguration {
             crt_bundle_attach: Some(esp_idf_svc::sys::esp_crt_bundle_attach),
             ..Default::default()

         };
        let mut client = HttpClient::wrap(EspHttpConnection::new(http_config).map_err(|_| SendingError::ConfigError)?);
        let content_len_header = format!("{}", payload.len());

        let headers = [
            ("Content-Type", "application/octet-stream"),
            ("Content-Length", &content_len_header),
            ("Authorization", &format!("Bearer {:032x}", token)),
        ];

        let url = format!("https://{}/api/v3/fixed_sessions/{}/measurements", domain, config.session_uuid);
        let mut request = client.request(Method::Post, &url, &headers).map_err(|_| SendingError::Retry)?;
        request.write_all(&payload).map_err(|_| SendingError::Overflow)?;
        request.flush().map_err(|_| SendingError::Retry)?;

        let mut response = request.submit().map_err(|_| SendingError::Retry)?;
        let status = response.status();

        if !(200..400).contains(&(status as i32)) {
            return Err(SendingError::ConfigError)
        }

        info!(
            "POST measurements → {status}, sent {} records ({} bytes)",
            measurements.len(),
            payload.len()
        );

        let mut buf = [0u8; 512];
        let n = response.read(&mut buf).unwrap_or(0);
        let body = core::str::from_utf8(&buf[..n]).unwrap_or("<invalid utf8>");

        if !(200..300).contains(&(status as i32)) {
            let mut buf = [0u8; 256];
            let n = response.read(&mut buf).unwrap_or(0);
            let body = core::str::from_utf8(&buf[..n]).unwrap_or("<binary>");
        }
        Ok(())
    }

    pub fn disconnect(&self) -> anyhow::Result<(), WifiError> {
        if let Some(mut wifi) = self.wifi.try_lock() {
            wifi.disconnect().map_err(|_| WifiError::Other)?;
        } else { return Err(WifiError::LockError) }
        Ok(())
    }

    pub fn is_connected(&self) -> Result<bool, WifiError> {
        if let Some(mut wifi) = self.wifi.try_lock() {
            wifi.is_connected().map_err(|_| WifiError::Other)
        } else { Err(WifiError::LockError) }
    }

    fn encode_measurements(&self, measurements: &[Measurement], pm1_index: u8, pm2_5_index: u8) -> Result<Vec<u8>, SendingError> {
        let count = measurements.len();

        if count > u16::MAX as usize { return Err(SendingError::Overflow) }
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