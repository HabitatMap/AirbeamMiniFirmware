use std::time::Duration;
use esp_idf_svc::nvs::{EspDefaultNvs, EspDefaultNvsPartition, EspNvs};
use esp_idf_svc::sys::EspError;
use uuid::Uuid;
use crate::storage::session_config::{SessionConfig, SessionType};

const NAMESPACE: &str = "session";
const KEY_UUID: &str = "uuid";
const KEY_WIFI_SSID: &str = "wifi_ssid";
const KEY_WIFI_PASS: &str = "wifi_pass";
const KEY_IS_MOBILE: &str = "is_mobile";
const KEY_MEASUREMENT_INTERVAL: &str = "interval";
const KEY_PM1_INDEX: &str = "pm1_index";
const KEY_PM2_5_INDEX: &str = "pm2_5_index";

/// Manages persistent session data stored in the ESP32's NVS flash.
pub struct NvsManager {
    nvs: EspDefaultNvs,
}

impl NvsManager {
    /// Opens the NVS namespace once upon initialization
    pub fn new(partition: EspDefaultNvsPartition) -> Result<Self, EspError> {
        let nvs = EspNvs::new(partition, NAMESPACE, true)?;
        Ok(Self { nvs })
    }

    pub fn clear_all(&mut self) {
        let _ = self.nvs.remove(KEY_UUID);
        let _ = self.nvs.remove(KEY_WIFI_SSID);
        let _ = self.nvs.remove(KEY_WIFI_PASS);
        let _ = self.nvs.remove(KEY_IS_MOBILE);
        let _ = self.nvs.remove(KEY_MEASUREMENT_INTERVAL);
        let _ = self.nvs.remove(KEY_PM1_INDEX);
        let _ = self.nvs.remove(KEY_PM2_5_INDEX);
    }

    pub fn get_uuid(&self) -> Result<Option<Uuid>, EspError> {
        let mut buffer = [0u8; 16];
        Ok(self.nvs.get_blob(KEY_UUID, &mut buffer)?
            .and_then(|bytes| Uuid::from_slice(bytes).ok()))
    }

    pub fn set_uuid(&mut self, uuid: &Uuid) -> Result<(), EspError> {
        self.nvs.set_blob(KEY_UUID, uuid.as_bytes())
    }

    pub fn get_wifi_ssid(&self) -> Result<Option<String>, EspError> {
        let mut buffer = [0u8; 33];
        Ok(self.nvs.get_str(KEY_WIFI_SSID, &mut buffer)?
            .map(|s| s.to_string()))
    }

    pub fn set_wifi_ssid(&mut self, ssid: &str) -> Result<(), EspError> {
        self.nvs.set_str(KEY_WIFI_SSID, ssid)
    }

    pub fn get_wifi_password(&self) -> Result<Option<String>, EspError> {
        let mut buffer = [0u8; 65];
        Ok(self.nvs.get_str(KEY_WIFI_PASS, &mut buffer)?
            .map(|p| p.to_string()))
    }

    pub fn set_wifi_password(&mut self, password: &str) -> Result<(), EspError> {
        self.nvs.set_str(KEY_WIFI_PASS, password)
    }

    pub fn get_is_mobile(&self) -> Result<Option<bool>, EspError> {
        Ok(self.nvs.get_u8(KEY_IS_MOBILE)?.map(|val| val != 0))
    }

    pub fn set_is_mobile(&mut self, is_mobile: bool) -> Result<(), EspError> {
        self.nvs.set_u8(KEY_IS_MOBILE, is_mobile as u8)
    }

    pub fn get_measurement_interval(&self) -> Result<Option<Duration>, EspError> {
        Ok(self.nvs.get_u32(KEY_MEASUREMENT_INTERVAL)?
            .map(|val| Duration::from_secs(val as u64)))
    }

    pub fn set_measurement_interval(&mut self, interval: Duration) -> Result<(), EspError> {
        self.nvs.set_u32(KEY_MEASUREMENT_INTERVAL, interval.as_secs() as u32)
    }

    pub fn get_pm1_index(&self) -> Result<Option<u8>, EspError> {
        self.nvs.get_u8(KEY_PM1_INDEX)
    }

    pub fn set_pm1_index(&mut self, pm_index: u8) -> Result<(), EspError> {
        self.nvs.set_u8(KEY_PM1_INDEX, pm_index)
    }

    pub fn get_pm2_5_index(&self) -> Result<Option<u8>, EspError> {
        self.nvs.get_u8(KEY_PM2_5_INDEX)
    }

    pub fn set_pm2_5_index(&mut self, pm_index: u8) -> Result<(), EspError> {
        self.nvs.set_u8(KEY_PM2_5_INDEX, pm_index)
    }

    pub fn get_session_config(&self) -> Result<Option<SessionConfig>, EspError> {
        let uuid = match self.get_uuid()? {
            Some(u) => u,
            None => return Ok(None),
        };

        let interval = match self.get_measurement_interval()? {
            Some(i) => i,
            None => return Ok(None),
        };

        let is_mobile = match self.get_is_mobile()? {
            Some(m) => m,
            None => return Ok(None),
        };

        // 2. Determine SessionType
        let Some(session_type) = (if is_mobile {
            Some(SessionType::MOBILE)
        } else {
            self.get_pm1_index()?.zip(self.get_pm2_5_index()?)
                .zip(self.get_wifi_ssid()?)
                .zip(self.get_wifi_password()?)
                .map(|(((p1, p2), ssid), pass)| SessionType::FIXED {
                    pm1_index: p1, pm2_5_index: p2, wifi_ssid: ssid, wifi_password: pass
                })
        }) else {
            return Ok(None);
        };

        Ok(Some(SessionConfig::new(uuid, interval, session_type)))
    }
    pub fn set_session_config(&mut self, config: &SessionConfig) -> Result<(), EspError> {
        self.set_uuid(&config.session_uuid)?;
        self.set_measurement_interval(config.interval)?;
        match &config.session_type {
            SessionType::MOBILE => {
                self.set_is_mobile(true)?;
            },
            SessionType::FIXED { pm1_index, pm2_5_index, wifi_ssid, wifi_password } => {
                self.set_is_mobile(false)?;
                self.set_pm1_index(*pm1_index)?;
                self.set_pm2_5_index(*pm2_5_index)?;
                self.set_wifi_ssid(&wifi_ssid)?;
                self.set_wifi_password(&wifi_password)?;
            }
        }
        Ok(())
    }
}