use esp32_nimble::utilities::BleUuid;
use esp32_nimble::uuid128;
use uuid::Uuid;
use crate::storage::session_config::{SessionConfig, SessionType};

pub const SERVICE_UUID: BleUuid = uuid128!("a0e1f000-0001-4b3c-8e9a-1f2d3c4b5a60");
pub const STATUS_CHAR_UUID: BleUuid = uuid128!("a0e1f000-0002-4b3c-8e9a-1f2d3c4b5a60");
pub const COMMAND_CHAR_UUID: BleUuid = uuid128!("a0e1f000-0003-4b3c-8e9a-1f2d3c4b5a60");
pub const RESPONSE_CHAR_UUID: BleUuid = uuid128!("a0e1f000-0004-4b3c-8e9a-1f2d3c4b5a60");
pub const PM_1_NAME: &str = "PM1";
pub const PM_2_5_NAME: &str = "PM2.5";
pub const PM_1_UNIT: &str = "μg/m3";
pub const PM_2_5_UNIT: &str = "μg/m3";

/// Commands the app writes to the device
/// All data in LowEndian
#[derive(Debug, Clone)]
pub enum AppCommand {
    ContinueSession,                // 0x10
    DiscardSession,                 // 0x11
    StartSync,                      // 0x12
    NewSessionConfig(SessionConfig),// 0x13 + 16B - uuid + u16 interval + u8 session_type (optional: + u8 pm1 index + u8 pm2 index + 32B wifi_ssid + 64B wifi_pass)
    GetSensors,                     // 0x14
    SetTime(i64)                    // 0x15 + i64
}

impl AppCommand {
    pub fn decode(data: &[u8]) -> Option<Self> {
        match *data.first()? {
            0x10 => Some(Self::ContinueSession),
            0x11 => Some(Self::DiscardSession),
            0x12 => Some(Self::StartSync),
            0x13 if data.len() >= 19 => {
                let uuid = Uuid::from_slice(&data[1..17]).ok()?;
                let interval_seconds = u16::from_le_bytes(data[17..19].try_into().ok()?);
                let interval = std::time::Duration::from_secs(interval_seconds as u64);
                let session_type = match data[19] {
                    0 => {
                        if data.len() < 118 { return None; }
                        let pm1_index = data[20];
                        let pm2_5_index = data[21];
                        let wifi_ssid = String::from_utf8(data[22..54].iter().take_while(|&&x| x != 0).cloned().collect()).ok()?;
                        let wifi_password = String::from_utf8(data[54..118].iter().take_while(|&&x| x != 0).cloned().collect()).ok()?;
                        SessionType::FIXED {pm1_index, pm2_5_index, wifi_ssid, wifi_password}
                    }
                    1 => {
                        SessionType::MOBILE
                    }
                    _ => return None,
                };
                Some(Self::NewSessionConfig(SessionConfig::new(uuid, interval, session_type)))
            }
            0x14 => Some(Self::GetSensors),
            0x15 if data.len() >= 9 => {
                let epoch = i64::from_le_bytes(data[1..9].try_into().ok()?);
                Some(Self::SetTime(epoch))
            }
            _ => None,
        }
    }
}

/// Device responses back to the app
#[derive(Debug, Clone)]
pub enum DeviceResponse {
    Ack,                    // 0x20
    Nack(ErrorCode),        // 0x21
    Ready,                  // 0x22
}
impl DeviceResponse {
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        match self {
            Self::Ack => {
                buf[0] = 0x20;
                buf.len()
            }
            Self::Nack(code) => {
                buf[0] = 0x21;
                buf[1] = *code as u8;
                buf.len()
            }
            Self::Ready => {
                buf[0] = 0x24;
                buf.len()
            }
        }
    }
}

pub enum DeviceStatus {
    Idle,
    HasSavedSession {
        session: Uuid,
        has_measurements: bool,
    }
}

impl DeviceStatus {
    pub fn encode(&self, buf: &mut [u8]) -> usize{
        match self {
            Self::Idle => {
                buf[0] = 0x00;
                buf.len()
            }
            Self::HasSavedSession { session, has_measurements } => {
                buf[0] = 0x01;
                buf[1..17].copy_from_slice(&session.to_bytes_le());
                buf[17] = *has_measurements as u8;
                buf.len()
            }
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum ErrorCode {
    InvalidState = 0x01,
    InvalidConfig = 0x02,
    StorageHasMeasurements = 0x03,
    ClearStorageFailed = 0x04,
}
