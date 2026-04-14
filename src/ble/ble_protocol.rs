use crate::storage::session_config::{SessionConfig, SessionType};
use crate::LoopEvent;
use uuid::Uuid;

pub const SENSOR_INFO: &str = "PM1,μg/m3;PM2.5,μg/m3";
/// Commands the app writes to the device
/// All data in LowEndian
#[derive(Debug, Clone)]
pub enum AppCommand {
    ContinueSession,                 // 0x10
    DiscardSession,                  // 0x11 (end session without syncing)
    StartSync,                       // 0x12 (end session when running)
    NewSessionConfig(SessionConfig), // 0x13 + 16B - uuid + u16 interval + u8 session_type (optional: + u8 pm1 index + u8 pm2 index + 32B wifi_ssid + 64B wifi_pass)
    GetSensors,                      // 0x14
    SetTime(i64),                    // 0x15 + i64
}

impl AppCommand {
    pub fn decode(data: &[u8]) -> Option<Self> {
        match *data.first()? {
            0x10 => Some(Self::ContinueSession),
            0x11 => Some(Self::DiscardSession),
            0x12 => Some(Self::StartSync),
            0x13 if data.len() >= 19 => {
                let uuid = Uuid::from_slice_le(&data[1..17]).ok()?;
                let interval_seconds = u16::from_le_bytes(data[17..19].try_into().ok()?);
                let interval = std::time::Duration::from_secs(interval_seconds as u64);
                let session_type = match data[19] {
                    0 => {
                        if data.len() < 118 {
                            return None;
                        }
                        let pm1_index = data[20];
                        let pm2_5_index = data[21];
                        let token = u128::from_le_bytes(data[22..38].try_into().ok()?);
                        let wifi_ssid = String::from_utf8(
                            data[38..70]
                                .iter()
                                .take_while(|&&x| x != 0)
                                .cloned()
                                .collect(),
                        )
                        .ok()?;
                        let wifi_password = String::from_utf8(
                            data[70..134]
                                .iter()
                                .take_while(|&&x| x != 0)
                                .cloned()
                                .collect(),
                        )
                        .ok()?;
                        SessionType::FIXED {
                            pm1_index,
                            pm2_5_index,
                            token,
                            wifi_ssid,
                            wifi_password,
                        }
                    }
                    1 => SessionType::MOBILE,
                    _ => return None,
                };
                Some(Self::NewSessionConfig(SessionConfig::new(
                    uuid,
                    interval,
                    session_type,
                )))
            }
            0x14 => Some(Self::GetSensors),
            0x15 if data.len() >= 9 => {
                let epoch = i64::from_le_bytes(data[1..9].try_into().ok()?);
                Some(Self::SetTime(epoch))
            }
            _ => None,
        }
    }
    pub fn as_loop_event(&self) -> Option<LoopEvent> {
        match self {
            AppCommand::SetTime(time) => Some(LoopEvent::TimeUpdate(*time)),
            AppCommand::DiscardSession => Some(LoopEvent::Stop { start_sync: false }),
            AppCommand::StartSync => Some(LoopEvent::Stop { start_sync: true }),
            _ => None,
        }
    }
}

/// Device responses back to the app
#[derive(Debug, Clone)]
pub enum DeviceResponse {
    Ack,             // 0x20
    Nack(ErrorCode), // 0x21
    Ready,           // 0x22
    SensorInfo,      // 0x23
}
impl DeviceResponse {
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        match self {
            Self::Ack => {
                buf[0] = 0x20;
                1
            }
            Self::Nack(code) => {
                buf[0] = 0x21;
                buf[1] = *code as u8;
                2
            }
            Self::Ready => {
                buf[0] = 0x22;
                1
            }
            Self::SensorInfo => {
                buf[0] = 0x23;
                let info = SENSOR_INFO.as_bytes();
                buf[1..1 + info.len()].copy_from_slice(info);
                info.len() + 1
            }
        }
    }
}

pub enum DeviceStatus {
    Idle(i8),
    HasSavedSession {
        battery_level: i8,
        session: Uuid,
        has_measurements: bool,
    },
    Running {
        battery_level: i8,
        session: Uuid,
    },
}

impl DeviceStatus {
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        match self {
            Self::Idle(battery_level) => {
                buf[0] = 0x00;
                buf[1] = *battery_level as u8;
                2
            }
            Self::HasSavedSession {
                battery_level,
                session,
                has_measurements,
            } => {
                buf[0] = 0x01;
                buf[1] = *battery_level as u8;
                buf[2..18].copy_from_slice(&session.to_bytes_le());
                buf[18] = *has_measurements as u8;
                19
            }
            Self::Running {
                battery_level,
                session,
            } => {
                buf[0] = 0x02;
                buf[1] = *battery_level as u8;
                buf[2..18].copy_from_slice(&session.to_bytes_le());
                18
            }
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum ErrorCode {
    NoSession = 0x01,
    InvalidConfig = 0x02,
    StorageHasMeasurements = 0x03,
    ClearStorageFailed = 0x04,
}
