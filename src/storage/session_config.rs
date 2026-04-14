use std::time::Duration;
use uuid::Uuid;

#[derive(Clone, Debug)]
pub struct SessionConfig {
    pub session_uuid: Uuid,
    pub interval: Duration,
    pub session_type: SessionType,
}

#[derive(Clone, Debug)]
pub enum SessionType {
    MOBILE,
    FIXED {
        pm1_index: u8,
        pm2_5_index: u8,
        token: u128,
        wifi_ssid: String,
        wifi_password: String,
    },
}

impl SessionConfig {
    pub const PM_1_NAME: &str = "PM1";
    pub const PM_2_5_NAME: &str = "PM2.5";
    pub const PM_1_UNIT: &str = "μg/m3";
    pub const PM_2_5_UNIT: &str = "μg/m3";

    pub fn new(session_uuid: Uuid, interval: Duration, session_type: SessionType) -> Self {
        Self {
            session_uuid,
            interval,
            session_type,
        }
    }
}
