use crate::sensor::sensor_parser::PmsMeasurement;
use crate::LoopEvent;

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub struct Measurement {
    pub pm1_0_avg: u16,
    pub pm2_5_avg: u16,
    pub timestamp: u32,
}
impl From<Measurement> for LoopEvent {
    fn from(value: Measurement) -> Self {
        LoopEvent::Measurement(value)
    }
}
impl Measurement {
    pub fn new(pm1_0_avg: u16, pm2_5_avg: u16, timestamp: u32) -> Self {
        Measurement {
            pm1_0_avg,
            pm2_5_avg,
            timestamp,
        }
    }

    pub fn from_pms_measurement(pms: PmsMeasurement, timestamp: u32) -> Self {
        Measurement::from_raw_pm1_atm(pms.pm1_0_atm as f32, timestamp)
    }

    pub fn from_raw_pm1_atm(raw_pm1_atm: f32, timestamp: u32) -> Self {
        let (pm1, pm25) = calibrate_pm(raw_pm1_atm);
        Measurement::new(pm1, pm25, timestamp)
    }
}

/// Piecewise-linear calibration ported from the original Plantower firmware:
/// raw PM1.0 atm → calibrated PM1.0, then PM1.0 → PM2.5.
pub fn calibrate_pm(raw_pm1_atm: f32) -> (u16, u16) {
    let pm1 = if raw_pm1_atm < 22.14 {
        1.0566 * raw_pm1_atm
    } else if raw_pm1_atm < 37.95 {
        -30.9677 + 2.4554 * raw_pm1_atm
    } else {
        11.2026 + 1.3442 * raw_pm1_atm
    };
    let pm25 = if pm1 < 5.16 {
        1.49 + 1.4 * pm1
    } else if pm1 < 30.16 {
        2.0 + 1.1 * pm1
    } else if pm1 < 126.16 {
        -4.0 + 1.2826 * pm1
    } else {
        -24.0 + 1.441 * pm1
    };
    (
        pm1.max(0.0).round() as u16,
        pm25.max(0.0).round() as u16,
    )
}

impl Ord for Measurement {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.timestamp.cmp(&other.timestamp)
    }
}

impl PartialOrd for Measurement {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
