use uuid::Timestamp;
use crate::LoopEvent;
use crate::sensor::sensor_parser::PmsMeasurement;

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
        Measurement::new(pms.pm1_0_atm, pms.pm2_5_atm, timestamp)
    }
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