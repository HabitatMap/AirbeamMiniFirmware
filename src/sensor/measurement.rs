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
        let pm2_5 = (1.23345 + 0.005157 * (pms.c03 as f32) + 0.211782 * (pms.c1 as f32)).max(0.0);
        let pm1 = (pm2_5 * (0.855 - 0.818 * (-pm2_5 / 6.12_f32).exp())).max(0.0);

        Measurement::new(pm1.round() as u16, pm2_5.round() as u16, timestamp)
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
