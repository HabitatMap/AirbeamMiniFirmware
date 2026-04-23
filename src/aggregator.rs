use std::time::Duration;
use crate::sensor::measurement::Measurement;

const MAX_DURATION_FOR_AVERAGE: u32 = 59;

pub struct MeasurementAggregator {
    duration: u32,
    records: Vec<Measurement>,
}

impl MeasurementAggregator {
    pub fn new(duration: Duration) -> Self {
        Self {
            duration: duration.as_secs() as u32,
            records: Vec::new(),
        }
    }

    pub fn average_measurement(
        &mut self,
        measurement: Measurement,
    ) -> Option<Measurement> {
        if self.duration > MAX_DURATION_FOR_AVERAGE {
            return Some(measurement);
        }
        if self.records.is_empty() {
            self.records.push(measurement);
            return None;
        }
        let earliest_timestamp = self.records.iter().min().unwrap().timestamp;
        if measurement.timestamp - earliest_timestamp > self.duration {
            let avg = Some(self.get_average());
            self.records.clear();
            avg
        } else {
            self.records.push(measurement);
            None
        }
    }

    fn get_average(&self) -> Measurement {
        let count = self.records.len() as u32;
        let pm1_avg = self.records.iter().map(|r| r.pm1_0_avg as u32).sum::<u32>() / count;
        let pm2_5_avg = self.records.iter().map(|r| r.pm2_5_avg as u32).sum::<u32>() / count;
        Measurement {
            timestamp: self.records.iter().max().unwrap().timestamp,
            pm1_0_avg: pm1_avg as u16,
            pm2_5_avg: pm2_5_avg as u16,
        }
    }
}
