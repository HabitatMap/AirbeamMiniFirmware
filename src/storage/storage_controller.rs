use crate::storage::storage_iterator::MeasurementIter;
use log::{error, info, warn};
use std::fs::{File, OpenOptions};
use std::io::{Read, Seek, SeekFrom, Write};
use std::sync::Mutex;
use std::time::Duration;
use crate::aggregator::MeasurementAggregator;
use crate::sensor::measurement::Measurement;

pub const MOUNT_POINT: &str = "/storage";
pub const FILE_PATH: &str = "/storage/psm.bin";
pub const START_BYTES: [u8; 2] = [0xAB, 0xBA]; // gimmie gimmie gimmie start bytes after midnight

// Buffer up to N records before flushing to flash.
const BUFFER_CAPACITY: usize = 10;
//Record size is 2 start bytes + 1 byte number of measurements in a record + 8 bytes (timestamp + raw data) for each record + 1 byte checksum
const MAX_RECORD_SIZE: usize = BUFFER_CAPACITY * 8 + 4;

struct StorageInner {
    buffer: Vec<Measurement>,
}

pub struct StorageManager {
    inner: Mutex<StorageInner>,
    aggregator: Option<MeasurementAggregator>
}

impl StorageManager {
    /// Create a new StorageManager.
    ///
    /// IMPORTANT: You must mount LittleFS before calling this.
    pub fn new() -> Self {
        // Ensure the file exists
        if let Err(e) = OpenOptions::new().append(true).create(true).open(FILE_PATH) {
            log::error!("Failed to create/open storage file: {}", e);
        }
        Self {
            inner: Mutex::new(StorageInner {
                buffer: Vec::with_capacity(BUFFER_CAPACITY),
            }),
            aggregator: None
        }
    }

    pub fn set_aggregator(&mut self, interval: Duration) {
        self.aggregator = if interval.as_secs() < 60 {
            Some(MeasurementAggregator::new(interval))
        } else {
            None
        };
    }

    /// Buffer a measurement. When the buffer is full, it automatically flushes to flash.
    pub fn save_measurement(&mut self, record: Measurement) -> anyhow::Result<()> {

        let to_save = if let Some(mut aggregator) = self.aggregator.as_mut() {
            aggregator.average_measurement(record.clone())
        } else {
            Some ( record )
        };

        if to_save.is_none() {
            return Ok(());
        }
        let mut inner = self.inner.lock().unwrap();
        inner.buffer.push(record);

        if inner.buffer.len() >= BUFFER_CAPACITY {
            Self::flush_buffer(&mut inner)
        } else {
            Ok(())
        }
    }

    /// Force flush any buffered records to flash.
    /// Call this before sleeping, shutting down, or when you need data persisted immediately.
    pub fn flush(&self) -> anyhow::Result<()> {
        let mut inner = self.inner.lock().unwrap();
        if !inner.buffer.is_empty() {
            Self::flush_buffer(&mut inner)
        } else {
            Ok(())
        }
    }

    /// Internal: write all buffered records to the file in one operation.
    fn flush_buffer(inner: &mut StorageInner) -> anyhow::Result<()> {
        let file = OpenOptions::new().append(true).open(FILE_PATH);
        if inner.buffer.is_empty() {
            return Ok(());
        }

        match file {
            Ok(mut file) => {
                // Pre-allocate a byte buffer for all records
                let mut bytes = Vec::with_capacity(inner.buffer.len() * MAX_RECORD_SIZE);
                bytes.extend_from_slice(&START_BYTES);
                bytes.push(inner.buffer.len() as u8);

                for record in &inner.buffer {
                    let ts_bytes = record.timestamp.to_le_bytes();
                    let pm1_bytes = record.pm1_0_avg.to_le_bytes();
                    let pm2_bytes = record.pm2_5_avg.to_le_bytes();
                    bytes.extend_from_slice(&ts_bytes);
                    bytes.extend_from_slice(&pm1_bytes);
                    bytes.extend_from_slice(&pm2_bytes);
                }

                // XOR checksum
                let mut checksum: u8 = 0;
                for b in bytes.iter() {
                    checksum ^= b;
                }
                bytes.push(checksum);

                if let Err(e) = file.write_all(&bytes) {
                    log::error!(
                        "Failed to write {} records to storage: {}",
                        inner.buffer.len(),
                        e
                    );
                    return Err(e.into());
                }

                info!("Flushed {} records to flash", inner.buffer.len());
                inner.buffer.clear();
                Ok(())
            }
            Err(e) => {
                log::error!("Failed to open storage file for writing: {}", e);
                Err(e.into())
            }
        }
    }
    pub fn has_measurements(&self) -> bool {
        let _guard = self.inner.lock().unwrap();

        match std::fs::metadata(FILE_PATH) {
            Ok(metadata) => metadata.len() > 0,
            Err(e) => {
                log::warn!("Failed to read storage metadata: {}", e);
                false
            }
        }
    }

    pub fn iter_measurements(&self) -> Option<MeasurementIter> {
        if let Err(e) = self.flush() {
            warn!("Failed to flush storage: {}", e);
        }
        let _guard = self.inner.lock().unwrap();
        info!("Reading measurements from storage");
        let file = File::open(FILE_PATH).ok()?;
        MeasurementIter::new(file).ok()
    }

    /// Clear all stored measurements and discard the buffer.
    pub fn clear_measurements(&self) -> anyhow::Result<()> {
        let mut inner = self.inner.lock().unwrap();
        inner.buffer.clear();
        if let Err(e) = File::create(FILE_PATH) {
            error!("Failed to create storage file: {}", e);
            Err(e.into())
        } else {
            Ok(())
        }
    }
    pub fn remove_last(&self, bytes_to_remove: usize) -> anyhow::Result<()> {
        let mut inner = self.inner.lock().unwrap();

        if !inner.buffer.is_empty() {
            Self::flush_buffer(&mut inner)?;
        }

        let metadata = std::fs::metadata(FILE_PATH)?;
        let current_size = metadata.len() as usize;

        if bytes_to_remove == 0 {
            return Ok(());
        }

        if bytes_to_remove >= current_size {
            drop(inner);
            self.clear_measurements()?;
            return Ok(());
        }

        let new_size = current_size - bytes_to_remove;
        let file = File::options().write(true).open(FILE_PATH)?;
        file.set_len(new_size as u64)?;
        Ok(())
    }
}

impl Drop for StorageManager {
    fn drop(&mut self) {
        // Flush any remaining buffered records before the manager is dropped
        let mut inner = self.inner.lock().unwrap();
        if !inner.buffer.is_empty() {
            let _ = Self::flush_buffer(&mut inner);
        }
    }
}
