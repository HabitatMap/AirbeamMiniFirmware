use crate::sensor::sensor_thread::Measurement;
use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::sync::Mutex;
use crate::storage::storage_iterator::MeasurementIter;

pub const MOUNT_POINT: &str = "/storage";
pub const FILE_PATH: &str = "/storage/psm.bin";
pub const START_BYTES: [u8; 2] = [0xAA, 0xBB];
pub const RECORD_SIZE: usize = 9; // 2 start + 4 timestamp + 2 raw + 1 checksum

// Buffer up to N records before flushing to flash.
// LittleFS block size is 4096 bytes on ESP32.
// 4096 / 9 = ~455 records per block, so 128 is a reasonable batch
// that reduces write cycles while not using too much RAM.
const BUFFER_CAPACITY: usize = 128;

#[derive(Debug, Clone, Copy)]
pub struct MeasurementRecord {
    pub timestamp: u32,
    pub pm1: u16,
    pub pm2_5: u16,
}

impl MeasurementRecord {
    pub fn new(timestamp: u32, pm1: u16, pm2_5: u16) -> Self {
        Self { timestamp, pm1, pm2_5 }
    }

    pub fn from_measurement(m: &Measurement, timestamp: u32) -> Self {
        Self {
            timestamp,
            pm1: m.pm1_0_avg,
            pm2_5: m.pm2_5_avg,
        }
    }
}

struct StorageInner {
    buffer: Vec<MeasurementRecord>,
}

pub struct StorageManager {
    inner: Mutex<StorageInner>,
}

impl StorageManager {
    /// Create a new StorageManager.
    ///
    /// IMPORTANT: You must mount LittleFS before calling this.
    pub fn new() -> Self {
        // Ensure the file exists
        if let Err(e) = OpenOptions::new()
            .append(true)
            .create(true)
            .open(FILE_PATH)
        {
            log::error!("Failed to create/open storage file: {}", e);
        }

        Self {
            inner: Mutex::new(StorageInner {
                buffer: Vec::with_capacity(BUFFER_CAPACITY),
            }),
        }
    }

    /// Buffer a measurement. When the buffer is full, it automatically flushes to flash.
    pub fn save_measurement(&self, record: MeasurementRecord) -> anyhow::Result<()> {
        let mut inner = self.inner.lock().unwrap();
        inner.buffer.push(record);

        if inner.buffer.len() >= BUFFER_CAPACITY {
            Self::flush_buffer(&mut inner)
        } else { Ok(()) }
    }

    /// Force flush any buffered records to flash.
    /// Call this before sleeping, shutting down, or when you need data persisted immediately.
    pub fn flush(&self) -> anyhow::Result<()> {
        let mut inner = self.inner.lock().unwrap();
        if !inner.buffer.is_empty() {
            Self::flush_buffer(&mut inner)
        } else { Ok(()) }
    }

    /// Internal: write all buffered records to the file in one operation.
    fn flush_buffer(inner: &mut StorageInner) -> anyhow::Result<()> {
        let file = OpenOptions::new().append(true).open(FILE_PATH);

        match file {
            Ok(mut file) => {
                // Pre-allocate a byte buffer for all records
                let mut bytes = Vec::with_capacity(inner.buffer.len() * RECORD_SIZE);

                for record in &inner.buffer {
                    let ts_bytes = record.timestamp.to_be_bytes();
                    let raw_bytes = record.pm1.to_be_bytes();

                    // XOR checksum over timestamp and raw bytes
                    let mut checksum: u8 = 0;
                    for b in ts_bytes.iter().chain(raw_bytes.iter()) {
                        checksum ^= b;
                    }

                    bytes.extend_from_slice(&START_BYTES);
                    bytes.extend_from_slice(&ts_bytes);
                    bytes.extend_from_slice(&raw_bytes);
                    bytes.push(checksum);
                }

                if let Err(e) = file.write_all(&bytes) {
                    log::error!(
                        "Failed to write {} records to storage: {}",
                        inner.buffer.len(),
                        e
                    );
                    return Err(e.into());
                }

                log::info!("Flushed {} records to flash", inner.buffer.len());
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
        self.measurement_count() > 0
    }

    /// Returns the number of records stored on flash (does NOT include buffered records).
    pub fn measurement_count(&self) -> usize {
        let _guard = self.inner.lock().unwrap();

        match std::fs::metadata(FILE_PATH) {
            Ok(metadata) => (metadata.len() as usize) / RECORD_SIZE,
            Err(e) => {
                log::warn!("Failed to read storage metadata: {}", e);
                0
            }
        }
    }

    /// Returns the total count including buffered (unflushed) records.
    pub fn total_measurement_count(&self) -> usize {
        let inner = self.inner.lock().unwrap();

        let on_flash = match std::fs::metadata(FILE_PATH) {
            Ok(metadata) => (metadata.len() as usize) / RECORD_SIZE,
            Err(_) => 0,
        };

        on_flash + inner.buffer.len()
    }

    pub fn iter_measurements(&self) -> Option<MeasurementIter> {
        self.flush(); //just in case there is anything in the buffer
        let _guard = self.inner.lock().unwrap();

        let file = File::open(FILE_PATH).ok()?;
        MeasurementIter::new(file).ok()
    }

    /// Clear all stored measurements and discard the buffer.
    pub fn clear_measurements(&self) {
        let mut inner = self.inner.lock().unwrap();
        inner.buffer.clear();

        // Truncate the file
        if let Err(e) = File::create(FILE_PATH) {
            log::error!("Failed to clear storage file: {}", e);
        } else {
            log::info!("Storage cleared");
        }
    }
    pub fn remove_last(&self, count: usize) -> anyhow::Result<()> {
        let mut inner = self.inner.lock().unwrap();

        if !inner.buffer.is_empty() {
            Self::flush_buffer(&mut inner)?;
        }

        let metadata = std::fs::metadata(FILE_PATH)?;
        let current_size = metadata.len() as usize;
        let total_records = current_size / RECORD_SIZE;

        if count == 0 {
            return Ok(());
        }

        if count >= total_records {
            drop(inner);
            self.clear_measurements();
            return Ok(());
        }

        let new_size = (total_records - count) * RECORD_SIZE;
        let file = File::options().write(true).open(FILE_PATH)?;
        file.set_len(new_size as u64)?;

        log::info!("Removed {} records from end of storage ({} remaining)", count, total_records - count);
        Ok(())
    }
}

impl Drop for StorageManager {
    fn drop(&mut self) {
        // Flush any remaining buffered records before the manager is dropped
        let mut inner = self.inner.lock().unwrap();
        if !inner.buffer.is_empty() {
            log::info!("Flushing {} remaining records on drop", inner.buffer.len());
            let _ = Self::flush_buffer(&mut inner);
        }
    }
}