use crate::sensor::measurement::Measurement;
use crate::storage::storage_controller::START_BYTES;
use esp_idf_svc::sys::vTaskDelay;
use std::fs::File;
use std::io::{Read, Seek, SeekFrom};

const MAX_LINE_MEASUREMENTS: usize = 10;
const MEASUREMENT_SIZE: usize = 8; // u32 + u16 + u16
const LINE_HEADER_SIZE: usize = 3; // 0xAB + 0xBA + count: u8
const MIN_LINE_SIZE: usize = LINE_HEADER_SIZE + MEASUREMENT_SIZE + 1;
const MAX_LINE_SIZE: usize = LINE_HEADER_SIZE + MAX_LINE_MEASUREMENTS * MEASUREMENT_SIZE + 1;
const BUF_CAPACITY: usize = 4096;
#[derive(Debug, Clone)]
pub struct MeasurementLine {
    pub measurements: Vec<Measurement>,
    /// Bytes from end of file to the start of this line.
    /// Truncate file to `file_len - offset_from_end` to discard
    /// this line and everything after it (including skipped corruption).
    pub offset_from_end: u64,
}

pub struct MeasurementIter {
    file: File,
    buf: Vec<u8>,
    buf_file_start: u64,
    file_len: u64,
    /// Points at the end of the next line to parse (moves left)
    cursor: u64,
    done: bool,
}

impl MeasurementIter {
    pub fn new(mut file: File) -> std::io::Result<Self> {
        let file_len = file.metadata()?.len();
        let read_size = (file_len as usize).min(BUF_CAPACITY);
        let start = file_len - read_size as u64;

        file.seek(SeekFrom::Start(start))?;
        let mut buf = vec![0u8; read_size];
        file.read_exact(&mut buf)?;

        Ok(Self {
            file,
            buf,
            buf_file_start: start,
            file_len,
            cursor: file_len,
            done: false,
        })
    }

    /// Extend buffer leftward to cover earlier file data.
    fn extend_left(&mut self) -> bool {
        if self.buf_file_start == 0 {
            return false;
        }

        // Trim past cursor
        let keep = (self.cursor - self.buf_file_start) as usize;
        self.buf.truncate(keep);

        let read_size = (self.buf_file_start as usize).min(BUF_CAPACITY);
        let new_start = self.buf_file_start - read_size as u64;

        if self.file.seek(SeekFrom::Start(new_start)).is_err() {
            return false;
        }

        let mut new_data = vec![0u8; read_size];
        if self.file.read_exact(&mut new_data).is_err() {
            return false;
        }

        new_data.extend_from_slice(&self.buf);
        self.buf = new_data;
        self.buf_file_start = new_start;
        true
    }

    /// Try parsing a line that starts at `buf_pos` and ends exactly at cursor.
    fn try_parse_at(&self, buf_pos: usize, end: usize) -> Option<Vec<Measurement>> {
        let slice = &self.buf[buf_pos..end];
        let len = slice.len();

        if len < MIN_LINE_SIZE {
            log::warn!("Skipping line with too few bytes: {}", len);
            return None;
        }
        if slice[0] != START_BYTES[0] || slice[1] != START_BYTES[1] {
            //log::warn!("Skipping line with invalid start bytes: {:02x}{:02x}", slice[0], slice[1]);
            return None;
        }

        let count = slice[2] as usize;
        if count == 0 || count > MAX_LINE_MEASUREMENTS {
            log::warn!("Skipping line with invalid measurement count: {}", count);
            return None;
        }

        let expected = LINE_HEADER_SIZE + count * MEASUREMENT_SIZE + 1;
        if len != expected {
            log::warn!("Skipping line with invalid length: {}", len);
            return None;
        }

        let stored_checksum = slice[len - 1];
        let mut checksum: u8 = 0;
        for &b in &slice[..len - 1] {
            checksum ^= b;
        }
        if checksum != stored_checksum {
            log::warn!(
                "Skipping line with invalid checksum: {:02x}",
                stored_checksum
            );
            return None;
        }

        let data = &slice[LINE_HEADER_SIZE..];
        let mut measurements = Vec::with_capacity(count);
        for i in 0..count {
            let o = i * MEASUREMENT_SIZE;
            measurements.push(Measurement {
                timestamp: u32::from_le_bytes(data[o..o + 4].try_into().unwrap()),
                pm1_0_avg: u16::from_le_bytes(data[o + 4..o + 6].try_into().unwrap()),
                pm2_5_avg: u16::from_le_bytes(data[o + 6..o + 8].try_into().unwrap()),
            });
        }

        Some(measurements)
    }
}

impl Iterator for MeasurementIter {
    type Item = MeasurementLine;

    fn next(&mut self) -> Option<Self::Item> {
        if self.done || self.cursor == 0 {
            self.done = true;
            return None;
        }

        loop {
            unsafe {
                vTaskDelay(1);
            }
            let cursor_in_buf = (self.cursor - self.buf_file_start) as usize;

            let scan_lo = cursor_in_buf.saturating_sub(MAX_LINE_SIZE);
            let scan_hi = cursor_in_buf.saturating_sub(MIN_LINE_SIZE);

            if scan_lo < cursor_in_buf {
                for pos in (scan_lo..=scan_hi).rev() {
                    if let Some(measurements) = self.try_parse_at(pos, cursor_in_buf) {
                        self.cursor = self.buf_file_start + pos as u64;
                        return Some(MeasurementLine {
                            measurements,
                            offset_from_end: self.file_len - self.cursor,
                        });
                    }
                }

                // Resync: step back 1 byte
                self.cursor -= 1;
                if self.cursor == 0 {
                    self.done = true;
                    return None;
                }

                continue;
            }

            if !self.extend_left() {
                self.done = true;
                return None;
            }
        }
    }
}
