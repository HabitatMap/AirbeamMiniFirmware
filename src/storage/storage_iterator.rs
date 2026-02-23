use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use crate::storage::storage_controller::{MeasurementRecord, RECORD_SIZE, START_BYTES};

const CHUNK_RECORDS: usize = 64;
const CHUNK_SIZE: usize = CHUNK_RECORDS * RECORD_SIZE;

pub struct MeasurementIter {
    file: File,
    buf: [u8; CHUNK_SIZE],
    /// Byte offset in the file where the current chunk starts
    file_offset: u64,
    /// Current position within buf (index into buf, counting backwards)
    buf_pos: usize,
    /// How many valid bytes are in buf
    buf_len: usize,
    done: bool,
}

impl MeasurementIter {
    pub fn new(file: File) -> std::io::Result<Self> {
        let file_len = file.metadata()?.len();
        Ok(Self {
            file,
            buf: [0u8; CHUNK_SIZE],
            file_offset: file_len,
            buf_pos: 0,
            buf_len: 0,
            done: false,
        })
    }

    fn load_prev_chunk(&mut self) -> bool {
        if self.file_offset == 0 {
            return false;
        }

        let read_size = (self.file_offset as usize).min(CHUNK_SIZE);
        self.file_offset -= read_size as u64;

        if self.file.seek(SeekFrom::Start(self.file_offset)).is_err() {
            return false;
        }

        match self.file.read_exact(&mut self.buf[..read_size]) {
            Ok(_) => {
                self.buf_len = read_size;
                self.buf_pos = read_size;
                true
            }
            Err(_) => false,
        }
    }

    fn try_parse_record_at(&self, pos: usize) -> Option<MeasurementRecord> {
        if pos + RECORD_SIZE > self.buf_len {
            return None;
        }

        let chunk = &self.buf[pos..pos + RECORD_SIZE];

        if chunk[0] != START_BYTES[0] || chunk[1] != START_BYTES[1] {
            return None;
        }

        let ts_bytes: [u8; 4] = chunk[2..6].try_into().unwrap();
        let raw_bytes: [u8; 2] = chunk[6..8].try_into().unwrap();
        let stored_checksum = chunk[8];

        let mut checksum: u8 = 0;
        for b in ts_bytes.iter().chain(raw_bytes.iter()) {
            checksum ^= b;
        }

        if checksum != stored_checksum {
            return None;
        }

        Some(MeasurementRecord {
            timestamp: u32::from_be_bytes(ts_bytes),
            raw: u16::from_be_bytes(raw_bytes),
        })
    }
}

impl Iterator for MeasurementIter {
    type Item = MeasurementRecord;

    fn next(&mut self) -> Option<Self::Item> {
        if self.done {
            return None;
        }

        loop {
            // Walk backwards through current buffer
            while self.buf_pos >= RECORD_SIZE {
                let candidate = self.buf_pos - RECORD_SIZE;

                if let Some(record) = self.try_parse_record_at(candidate) {
                    self.buf_pos = candidate;
                    return Some(record);
                }

                // Resync: step back by 1 byte
                self.buf_pos -= 1;
            }

            // Remaining bytes at the start of buf might be a partial record
            // that spans the previous chunk — we need to handle the boundary.
            // For simplicity, we just load the next chunk. If the file is
            // well-formed, records align and we lose nothing. If corrupt,
            // we skip at most one record at a chunk boundary.
            if !self.load_prev_chunk() {
                self.done = true;
                return None;
            }
        }
    }
}