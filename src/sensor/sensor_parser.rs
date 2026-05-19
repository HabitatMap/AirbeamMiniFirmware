use byteorder::{BigEndian, ByteOrder};

#[derive(Debug, Default, Clone, Copy)]
pub struct PmsMeasurement {
    pub(crate) pm1_0_atm: u16,
}
pub fn parse_sensor(buffer: &[u8; 32]) -> Option<PmsMeasurement> {
    // Checksum is the last 2 bytes. It should be equal to the sum of the first 30 bytes.
    let checksum_received = BigEndian::read_u16(&buffer[30..32]);
    let checksum_calculated: u16 = buffer[0..30].iter().map(|&b| b as u16).sum();

    if checksum_received != checksum_calculated {
        return None;
    }

    let pm1_0_atm = BigEndian::read_u16(&buffer[10..12]);

    Some(PmsMeasurement { pm1_0_atm })
}
