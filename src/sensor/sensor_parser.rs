use byteorder::{BigEndian, ByteOrder};

#[derive(Debug, Default, Clone)]
pub struct PmsMeasurement {
    pub(crate) c03: u16,
    pub(crate) c10: u16,
}
pub fn parse_sensor(buffer: &[u8; 32]) -> Option<PmsMeasurement> {
    // Checksum is the last 2 bytes. It should be equal to the sum of the first 30 bytes.
    let checksum_received = BigEndian::read_u16(&buffer[30..32]);
    let checksum_calculated: u16 = buffer[0..30].iter().map(|&b| b as u16).sum();

    if checksum_received != checksum_calculated {
        return None;
    }

    let c03 = BigEndian::read_u16(&buffer[16..18]);
    let c10 = BigEndian::read_u16(&buffer[26..28]);

    Some(PmsMeasurement { c03, c10 })
}
