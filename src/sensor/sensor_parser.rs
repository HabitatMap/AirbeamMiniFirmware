use byteorder::{BigEndian, ByteOrder};

#[derive(Debug, Default, Clone)]
pub(crate) struct PmsMeasurement {
    pm1_0_std: u16,
    pm2_5_std: u16,
    pm10_std: u16,
    pub(crate) pm1_0_atm: u16,
    pub(crate) pm2_5_atm: u16,
    pub(crate) pm10_atm: u16,
}

pub fn parse_sensor(buffer: &[u8; 32]) -> Option<PmsMeasurement> {
    // Checksum is the last 2 bytes. It should be equal to the sum of the first 30 bytes.
    let checksum_received = BigEndian::read_u16(&buffer[30..32]);
    let checksum_calculated: u16 = buffer[0..30].iter().map(|&b| b as u16).sum();

    if checksum_received != checksum_calculated {
        return None;
    }

    // Standard Particles (CF=1, standard particle)
    let pm1_0_std = BigEndian::read_u16(&buffer[4..6]);
    let pm2_5_std = BigEndian::read_u16(&buffer[6..8]);
    let pm10_std = BigEndian::read_u16(&buffer[8..10]);

    // Atmospheric Environment (This is usually what you want for air quality)
    let pm1_0_atm = BigEndian::read_u16(&buffer[10..12]);
    let pm2_5_atm = BigEndian::read_u16(&buffer[12..14]);
    let pm10_atm = BigEndian::read_u16(&buffer[14..16]);

    Some(PmsMeasurement {
        pm1_0_std,
        pm2_5_std,
        pm10_std,
        pm1_0_atm,
        pm2_5_atm,
        pm10_atm,
    })
}