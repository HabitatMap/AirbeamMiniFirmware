#![allow(dead_code)]

#[path = "../src/ble/ble_protocol.rs"]
mod ble_protocol;

mod storage {
    pub mod session_config {
        include!("../src/storage/session_config.rs");
    }
}

// The protocol only needs these event payloads, not the hardware-specific main module.
enum LoopEvent {
    TimeUpdate(i64),
    Stop {
        start_wifi_sync: bool,
        start_ble_sync: bool,
    },
}

use ble_protocol::AppCommand;
use std::time::Duration;
use storage::session_config::{SessionConfig, SessionType};
use uuid::Uuid;

const SESSION_UUID: Uuid = Uuid::from_u128(0x01234567_89ab_cdef_0123_456789abcdef);
const TOKEN: u128 = 0xfedcba98_7654_3210_fedc_ba9876543210;

fn mobile_config_packet() -> Vec<u8> {
    let mut packet = vec![0; 20];
    packet[0] = 0x13;
    packet[1..17].copy_from_slice(&SESSION_UUID.to_bytes_le());
    packet[17..19].copy_from_slice(&300_u16.to_le_bytes());
    packet[19] = 1;
    packet
}

fn fixed_config_packet() -> Vec<u8> {
    let mut packet = mobile_config_packet();
    packet.resize(134, 0);
    packet[19] = 0;
    packet[20] = 3;
    packet[21] = 7;
    packet[22..38].copy_from_slice(&TOKEN.to_le_bytes());
    packet[38..50].copy_from_slice(b"test-network");
    packet[70..83].copy_from_slice(b"test-password");
    packet
}

fn decode_config(packet: &[u8]) -> SessionConfig {
    match AppCommand::decode(packet) {
        Some(AppCommand::NewSessionConfig(config)) => config,
        command => panic!("Expected session configuration, got {command:?}"),
    }
}

#[test]
fn rejects_every_truncated_mobile_configuration() {
    let packet = mobile_config_packet();
    for length in 0..packet.len() {
        assert!(
            AppCommand::decode(&packet[..length]).is_none(),
            "Accepted a truncated mobile configuration of {length} bytes"
        );
    }
}

#[test]
fn rejects_every_truncated_fixed_configuration() {
    let packet = fixed_config_packet();
    for length in 20..packet.len() {
        assert!(
            AppCommand::decode(&packet[..length]).is_none(),
            "Accepted a truncated fixed configuration of {length} bytes"
        );
    }
}

#[test]
fn decodes_minimum_length_mobile_configuration() {
    let config = decode_config(&mobile_config_packet());
    assert_eq!(config.session_uuid, SESSION_UUID);
    assert_eq!(config.interval, Duration::from_secs(300));
    assert!(matches!(config.session_type, SessionType::MOBILE));
}

#[test]
fn decodes_minimum_length_fixed_configuration() {
    let config = decode_config(&fixed_config_packet());
    assert_eq!(config.session_uuid, SESSION_UUID);
    assert_eq!(config.interval, Duration::from_secs(300));
    match config.session_type {
        SessionType::FIXED {
            pm1_index,
            pm2_5_index,
            token,
            wifi_ssid,
            wifi_password,
        } => {
            assert_eq!(pm1_index, 3);
            assert_eq!(pm2_5_index, 7);
            assert_eq!(token, TOKEN);
            assert_eq!(wifi_ssid, "test-network");
            assert_eq!(wifi_password, "test-password");
        }
        session_type => panic!("Expected fixed session, got {session_type:?}"),
    }
}

#[test]
fn accepts_configurations_with_trailing_bytes() {
    for mut packet in [mobile_config_packet(), fixed_config_packet()] {
        packet.extend_from_slice(&[0xFE, 0xFF]);
        decode_config(&packet);
    }
}
