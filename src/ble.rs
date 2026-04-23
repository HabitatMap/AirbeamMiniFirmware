mod ble_protocol;

use crate::ble::ble_protocol::{AppCommand, DeviceResponse, DeviceStatus, ErrorCode};
use crate::storage::session_config::{SessionConfig, SessionType};
use crate::{LoopEvent, SendingError};
use esp32_nimble::enums::AuthReq;
use esp32_nimble::utilities::mutex::Mutex as NimbleMutex;
use esp32_nimble::utilities::BleUuid;
use esp32_nimble::{
    enums::{ConnMode, DiscMode},
    uuid128, BLEAdvertisementData, BLECharacteristic, BLEDevice, BLEServer, NimbleProperties,
    NotifyTxStatus,
};
use esp_idf_svc::sys::{settimeofday, timeval};
use log::{info, warn};
use std::sync::mpsc::Sender;
use std::sync::Arc;
use std::time::Duration;
use uuid::Uuid;
use crate::sensor::measurement::Measurement;

const SERVICE_UUID: BleUuid = uuid128!("a0e1f000-0001-4b3c-8e9a-1f2d3c4b5a60");
const STATUS_CHAR_UUID: BleUuid = uuid128!("a0e1f000-0002-4b3c-8e9a-1f2d3c4b5a60");
const COMMAND_CHAR_UUID: BleUuid = uuid128!("a0e1f000-0003-4b3c-8e9a-1f2d3c4b5a60");
const RESPONSE_CHAR_UUID: BleUuid = uuid128!("a0e1f000-0004-4b3c-8e9a-1f2d3c4b5a60");
const MEASUREMENT_CHAR_UUID: BleUuid = uuid128!("a0e1f000-0005-4b3c-8e9a-1f2d3c4b5a60");
const SYNC_CHAR_UUID: BleUuid = uuid128!("a0e1f000-0006-4b3c-8e9a-1f2d3c4b5a60");
const FIXED_SESSION_TIMEOUT: Duration = Duration::from_secs(120);
#[derive(Debug)]
pub enum SetupResult {
    Continue,
    StartNew(SessionConfig),
}

pub struct BleManager {
    // characteristic handles — set once during init, then read-only
    status_chr: Arc<NimbleMutex<BLECharacteristic>>,
    response_chr: Arc<NimbleMutex<BLECharacteristic>>,
    measurement_chr: Arc<NimbleMutex<BLECharacteristic>>,
    sync_chr: Arc<NimbleMutex<BLECharacteristic>>,
    //channels for BLE data
    notify_status: std::sync::mpsc::Receiver<NotifyTxStatus>,
    cmd_rx: std::sync::mpsc::Receiver<AppCommand>,
    cmd_tx: std::sync::mpsc::Sender<AppCommand>,
    // keep server alive
    _ble_device: &'static BLEDevice,
}

impl BleManager {
    pub fn new(device_name: &str, event_tx: Sender<LoopEvent>) -> anyhow::Result<Self> {
        let (cmd_tx, cmd_rx) = std::sync::mpsc::channel();
        let (notify_status_tx, notify_status_rx) = std::sync::mpsc::channel();
        // ── 1. Get the NimBLE singleton ──────────────────────────────────
        let ble_device = BLEDevice::take();
        ble_device.security().set_auth(AuthReq::Bond).resolve_rpa();

        // ── 2. Set up GATT server ────────────────────────────────────────
        let server = ble_device.get_server();

        // connection / disconnection callback
        server.on_connect(move |server, desc| {
            info!("BLE client connected, conn_handle={}", desc.conn_handle());
            let _ = server.update_conn_params(desc.conn_handle(), 6, 24, 0, 200);
        });

        server.on_disconnect(move |_desc, reason| {
            info!("BLE client disconnected");
        });

        // ── 3. Create service + characteristics ──────────────────────────
        let service = server.create_service(SERVICE_UUID);

        // Status: notify-only (device → app on connect)
        let status_chr = service.lock().create_characteristic(
            STATUS_CHAR_UUID,
            NimbleProperties::READ | NimbleProperties::NOTIFY,
        );

        // Command: write (app → device)
        let command_chr = service
            .lock()
            .create_characteristic(COMMAND_CHAR_UUID, NimbleProperties::WRITE);
        let cmd_tx_clone = cmd_tx.clone();
        command_chr.lock().on_write(move |args| {
            let data = args.recv_data();
            match AppCommand::decode(data) {
                Some(cmd) => {
                    info!("BLE command received: {:?}", cmd);
                    if let Err(e) = cmd_tx_clone.send(cmd.clone()) {
                        warn!("BLE command send failed: {:?}", e);
                    }
                    if let Some(event) = cmd.as_loop_event() {
                        let _ = event_tx.send(event);
                    }
                }
                None => {
                    warn!("BLE: unparseable command ({} bytes)", data.len());
                }
            }
        });

        let measurement_chr = service
            .lock()
            .create_characteristic(MEASUREMENT_CHAR_UUID, NimbleProperties::INDICATE);
        let clone_notify_status_tx = notify_status_tx.clone();
        measurement_chr.lock().on_notify_tx(move |tx| {
            let status = tx.status();
            let _ = clone_notify_status_tx.send(status);
        });

        let sync_chr = service
            .lock()
            .create_characteristic(SYNC_CHAR_UUID, NimbleProperties::INDICATE);
        let clone_notify_status_tx = notify_status_tx.clone();
        sync_chr.lock().on_notify_tx(move |tx| {
            let status = tx.status();
            let _ = clone_notify_status_tx.send(status);
        });

        // Response: notify-only (device → app for ack/nack/sync chunks)
        let response_chr = service.lock().create_characteristic(
            RESPONSE_CHAR_UUID,
            NimbleProperties::READ | NimbleProperties::NOTIFY,
        );

        // ── 4. Start advertising ─────────────────────────────────────────
        let advertising = ble_device.get_advertising();
        advertising.lock().set_data(
            BLEAdvertisementData::new()
                .name(device_name)
                .add_service_uuid(SERVICE_UUID),
        )?;

        advertising
            .lock()
            .advertisement_type(ConnMode::Und)
            .disc_mode(DiscMode::Gen);

        advertising.lock().start()?;
        info!("BLE advertising started as '{}'", device_name);

        Ok(Self {
            status_chr,
            response_chr,
            measurement_chr,
            sync_chr,
            notify_status: notify_status_rx,
            cmd_rx,
            cmd_tx,
            _ble_device: ble_device,
        })
    }

    /// Run the setup handshake. Blocks the calling thread until a config is obtained.
    pub fn run_setup<F0, F1, F2, W>(
        &mut self,
        saved_config: Option<SessionConfig>,
        has_measurements: bool,
        mut battery_level: F0,
        clear_storage: F1,
        sync_storage: F2,
        connect_to_wifi: W,
    ) -> anyhow::Result<SetupResult>
    where
        F0: FnMut() -> i8,
        F1: Fn() -> anyhow::Result<()>,
        F2: Fn() -> anyhow::Result<()>,
        W: Fn(&str, &str) -> anyhow::Result<()>,
    {
        self.wait_for_connection(Self::get_timeout(saved_config.clone()))?;
        // small delay so the client has time to subscribe to notifications
        std::thread::sleep(std::time::Duration::from_millis(300));

        let status = if let Some(config) = saved_config.clone() {
            DeviceStatus::HasSavedSession {
                battery_level: battery_level(),
                session: config.session_uuid,
                has_measurements,
            }
        } else {
            DeviceStatus::Idle(battery_level())
        };

        self.notify_status(&status)?;

        loop {
            // blocks until the app writes to the command characteristic
            let cmd = self.cmd_rx.recv()?;

            match cmd {
                AppCommand::ContinueSession => {
                    if has_measurements {
                        self.send_response(DeviceResponse::Nack(
                            ErrorCode::StorageHasMeasurements,
                        ))?;
                    } else {
                        match saved_config {
                            Some(_) => {
                                self.send_response(DeviceResponse::Ack)?;
                                return Ok(SetupResult::Continue);
                            }
                            None => {
                                self.send_response(DeviceResponse::Nack(ErrorCode::NoSession))?;
                            }
                        }
                    }
                }

                AppCommand::DiscardSession => {
                    self.send_response(DeviceResponse::Ack)?;
                    match clear_storage() {
                        Ok(()) => self.send_response(DeviceResponse::Ready)?,
                        Err(e) => {
                            self.send_response(DeviceResponse::Nack(ErrorCode::ClearStorageFailed))?
                        }
                    }
                }

                AppCommand::StartSync => {
                    self.send_response(DeviceResponse::Ack)?;
                    match sync_storage() {
                        Ok(()) => self.send_response(DeviceResponse::Ready)?,
                        Err(e) => {
                            self.send_response(DeviceResponse::Nack(ErrorCode::ClearStorageFailed))?
                        }
                    }
                }

                AppCommand::NewSessionConfig(config) => {
                    self.send_response(DeviceResponse::Ack)?;
                    if let SessionType::FIXED {
                        pm1_index: _,
                        pm2_5_index: _,
                        token,
                        wifi_ssid,
                        wifi_password,
                    } = &config.session_type
                    {
                        match connect_to_wifi(wifi_ssid, wifi_password) {
                            Ok(()) => {
                                return Ok(SetupResult::StartNew(config));
                            }
                            Err(_) => {
                                self.send_response(DeviceResponse::Nack(ErrorCode::InvalidConfig))?
                            }
                        }
                    } else {
                        self.send_response(DeviceResponse::Ready)?;
                        return Ok(SetupResult::StartNew(config));
                    }
                }
                AppCommand::GetSensors => {
                    self.send_response(DeviceResponse::SensorInfo)?;
                    info!("BLE: Return sensors");
                }
                AppCommand::SetTime(time_epoch) => {
                    let tv = timeval {
                        tv_sec: time_epoch,
                        tv_usec: 0,
                    };
                    unsafe { settimeofday(&tv, std::ptr::null()) };
                    info!("BLE: Set time to {}", time_epoch);
                }
            }
        }
    }

    pub fn send_measurement(
        &self,
        measurement: &Measurement,
        battery_level: i8,
        session: Uuid,
    ) -> Result<(), SendingError> {
        if !self.is_connected() {
            return Err(SendingError::ConnectionError);
        }

        let mut buf = [0u8; 9];
        buf[0] = 1_u8;
        buf[1..5].copy_from_slice(measurement.timestamp.to_le_bytes().as_slice());
        buf[5..7].copy_from_slice(measurement.pm1_0_avg.to_le_bytes().as_slice());
        buf[7..9].copy_from_slice(measurement.pm2_5_avg.to_le_bytes().as_slice());

        match self.indicate_measurement_chr(&buf, false) {
            Ok(()) => {
                let mut status = [0u8; 18];
                DeviceStatus::Running {
                    battery_level,
                    session,
                }
                .encode(&mut status);
                self.status_chr.lock().set_value(&status).notify();
                Ok(())
            }
            Err(e) => Err(e),
        }
    }

    pub fn send_measurements(
        &self,
        measurements: &[Measurement],
    ) -> Result<(), SendingError> {
        let mut buf = [0u8; 244];
        let count = measurements.len() as u8;
        buf[0] = count;
        for (i, measurement) in measurements.iter().enumerate() {
            let offset = 3 + i * 8;
            if offset + 8 > buf.len() {
                return Err(SendingError::Overflow);
            }
            buf[offset..offset + 4].copy_from_slice(measurement.timestamp.to_le_bytes().as_slice());
            buf[offset + 4..offset + 6].copy_from_slice(measurement.pm1_0_avg.to_le_bytes().as_slice());
            buf[offset + 6..offset + 8].copy_from_slice(measurement.pm2_5_avg.to_le_bytes().as_slice());
        }
        self.indicate_measurement_chr(&buf, true)
    }

    fn indicate_measurement_chr(&self, buf: &[u8], is_sync: bool) -> Result<(), SendingError> {
        while self.notify_status.try_recv().is_ok() {} //empty notify chanel in case of old status

        if is_sync {
            self.sync_chr.lock().set_value(buf).notify();
        } else {
            self.measurement_chr.lock().set_value(buf).notify();
        }

        if let Ok(status) = self.notify_status.recv_timeout(Duration::from_secs(1)) {
            match status {
                NotifyTxStatus::SuccessIndicate => Ok(()),
                NotifyTxStatus::ErrorNoClient => Err(SendingError::ConnectionError),
                NotifyTxStatus::ErrorIndicateTimeout => Err(SendingError::Retry),
                NotifyTxStatus::ErrorIndicateDisabled | NotifyTxStatus::ErrorGatt => {
                    Err(SendingError::ConfigError)
                }
                _ => Err(SendingError::Retry),
            }
        } else {
            Err(SendingError::Retry)
        }
    }

    /// Restart advertising after a disconnect
    pub fn restart_advertising(&self) -> anyhow::Result<()> {
        self._ble_device.get_advertising().lock().start()?;
        info!("BLE re-advertising");
        Ok(())
    }
    /// returns true if we connected, false if we timed out
    fn wait_for_connection(&self, timeout: Option<Duration>) -> anyhow::Result<bool> {
        let server = self._ble_device.get_server();
        let start = std::time::Instant::now();

        loop {
            if server.connected_count() > 0 {
                log::info!("BLE client connected");
                return Ok(true);
            }

            if let Some(t) = timeout {
                if start.elapsed() >= t {
                    return Ok(false);
                }
            }

            std::thread::sleep(Duration::from_millis(100));
        }
    }

    fn get_timeout(session_config: Option<SessionConfig>) -> Option<Duration> {
        if let Some(config) = session_config {
            if matches!(config.session_type, SessionType::FIXED { .. }) {
                Some(FIXED_SESSION_TIMEOUT)
            } else {
                None
            }
        } else {
            None
        }
    }

    fn notify_status(&self, status: &DeviceStatus) -> anyhow::Result<()> {
        let mut buf = [0u8; 20];
        let len = status.encode(&mut buf);
        self.status_chr.lock().set_value(&buf[..len]).notify();
        Ok(())
    }

    fn send_response(&self, resp: DeviceResponse) -> anyhow::Result<()> {
        let mut buf = [0u8; 244]; // max we'll send in one notification
        let len = resp.encode(&mut buf);
        self.response_chr.lock().set_value(&buf[..len]).notify();
        Ok(())
    }
    pub fn is_connected(&self) -> bool {
        self._ble_device.get_server().connected_count() > 0
    }
}
