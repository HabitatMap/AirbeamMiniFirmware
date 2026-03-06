mod ble_protocol;

use std::sync::{Arc, Mutex, Condvar};

use esp32_nimble::{
    enums::{ConnMode, DiscMode},
    uuid128,
    BLEAdvertisementData, BLECharacteristic, BLEDevice, BLEServer, NimbleProperties,
};
use esp32_nimble::enums::AuthReq;
use esp32_nimble::utilities::BleUuid;
use log::{info, warn};
use uuid::Uuid;
use crate::ble::ble_protocol::{AppCommand, DeviceResponse, DeviceStatus, ErrorCode};
use crate::storage::session_config::{SessionConfig, SessionType};
use esp32_nimble::utilities::mutex::Mutex as NimbleMutex;
use esp_idf_svc::sys::ble_sm_sc_oob_data;
use crate::ble::ble_protocol::AppCommand::{ContinueSession, NewSessionConfig};

const SERVICE_UUID: BleUuid = uuid128!("a0e1f000-0001-4b3c-8e9a-1f2d3c4b5a60");
const STATUS_CHAR_UUID: BleUuid = uuid128!("a0e1f000-0002-4b3c-8e9a-1f2d3c4b5a60");
const COMMAND_CHAR_UUID: BleUuid = uuid128!("a0e1f000-0003-4b3c-8e9a-1f2d3c4b5a60");
const RESPONSE_CHAR_UUID: BleUuid = uuid128!("a0e1f000-0004-4b3c-8e9a-1f2d3c4b5a60");

pub enum SetupResult {
    Continue,
    StartNew(SessionConfig),
}

pub struct BleManager {
    // characteristic handles — set once during init, then read-only
    status_chr: Arc<NimbleMutex<BLECharacteristic>>,
    response_chr: Arc<NimbleMutex<BLECharacteristic>>,
    // keep server alive
    _ble_device: &'static BLEDevice,

    cmd_rx: std::sync::mpsc::Receiver<AppCommand>,
    cmd_tx: std::sync::mpsc::Sender<AppCommand>,
}


impl BleManager {
    pub fn new(device_name: &str) -> anyhow::Result<Self> {
        let (cmd_tx, cmd_rx) = std::sync::mpsc::channel();
        // ── 1. Get the NimBLE singleton ──────────────────────────────────
        let ble_device = BLEDevice::take();
        ble_device
            .security()
            .set_auth(AuthReq::Bond)
            .resolve_rpa();

        // ── 2. Set up GATT server ────────────────────────────────────────
        let server = ble_device.get_server();

        // connection / disconnection callback
        server.on_connect(move |server, desc| {
            info!("BLE client connected, conn_handle={}", desc.conn_handle());
            // after connect, update connection params for faster throughput
            //TODO
            // min_interval=6 (7.5ms), max_interval=24 (30ms), latency=0, timeout=200 (2s)
            server.update_conn_params(desc.conn_handle(), 6, 24, 0, 200);
        });

        server.on_disconnect(move |_desc, reason| {
            info!("BLE client disconnected");
            //TODO reconnect attempt if session saved
        });

        // ── 3. Create service + characteristics ──────────────────────────
        let service = server.create_service(SERVICE_UUID);

        // Status: notify-only (device → app on connect)
        let status_chr = service.lock().create_characteristic(
            STATUS_CHAR_UUID,
            NimbleProperties::READ | NimbleProperties::NOTIFY,
        );

        // Command: write (app → device)
        let command_chr = service.lock().create_characteristic(
            COMMAND_CHAR_UUID,
            NimbleProperties::WRITE,
        );
        let cmd_tx_clone = cmd_tx.clone();
        command_chr.lock().on_write(move |args| {
            let data = args.recv_data();
            match AppCommand::decode(data) {
                Some(cmd) => {
                    info!("BLE command received: {:?}", cmd);
                    if let Err(e) = cmd_tx_clone.send(cmd) {
                        warn!("BLE command send failed: {:?}", e);
                    }
                }
                None => {
                    warn!("BLE: unparseable command ({} bytes)", data.len());
                }
            }
        });

        // Response: notify-only (device → app for ack/nack/sync chunks)
        let response_chr = service.lock().create_characteristic(
            RESPONSE_CHAR_UUID,
            NimbleProperties::READ | NimbleProperties::NOTIFY,
        );

        // ── 4. Start advertising ─────────────────────────────────────────
        let advertising = ble_device.get_advertising();
        advertising
            .lock()
            .set_data(
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
            _ble_device: ble_device,
            cmd_rx,
            cmd_tx,
        })
    }

    /// Run the setup handshake. Blocks the calling thread until a config is obtained.
    pub fn run_setup<F, W>(
        &mut self,
        saved_config: Option<SessionConfig>,
        has_measurements: bool,
        clear_storage: F,
        sync_storage: F,
        connect_to_wifi: W,
    ) -> anyhow::Result<SetupResult>
    where F: FnOnce() -> anyhow::Result<()>,
    W: FnOnce(&str, &str) -> anyhow::Result<()>{
        self.wait_for_connection(&saved_config)?;

        // small delay so the client has time to subscribe to notifications
        std::thread::sleep(std::time::Duration::from_millis(300));

        let status = if let Some(config) = saved_config {
           DeviceStatus::HasSavedSession { session: config.session_uuid, has_measurements}
        } else {
            DeviceStatus::Idle
        };

        self.notify_status(&status)?;

        loop {
            // blocks until the app writes to the command characteristic
            let cmd = self.cmd_rx.recv()?;

            // if we disconnected mid-setup, bail out
            if !self.is_connected() {
                anyhow::bail!("BLE disconnected during setup");
            }

            match cmd {
                ContinueSession => {
                    if has_measurements {
                        self.send_response(DeviceResponse::Nack(ErrorCode::StorageHasMeasurements))?;
                    } else {
                        self.send_response(DeviceResponse::Ack)?;
                        return Ok(SetupResult::Continue);
                    }
                },

                DiscardSession => {
                    self.send_response(DeviceResponse::Ack)?;
                    match clear_storage() {
                        Ok(()) => self.send_response(DeviceResponse::Ready)?,
                        Err(e) => self.send_response(DeviceResponse::Nack(ErrorCode::ClearStorageFailed))?,
                    }
                },

                StartSync => {
                    self.send_response(DeviceResponse::Ack)?;
                    match sync_storage() {
                        Ok(()) => self.send_response(DeviceResponse::Ready)?,
                        Err(e) => self.send_response(DeviceResponse::Nack(ErrorCode::ClearStorageFailed))?,
                    }
                },

                NewSessionConfig(config) => {
                    self.send_response(DeviceResponse::Ack)?;
                    if let SessionType::FIXED {
                        pm1_index,
                        pm2_5_index,
                        wifi_ssid,
                        wifi_password
                    } = &config.session_type {
                        match connect_to_wifi(wifi_ssid, wifi_password) {
                            Ok(()) => {
                                self.send_response(DeviceResponse::Ready)?;
                                 return Ok(SetupResult::StartNew(config));
                            },
                            Err(e) => self.send_response(DeviceResponse::Nack(ErrorCode::InvalidConfig))?,
                        }
                    } else {
                        self.send_response(DeviceResponse::Ready)?;
                        return Ok(SetupResult::StartNew(config))
                    }
                }
            }
        }
    }

    /// Restart advertising after a disconnect (call from your reconnect logic)
    pub fn restart_advertising(&self) -> anyhow::Result<()> {
        self._ble_device.get_advertising().lock().start()?;
        info!("BLE re-advertising");
        Ok(())
    }

    fn wait_for_connection(&self, saved_config: &Option<SessionConfig>) -> anyhow::Result<()> {
        //TODO: wait unless WIFI
        Ok(())
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
    fn is_connected(&self) -> bool {
        self._ble_device.get_server().connected_count() > 0
    }
}

