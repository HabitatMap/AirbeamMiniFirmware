use crate::sensor::measurement::Measurement;
use crate::storage::session_config::{SessionConfig, SessionType};
use crate::storage::storage_controller::FILE_PATH;
use crate::{LoopEvent, SendingError};
use embedded_svc::http::Method;
use embedded_svc::io::Write;
use embedded_svc::wifi::AccessPointConfiguration;
use embedded_svc::{
    http::client::Client as HttpClient,
    wifi::{AuthMethod, ClientConfiguration, Configuration},
};
use esp32_nimble::utilities::mutex::Mutex;
use esp_idf_svc::http::client::EspHttpConnection;
use esp_idf_svc::sys::{
    esp_err_t, esp_get_free_heap_size, esp_get_minimum_free_heap_size, esp_random,
    heap_caps_get_largest_free_block, http_method_HTTP_GET, httpd_config_t, httpd_handle_t,
    httpd_register_uri_handler, httpd_req_t, httpd_resp_send_chunk, httpd_resp_set_hdr,
    httpd_resp_set_type, httpd_start, httpd_stop, httpd_uri_t, ESP_FAIL, ESP_OK,
    MALLOC_CAP_8BIT, MALLOC_CAP_INTERNAL,
};
use esp_idf_svc::wifi::{BlockingWifi, EspWifi};
use log::{error, info};
use std::ffi::{c_int, c_void, CString};
use std::fs::File;
use std::io::{BufReader, Read};
use std::ptr;
use std::sync::mpsc::{Receiver, Sender};
use uuid::Uuid;

const CHUNK_SIZE: usize = 4096;
const MAGIC: &[u8; 2] = &[0xAB, 0xBA];
// Default 5s send_wait_timeout trips when the phone reads the body slowly under
// BLE/Wi-Fi coex. 30s eats real-world stalls.
const HTTPD_TIMEOUT_SECS: u16 = 30;

pub enum SyncStatus {
    Ready { password: String },
    Syncing,
    Done,
}

struct SyncHandlerCtx {
    file_path: CString,
    tx: Sender<SyncStatus>,
}

struct SyncServer {
    handle: httpd_handle_t,
    ctx: *mut SyncHandlerCtx,
}

impl Drop for SyncServer {
    fn drop(&mut self) {
        unsafe {
            let _ = httpd_stop(self.handle);
            drop(Box::from_raw(self.ctx));
        }
    }
}

pub struct WifiManager {
    wifi: Mutex<BlockingWifi<EspWifi<'static>>>,
    sync_server: Mutex<Option<SyncServer>>,
}

impl WifiManager {
    pub fn new(wifi: BlockingWifi<EspWifi<'static>>) -> Self {
        Self {
            wifi: Mutex::new(wifi),
            sync_server: Mutex::new(None),
        }
    }

    /// Server is parked on `WifiManager` so it outlives BLE disconnects;
    /// caller must invoke [`Self::cancel_manual_sync`] when done.
    ///
    /// Uses the raw esp-idf httpd FFI so the recv/send wait timeouts can be
    /// bumped from the crate-hardcoded 5s — `esp-idf-svc::http::server::Configuration`
    /// does not expose those fields.
    pub fn manual_sync(&self) -> anyhow::Result<Receiver<SyncStatus>> {
        if let Some(mut wifi) = self.wifi.try_lock() {
            let ssid = "AirBeamMini Sync";
            let n = unsafe { esp_random() } % 100_000_000;
            let password = format!("{:08}", n);
            info!("manual_sync: stop prior wifi");
            let _ = wifi.stop(); // if already started
            // Channel 1 is dense in real-world scans (23+ APs observed); pin
            // SoftAP to 11 to dodge the congestion that was retransmitting /sync
            // chunks until the phone TCP gave up at ~14s.
            info!("manual_sync: set AP config");
            wifi.set_configuration(&Configuration::AccessPoint(AccessPointConfiguration {
                ssid: ssid.parse()?,
                password: password.parse()?,
                auth_method: AuthMethod::WPA2Personal,
                channel: 11,
                max_connections: 1,
                ..Default::default()
            }))?;
            info!("manual_sync: wifi.start()");
            wifi.start()?;
            info!("manual_sync: wait_netif_up()");
            wifi.wait_netif_up()?;
            log_heap("before httpd_start");

            let (tx, rx) = std::sync::mpsc::channel();
            let ctx = Box::into_raw(Box::new(SyncHandlerCtx {
                file_path: CString::new(FILE_PATH)?,
                tx: tx.clone(),
            }));

            // 6144B previously panicked because the GET handler had a 4 KiB
            // on-stack chunk buffer (now heap-allocated). 8 KiB leaves
            // comfortable headroom for the trimmed handler. task_caps of 0
            // means xTaskCreatePinnedToCoreWithCaps finds no matching region
            // and returns null -> ESP_ERR_HTTPD_TASK; pass the same caps that
            // esp-idf-svc's safe wrapper uses.
            let config = httpd_config_t {
                task_priority: 5,
                stack_size: 8192,
                core_id: i32::MAX as c_int,
                task_caps: MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT,
                server_port: 80,
                ctrl_port: 32768,
                max_open_sockets: 4,
                max_uri_handlers: 4,
                max_resp_headers: 8,
                backlog_conn: 5,
                lru_purge_enable: true,
                recv_wait_timeout: HTTPD_TIMEOUT_SECS,
                send_wait_timeout: HTTPD_TIMEOUT_SECS,
                ..Default::default()
            };

            let mut handle: httpd_handle_t = ptr::null_mut();
            info!("manual_sync: httpd_start (stack={})", config.stack_size);
            let start_res = unsafe { httpd_start(&mut handle, &config) };
            info!("manual_sync: httpd_start ret={}", start_res);
            if start_res != ESP_OK {
                error!("manual_sync: httpd_start FAILED: {}", start_res);
                log_heap("after httpd_start failure");
                unsafe { drop(Box::from_raw(ctx)) };
                return Err(anyhow::Error::msg(format!(
                    "httpd_start failed: {}",
                    start_res
                )));
            }

            let uri = c"/sync";
            let uri_handler = httpd_uri_t {
                uri: uri.as_ptr(),
                method: http_method_HTTP_GET,
                handler: Some(sync_get_handler),
                user_ctx: ctx as *mut c_void,
            };
            info!("manual_sync: register_uri_handler");
            let reg_res = unsafe { httpd_register_uri_handler(handle, &uri_handler) };
            info!("manual_sync: register_uri_handler ret={}", reg_res);
            if reg_res != ESP_OK {
                error!("manual_sync: register_uri_handler FAILED: {}", reg_res);
                unsafe {
                    let _ = httpd_stop(handle);
                    drop(Box::from_raw(ctx));
                }
                return Err(anyhow::Error::msg(format!(
                    "httpd_register_uri_handler failed: {}",
                    reg_res
                )));
            }
            log_heap("after httpd_start success");

            tx.send(SyncStatus::Ready { password })?;
            *self.sync_server.lock() = Some(SyncServer { handle, ctx });
            Ok(rx)
        } else {
            Err(anyhow::Error::msg("Wifi lock fail"))
        }
    }

    pub fn cancel_manual_sync(&self) {
        // Drop runs httpd_stop and frees the handler ctx box.
        drop(self.sync_server.lock().take());
        if let Some(mut wifi) = self.wifi.try_lock() {
            let _ = wifi.stop();
        }
    }

    pub fn connect(&self, wifi_ssid: &str, wifi_password: &str) -> anyhow::Result<()> {
        if let Some(mut wifi) = self.wifi.try_lock() {
            let _ = wifi.stop(); // if already started
            let wifi_config = Configuration::Client(ClientConfiguration {
                ssid: wifi_ssid.try_into()?,
                bssid: None,
                auth_method: AuthMethod::WPA2Personal,
                password: wifi_password.try_into()?,
                channel: None,
                ..Default::default()
            });
            info!(
                "calling start, connect, wait_netif_up on wifi manager: {:?} ",
                wifi_config
            );
            wifi.set_configuration(&wifi_config)?;
            wifi.start()?;
            wifi.connect()?;
            wifi.wait_netif_up()?;
        }
        Ok(())
    }

    pub fn get_time(
        &self,
        domain: &str,
        token: u128,
        uuid: Uuid,
        event_tx: Sender<LoopEvent>,
    ) -> Result<(), SendingError> {
        if !self.is_connected() {
            return Err(SendingError::ConnectionError);
        }

        use esp_idf_svc::http::client::Configuration as HttpConfiguration;
        let http_config = &HttpConfiguration {
            crt_bundle_attach: Some(esp_idf_svc::sys::esp_crt_bundle_attach),
            ..Default::default()
        };
        let mut client = HttpClient::wrap(
            EspHttpConnection::new(http_config).map_err(|_| SendingError::ConfigError)?,
        );

        let headers = [
            ("Content-Type", "application/octet-stream"),
            ("Authorization", &format!("Bearer {:032x}", token)),
        ];

        let url = format!(
            "https://{}/api/v3/fixed_sessions/{}/measurements",
            domain, uuid
        );
        let mut request = client
            .request(Method::Post, &url, &headers)
            .map_err(|_| SendingError::Retry)?;
        request.flush().map_err(|_| SendingError::Retry)?;
        let mut response = request.submit().map_err(|_| SendingError::Retry)?;

        if let Some(epoch) = response.header("X-Server-Time") {
            if let Ok(epoch) = epoch.parse::<i64>() {
                info!("Server time: {}", epoch);
                event_tx.send(LoopEvent::TimeUpdate(epoch)).unwrap();
            }
        }
        Ok(())
    }

    pub fn send_measurements(
        &self,
        measurements: &[Measurement],
        domain: &str,
        config: SessionConfig,
        event_tx: Sender<LoopEvent>,
    ) -> Result<(), SendingError> {
        if measurements.is_empty() {
            return Ok(());
        }
        if !self.is_connected() {
            return Err(SendingError::ConnectionError);
        }
        let SessionType::FIXED {
            pm1_index,
            pm2_5_index,
            token,
            wifi_ssid: _,
            wifi_password: _,
        } = config.session_type.clone()
        else {
            panic!("Config error, expected fixed session")
        };
        let payload = self.encode_measurements(measurements, pm1_index, pm2_5_index)?;
        use esp_idf_svc::http::client::Configuration as HttpConfiguration;

        let http_config = &HttpConfiguration {
            crt_bundle_attach: Some(esp_idf_svc::sys::esp_crt_bundle_attach),
            ..Default::default()
        };
        let mut client = HttpClient::wrap(
            EspHttpConnection::new(http_config).map_err(|_| SendingError::ConfigError)?,
        );
        let content_len_header = format!("{}", payload.len());

        let headers = [
            ("Content-Type", "application/octet-stream"),
            ("Content-Length", &content_len_header),
            ("Authorization", &format!("Bearer {:032x}", token)),
        ];

        let url = format!(
            "https://{}/api/v3/fixed_sessions/{}/measurements",
            domain, config.session_uuid
        );
        let mut request = client
            .request(Method::Post, &url, &headers)
            .map_err(|_| SendingError::Retry)?;
        request
            .write_all(&payload)
            .map_err(|_| SendingError::Overflow)?;
        request.flush().map_err(|_| SendingError::Retry)?;
        let mut response = request.submit().map_err(|_| SendingError::Retry)?;
        let status = response.status();

        if let Some(epoch) = response.header("X-Server-Time") {
            if let Ok(epoch) = epoch.parse::<i64>() {
                event_tx.send(LoopEvent::TimeUpdate(epoch)).unwrap();
            }
        }

        info!(
            "POST measurements → {status}, sent {} records ({} bytes)",
            measurements.len(),
            payload.len()
        );

        if !(200..300).contains(&(status as i32)) {
            return Err(SendingError::ConfigError);
        }

        Ok(())
    }

    pub fn disconnect(&mut self) {
        if let Some(mut wifi) = self.wifi.try_lock() {
            let _ = wifi.disconnect();
            let _ = wifi.stop();
        }
    }

    pub fn is_connected(&self) -> bool {
        if let Some(mut wifi) = self.wifi.try_lock() {
            wifi.is_connected().unwrap_or(false)
        } else {
            false
        }
    }

    fn encode_measurements(
        &self,
        measurements: &[Measurement],
        pm1_index: u8,
        pm2_5_index: u8,
    ) -> Result<Vec<u8>, SendingError> {
        let count = measurements.len();

        if count > u16::MAX as usize {
            return Err(SendingError::Overflow);
        }
        let count = (count * 2) as u16;

        // 0xAB + 0xBA + u16 + N * 2 * (u32 + u8 + float) + u8
        let capacity = 2 + 2 + count * 18 + 1;
        let mut buffer = Vec::with_capacity(capacity as usize);
        buffer.extend_from_slice(MAGIC);
        buffer.extend_from_slice(&count.to_be_bytes());
        for m in measurements {
            buffer.extend_from_slice(&m.timestamp.to_be_bytes());
            buffer.push(pm1_index);
            buffer.extend_from_slice(&f32::from(m.pm1_0_avg).to_be_bytes());
            buffer.extend_from_slice(&m.timestamp.to_be_bytes());
            buffer.push(pm2_5_index);
            buffer.extend_from_slice(&f32::from(m.pm2_5_avg).to_be_bytes());
        }
        let checksum = buffer.iter().fold(0u8, |acc, &b| acc ^ b);
        buffer.push(checksum);
        Ok(buffer)
    }
}

#[derive(Debug)]
pub enum WifiError {
    NotConnected,
    Config,
    NotStarted,
    LockError,
    Other,
}

fn log_heap(tag: &str) {
    let (free, min_free, largest_internal) = unsafe {
        (
            esp_get_free_heap_size(),
            esp_get_minimum_free_heap_size(),
            heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
        )
    };
    info!(
        "heap[{}]: free={} min_free={} largest_internal_8bit={}",
        tag, free, min_free, largest_internal
    );
}

extern "C" fn sync_get_handler(req: *mut httpd_req_t) -> esp_err_t {
    info!("sync_get: entered, opening file");
    let ctx_ptr = unsafe { (*req).user_ctx as *const SyncHandlerCtx };
    if ctx_ptr.is_null() {
        error!("sync_get: null user_ctx");
        return ESP_FAIL;
    }
    let ctx = unsafe { &*ctx_ptr };

    let path = match ctx.file_path.to_str() {
        Ok(p) => p,
        Err(_) => {
            error!("sync_get: file_path not utf8");
            return ESP_FAIL;
        }
    };
    let file = match File::open(path) {
        Ok(f) => f,
        Err(e) => {
            error!("sync_get: file open failed: {:?}", e);
            return ESP_FAIL;
        }
    };
    let file_size = match file.metadata() {
        Ok(m) => m.len(),
        Err(e) => {
            error!("sync_get: metadata failed: {:?}", e);
            return ESP_FAIL;
        }
    };
    info!("sync_get: file_size = {}", file_size);
    let size_str = match CString::new(file_size.to_string()) {
        Ok(s) => s,
        Err(_) => return ESP_FAIL,
    };

    // Receiver may be gone (notify_status failure post-BLE-drop); must not
    // abort the GET handler before the body goes out.
    let _ = ctx.tx.send(SyncStatus::Syncing);

    unsafe {
        if httpd_resp_set_type(req, c"application/octet-stream".as_ptr()) != ESP_OK {
            error!("sync_get: set_type failed");
            return ESP_FAIL;
        }
        if httpd_resp_set_hdr(req, c"Content-Length".as_ptr(), size_str.as_ptr()) != ESP_OK {
            error!("sync_get: set_hdr Content-Length failed");
            return ESP_FAIL;
        }
        if httpd_resp_set_hdr(
            req,
            c"Content-Disposition".as_ptr(),
            c"attachment".as_ptr(),
        ) != ESP_OK
        {
            error!("sync_get: set_hdr Content-Disposition failed");
            return ESP_FAIL;
        }
    }

    let mut reader = BufReader::with_capacity(CHUNK_SIZE, file);
    // Heap-allocate the chunk buffer — a 4 KiB stack array on top of httpd's
    // internal frames was overflowing the task stack.
    let mut buf = vec![0u8; CHUNK_SIZE];
    let mut sent_total: u64 = 0;
    let mut chunk_idx: u32 = 0;
    loop {
        let n = match reader.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => n,
            Err(e) => {
                error!("sync_get: read err: {:?}", e);
                return ESP_FAIL;
            }
        };
        info!(
            "sync_get: sending chunk #{} n={} sent_so_far={}",
            chunk_idx, n, sent_total
        );
        let res = unsafe { httpd_resp_send_chunk(req, buf.as_ptr() as *const _, n as isize) };
        if res != ESP_OK {
            error!(
                "sync_get: send_chunk err: {} at chunk #{} sent_so_far={}",
                res, chunk_idx, sent_total
            );
            return res;
        }
        sent_total += n as u64;
        chunk_idx += 1;
    }
    let res = unsafe { httpd_resp_send_chunk(req, ptr::null(), 0) };
    if res != ESP_OK {
        error!("sync_get: terminator send_chunk err: {}", res);
        return res;
    }
    info!(
        "sync_get: end-of-stream sent, returning OK ({} bytes in {} chunks)",
        sent_total, chunk_idx
    );
    let _ = ctx.tx.send(SyncStatus::Done);
    ESP_OK
}
