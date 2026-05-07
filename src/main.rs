mod aggregator;
mod autosync;
mod battery;
mod ble;
mod led;
mod sensor;
mod storage;
mod wifi;

use crate::autosync::sync_from_storage;
use crate::battery::BatteryMonitor;
use crate::ble::ble_protocol::{DeviceResponse, DeviceStatus, ErrorCode};
use crate::ble::SetupResult;
use crate::led::led_thread::{start_led_thread, Color, LedCommand, LedPins};
use crate::sensor::measurement::Measurement;
use crate::sensor::sensor_thread::SensorDriver;
use crate::storage::nvs_manager::NvsManager;
use crate::storage::session_config::SessionType;
use crate::storage::storage_controller::{StorageManager, MOUNT_POINT};
use crate::wifi::wifi_manager::{SyncStatus, WifiManager};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::fs::littlefs::Littlefs;
use esp_idf_svc::hal::adc::attenuation::DB_12;
use esp_idf_svc::hal::adc::oneshot::config::{AdcChannelConfig, Calibration};
use esp_idf_svc::hal::adc::oneshot::{AdcChannelDriver, AdcDriver};
use esp_idf_svc::hal::adc::Resolution;
use esp_idf_svc::hal::gpio;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::uart::config::{DataBits, StopBits};
use esp_idf_svc::hal::uart::{UartConfig, UartDriver};
use esp_idf_svc::hal::units::Hertz;
use esp_idf_svc::io::vfs::MountedLittlefs;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::sys::{esp, esp_pm_config_t, esp_pm_configure, settimeofday, timeval};
use esp_idf_svc::wifi::{BlockingWifi, EspWifi};
use log::{error, info};
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("Starting up!");

    let peripherals = Peripherals::take()?;
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    let mut mac = [0u8; 6];
    unsafe {
        esp_idf_svc::sys::esp_read_mac(
            mac.as_mut_ptr(),
            esp_idf_svc::sys::esp_mac_type_t_ESP_MAC_BT,
        );
    }
    let mac_str = format!(
        "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
    );
    let lfs = unsafe { Littlefs::<()>::new_partition("storage") }?;
    let mounted = MountedLittlefs::mount(lfs, MOUNT_POINT).unwrap_or_else(|e| {
        log::error!("Formatting storage, Failed to mount filesystem: {:?}", e);
        let mut lfs = unsafe { Littlefs::<()>::new_partition("storage") }.unwrap();
        let _ = lfs.format();
        MountedLittlefs::mount(lfs, MOUNT_POINT).unwrap()
    }); //this MUST be kept alive, otherwise filesystem will unmount
    info!("Filesystem info: {:?}", mounted.info());
    let led_pins = LedPins {
        timer: peripherals.ledc.timer0,
        channel_r: peripherals.ledc.channel0,
        channel_g: peripherals.ledc.channel1,
        channel_b: peripherals.ledc.channel2,
        pin_r: peripherals.pins.gpio2,
        pin_g: peripherals.pins.gpio1,
        pin_b: peripherals.pins.gpio0,
    };

    let tx_pin = peripherals.pins.gpio7; // TX on ESP32-C3 (Connects to RX on Sensor)
    let rx_pin = peripherals.pins.gpio6; // RX on ESP32-C3 (Connects to TX on Sensor)

    // ADC driver + channel live here in main (can't be in the same struct)
    let adc = AdcDriver::new(peripherals.adc1)?;
    let adc_config = AdcChannelConfig {
        attenuation: DB_12,
        calibration: Calibration::Curve,
        resolution: Resolution::Resolution12Bit,
    };
    let mut vbat_pin = AdcChannelDriver::new(&adc, peripherals.pins.gpio3, &adc_config)?;

    // Battery monitor owns the USB sense pin
    let mut batt = BatteryMonitor::new(peripherals.pins.gpio4)?;

    let config = UartConfig::new()
        .baudrate(Hertz(9600))
        .data_bits(DataBits::DataBits8)
        .stop_bits(StopBits::STOP1);

    let uart = UartDriver::new(
        peripherals.uart1,
        tx_pin,
        rx_pin,
        Option::<gpio::Gpio0>::None, // CTS
        Option::<gpio::Gpio1>::None, // RTS
        &config,
    )?;

    let (event_tx, event_rx) = mpsc::channel();
    let sensor = SensorDriver::new(uart);
    let led_command = start_led_thread(led_pins)?;
    let mut storage = StorageManager::new();
    let mut nvs_manager = NvsManager::new(nvs.clone())?;
    let name = format!("AirBeamMini:{}", mac_str);
    let mut ble = ble::BleManager::new(name.as_str(), event_tx.clone())?;
    let esp_wifi = EspWifi::new(peripherals.modem.split().0, sys_loop.clone(), Some(nvs))?;
    let blocking = BlockingWifi::wrap(esp_wifi, sys_loop)?;
    let wifi_manager = WifiManager::new(blocking);

    unsafe {
        let pm = esp_pm_config_t {
            max_freq_mhz: 160,
            min_freq_mhz: 40,
            light_sleep_enable: true,
        };
        esp!(esp_pm_configure(
            &pm as *const _ as *const core::ffi::c_void
        ))?;
    }

    info!(
        "Startup complete!, AirBeamMini MAC: {}, Version: {}",
        mac_str,
        env!("CARGO_PKG_VERSION")
    );

    loop {
        let config = nvs_manager.get_session_config().unwrap_or_else(|e| {
            nvs_manager.clear_session_config();
            error!("Failed to get session config: {:?}", e);
            None
        });

        let result = ble.run_setup(
            config,
            storage.has_measurements(),
            storage.get_file_size().unwrap_or(1),
            || batt.read(&adc, &mut vbat_pin).signed_percent,
            || storage.clear_measurements(),
            || wifi_manager.manual_sync(),
            || wifi_manager.cancel_manual_sync(),
            |ssid, password| wifi_manager.connect(ssid, password),
        )?;
        info!("BLE setup result: {:?}", result);

        let config = if let SetupResult::StartNew(config) = &result {
            nvs_manager.set_session_config(&config)?;
            config
        } else {
            &nvs_manager.get_session_config()?.unwrap()
        };

        storage.set_aggregator(config.interval);
        let domain = nvs_manager.get_domain()?;

        let mut send_measurement = |m: Measurement| -> Result<(), SendingError> {
            match &config.session_type {
                SessionType::MOBILE => ble.send_measurement(
                    &m,
                    batt.read(&adc, &mut vbat_pin).signed_percent,
                    config.session_uuid,
                ),
                _ => wifi_manager.send_measurements(
                    &[m],
                    domain.as_str(),
                    config.clone(),
                    event_tx.clone(),
                ),
            }
        };

        let send_measurements = |measurements: &[Measurement]| -> Result<(), SendingError> {
            match &config.session_type {
                SessionType::MOBILE => ble.send_measurements(&measurements),
                _ => wifi_manager.send_measurements(
                    &measurements,
                    domain.as_str(),
                    config.clone(),
                    event_tx.clone(),
                ),
            }
        };

        let connected = || match &config.session_type {
            SessionType::MOBILE => ble.is_connected(),
            SessionType::FIXED { .. } => wifi_manager.is_connected(),
        };

        while event_rx.try_recv().is_ok() {
            //Drop set time from setup
            thread::sleep(Duration::from_millis(10));
        }

        if let SessionType::FIXED { token, .. } = config.session_type {
            if connected() {
                let _ = wifi_manager.get_time(
                    domain.as_str(),
                    token,
                    config.session_uuid,
                    event_tx.clone(),
                );
            }
        }

        let stop_tx = sensor.start_sensor_task(config.interval, event_tx.clone());
        let mut last_time_update = SystemTime::now().duration_since(UNIX_EPOCH)?.as_secs() as i64;

        let mut on_wifi_error = if let SessionType::FIXED { .. } = config.session_type {
            if let SetupResult::Continue = result {
                None
            } else {
                Some(|| {
                    if ble.is_connected() {
                        let _ = ble.send_response(DeviceResponse::Nack(ErrorCode::InvalidConfig));
                    };
                })
            }
        } else {
            None
        };

        if let SessionType::MOBILE = config.session_type {
            wifi_manager.disconnect();
        }

        loop {
            let event = event_rx.recv_timeout(Duration::from_millis(100));
            if let Ok(event) = event {
                match event {
                    LoopEvent::Measurement(m) => {
                        let notify = on_wifi_error.take();
                        info!("Got measurement: {:?}", m);
                        if send_measurement(m).is_err() {
                            if let Some(f) = notify {
                                f();
                                break;
                            }
                            let _ = storage.save_measurement(m);
                        } else {
                            if ble.is_connected() {
                                let _ = ble.send_response(DeviceResponse::Ready);
                                if let SessionType::FIXED { .. } = config.session_type {
                                    ble.stop()
                                }
                            }
                        }
                    }

                    LoopEvent::TimeUpdate(time_epoch) => {
                        let tv = timeval {
                            tv_sec: time_epoch,
                            tv_usec: 0,
                        };
                        let now = SystemTime::now().duration_since(UNIX_EPOCH)?.as_secs() as i64;
                        if now != time_epoch && time_epoch - last_time_update >= 60 {
                            unsafe { settimeofday(&tv, std::ptr::null()) };
                            last_time_update = time_epoch;
                            info!("Set time to {}", time_epoch);
                        }
                    }

                    LoopEvent::Stop { start_sync } => {
                        let _ = stop_tx.send(());
                        if start_sync {
                            let sync_status = wifi_manager.manual_sync()?;
                            loop {
                                match sync_status.recv()? {
                                    SyncStatus::Ready { password } => {
                                        let file_size = storage.get_file_size().unwrap_or(1);
                                        let _ = ble.notify_status(&DeviceStatus::ReadyToSync {
                                            file_size,
                                            password,
                                        });
                                    }
                                    SyncStatus::Done => break,
                                    SyncStatus::Syncing => {}
                                }
                            }
                            wifi_manager.cancel_manual_sync();
                            if ble.is_connected() {
                                let _ = ble.send_response(DeviceResponse::Ready);
                            }
                        }
                        info!("Stopping");
                        let _ = storage.clear_measurements();
                        nvs_manager.clear_session_config();
                        break;
                    }
                }
            }

            if storage.has_measurements() && connected() {
                if let Err(_) = sync_from_storage(&config, &storage, |m| send_measurements(m)) {
                    error!("Failed to sync");
                }
            }
        }
    }
}

#[derive(Debug)]
pub enum LoopEvent {
    TimeUpdate(i64),
    Measurement(Measurement),
    Stop { start_sync: bool },
}

#[derive(Debug)]
enum SendingError {
    ConfigError,
    ConnectionError,
    Retry,
    Overflow,
}
