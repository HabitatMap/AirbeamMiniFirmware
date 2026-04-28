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
use crate::ble::ble_protocol::{DeviceResponse, ErrorCode};
use crate::ble::SetupResult;
use crate::led::led_thread::{start_led_thread, Color, LedCommand, LedPins};
use crate::sensor::measurement::Measurement;
use crate::sensor::sensor_thread::SensorDriver;
use crate::storage::nvs_manager::NvsManager;
use crate::storage::session_config::{SessionConfig, SessionType};
use crate::storage::storage_controller::{StorageManager, MOUNT_POINT};
use crate::wifi::wifi_manager::WifiManager;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::fs::littlefs::Littlefs;
use esp_idf_svc::hal::adc::attenuation::DB_12;
use esp_idf_svc::hal::adc::oneshot::config::{AdcChannelConfig, Calibration};
use esp_idf_svc::hal::adc::oneshot::{AdcChannelDriver, AdcDriver};
use esp_idf_svc::hal::gpio;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::uart::config::{DataBits, StopBits};
use esp_idf_svc::hal::uart::{UartConfig, UartDriver};
use esp_idf_svc::hal::units::Hertz;
use esp_idf_svc::io::vfs::MountedLittlefs;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::sys::{settimeofday, timeval};
use esp_idf_svc::wifi::{BlockingWifi, EspWifi};
use log::{error, info, warn};
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use uuid::Uuid;

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
        calibration: Calibration::None,
        ..Default::default()
    };
    let mut vbat_pin = AdcChannelDriver::new(&adc, peripherals.pins.gpio3, &adc_config)?;

    // Battery monitor owns the USB sense pin
    let batt = BatteryMonitor::new(peripherals.pins.gpio4)?;

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

    loop {
        let config = nvs_manager.get_session_config().unwrap_or_else(|e| {
            nvs_manager.clear_session_config();
            error!("Failed to get session config: {:?}", e);
            None
        });

        let result = ble.run_setup(
            config,
            storage.has_measurements(),
            || batt.read(&adc, &mut vbat_pin).signed_percent,
            || storage.clear_measurements(),
            || {
                info!("TODO: sync storage");
                Ok(())
            },
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
                            //TODO: wifi sync
                        }
                        info!("Stopping");
                        let _ = storage.clear_measurements();
                        nvs_manager.clear_session_config();
                        break;
                    }
                }
            }

            if storage.has_measurements() && connected() {
                if let Err(e) = sync_from_storage(&config, &storage, |m| send_measurements(m)) {
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
