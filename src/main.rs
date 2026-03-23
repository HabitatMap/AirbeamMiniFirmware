mod ble;
mod led;
mod sensor;
mod storage;

use crate::ble::SetupResult;
use crate::led::led_thread::{start_led_thread, Color, LedCommand, LedPins};
use crate::sensor::sensor_thread::{Measurement, SensorDriver};
use crate::storage::nvs_manager::NvsManager;
use crate::storage::session_config::SessionType;
use crate::storage::storage_controller::{MeasurementRecord, StorageManager, MOUNT_POINT};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::fs::littlefs::Littlefs;
use esp_idf_svc::hal::gpio;
use esp_idf_svc::hal::prelude::*;
use esp_idf_svc::hal::uart::config::{DataBits, StopBits};
use esp_idf_svc::hal::uart::{UartConfig, UartDriver};
use esp_idf_svc::io::vfs::MountedLittlefs;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use log::{error, info};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("Starting up!");

    let peripherals = Peripherals::take()?;
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

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

    let sensor = SensorDriver::new(uart);
    sensor.sleep(); //no need for sensor to be running during setup process
    let led_command = start_led_thread(led_pins)?;
    let storage = StorageManager::new();
    let mut nvs_manager = NvsManager::new(nvs)?;
    let mut ble = ble::BleManager::new("AirBeamMini2")?;

    let config = nvs_manager.get_session_config().unwrap_or_else(|e| {
        nvs_manager.clear_all();
        error!("Failed to get session config: {:?}", e);
        None
    });

    let result = ble.run_setup(
        config,
        storage.has_measurements(),
        102_u8, //TODO: get battery level
        || storage.clear_measurements(),
        || {
            info!("TODO: sync storage");
            Ok(())
        },
        |ssid, password| {
            info!("TODO: connect to wifi '{}' / '{}'", ssid, password);
            Ok(())
        },
    )?;

    let connected = || true;
    let sync_stopped = || false;

    let config = if let SetupResult::StartNew(config) = result {
        nvs_manager.set_session_config(&config)?;
        config
    } else {
        nvs_manager.get_session_config()?.unwrap()
    };

    let send_measurement = |m: Measurement, t: u32| -> Result<(), SendingError> {
        match &config.session_type {
            SessionType::MOBILE => ble.send_measurement(&m, t),
            SessionType::FIXED {
                pm1_index,
                pm2_5_index,
                wifi_ssid,
                wifi_password,
            } => Ok(()),
        }
    };

    let (measurement_rx, stop_tx) = sensor.start_sensor_task(config.interval);

    loop {
        let measurement = measurement_rx.recv()?;
        let measurement_time = SystemTime::now().duration_since(UNIX_EPOCH)?.as_secs() as u32;

        if let Err(e) = send_measurement(measurement, measurement_time) {
            match e {
                SendingError::Retry => {
                    if let Err(e) = send_measurement(measurement, measurement_time) {
                        storage.save_measurement(MeasurementRecord::from_measurement(
                            &measurement,
                            measurement_time,
                        ))?;
                    }
                }
                SendingError::ConfigError => {} //TODO: break the session loop
                SendingError::ConnectionError => storage.save_measurement(
                    MeasurementRecord::from_measurement(&measurement, measurement_time),
                )?, //TODO: on error hold until connection
            }
        }

        if sync_stopped() && storage.has_measurements() && connected() {
            //TODO: start sync thread
        }
    }
}

enum SendingError {
    ConfigError,
    ConnectionError,
    Retry,
}
