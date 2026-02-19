mod led;
mod sensor;
mod storage;

use esp_idf_svc::hal::prelude::*;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use std::thread;
use std::time::{Duration, Instant};
use esp_idf_svc::hal::gpio;
use esp_idf_svc::hal::uart::config::{DataBits, StopBits};
use esp_idf_svc::hal::uart::{UartConfig, UartDriver};
use esp_idf_svc::fs::littlefs::Littlefs;
use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::io::vfs::MountedLittlefs;
use log::info;
use crate::led::led_thread::{start_led_thread, Color, LedCommand, LedPins};
use crate::sensor::sensor_thread::SensorDriver;
use crate::storage::storage_controller::{MeasurementRecord, StorageManager, MOUNT_POINT};


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
    let led_command = start_led_thread(led_pins)?;
    let storage = StorageManager::new();
    loop {
        let size = storage.total_measurement_count();
        info!("Stored {} measurements", size);
        led_command.send(LedCommand::Continuous(Color::RED))?;
        info!("Red LED on");
        led_command.send(LedCommand::Off)?;
        info!("Red LED off");
        thread::sleep(Duration::from_secs(3));
    }
}