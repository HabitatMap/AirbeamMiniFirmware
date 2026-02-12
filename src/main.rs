mod led;
mod sensor;

use esp_idf_svc::hal::prelude::*;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use std::thread;
use std::time::Duration;
use esp_idf_svc::hal::gpio;
use esp_idf_svc::hal::uart::config::{DataBits, StopBits};
use esp_idf_svc::hal::uart::{UartConfig, UartDriver};
use crate::led::led_thread::{start_led_thread, Color, LedCommand, LedPins};
use crate::sensor::sensor_thread::SensorDriver;

fn main() -> anyhow::Result<()> {
    // 1. Link patches to the ESP-IDF system (Required for std)
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Starting up!");

    // 2. Get Peripherals
    let peripherals = Peripherals::take()?;
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    
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
    let tx = start_led_thread(led_pins)?;
    loop {
        tx.send(LedCommand::Continuous(Color::RED))?;
        log::info!("Red LED on");
        let (meas, stop) = sensor.start_sensor_task(Duration::from_secs(0));
        for _ in 0..10 {
            let mes = meas.recv()?;
            log::info!("Measurement received: {:?}", mes);
        }
        stop.send(())?;
        tx.send(LedCommand::Continuous(Color::GREEN))?;
        thread::sleep(Duration::from_secs(5));
        let (meas, stop) = sensor.start_sensor_task(Duration::from_secs(20));
        for _ in 0..10 {
            let mes = meas.recv()?;
            log::info!("Measurement received: {:?}", mes);
        }
        stop.send(())?;
        tx.send(LedCommand::Blinking(Color::BLUE, Duration::from_secs(2)))?;
        thread::sleep(Duration::from_secs(5));
        let (meas, stop) = sensor.start_sensor_task(Duration::from_secs(100));
        for _ in 0..3 {
            let mes = meas.recv()?;
            log::info!("Measurement received: {:?}", mes);
        }
        stop.send(())?;
        thread::sleep(Duration::from_secs(5));
    }
}