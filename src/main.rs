mod led;

use esp_idf_svc::hal::prelude::*;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use std::thread;
use std::time::Duration;
use crate::led::led_thread::{start_led_thread, Color, LedCommand, LedPins};


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
    
    let tx = start_led_thread(led_pins)?;
    // --- MAIN LOOP ---
    // Since we are in std, we can just loop here or let the main thread sleep
    // FreeRTOS is handling the WiFi/BLE tasks in the background.
    loop {
        tx.send(LedCommand::Continuous(Color::RED))?;
        log::info!("Red LED on");
        thread::sleep(Duration::from_secs(1));
        tx.send(LedCommand::Continuous(Color::GREEN))?;
        log::info!("Green LED on");
        thread::sleep(Duration::from_secs(1));
        tx.send(LedCommand::Continuous(Color::BLUE))?;
        log::info!("Blue LED on");
        thread::sleep(Duration::from_secs(1));
    }
}