use esp_idf_svc::hal::prelude::*;
use esp_idf_svc::wifi::{ClientConfiguration, Configuration, EspWifi};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp32_nimble::{BLEDevice, BLEServer, NimbleProperties};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

fn main() -> anyhow::Result<()> {
    // 1. Link patches to the ESP-IDF system (Required for std)
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Starting up!");

    // 2. Get Peripherals
    let peripherals = Peripherals::take()?;
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

    // --- WIFI SETUP ---
    log::info!("Initializing WiFi...");
    let mut wifi = EspWifi::new(
        peripherals.modem,
        sys_loop,
        Some(nvs),
    )?;

    wifi.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: "ssid-here".try_into().unwrap(),
        password: "pass-here".try_into().unwrap(),
        ..Default::default()
    }))?;

    wifi.start()?;
    wifi.connect()?;

    // Wait for connection (simple polling)
    while !wifi.is_connected()? {
        let config = wifi.get_configuration()?;
        log::info!("Waiting for connection...");
        thread::sleep(Duration::from_secs(1));
    }
    log::info!("WiFi Connected!");

    // --- BLE SETUP ---
    log::info!("Initializing BLE...");
    let ble_device = BLEDevice::take();
    let server = ble_device.get_server();

    // Create a Service (UUID)
    let service = server.create_service(esp32_nimble::uuid128!("91bad492-b950-4226-aa2b-4ede9fa42f59"));

    // Create a Characteristic
    let characteristic = service.lock().create_characteristic(
        esp32_nimble::uuid128!("cba1d466-344c-4be3-ab3f-189f80dd7518"),
        NimbleProperties::READ | NimbleProperties::NOTIFY,
    );

    characteristic.lock().set_value(b"Hello Rust!");

    // Start Advertising
    //let ble_advertising = ble_device.get_advertising();

    log::info!("BLE Advertising started!");

    // --- MAIN LOOP ---
    // Since we are in std, we can just loop here or let the main thread sleep
    // FreeRTOS is handling the WiFi/BLE tasks in the background.
    loop {
        log::info!("System running... IP info: {:?}", wifi.sta_netif().get_ip_info());
        thread::sleep(Duration::from_secs(5));
    }
}