use std::sync::mpsc;
use std::thread;
use std::time::Duration;
use esp_idf_svc::hal::ledc::config::TimerConfig;
use esp_idf_svc::hal::ledc::{LedcTimerDriver, Resolution};
use crate::led::led_control::RgbLed;

use esp_idf_svc::hal::gpio::OutputPin;
use esp_idf_svc::hal::ledc::{LedcChannel, LedcTimer, LowSpeed};
use esp_idf_svc::hal::peripheral::Peripheral;

#[derive(Clone, Copy)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}
impl Color {
    //255 - off; 0 - max brightness
    pub const RED: Color = Color { r: 0, g: 255, b: 255 };
    pub const GREEN: Color = Color { r: 255, g: 0, b: 255 };
    pub const BLUE: Color = Color { r: 255, g: 255, b: 0 };
    pub const WHITE: Color = Color { r: 0, g: 0, b: 0 };
}

pub enum LedCommand {
    Off,
    Continuous(Color),
    Blinking(Color, Duration),
}

pub struct LedPins<T, C0, C1, C2, R, G, B> {
    pub timer: T,
    pub channel_r: C0,
    pub channel_g: C1,
    pub channel_b: C2,
    pub pin_r: R,
    pub pin_g: G,
    pub pin_b: B,
}

pub fn start_led_thread<T, C0, C1, C2, R, G, B>(
    pins: LedPins<T, C0, C1, C2, R, G, B>,
) -> anyhow::Result<mpsc::Sender<LedCommand>>
where
    T: Peripheral + 'static,
    T::P: LedcTimer<SpeedMode = LowSpeed>,
    C0: Peripheral + 'static,
    C0::P: LedcChannel<SpeedMode = LowSpeed>,
    C1: Peripheral + 'static,
    C1::P: LedcChannel<SpeedMode = LowSpeed>,
    C2: Peripheral + 'static,
    C2::P: LedcChannel<SpeedMode = LowSpeed>,
    R: Peripheral + 'static,
    R::P: OutputPin,
    G: Peripheral + 'static,
    G::P: OutputPin,
    B: Peripheral + 'static,
    B::P: OutputPin,
{
    let (tx, rx) = mpsc::channel::<LedCommand>();

    // --- Hardware Initialization (Main Thread) ---
    // Perform initialization here so we can fail fast if hardware is missing/busy
    let config = TimerConfig::new().frequency(5000.into()).resolution(Resolution::Bits8);
    let timer = LedcTimerDriver::new(pins.timer, &config)?;

    let mut led = RgbLed::new(
        &timer,
        pins.pin_r, pins.pin_g, pins.pin_b,
        pins.channel_r, pins.channel_g, pins.channel_b,
    )?;

    thread::spawn(move || {
        // --- Logic Loop (Worker Thread) ---
        let mut current_mode = LedCommand::Off;
        let mut blink_is_on = true;

        loop {
            match current_mode {
                LedCommand::Off => {
                    if let Err(e) = led.off() {
                        log::error!("Failed to turn off LED: {:?}", e);
                    }

                    // Wait indefinitely for a new command (Blocks thread, saves CPU)
                    match rx.recv() {
                        Ok(cmd) => {
                            current_mode = cmd;
                            blink_is_on = true; // Reset phase for new command
                        },
                        Err(_) => {
                            log::info!("LED Control Channel closed, exiting thread");
                            break;
                        }
                    }
                }
                LedCommand::Continuous(color) => {
                    if let Err(e) = led.set_color(color.r, color.g, color.b) {
                        log::error!("Failed to set solid LED color: {:?}", e);
                    }

                    // Wait indefinitely for a new command
                    match rx.recv() {
                        Ok(cmd) => {
                            current_mode = cmd;
                            blink_is_on = true;
                        },
                        Err(_) => {
                            log::info!("LED Control Channel closed, exiting thread");
                            break;
                        }
                    }
                }

                // CASE 3: LED is BLINKING
                LedCommand::Blinking(color, timeout) => {
                    // 1. Apply the current blink state (On or Off)
                    let set_result = if blink_is_on {
                        led.set_color(color.r, color.g, color.b)
                    } else {
                        led.off()
                    };

                    if let Err(e) = set_result {
                        log::error!("Failed to update blink state: {:?}", e);
                    }

                    // 2. Wait for the period duration OR a new command
                    match rx.recv_timeout(timeout) {
                        Ok(cmd) => {
                            current_mode = cmd;
                            blink_is_on = true; // Reset phase
                        },
                        Err(mpsc::RecvTimeoutError::Timeout) => {
                            blink_is_on = !blink_is_on;
                        },
                        Err(mpsc::RecvTimeoutError::Disconnected) => {
                            log::info!("LED Control Channel closed, exiting thread");
                            break;
                        }
                    }
                }
            }
        }
    });

    Ok(tx)
}