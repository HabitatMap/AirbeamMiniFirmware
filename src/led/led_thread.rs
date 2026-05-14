use crate::led::led_control::RgbLed;
use esp_idf_svc::hal::ledc::config::TimerConfig;
use esp_idf_svc::hal::ledc::{LedcTimerDriver, Resolution, SpeedMode};
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

use esp_idf_svc::hal::gpio::OutputPin;
use esp_idf_svc::hal::ledc::{LedcChannel, LedcTimer, LowSpeed};

#[derive(Debug, Clone, Copy)]
pub enum LedEvent {
    Idle,
    BleConnected,
    SessionStarted { is_fixed: bool },
    UpdateBleState(bool),
    UpdateBattery(bool),
    SyncStarted,
    SyncFinished,
}

#[derive(Clone, Copy, PartialEq)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}
impl Color {
    //255 - off; 0 - max brightness
    pub const RED: Color = Color {
        r: 0,
        g: 255,
        b: 255,
    };
    pub const GREEN: Color = Color {
        r: 255,
        g: 0,
        b: 255,
    };
    pub const BLUE: Color = Color {
        r: 255,
        g: 255,
        b: 0,
    };
    pub const MAGENTA: Color = Color { r: 0, g: 255, b: 0 };
    pub const CYAN: Color = Color { r: 255, g: 0, b: 0 };
    pub const YELLOW: Color = Color { r: 0, g: 0, b: 255 };
    pub const WHITE: Color = Color { r: 0, g: 0, b: 0 };
}

#[derive(PartialEq)]
enum LedCommand {
    Off,
    Continuous(Color),
    Blinking(Color, Duration),
}

enum Phase {
    Idle,
    SetupConnected,
    Session,
}

struct LedStateManager {
    current_phase: Phase,
    is_fixed: bool,
    ble_connected: bool,
    battery_low: bool,
    session_start_time: Option<Instant>,
    syncing: bool,
}

impl LedStateManager {
    fn new() -> Self {
        Self {
            current_phase: Phase::Idle,
            is_fixed: false,
            ble_connected: false,
            battery_low: false,
            session_start_time: None,
            syncing: false,
        }
    }

    fn apply_event(&mut self, event: LedEvent) {
        match event {
            LedEvent::Idle => self.current_phase = Phase::Idle,
            LedEvent::BleConnected => self.current_phase = Phase::SetupConnected,
            LedEvent::SessionStarted { is_fixed } => {
                self.current_phase = Phase::Session;
                self.is_fixed = is_fixed;
                self.session_start_time = Some(Instant::now());
            }
            LedEvent::UpdateBleState(connected) => self.ble_connected = connected,
            LedEvent::UpdateBattery(low) => self.battery_low = low,
            LedEvent::SyncStarted => self.syncing = true,
            LedEvent::SyncFinished => self.syncing = false,
        }
    }

    fn get_current_command(&self) -> LedCommand {
        if self.syncing {
            return LedCommand::Continuous(Color::CYAN);
        }

        match self.current_phase {
            Phase::Idle => LedCommand::Continuous(Color::GREEN),
            Phase::SetupConnected => LedCommand::Continuous(Color::BLUE),
            Phase::Session => {
                if let Some(start) = self.session_start_time {
                    if start.elapsed() < Duration::from_secs(120) {
                        return LedCommand::Continuous(Color::WHITE);
                    }
                }

                if self.is_fixed {
                    LedCommand::Off
                } else {
                    if self.battery_low {
                        LedCommand::Blinking(Color::MAGENTA, Duration::from_secs(10))
                    } else if self.ble_connected {
                        LedCommand::Blinking(Color::WHITE, Duration::from_secs(10))
                    } else {
                        LedCommand::Blinking(Color::YELLOW, Duration::from_secs(10))
                    }
                }
            }
        }
    }
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
) -> anyhow::Result<mpsc::Sender<LedEvent>>
where
    T: LedcTimer<SpeedMode = LowSpeed> + 'static,
    C0: LedcChannel<SpeedMode = LowSpeed> + 'static,
    C1: LedcChannel<SpeedMode = LowSpeed> + 'static,
    C2: LedcChannel<SpeedMode = LowSpeed> + 'static,
    R: OutputPin + 'static,
    G: OutputPin + 'static,
    B: OutputPin + 'static,
{
    let (tx, rx) = mpsc::channel::<LedEvent>();

    // --- Hardware Initialization (Main Thread) ---
    // Perform initialization here so we can fail fast if hardware is missing/busy
    let config = TimerConfig::new()
        .frequency(5000.into())
        .resolution(Resolution::Bits8);
    let timer = LedcTimerDriver::new(pins.timer, &config)?;

    let mut led = RgbLed::new(
        &timer,
        pins.pin_r,
        pins.pin_g,
        pins.pin_b,
        pins.channel_r,
        pins.channel_g,
        pins.channel_b,
    )?;

    thread::spawn(move || {
        // --- Logic Loop (Worker Thread) ---
        let mut state = LedStateManager::new();
        let mut current_command = state.get_current_command();
        let mut blink_is_on = true;

        loop {
            // Apply current command to hardware
            let mut timeout_to_use = None;

            match &current_command {
                LedCommand::Off => {
                    if let Err(e) = led.off() {
                        log::error!("Failed to turn off LED: {:?}", e);
                    }
                }
                LedCommand::Continuous(color) => {
                    if let Err(e) = led.set_color(color.r, color.g, color.b, Some(60)) {
                        log::error!("Failed to set solid LED color: {:?}", e);
                    }
                }
                LedCommand::Blinking(color, timeout) => {
                    if blink_is_on {
                        if let Err(e) = led.set_color(color.r, color.g, color.b, Some(70)) {
                            log::error!("Failed to set blink LED state: {:?}", e);
                        }
                        timeout_to_use = Some(Duration::from_secs(1));
                    } else {
                        if let Err(e) = led.off() {
                            log::error!("Failed to turn off LED: {:?}", e);
                        }
                        timeout_to_use = Some(*timeout);
                    }
                }
            }

            // Check if we need to wake up for the 120s timer expiration
            if let Phase::Session = state.current_phase {
                if let Some(start) = state.session_start_time {
                    let elapsed = start.elapsed();
                    let max_dur = Duration::from_secs(120);
                    if elapsed < max_dur {
                        let remaining = max_dur - elapsed;
                        timeout_to_use = match timeout_to_use {
                            Some(t) => Some(std::cmp::min(t, remaining)),
                            None => Some(remaining),
                        };
                    }
                }
            }

            let rx_result = if let Some(t) = timeout_to_use {
                rx.recv_timeout(t)
            } else {
                rx.recv().map_err(|_| mpsc::RecvTimeoutError::Disconnected)
            };

            match rx_result {
                Ok(event) => {
                    state.apply_event(event);
                    let new_cmd = state.get_current_command();
                    if new_cmd != current_command {
                        current_command = new_cmd;
                        blink_is_on = true; // Reset phase for new command
                    }
                }
                Err(mpsc::RecvTimeoutError::Timeout) => {
                    if matches!(current_command, LedCommand::Blinking(_, _)) {
                        blink_is_on = !blink_is_on;
                    }
                    // Re-evaluate command (in case of 120s timer expiration)
                    let new_cmd = state.get_current_command();
                    if new_cmd != current_command {
                        current_command = new_cmd;
                        blink_is_on = true;
                    }
                }
                Err(mpsc::RecvTimeoutError::Disconnected) => {
                    log::info!("LED Control Channel closed, exiting thread");
                    break;
                }
            }
        }
    });

    Ok(tx)
}
