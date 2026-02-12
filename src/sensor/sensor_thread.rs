use crate::sensor::sensor_parser::{parse_sensor, PmsMeasurement};
use esp_idf_svc::hal::uart::UartDriver;
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender};
use std::sync::{mpsc, Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use crate::led::led_thread::{Color, LedCommand};

pub const START_BYTE_1: u8 = 0x42;
pub const START_BYTE_2: u8 = 0x4D;
pub const FRAME_LEN: usize = 32;
const CMD_PASSIVE: [u8; 7] = [0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70];
const CMD_READ: [u8; 7] = [0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71];
const CMD_SLEEP: [u8; 7] = [0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73];
const CMD_WAKE: [u8; 7] = [0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74];
const WAKE_UP_SECONDS: u64 = 5;
#[derive(Debug, Clone, Copy)]
pub struct Measurement {
    pm1_0_avg: u32,
    pm2_5_avg: u32,
}
impl From<PmsMeasurement> for Measurement {
    fn from(value: PmsMeasurement) -> Self {
        Measurement {
            pm1_0_avg: value.pm1_0_atm as u32,
            pm2_5_avg: value.pm2_5_atm as u32,
        }
    }
}

pub struct SensorDriver {
    uart: Arc<Mutex<UartDriver<'static>>>,
}

impl SensorDriver {
    pub fn new(uart: UartDriver<'static>) -> Self {
            Self {
                uart: Arc::new(Mutex::new(uart)),
            }
    }
    pub fn start_sensor_task(&self, period: Duration) -> (Receiver<Measurement>, Sender<()>) {
        // 1. Take the receiver out of the struct (leaving None behind)
        // This ensures we can't start the thread twice.

        let (stop_tx, stop_rx) = mpsc::channel();
        let (data_tx, data_rx) = mpsc::channel();
        let uart_shared = self.uart.clone();
        let (sleep_time, averaging_time) = Self::get_loop_durations(period);
        let should_sleep = Self::should_sleep(period);

        thread::spawn(move || {
            log::info!("Sensor Thread: Started.");
            if let Ok(uart) = uart_shared.lock() { //wake up sensor for active mode

                //We need to wake up the sensor for active mode,
                // assumption is that sensor will be in sleep on start
                if !should_sleep {
                    let _ = uart.write(&CMD_WAKE);
                    thread::sleep(Duration::from_millis(100));
                }
            }

            loop {
                //we wait for stop signal or sleep time to expire
                match stop_rx.recv_timeout(sleep_time) {
                    Ok(_) | Err(RecvTimeoutError::Disconnected) => {
                        log::info!("Stop signal received. Shutting down...");
                        break; // Exit the loop
                    }
                    Err(RecvTimeoutError::Timeout) => {
                        // Timeout passed, just continue the loop
                    }
                }
                if let Ok(uart) = uart_shared.lock() {
                    //wake up sensor for passive mode
                    if should_sleep {
                        let _ = uart.write(&CMD_WAKE);
                        thread::sleep(Duration::from_secs(WAKE_UP_SECONDS));
                        let _ = uart.write(&CMD_PASSIVE);
                    }

                    let mut byte_buf = [0u8; 1];
                    let read_byte=
                        || match uart.read(&mut byte_buf, 100) {
                            Ok(_bytes) => Some(byte_buf),
                            _ => None,
                        };
                    let read_command = || {
                        match uart.write(&CMD_READ) {
                            Ok(_bytes) => Some(()),
                            _ => None,
                        }
                    };
                    let _ = uart.clear_rx();
                    //for averaging_time <= 3 seconds, we read in active mode
                    let measurement = if averaging_time.as_secs() <= 3 {
                        Self::read_uart(read_byte, Duration::from_secs(3))
                    } else {
                        //for averaging_time > 3 seconds, we read for that period of time and average into one result
                        let measurement = Self::averaging_loop(averaging_time, read_byte, read_command);
                        if should_sleep {
                            let _ = uart.write(&CMD_SLEEP);
                        }
                        measurement
                    };
                    if let Some(measurement) = measurement {
                        data_tx.send(measurement).unwrap_or_else(|e| {
                            log::error!("Error sending measurement: {:?}", e);
                        });
                    }
                }
            }
            // When loop breaks due to stop command, put sensor to sleep
            if let Ok(uart) = uart_shared.lock() {
                let _ = uart.write(&CMD_SLEEP);
                log::info!("Sensor command: SLEEP sent.");
            }
        });
        (data_rx, stop_tx)
    }

    fn averaging_loop<F, G>(
        duration: Duration,
        mut read_byte: F,
        read_command: G,
    ) -> Option<Measurement>
    where
        F: FnMut() -> Option<[u8; 1]>,
        G: Fn() -> Option<()>,
    {
        let mut pm1_0_sum = 0_u32;
        let mut pm2_5_sum = 0_u32;
        let mut count = 0_u32;
        let instant = Instant::now();

        while duration > instant.elapsed() {
            read_command();
            if let Some(parsed) = Self::read_uart(|| read_byte(), duration) {
                pm1_0_sum += parsed.pm1_0_avg;
                pm2_5_sum += parsed.pm2_5_avg;
                count += 1;
                log::info!("Read successful. Count: {}", count);
                read_command();
            }
        }
        thread::sleep(Duration::from_millis(1000));
        if count > 0 {
            let final_pm1 = pm1_0_sum / count;
            let final_pm25 = pm2_5_sum / count;
            Some(Measurement {
                pm1_0_avg: final_pm1,
                pm2_5_avg: final_pm25,
            })
        } else {
            None
        }
    }

    fn read_uart<F>(mut read_byte: F, timeout: Duration) -> Option<Measurement>
    where
        F: FnMut() -> Option<[u8; 1]>,
    {
        let mut buf = [0u8; FRAME_LEN];
        let mut byte_buf: Option<[u8; 1]> = None;
        let mut frame_idx = 0;
        let instant = Instant::now();
        //we make sure that first two bytes are 0x42 0x4D
        //when they are rest of the readout is collected into buf,
        //until we get FRAME_LEN bytes
        while frame_idx < FRAME_LEN && instant.elapsed() < timeout {
            byte_buf = read_byte();
            match byte_buf {
                Some(byte_buf) => {
                    let b = byte_buf[0];
                    match frame_idx {
                        0 => {
                            if b == START_BYTE_1 {
                                buf[0] = b;
                                frame_idx = 1;
                            }
                        }
                        1 => {
                            if b == START_BYTE_2 {
                                buf[1] = b;
                                frame_idx = 2;
                            } else if b == START_BYTE_1 {
                                buf[0] = b; // Handle overlapping 0x42s
                            } else {
                                frame_idx = 0; // Invalid header, reset
                            }
                        }
                        _ => {
                            buf[frame_idx] = b;
                            frame_idx += 1;
                        }
                    }
                }
                _ => {
                    log::warn!("UART read timeout or error.");
                    return None;
                }
            }
        }
        parse_sensor(&buf).map(Measurement::from)
    }

    ///returns sleep duration and measurement collection time
    pub(self) fn get_loop_durations(period: Duration) -> (Duration, Duration) {
        let seconds = period.as_secs();
        match seconds {
            0..=3 => (Duration::from_millis(10), Duration::from_millis(0)), //no sleep, active mode
            4..=59 => (Duration::from_millis(10), Duration::from_secs(seconds)), //no sleep, passive mode
            _ => (
                Duration::from_secs(seconds - WAKE_UP_SECONDS - 30),
                Duration::from_secs(30),
            ), //sleep - wakeup (5s), and passive mode
        }
    }
    ///put sensor to sleep if the period is greater or equal than 60 seconds
    pub(self) fn should_sleep(period: Duration) -> bool {
        period >= Duration::from_secs(60)
    }
}
