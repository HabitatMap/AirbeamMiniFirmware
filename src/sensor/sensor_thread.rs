use crate::sensor::measurement::Measurement;
use crate::sensor::sensor_parser::parse_sensor;
use crate::LoopEvent;
use esp_idf_svc::hal::uart::UartDriver;
use log::{info, warn};
use std::sync::mpsc::{Receiver, Sender};
use std::sync::{mpsc, Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

pub const START_BYTE_1: u8 = 0x42;
pub const START_BYTE_2: u8 = 0x4D;
pub const FRAME_LEN: usize = 32;
const CMD_PASSIVE: [u8; 7] = [0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70];
const CMD_READ: [u8; 7] = [0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71];
const CMD_SLEEP: [u8; 7] = [0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73];
const CMD_WAKE: [u8; 7] = [0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74];
const WAKE_UP_SECONDS: u64 = 10;
const PASSIVE_THRESHOLD: u64 = 3;

const SENSOR_READOUT_TIMEOUT: u32 = 2300; //Longest possible time between the readouts

pub struct SensorDriver {
    uart: Arc<Mutex<UartDriver<'static>>>,
}

impl SensorDriver {
    pub fn new(uart: UartDriver<'static>) -> Self {
        Self {
            uart: Arc::new(Mutex::new(uart)),
        }
    }

    pub fn start_sensor_task(&self, period: Duration, event_tx: Sender<LoopEvent>) -> Sender<()> {
        // 1. Take the receiver out of the struct (leaving None behind)
        // This ensures we can't start the thread twice.

        let (stop_tx, stop_rx) = mpsc::channel();
        let uart_shared = self.uart.clone();
        let (sleep_time, averaging_time) = Self::get_loop_durations(period);
        let should_sleep = Self::should_sleep(period);

        thread::spawn(move || {
            info!("Sensor Thread: Started.");
            if let Ok(uart) = uart_shared.lock() {
                //wake up sensor for active mode

                //We need to wake up the sensor for active mode,
                // assumption is that sensor will be in sleep on start
                let _ = uart.clear_rx();
                let read_byte = || {
                    let mut byte_buf = [0u8; 1];
                    match uart.read(&mut byte_buf, SENSOR_READOUT_TIMEOUT) {
                        Ok(_bytes) => Some(byte_buf),
                        _ => None,
                    }
                };
                if let Some(m) = Self::read_uart(read_byte, Duration::from_secs(5)) {
                    info!("Read successful. Sending inital measurement.");
                    let _ = event_tx.send(m.into());
                }
                if should_sleep {
                    let _ = uart.write(&CMD_SLEEP);
                    thread::sleep(Duration::from_millis(100));
                } else {
                    let _ = uart.write(&CMD_WAKE);
                    thread::sleep(Duration::from_millis(100));
                }
            }

            loop {
                info!("Sensor Thread: Loop OK");
                if let Ok(uart) = uart_shared.lock() {
                    let read_byte = || {
                        let mut byte_buf = [0u8; 1];
                        match uart.read(&mut byte_buf, SENSOR_READOUT_TIMEOUT) {
                            Ok(_bytes) => Some(byte_buf),
                            _ => None,
                        }
                    };
                    let read_command = || {
                        if averaging_time.as_secs() >= PASSIVE_THRESHOLD {
                            uart.write(&CMD_READ).ok()
                        } else {
                            None
                        }
                    };
                    //wake up sensor for passive mode
                    if should_sleep {
                        let _ = uart.write(&CMD_WAKE);
                        thread::sleep(Duration::from_secs(WAKE_UP_SECONDS));
                        let _ = uart.write(&CMD_PASSIVE);
                    }
                    if averaging_time > Duration::from_secs(PASSIVE_THRESHOLD) {
                        let _ = uart.write(&CMD_PASSIVE);
                    }
                    let _ = uart.clear_rx();

                    //for averaging_time <= 3 seconds, we read in active mode
                    let (measurement, is_stopped) =
                        Self::averaging_loop(averaging_time, read_byte, read_command, &stop_rx);

                    if is_stopped {
                        break;
                    }

                    if measurement.is_none() {
                        warn!("No measurement scanned. Continuing...");
                    }
                    if should_sleep {
                        let _ = uart.write(&CMD_SLEEP);
                    }

                    if let Some(measurement) = measurement {
                        event_tx.send(measurement.into()).unwrap_or_else(|e| {
                            log::error!("Error sending measurement: {:?}", e);
                        });
                    }
                }
            }
            info!("Sensor Thread: Loop stopped.");
            // When loop breaks due to stop command, put sensor to sleep
            if let Ok(uart) = uart_shared.lock() {
                let _ = uart.write(&CMD_SLEEP);
                info!("Sensor command: SLEEP sent.");
            }
        });
        stop_tx
    }

    fn averaging_loop<F, G>(
        duration: Duration,
        mut read_byte: F,
        read_command: G,
        stop: &Receiver<()>,
    ) -> (Option<Measurement>, bool)
    where
        F: FnMut() -> Option<[u8; 1]>,
        G: Fn() -> Option<usize>,
    {
        let mut pm1_0_sum = 0_u32;
        let mut pm2_5_sum = 0_u32;
        let mut count = 0_u32;
        let instant = Instant::now();
        let mut stopped = false;

        while duration > instant.elapsed() {
            read_command(); //TODO: slowdown? current speed 800meas/min
            if let Some(parsed) = Self::read_uart(&mut read_byte, Duration::from_secs(5)) {
                pm1_0_sum += parsed.pm1_0_avg as u32;
                pm2_5_sum += parsed.pm2_5_avg as u32;
                count += 1;
            }
            if stop.try_recv().is_ok() {
                //break the loop if stop signal is received
                stopped = true;
                break;
            }
        }
        let timestamp = SystemTime::now().duration_since(UNIX_EPOCH).ok();
        if count > 0 && timestamp.is_some() {
            let final_pm1 = pm1_0_sum / count;
            let final_pm25 = pm2_5_sum / count;
            (
                Some(Measurement {
                    pm1_0_avg: final_pm1 as u16,
                    pm2_5_avg: final_pm25 as u16,
                    timestamp: timestamp.unwrap().as_secs() as u32,
                }),
                stopped,
            )
        } else {
            (None, stopped)
        }
    }

    fn read_uart<F>(mut read_byte: F, timeout: Duration) -> Option<Measurement>
    where
        F: FnMut() -> Option<[u8; 1]>,
    {
        let mut buf = [0u8; FRAME_LEN];
        let mut frame_idx = 0;
        let instant = Instant::now();
        //we make sure that first two bytes are 0x42 0x4D
        //when they are rest of the readout is collected into buf,
        //until we get FRAME_LEN bytes
        while frame_idx < FRAME_LEN && instant.elapsed() < timeout {
            let byte_buf = read_byte();
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
                    warn!("UART read timeout or error.");
                    return None;
                }
            }
        }
        parse_sensor(&buf).map(|pms| {
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .ok()
                .map(|now| Measurement::from_pms_measurement(pms, now.as_secs() as u32))
        })?
    }

    ///returns sleep duration and measurement collection time
    pub(self) fn get_loop_durations(period: Duration) -> (Duration, Duration) {
        let seconds = period.as_secs();
        match seconds {
            0..=59 => (Duration::from_millis(10), Duration::from_secs(1)), //no sleep, active mode
            60 => (Duration::from_millis(10), Duration::from_secs(60)),    //
            _ => {
                let collection_time = (seconds / 2).clamp(30, 60);
                (
                    Duration::from_secs(seconds - WAKE_UP_SECONDS - collection_time),
                    Duration::from_secs(collection_time),
                )
            } //sleep - wakeup (5s), and passive mode
        }
    }
    ///put sensor to sleep if the period is greater than 60 seconds
    pub(self) fn should_sleep(period: Duration) -> bool {
        period > Duration::from_secs(60)
    }
}
