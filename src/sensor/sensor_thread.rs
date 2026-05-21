use crate::sensor::measurement::Measurement;
use crate::sensor::sensor_parser::{parse_sensor, PmsMeasurement};
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
const CMD_ACTIVE: [u8; 7] = [0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71];
const CMD_PASSIVE: [u8; 7] = [0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70];
const CMD_READ: [u8; 7] = [0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71];
const CMD_SLEEP: [u8; 7] = [0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73];
const CMD_WAKE: [u8; 7] = [0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74];
const WAKE_UP_SECONDS: u64 = 15;
const PASSIVE_THRESHOLD: u64 = 3;
/// Number of raw frames averaged for the initial emit of a fixed-minute
/// session. A single post-warmup frame is statistically noisier (and biased
/// low on the PMS) than the ~60-frame per-minute averages that follow, so
/// average a few frames so the first reading matches the character of
/// subsequent emits.
const FIXED_INITIAL_FRAME_COUNT: u32 = 5;

const SENSOR_READOUT_TIMEOUT: u32 = 2300; //Longest possible time between the readouts

#[derive(Clone, Copy, Debug)]
enum WarmupState {
    Cold,
    Warming { since: Instant },
    Warm,
}

pub struct SensorDriver {
    uart: Arc<Mutex<UartDriver<'static>>>,
    warmup: Arc<Mutex<WarmupState>>,
}

impl SensorDriver {
    pub fn new(uart: UartDriver<'static>) -> Self {
        Self {
            uart: Arc::new(Mutex::new(uart)),
            warmup: Arc::new(Mutex::new(WarmupState::Cold)),
        }
    }

    /// Kick off PMS warmup in the background so the first measurement is
    /// available without paying the 15 s wake delay when a session starts.
    /// Idempotent: a call while already Warming/Warm is a no-op.
    pub fn pre_warm(&self) {
        let uart = self.uart.clone();
        let warmup = self.warmup.clone();
        thread::spawn(move || {
            {
                let mut w = match warmup.lock() {
                    Ok(g) => g,
                    Err(_) => return,
                };
                if !matches!(*w, WarmupState::Cold) {
                    return;
                }
                *w = WarmupState::Warming {
                    since: Instant::now(),
                };
            }
            if let Ok(uart) = uart.lock() {
                let _ = uart.clear_rx();
                uart.write(&CMD_ACTIVE).ok();
                thread::sleep(Duration::from_millis(100));
                uart.write(&CMD_WAKE).ok();
            }
            thread::sleep(Duration::from_secs(WAKE_UP_SECONDS));
            if let Ok(mut w) = warmup.lock() {
                if matches!(*w, WarmupState::Warming { .. }) {
                    *w = WarmupState::Warm;
                }
            }
            info!("Sensor pre-warm: complete.");
        });
    }

    fn consume_warmup(
        uart_shared: &Arc<Mutex<UartDriver<'static>>>,
        warmup: &Arc<Mutex<WarmupState>>,
    ) -> Duration {
        let mut w = match warmup.lock() {
            Ok(g) => g,
            Err(_) => return Duration::from_secs(WAKE_UP_SECONDS),
        };
        let remaining = match *w {
            WarmupState::Warm => Duration::ZERO,
            WarmupState::Warming { since } => {
                Duration::from_secs(WAKE_UP_SECONDS).saturating_sub(since.elapsed())
            }
            WarmupState::Cold => {
                if let Ok(uart) = uart_shared.lock() {
                    let _ = uart.clear_rx();
                    uart.write(&CMD_ACTIVE).ok();
                    thread::sleep(Duration::from_millis(100));
                    uart.write(&CMD_WAKE).ok();
                }
                Duration::from_secs(WAKE_UP_SECONDS)
            }
        };
        *w = WarmupState::Cold;
        remaining
    }

    pub fn start_sensor_task(&self, period: Duration, event_tx: Sender<LoopEvent>) -> Sender<()> {
        // 1. Take the receiver out of the struct (leaving None behind)
        // This ensures we can't start the thread twice.

        let (stop_tx, stop_rx) = mpsc::channel();
        let uart_shared = self.uart.clone();
        let warmup = self.warmup.clone();

        if period == Duration::from_secs(60) {
            return Self::start_fixed_minute_task(uart_shared, warmup, event_tx, stop_tx, stop_rx);
        }

        let (sleep_time, averaging_time) = Self::get_loop_durations(period);
        let should_sleep = Self::should_sleep(period);

        thread::spawn(move || {
            info!("Sensor Thread: Started.");
            // Sensor wake commands are issued by pre_warm() in the background; here we
            // only sleep for any remaining settle time before reading the first frame.
            let remaining = Self::consume_warmup(&uart_shared, &warmup);
            if remaining > Duration::ZERO {
                thread::sleep(remaining);
            }
            // PMS active-mode frame rate is slightly faster than 1 Hz, so two
            // reads occasionally land in the same wall-clock second. Track the
            // last emitted timestamp and drop duplicates so each second yields
            // at most one record.
            let mut last_emitted_ts: u32 = 0;
            if let Ok(uart) = uart_shared.lock() {
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
                    last_emitted_ts = m.timestamp;
                    let _ = event_tx.send(m.into());
                }
                // In the 1 s active-mode path the sensor is already awake and
                // streaming from pre_warm, so re-issuing CMD_ACTIVE/CMD_WAKE
                // here just stalls the next frame by ~1 s and adds 200 ms of
                // dead sleep. Only run the mode/wake handshake when we
                // actually need to switch into passive mode or put the sensor
                // to sleep.
                if should_sleep || averaging_time > Duration::from_secs(PASSIVE_THRESHOLD) {
                    if averaging_time > Duration::from_secs(PASSIVE_THRESHOLD) {
                        let _ = uart.write(&CMD_PASSIVE);
                    } else {
                        let _ = uart.write(&CMD_ACTIVE);
                    }
                    if should_sleep {
                        let _ = uart.write(&CMD_SLEEP);
                    } else {
                        let _ = uart.write(&CMD_WAKE);
                    }
                    thread::sleep(Duration::from_millis(100));
                }
            }

            loop {
                info!("Sensor Thread: Loop OK");
                thread::sleep(sleep_time);
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
                    // In active mode the PMS streams ~1 Hz; clearing the RX
                    // buffer here would discard the next pending frame and
                    // force a fresh ~1 s wait every iteration. Only flush
                    // when we just woke the sensor or are about to request a
                    // passive read.
                    if should_sleep || averaging_time > Duration::from_secs(PASSIVE_THRESHOLD) {
                        let _ = uart.clear_rx();
                    }

                    // Fast path: averaging_time <= 1 s. PMS active-mode
                    // frame rate is already ~1 Hz, so averaging would only
                    // double the wall-clock period. Read a single frame.
                    let (measurement, is_stopped) =
                        if averaging_time <= Duration::from_secs(1) && !should_sleep {
                            if stop_rx.try_recv().is_ok() {
                                (None, true)
                            } else {
                                let m = Self::read_uart(read_byte, Duration::from_secs(5));
                                (m, false)
                            }
                        } else {
                            //for averaging_time <= 3 seconds, we read in active mode
                            Self::averaging_loop(averaging_time, read_byte, read_command, &stop_rx)
                        };

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
                        if measurement.timestamp == last_emitted_ts {
                            // Same wall-clock second as the previous record;
                            // drop to avoid duplicate timestamps. Next read
                            // will land in the following second.
                            continue;
                        }
                        last_emitted_ts = measurement.timestamp;
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

    fn start_fixed_minute_task(
        uart_shared: Arc<Mutex<UartDriver<'static>>>,
        warmup: Arc<Mutex<WarmupState>>,
        event_tx: Sender<LoopEvent>,
        stop_tx: Sender<()>,
        stop_rx: Receiver<()>,
    ) -> Sender<()> {
        thread::spawn(move || {
            info!("Sensor Thread: Started (fixed-minute mode).");
            let remaining = Self::consume_warmup(&uart_shared, &warmup);
            if remaining > Duration::ZERO {
                thread::sleep(remaining);
            }
            if let Ok(uart) = uart_shared.lock() {
                let _ = uart.clear_rx();

                let read_byte = || {
                    let mut byte_buf = [0u8; 1];
                    match uart.read(&mut byte_buf, SENSOR_READOUT_TIMEOUT) {
                        Ok(_bytes) => Some(byte_buf),
                        _ => None,
                    }
                };
                let initial_avg = Self::read_initial_avg_pms(
                    read_byte,
                    FIXED_INITIAL_FRAME_COUNT,
                    Duration::from_secs(5),
                );
                let mut current_minute: u64 = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .map(|d| d.as_secs() / 60)
                    .unwrap_or(0);
                let initial_minute = current_minute;
                if let Some(avg_pms) = initial_avg {
                    info!("Read successful. Sending initial measurement.");
                    let m = Measurement::from_pms_measurement(avg_pms, (current_minute * 60) as u32);
                    let _ = event_tx.send(m.into());
                }

                let mut sum_c03: u32 = 0;
                let mut sum_c10: u32 = 0;
                let mut count: u32 = 0;

                let read_byte_loop = || {
                    let mut byte_buf = [0u8; 1];
                    match uart.read(&mut byte_buf, SENSOR_READOUT_TIMEOUT) {
                        Ok(_bytes) => Some(byte_buf),
                        _ => None,
                    }
                };

                loop {
                    if stop_rx.try_recv().is_ok() {
                        break;
                    }
                    if let Some(frame) = Self::read_raw_frame(read_byte_loop, Duration::from_secs(5)) {
                        let now_min = SystemTime::now()
                            .duration_since(UNIX_EPOCH)
                            .map(|d| d.as_secs() / 60)
                            .unwrap_or(current_minute);
                        if now_min == initial_minute {
                            // Initial minute already emitted; skip accumulation.
                            continue;
                        }
                        if now_min != current_minute {
                            if count > 0 {
                                let avg_pms = PmsMeasurement {
                                    c03: (sum_c03 / count) as u16,
                                    c1: (sum_c10 / count) as u16,
                                };
                                let m = Measurement::from_pms_measurement(avg_pms, (current_minute * 60) as u32);
                                event_tx.send(m.into()).unwrap_or_else(|e| {
                                    log::error!("Error sending measurement: {:?}", e);
                                });
                            } else if current_minute != initial_minute {
                                warn!("No samples in minute {}, skipping emit.", current_minute);
                            }
                            sum_c03 = 0;
                            sum_c10 = 0;
                            count = 0;
                            current_minute = now_min;
                        }
                        sum_c03 += frame.c03 as u32;
                        sum_c10 += frame.c1 as u32;
                        count += 1;
                    }
                }

                info!("Sensor Thread: Loop stopped.");
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
        let mut c03_sum = 0_u32;
        let mut c1_sum = 0_u32;
        let mut count = 0_u32;
        let instant = Instant::now();
        let mut stopped = false;

        while duration > instant.elapsed() {
            let is_passive = read_command().is_some();
            if let Some(parsed) = Self::read_raw_frame(&mut read_byte, Duration::from_secs(5)) {
                c03_sum += parsed.c03 as u32;
                c1_sum += parsed.c1 as u32;
                count += 1;
            }
            if stop.try_recv().is_ok() {
                //break the loop if stop signal is received
                stopped = true;
                break;
            }
            if is_passive {
                thread::sleep(Duration::from_millis(500));
            }
        }
        let timestamp = SystemTime::now().duration_since(UNIX_EPOCH).ok();
        if count > 0 && timestamp.is_some() {
            let avg_pms = PmsMeasurement {
                c03: (c03_sum / count) as u16,
                c1: (c1_sum / count) as u16,
            };
            let m = Measurement::from_pms_measurement(avg_pms, timestamp.unwrap().as_secs() as u32);
            (Some(m), stopped)
        } else {
            (None, stopped)
        }
    }

    fn read_uart<F>(read_byte: F, timeout: Duration) -> Option<Measurement>
    where
        F: FnMut() -> Option<[u8; 1]>,
    {
        let pms = Self::read_raw_frame(read_byte, timeout)?;
        let now = SystemTime::now().duration_since(UNIX_EPOCH).ok()?;
        Some(Measurement::from_pms_measurement(pms, now.as_secs() as u32))
    }

    fn read_raw_frame<F>(mut read_byte: F, timeout: Duration) -> Option<PmsMeasurement>
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
        parse_sensor(&buf)
    }

    /// Reads up to `n` raw frames and returns the per-field averages as a
    /// PmsMeasurement. Used for the first emit of a fixed-minute session so the
    /// initial reading is built from the same statistical base as the
    /// per-minute averaged emits that follow it. Averages the calibration
    /// inputs (`c03`, `c10`) so the nonlinear pm1/pm2.5 equation is applied
    /// once on smoothed counts.
    fn read_initial_avg_pms<F>(
        mut read_byte: F,
        n: u32,
        per_frame_timeout: Duration,
    ) -> Option<PmsMeasurement>
    where
        F: FnMut() -> Option<[u8; 1]>,
    {
        let mut sum_c03: u32 = 0;
        let mut sum_c10: u32 = 0;
        let mut count: u32 = 0;
        for _ in 0..n {
            if let Some(frame) = Self::read_raw_frame(&mut read_byte, per_frame_timeout) {
                sum_c03 += frame.c03 as u32;
                sum_c10 += frame.c1 as u32;
                count += 1;
            }
        }
        if count == 0 {
            None
        } else {
            Some(PmsMeasurement {
                c03: (sum_c03 / count) as u16,
                c1: (sum_c10 / count) as u16,
            })
        }
    }

    ///returns sleep duration and measurement collection time
    pub(self) fn get_loop_durations(period: Duration) -> (Duration, Duration) {
        let seconds = period.as_secs();
        match seconds {
            0..=60 => (Duration::from_millis(10), Duration::from_secs(seconds)), //no sleep, active mode
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
