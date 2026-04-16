use std::borrow::Borrow;
use esp_idf_svc::hal::adc::oneshot::{AdcChannelDriver, AdcDriver};
use esp_idf_svc::hal::adc::{AdcChannel, ADC1, ADCU1};
use esp_idf_svc::hal::gpio::{Gpio3, Gpio4, Input, PinDriver, Pull};
use std::time::Instant;
// Divider ratio: V_bat = V_pin * 1499 / 1000  (e.g. 499kΩ top + 1000kΩ bottom)
const DIVIDER_RATIO_NUM: u32 = 1499;
const DIVIDER_RATIO_DEN: u32 = 1000;
const VBAT_EMPTY_MV: u32 = 3050;
const VBAT_FULL_MV: u32 = 4000;
const SAMPLE_WINDOW_MS: u128 = 20;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BatteryState {
    /// -100..=-1 discharging, 1..=100 charging, 0 = read error
    pub signed_percent: i8,
    /// actual battery voltage in mV (after divider correction)
    pub voltage_mv: u32,
}

pub struct BatteryMonitor<'a> {
    usb_pin: PinDriver<'a, Input>,
}

impl<'a> BatteryMonitor<'a> {
    pub fn new(usb_gpio: esp_idf_svc::hal::gpio::Gpio4<'a>) -> anyhow::Result<Self> {
        Ok(Self {
            usb_pin: PinDriver::input(usb_gpio, Pull::Floating)?,
        })
    }

    /// Burst-sample for 20ms, return battery state.
    /// ADC driver and channel live in main — we just borrow them here.
    pub fn read(
        &self,
        adc: &AdcDriver<'a, ADCU1>,
        // We replace `Gpio3` and the exact Borrow type with `impl` constraints
        pin: &mut AdcChannelDriver<'a, impl AdcChannel<AdcUnit = ADCU1>, impl Borrow<AdcDriver<'a, ADCU1>>>,
    ) -> BatteryState { // Assuming BatteryState is defined in your code
        let mut sum: u32 = 0;
        let mut count: u16 = 0;
        let start = Instant::now();

        while start.elapsed().as_millis() < SAMPLE_WINDOW_MS { // Assuming SAMPLE_WINDOW_MS is defined
            if let Ok(mv) = adc.read(pin) {
                sum += mv as u32;
                count += 1;
            }
        }

        if count == 0 {
            return BatteryState {
                signed_percent: 0,
                voltage_mv: 0,
            };
        }

        let avg_adc_mv = sum / count as u32;
        let voltage_mv = avg_adc_mv * DIVIDER_RATIO_NUM / DIVIDER_RATIO_DEN;
        let usb = self.usb_pin.is_high();

        let signed = match Self::map_percent(voltage_mv) {
            Some(p) if usb => p as i8,
            Some(p) => -(p as i8),
            None => 0i8,
        };

        BatteryState {
            signed_percent: signed,
            voltage_mv,
        }
    }

    fn map_percent(mv: u32) -> Option<u8> {
        if mv < VBAT_EMPTY_MV {
            return Some(1);
        }
        if mv > VBAT_FULL_MV + 200 {
            return None;
        }
        let clamped = mv.min(VBAT_FULL_MV);
        let pct = ((clamped - VBAT_EMPTY_MV) * 100) / (VBAT_FULL_MV - VBAT_EMPTY_MV);
        Some((pct as u8).max(1))
    }
}
