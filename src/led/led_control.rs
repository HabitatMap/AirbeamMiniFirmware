use esp_idf_svc::hal::gpio::OutputPin;
use esp_idf_svc::hal::ledc::{LedcChannel, LedcDriver, LedcTimer, LedcTimerDriver, LowSpeed};
use esp_idf_svc::hal::peripheral::Peripheral;
pub struct RgbLed<'a> {
    red: LedcDriver<'a>,
    green: LedcDriver<'a>,
    blue: LedcDriver<'a>,
}

impl<'a> RgbLed<'a> {
    pub fn new(
        timer: &LedcTimerDriver<'a, impl LedcTimer<SpeedMode=LowSpeed>>,
        red_pin: impl Peripheral<P=impl OutputPin> + 'a,
        green_pin: impl Peripheral<P=impl OutputPin> + 'a,
        blue_pin: impl Peripheral<P=impl OutputPin> + 'a,
        c_red: impl Peripheral<P=impl LedcChannel<SpeedMode=LowSpeed>> + 'a,
        c_green: impl Peripheral<P=impl LedcChannel<SpeedMode=LowSpeed>> + 'a,
        c_blue: impl Peripheral<P=impl LedcChannel<SpeedMode=LowSpeed>> + 'a,
    ) -> anyhow::Result<Self> {
        Ok(Self {
            red: LedcDriver::new(c_red, timer, red_pin)?,
            green: LedcDriver::new(c_green, timer, green_pin)?,
            blue: LedcDriver::new(c_blue, timer, blue_pin)?,
        })
    }

    pub fn set_color(&mut self, r: u8, g: u8, b: u8) -> anyhow::Result<()> {
        self.red.set_duty(r as u32)?;
        self.green.set_duty(g as u32)?;
        self.blue.set_duty(b as u32)?;
        Ok(())
    }

    pub fn off(&mut self) -> anyhow::Result<()> {
        self.set_color(0, 0, 0)
    }
}