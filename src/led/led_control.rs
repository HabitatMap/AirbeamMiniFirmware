use std::borrow::Borrow;
use esp_idf_svc::hal::gpio::OutputPin;
use esp_idf_svc::hal::ledc::{LedcChannel, LedcDriver, LedcTimer, LedcTimerDriver, LowSpeed, SpeedMode};
pub struct RgbLed<'a> {
    red: LedcDriver<'a>,
    green: LedcDriver<'a>,
    blue: LedcDriver<'a>,
}

impl<'a> RgbLed<'a> {
    pub fn new<C0, C1, C2, T>(
        timer: T,
        red_pin: impl OutputPin + 'a,
        green_pin: impl OutputPin + 'a,
        blue_pin: impl OutputPin + 'a,
        c_red: C0,
        c_green: C1,
        c_blue: C2,
    ) -> anyhow::Result<Self>
    where
    // We tell the compiler EXACTLY what these types are allowed to be.
    // T must be something that borrows a LowSpeed timer (like a & reference)
    // and it must be Copy so we can use it three times.
        T: Borrow<LedcTimerDriver<'a, LowSpeed>> + Copy,
        C0: LedcChannel<SpeedMode = LowSpeed> + 'a,
        C1: LedcChannel<SpeedMode = LowSpeed> + 'a,
        C2: LedcChannel<SpeedMode = LowSpeed> + 'a,
    {
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
