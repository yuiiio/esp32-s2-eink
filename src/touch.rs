use esp_hal::{
    analog::adc::{Adc, AdcCalScheme, AdcChannel, AdcPin},
    delay::Delay,
    gpio::Output,
    peripherals::ADC1,
    Blocking,
};

pub struct TouchState {
    pub left: bool,
    pub right: bool,
    pub center: bool,
    pub top: bool,
}

pub struct TouchThresholds {
    pub left: u16,
    pub right: u16,
    pub center: u16,
    pub top: u16,
}

impl TouchThresholds {
    pub const fn new(left: u16, right: u16, center: u16, top: u16) -> Self {
        Self { left, right, center, top }
    }

    pub const fn default_thresholds() -> Self {
        Self {
            left: 2950,
            right: 2950,
            center: 2950,
            top: 3100,
        }
    }
}

pub struct TouchInput<'a> {
    touch_out: Output<'a>,
    delay: Delay,
    pulse_delay_ns: u32,
    thresholds: TouchThresholds,
}

impl<'a> TouchInput<'a> {
    pub const DEFAULT_PULSE_DELAY_NS: u32 = 400_000;

    pub fn new(
        touch_out: Output<'a>,
        delay: Delay,
        pulse_delay_ns: u32,
        thresholds: TouchThresholds,
    ) -> Self {
        Self {
            touch_out,
            delay,
            pulse_delay_ns,
            thresholds,
        }
    }

    fn pulse_and_read<PIN, CAL>(
        &mut self,
        adc: &mut Adc<'a, ADC1<'a>, Blocking>,
        pin: &mut AdcPin<PIN, ADC1<'a>, CAL>,
    ) -> u16
    where
        PIN: AdcChannel,
        CAL: AdcCalScheme<ADC1<'a>>,
    {
        self.touch_out.set_high();
        self.delay.delay_nanos(self.pulse_delay_ns);
        self.touch_out.set_low();
        adc.read_blocking(pin)
    }

    pub fn read_all<L, R, C, T, CAL>(
        &mut self,
        adc: &mut Adc<'a, ADC1<'a>, Blocking>,
        left: &mut AdcPin<L, ADC1<'a>, CAL>,
        right: &mut AdcPin<R, ADC1<'a>, CAL>,
        center: &mut AdcPin<C, ADC1<'a>, CAL>,
        top: &mut AdcPin<T, ADC1<'a>, CAL>,
    ) -> TouchState
    where
        L: AdcChannel,
        R: AdcChannel,
        C: AdcChannel,
        T: AdcChannel,
        CAL: AdcCalScheme<ADC1<'a>>,
    {
        let left_val = self.pulse_and_read(adc, left);
        let right_val = self.pulse_and_read(adc, right);
        let center_val = self.pulse_and_read(adc, center);
        let top_val = self.pulse_and_read(adc, top);

        TouchState {
            left: left_val > self.thresholds.left,
            right: right_val > self.thresholds.right,
            center: center_val > self.thresholds.center,
            top: top_val > self.thresholds.top,
        }
    }

    pub fn delay(&self) -> Delay {
        self.delay
    }
}
