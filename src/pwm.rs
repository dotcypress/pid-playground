use crate::hal::hal;
use crate::hal::timer::pwm::PwmPin;
use crate::*;

pub struct PwmPack {
    ch_abs: PwmPin<stm32::TIM3, Channel3>,
    ch_p: PwmPin<stm32::TIM3, Channel2>,
    ch_n: PwmPin<stm32::TIM3, Channel1>,
    phase_pin_p: gpioa::PA5<Output<PushPull>>,
    phase_pin_n: gpioa::PA4<Output<PushPull>>,
}

impl PwmPack {
    pub fn new(
        ch_abs: PwmPin<stm32::TIM3, Channel3>,
        ch_p: PwmPin<stm32::TIM3, Channel2>,
        ch_n: PwmPin<stm32::TIM3, Channel1>,
        phase_pin_p: gpioa::PA5<Output<PushPull>>,
        phase_pin_n: gpioa::PA4<Output<PushPull>>,
    ) -> Self {
        Self {
            ch_abs,
            ch_p,
            ch_n,
            phase_pin_p,
            phase_pin_n,
        }
    }
}

impl hal::PwmPin for PwmPack {
    type Duty = i32;

    fn disable(&mut self) {
        self.ch_abs.disable();
        self.ch_p.disable();
        self.ch_n.disable();
    }

    fn enable(&mut self) {
        self.ch_abs.enable();
        self.ch_p.enable();
        self.ch_n.enable();
    }

    fn get_duty(&self) -> Self::Duty {
        let p = self.ch_p.get_duty();
        if p > 0 {
            p as _
        } else {
            -(self.ch_n.get_duty() as i32)
        }
    }

    fn get_max_duty(&self) -> Self::Duty {
        self.ch_abs.get_duty() as _
    }

    fn set_duty(&mut self, duty: Self::Duty) {
        let duty_abs = duty.unsigned_abs();
        self.ch_abs.set_duty(duty_abs);
        if duty.is_positive() {
            self.ch_n.set_duty(0);
            self.phase_pin_n.set_low().ok();
            self.phase_pin_p.set_high().ok();
            self.ch_p.set_duty(duty_abs);
        } else {
            self.ch_p.set_duty(0);
            self.phase_pin_p.set_low().ok();
            self.phase_pin_n.set_high().ok();
            self.ch_n.set_duty(duty_abs);
        }
    }
}
