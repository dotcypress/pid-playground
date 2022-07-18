#![no_std]
#![no_main]
#![deny(warnings)]

extern crate cortex_m_rt as rt;
extern crate panic_halt;
extern crate stm32g0xx_hal as hal;

mod pwm;
mod regulator;
mod shell;

use crate::pwm::*;
use crate::regulator::*;
use crate::shell::*;
use dyadic::DF;
use hal::{analog::adc, gpio::*, prelude::*, serial, stm32, timer::*};
use ushell::{history::LRUHistory, UShell};

#[rtic::app(device = hal::stm32, peripherals = true, dispatchers = [USART1])]
mod app {
    use super::*;

    #[local]
    struct Local {
        adc: adc::Adc,
        adc_pin: gpioa::PA1<Analog>,
        shell: shell::Shell,
    }

    #[shared]
    struct Shared {
        timer: Timer<stm32::TIM2>,
        pwm: PwmPack,
        inverse: bool,
        reg: Regulator,
        freq: u32,
        on: bool,
        trace: bool,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut rcc = ctx.device.RCC.constrain();
        let port_a = ctx.device.GPIOA.split(&mut rcc);
        let port_b = ctx.device.GPIOB.split(&mut rcc);

        let mut delay = ctx.device.TIM1.delay(&mut rcc);

        let mut timer = ctx.device.TIM2.timer(&mut rcc);
        timer.listen();

        let pwm = ctx.device.TIM3.pwm(320.hz(), &mut rcc);
        let mut pwm = PwmPack::new(
            pwm.bind_pin(port_b.pb0),
            pwm.bind_pin(port_a.pa7),
            pwm.bind_pin(port_a.pa6),
            port_a.pa5.into(),
            port_a.pa4.into(),
        );
        pwm.set_duty(0);
        pwm.enable();

        let mut adc = ctx.device.ADC.constrain(&mut rcc);
        adc.set_sample_time(adc::SampleTime::T_80);
        adc.set_precision(adc::Precision::B_12);
        adc.set_oversampling_ratio(adc::OversamplingRatio::X_256);
        adc.set_oversampling_shift(20);
        adc.oversampling_enable(true);
        delay.delay(100.us());
        adc.calibrate();

        let adc_pin = port_a.pa1.into_analog();

        let uart_cfg = serial::BasicConfig::default().baudrate(115_200.bps());
        let mut uart = ctx
            .device
            .USART2
            .usart(port_a.pa2, port_a.pa3, uart_cfg, &mut rcc)
            .expect("Failed to init serial port");
        uart.listen(serial::Event::Rxne);

        let shell = UShell::new(uart, AUTOCOMPLETE, LRUHistory::default());
        let reg = Regulator::new(pwm.get_max_duty());

        (
            Shared {
                reg,
                timer,
                pwm,
                freq: 0,
                on: false,
                inverse: true,
                trace: false,
            },
            Local {
                adc,
                shell,
                adc_pin,
            },
            init::Monotonics(),
        )
    }

    #[task(
        capacity = 8,
        local = [shell],
        shared = [freq, inverse, on, trace, reg, timer, pwm],
        priority = 1
    )]
    fn env(ctx: env::Context, sig: EnvSignal) {
        let mut env = ctx.shared;
        env.on_signal(ctx.local.shell, sig).ok();
    }

    #[task(binds = USART2, priority = 1)]
    fn uart_rx(_: uart_rx::Context) {
        env::spawn(EnvSignal::SpinShell).ok();
    }

    #[task(
        binds = TIM2,
        local = [adc, adc_pin],
        shared = [trace, inverse, reg, timer, pwm],
        priority = 2
    )]
    fn timer_tick(ctx: timer_tick::Context) {
        let timer_tick::SharedResources {
            mut reg,
            mut timer,
            mut trace,
            mut pwm,
            mut inverse,
        } = ctx.shared;
        let timer_tick::LocalResources { adc, adc_pin } = ctx.local;

        let raw = adc.read(adc_pin).unwrap_or(0);

        pub const MAX: i32 = 65_535;
        let val = raw.clamp(0, MAX);
        let val = if inverse.lock(|inverse| *inverse) {
            MAX - val
        } else {
            val
        };

        let duty = reg.lock(|reg| reg.update(val));
        pwm.lock(|pwm| pwm.set_duty(duty));

        if trace.lock(|trace| *trace) {
            env::spawn(EnvSignal::TraceState).ok();
        }

        timer.lock(|timer| timer.clear_irq());
    }
}
