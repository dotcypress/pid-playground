#![no_std]
#![no_main]
#![deny(warnings)]

extern crate cortex_m_rt as rt;
extern crate panic_halt;
extern crate stm32g0xx_hal as hal;

mod regulator;
mod shell;

use crate::regulator::*;
use crate::shell::*;
use dyadic::DF;
use hal::{analog::adc, gpio::*, prelude::*, serial, stm32, timer::*};
use ushell::{autocomplete, history::LRUHistory, UShell};

#[rtic::app(device = hal::stm32, peripherals = true, dispatchers = [USART1])]
mod app {
    use super::*;

    #[local]
    struct Local {
        adc: adc::Adc,
        temp_pin: gpioa::PA1<Analog>,
        temp_cal: SensorCalibration,
        shell: shell::Shell,
    }

    #[shared]
    struct Shared {
        timer: Timer<stm32::TIM2>,
        heater: pwm::PwmPin<stm32::TIM3, Channel2>,
        reg: Regulator,
        freq: u32,
        on: bool,
        log: bool,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut rcc = ctx.device.RCC.constrain();
        let port_a = ctx.device.GPIOA.split(&mut rcc);

        let mut timer = ctx.device.TIM2.timer(&mut rcc);
        timer.listen();

        let pwm = ctx.device.TIM3.pwm(320.hz(), &mut rcc);
        let mut heater = pwm.bind_pin(port_a.pa7);
        heater.set_duty(0);
        heater.enable();

        let mut adc = ctx.device.ADC.constrain(&mut rcc);
        adc.set_sample_time(adc::SampleTime::T_80);
        adc.set_precision(adc::Precision::B_12);
        adc.set_oversampling_ratio(adc::OversamplingRatio::X_256);
        adc.set_oversampling_shift(20);
        adc.oversampling_enable(true);

        let mut delay = ctx.device.TIM1.delay(&mut rcc);
        delay.delay(100.us());
        adc.calibrate();

        let temp_pin = port_a.pa1.into_analog();

        let reg = Regulator::new(heater.get_max_duty());

        let temp_cal = SensorCalibration::new(1_900, DF::new(1_173, 10));

        let uart_cfg = serial::BasicConfig::default().baudrate(115_200.bps());
        let mut uart = ctx
            .device
            .USART2
            .usart(port_a.pa2, port_a.pa3, uart_cfg, &mut rcc)
            .expect("Failed to init serial port");
        uart.listen(serial::Event::Rxne);

        let shell = UShell::new(uart, autocomplete::NoAutocomplete, LRUHistory::default());

        (
            Shared {
                reg,
                timer,
                heater,
                freq: 0,
                on: false,
                log: false,
            },
            Local {
                adc,
                shell,
                temp_cal,
                temp_pin,
            },
            init::Monotonics(),
        )
    }

    #[task(capacity = 8, local = [shell], shared = [freq, on, log, reg, timer, heater], priority = 1)]
    fn env(ctx: env::Context, sig: EnvSignal) {
        let mut env = ctx.shared;
        env.on_signal(ctx.local.shell, sig).ok();
    }

    #[task(binds = USART2, priority = 1)]
    fn uart_rx(_: uart_rx::Context) {
        env::spawn(EnvSignal::SpinShell).ok();
    }

    #[task(binds = TIM2, local = [adc, temp_cal, temp_pin], shared = [log, reg, timer, heater], priority = 2)]
    fn timer_tick(ctx: timer_tick::Context) {
        let timer_tick::SharedResources {
            mut reg,
            mut timer,
            mut log,
            mut heater,
        } = ctx.shared;
        let timer_tick::LocalResources {
            adc,
            temp_cal,
            temp_pin,
        } = ctx.local;

        let temp = temp_cal.scale(adc.read(temp_pin).unwrap_or(0));
        let duty = reg.lock(|reg| reg.update(temp));
        heater.lock(|heater| heater.set_duty(duty));

        if log.lock(|log| *log) {
            env::spawn(EnvSignal::LogState).ok();
        }

        timer.lock(|timer| timer.clear_irq());
    }
}
