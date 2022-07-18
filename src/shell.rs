use crate::*;
use core::fmt::Write;
use rtic::Mutex;
use ushell::*;

pub type Uart = serial::Serial<stm32::USART2, serial::BasicConfig>;
pub type Autocomplete = autocomplete::StaticAutocomplete<12>;
pub type History = LRUHistory<24, 24>;
pub type Shell = UShell<Uart, Autocomplete, History, 24>;
pub type Env<'a> = app::env::SharedResources<'a>;
pub type EnvResult = SpinResult<Uart, ()>;

pub enum EnvSignal {
    SpinShell,
    TraceState,
}

impl Env<'_> {
    pub fn on_signal(&mut self, shell: &mut Shell, sig: EnvSignal) -> EnvResult {
        match sig {
            EnvSignal::TraceState => self.print_trace(shell),
            EnvSignal::SpinShell => shell.spin(self),
        }
    }

    fn print_trace(&mut self, shell: &mut Shell) -> EnvResult {
        let (sp, last, duty, last_duty) = self
            .reg
            .lock(|reg| (reg.sp, reg.last, reg.duty, reg.last_duty));
        write!(
            shell,
            "Δ {: <6} | sp: {} | fb: {} | {: >5} [{}]{}",
            sp - last,
            sp,
            last,
            duty.floor(),
            duty.floor() - last_duty.floor(),
            CR
        )
        .ok();
        Ok(())
    }

    fn reset_cmd(&mut self, shell: &mut Shell, _args: &str) -> ShellResult<Uart> {
        self.reg.lock(|reg| reg.reset());
        self.pwm.lock(|pwm| pwm.set_duty(0));
        shell.write_str(CR)?;
        Ok(())
    }

    fn set_cmd(&mut self, shell: &mut Shell, args: &str) -> ShellResult<Uart> {
        match args.split_once(' ') {
            Some((k, gain)) => {
                let gain = match gain.split_once(' ') {
                    Some((num, den_pwr)) => {
                        match (
                            btoi::btoi::<i32>(num.as_bytes()),
                            btoi::btoi::<i8>(den_pwr.as_bytes()),
                        ) {
                            (Ok(num), Ok(den_pwr)) => DF::new(num, den_pwr),
                            _ => {
                                shell.write_str(BAD_INPUT_ERR)?;
                                return Ok(());
                            }
                        }
                    }
                    None => match btoi::btoi::<i32>(gain.as_bytes()) {
                        Ok(num) => DF::from(num),
                        _ => {
                            shell.write_str(BAD_INPUT_ERR)?;
                            return Ok(());
                        }
                    },
                };

                match k {
                    "kp" => self.reg.lock(|reg| reg.pid.kp = gain),
                    "ki" => self.reg.lock(|reg| reg.pid.ki = gain),
                    "kd" => self.reg.lock(|reg| reg.pid.kd = gain),
                    "kf" => self.reg.lock(|reg| reg.pid.kf = gain),
                    "kv" => self.reg.lock(|reg| reg.pid.kv = gain),
                    _ => {
                        shell.write_str(BAD_INPUT_ERR)?;
                        return Ok(());
                    }
                }
                shell.write_str(CR)?;
            }
            _ => shell.write_str(BAD_INPUT_ERR)?,
        }

        Ok(())
    }

    fn start_cmd(&mut self, shell: &mut Shell, args: &str) -> ShellResult<Uart> {
        match btoi::btoi::<u32>(args.as_bytes()) {
            Ok(freq) if freq <= 1_000 => {
                self.freq.lock(|f| *f = freq);
                self.reg.lock(|reg| reg.pid.reset());
                self.timer.lock(|timer| timer.start(freq.hz()));
                shell.write_str(CR)?;
            }
            _ => shell.write_str(BAD_INPUT_ERR)?,
        }
        Ok(())
    }

    fn inverse_cmd(&mut self, shell: &mut Shell, args: &str) -> ShellResult<Uart> {
        match args {
            "on" => self.inverse.lock(|inverse| *inverse = true),
            "off" => self.inverse.lock(|inverse| *inverse = false),
            _ => shell.write_str(BAD_INPUT_ERR)?,
        }
        shell.write_str(CR)?;
        Ok(())
    }

    fn stop_cmd(&mut self, shell: &mut Shell, _args: &str) -> ShellResult<Uart> {
        self.pwm.lock(|pwm| pwm.set_duty(0));
        self.timer.lock(|timer| timer.pause());
        self.freq.lock(|freq| *freq = 0);
        shell.write_str(CR)?;
        Ok(())
    }

    fn trace_cmd(&mut self, _: &mut Shell, _args: &str) -> ShellResult<Uart> {
        self.trace.lock(|trace| *trace = true);
        Ok(())
    }

    fn target_cmd(&mut self, shell: &mut Shell, args: &str) -> ShellResult<Uart> {
        match btoi::btoi::<i32>(args.as_bytes()) {
            Ok(val) => {
                self.reg.lock(|reg| reg.sp = val);
                shell.write_str(CR)?;
            }
            _ => shell.write_str(BAD_INPUT_ERR)?,
        }
        Ok(())
    }

    fn print_config_cmd(&mut self, shell: &mut Shell, _args: &str) -> ShellResult<Uart> {
        let freq = self.freq.lock(|freq| *freq);
        let inverse = self.inverse.lock(|inverse| *inverse);
        let (sp, last, kp, ki, kd, kf, kv) = self.reg.lock(|reg| {
            (
                reg.sp, reg.last, reg.pid.kp, reg.pid.ki, reg.pid.kd, reg.pid.kf, reg.pid.kv,
            )
        });

        write!(shell, "{0}reg:\t{1}{0}f:\t{2} Hz{0}adc:\t{3}{0}sp:\t{4}{0}fb:\t{5}{0}kp:\t{6}{0}ki:\t{7}{0}kd:\t{8}{0}kf:\t{9}{0}kv:\t{10}{0}",
            CR,
            if freq > 0 { "on" } else { "off" },
            freq,
            if inverse { "inverted" } else { "non inverted" },
            sp,
            last,
            kp,
            ki,
            kd,
            kf,
            kv,
        ).ok();

        Ok(())
    }

    fn help_cmd(&mut self, shell: &mut Shell, args: &str) -> EnvResult {
        match args {
            "pinout" => shell.write_str(PINOUT)?,
            "usage" => shell.write_str(USAGE)?,
            _ => shell.write_str(HELP)?,
        }
        Ok(())
    }
}

impl Environment<Uart, Autocomplete, History, (), 24> for Env<'_> {
    fn command(&mut self, shell: &mut Shell, cmd: &str, args: &str) -> EnvResult {
        match cmd {
            "clear" => shell.clear()?,
            "help" => self.help_cmd(shell, args)?,
            "config" => self.print_config_cmd(shell, args)?,
            "reset" => self.reset_cmd(shell, args)?,
            "set" => self.set_cmd(shell, args)?,
            "start" => self.start_cmd(shell, args)?,
            "stop" => self.stop_cmd(shell, args)?,
            "trace" => self.trace_cmd(shell, args)?,
            "target" => self.target_cmd(shell, args)?,
            "inverse" => self.inverse_cmd(shell, args)?,
            "" => shell.write_str(CR)?,
            _ => write!(shell, "{0}unsupported command: \"{1}\"{0}", CR, cmd)?,
        }
        shell.write_str(SHELL_PROMPT)?;
        Ok(())
    }

    fn control(&mut self, shell: &mut Shell, code: u8) -> EnvResult {
        match code {
            control::CTRL_X => self.stop_cmd(shell, "")?,
            control::CTRL_C => {
                self.trace.lock(|trace| *trace = !*trace);
                shell.write_str(SHELL_PROMPT)?;
            }
            _ => {
                shell.bell()?;
            }
        }
        Ok(())
    }
}

pub const AUTOCOMPLETE: Autocomplete = autocomplete::StaticAutocomplete([
    "config",
    "clear",
    "help ",
    "help pinout",
    "help usage",
    "inverse ",
    "reset",
    "set k",
    "start ",
    "stop",
    "target",
    "trace",
]);

const BAD_INPUT_ERR: &str = "\r\nbad input\r\n";
const CR: &str = "\r\n";
const SHELL_PROMPT: &str = "\x1b[35m» \x1b[0m";
const HELP: &str = "\r\n\
\x1b[33mPID Shell\x1b[0m v0.2\r\n\r\n\
COMMANDS:\r\n\
\x20 config               Print regulator config\r\n\
\x20 start <freq>         Start regulator with provided frequency\r\n\
\x20 stop                 Stop regulator\r\n\
\x20 target <val>         Set target\r\n\
\x20 set k<x> <n> (dp)    Set PID coefficients(kp, ki, kd, kf, kv)\r\n\
\x20 inverse <on|off>     Set ADC inversion mode\r\n\
\x20 reset                Reser regulator internal state\r\n\
\x20 trace                Enable serial logger\r\n\
\x20 help [pinout|usage]  Print help message\r\n\
\x20 clear                Clear screen\r\n\r\n\
CONTROL KEYS:\r\n\
\x20 Ctrl+C               Toggle serial logger\r\n\r\n\
\x20 Ctrl+X               Stop regulator\r\n\r\n\
";

const USAGE: &str = "\r\n\
USAGE EXAMPLES:\r\n\
\x20 help pinout     Print pinout\r\n\
\x20 target 3400     Set targer to 3400\r\n\
\x20 start 4         Start regulator with 4 Hz frequency\r\n\
\x20 stop            Stop regulator\r\n\
\x20 trace           Enable serial logger\r\n\
\x20 inverse off     Disable adc inversion\r\n\
\x20 config          Print regulator config\r\n\
\x20 set kp 75 11    Set proportional gain to 75/2048\r\n\
\x20 set ki 2        Set integral gain to 2\r\n\
\x20 set kd 51 8     Set derivative gain to 51/256\r\n\
\x20 set kv 12 10    Set velocity gain to 12/1024\r\n\r\n\
";

const PINOUT: &str = "\r\n\
\x20             STM32G0xxFx  \r\n\
\x20            ╔═══════════╗ \r\n\
\x20    PB7|PB8 ╣1 ¤      20╠ PB3|PB4|PB5|PB6      \r\n\
\x20   PC9|PC14 ╣2        19╠ PA14|PA15     (SWDIO)\r\n\
\x20       PC15 ╣3        18╠ PA13         (SWDCLK)\r\n\
\x20        Vdd ╣4        17╠ PA12[PA10]           \r\n\
\x20        Vss ╣5        16╠ PA11[PA9]            \r\n\
\x20       nRst ╣6        15╠ PA8|PB0|PB1|PB2 (PWM)\r\n\
\x20        PA0 ╣7        14╠ PA7           (PWM_P)\r\n\
\x20 (ADC)  PA1 ╣8        13╠ PA6           (PWM_N)\r\n\
\x20 (TX)   PA2 ╣9        12╠ PA5         (PHASE_P)\r\n\
\x20 (RX)   PA3 ╣10       11╠ PA4         (PHASE_N)\r\n\
\x20            ╚═══════════╝ \r\n\r\n\
";
