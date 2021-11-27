use crate::*;
use core::fmt::Write;
use rtic::Mutex;
use ushell::*;

pub type Uart = serial::Serial<stm32::USART2, serial::BasicConfig>;
pub type Autocomplete = autocomplete::StaticAutocomplete<11>;
pub type History = LRUHistory<24, 24>;
pub type Shell = UShell<Uart, Autocomplete, History, 24>;
pub type Env<'a> = app::env::SharedResources<'a>;
pub type EnvResult = SpinResult<Uart, ()>;

pub enum EnvSignal {
    SpinShell,
    LogState,
}

impl Env<'_> {
    pub fn on_signal(&mut self, shell: &mut Shell, sig: EnvSignal) -> EnvResult {
        match sig {
            EnvSignal::SpinShell => shell.spin(self),
            EnvSignal::LogState => {
                let (target_temp, last_temp, duty, last_duty) = self
                    .reg
                    .lock(|reg| (reg.target_temp, reg.last_temp, reg.duty(), reg.last_duty));

                write!(
                    shell,
                    "Δ {: <6} | sp: {} m°C | fb: {} m°C | {: >5} [{}] {}",
                    target_temp as i32 - last_temp as i32,
                    target_temp,
                    last_temp,
                    duty,
                    duty as i32 - last_duty as i32,
                    CR
                )
                .ok();
                Ok(())
            }
        }
    }

    fn reset_cmd(&mut self, shell: &mut Shell, _args: &str) -> ShellResult<Uart> {
        self.reg.lock(|reg| reg.pid.reset());
        shell.write_str(CR)?;
        Ok(())
    }

    fn set_cmd(&mut self, shell: &mut Shell, args: &str) -> ShellResult<Uart> {
        match args.split_once(" ") {
            Some((k, gain)) => {
                let gain = match gain.split_once(" ") {
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
                self.timer.lock(|timer| timer.start(freq.hz()));
                self.reg.lock(|reg| reg.pid.reset());
                self.freq.lock(|f| *f = freq);
                shell.write_str(CR)?;
            }
            _ => shell.write_str(BAD_INPUT_ERR)?,
        }
        Ok(())
    }

    fn stop_cmd(&mut self, shell: &mut Shell, _args: &str) -> ShellResult<Uart> {
        self.heater.lock(|heater| heater.set_duty(0));
        self.timer.lock(|timer| timer.pause());
        self.freq.lock(|freq| *freq = 0);
        shell.write_str(CR)?;
        Ok(())
    }

    fn trace_cmd(&mut self, _shell: &mut Shell, _args: &str) -> ShellResult<Uart> {
        self.trace.lock(|trace| *trace = true);
        Ok(())
    }

    fn target_cmd(&mut self, shell: &mut Shell, args: &str) -> ShellResult<Uart> {
        match btoi::btoi::<i32>(args.as_bytes()) {
            Ok(temp) if temp >= TEMP_MIN && temp <= TEMP_MAX => {
                self.reg.lock(|reg| reg.target_temp = temp);
                shell.write_str(CR)?;
            }
            _ => shell.write_str(BAD_INPUT_ERR)?,
        }
        Ok(())
    }

    fn info_cmd(&mut self, shell: &mut Shell, _args: &str) -> ShellResult<Uart> {
        let freq = self.freq.lock(|freq| *freq);
        let (target_temp, last_temp, kp, ki, kd, kf, kv) = self.reg.lock(|reg| {
            (
                reg.target_temp,
                reg.last_temp,
                reg.pid.kp,
                reg.pid.ki,
                reg.pid.kd,
                reg.pid.kf,
                reg.pid.kv,
            )
        });

        write!(shell, "{0}reg:\t{1}{0}f:\t{2} Hz{0}sp:\t{3} m°C{0}fb:\t{4} m°C{0}kp:\t{5}{0}ki:\t{6}{0}kd:\t{7}{0}kf:\t{8}{0}kv:\t{9}{0}",
            CR,
            if freq > 0 { "on" } else { "off" },
            freq,
            target_temp,
            last_temp,
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
            "info" => self.info_cmd(shell, args)?,
            "reset" => self.reset_cmd(shell, args)?,
            "set" => self.set_cmd(shell, args)?,
            "start" => self.start_cmd(shell, args)?,
            "stop" => self.stop_cmd(shell, args)?,
            "trace" => self.trace_cmd(shell, args)?,
            "target" => self.target_cmd(shell, args)?,
            "" => shell.write_str(CR)?,
            _ => write!(shell, "{0}unsupported command: \"{1}\"{0}", CR, cmd)?,
        }
        shell.write_str(SHELL_PROMPT)?;
        Ok(())
    }

    fn control(&mut self, shell: &mut Shell, code: u8) -> EnvResult {
        match code {
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
    "clear",
    "help ",
    "help pinout",
    "help usage",
    "info",
    "reset",
    "set k",
    "start ",
    "stop",
    "target",
    "trace",
]);

const BAD_INPUT_ERR: &'static str = "\r\nbad input\r\n";
const CR: &str = "\r\n";
const SHELL_PROMPT: &str = "\x1b[35m» \x1b[0m";
const HELP: &str = "\r\n\
\x1b[33mPID Shell \x1b[32mv0.0.1\x1b[0m\r\n\r\n\
COMMANDS:\r\n\
\x20 start [freq]         Start regulator with provided frequency\r\n\
\x20 stop                 Stop regulator\r\n\
\x20 target <temp>        Set target temperature in m°C\r\n\
\x20 set k<x> <n> (dp)    Set PID coefficients(kp, ki, kd, kf, kv)\r\n\
\x20 reset                Reser regulator internal state\r\n\
\x20 trace                Enable serial logger, Ctrl+C to toggle\r\n\
\x20 info                 Print regulator status\r\n\
\x20 help [pinout|usage]  Print help message\r\n\
\x20 clear                Clear screen\r\n\r\n\
CONTROL KEYS:\r\n\
\x20 Ctrl+C               Toggle serial logger\r\n\r\n\
";

const USAGE: &str = "\r\n\
USAGE EXAMPLES:\r\n\
\x20 help pinout     Print pinout\r\n\
\x20 target 3400     Set targer temperature to 34°C\r\n\
\x20 start 4         Start regulator with 4 Hz frequency\r\n\
\x20 stop            Stop regulator\r\n\
\x20 trace           Enable serial logger\r\n\
\x20 info            Print regulator status\r\n\
\x20 set fp 75 11    Set proportional gain to 75/2048\r\n\
\x20 set fi 2        Set integral gain to 2\r\n\
\x20 set fd 51 8     Set derivative gain to 51/256\r\n\
\x20 set fv 12 10    Set velocity gain to 12/1024\r\n\r\n\
";

const PINOUT: &str = "\r\n\
\x20             STM32G031F6  \r\n\
\x20            ╔═══════════╗ \r\n\
\x20    PB7|PB8 ╣1 ¤      20╠ PB3|PB4|PB5|PB6    \r\n\
\x20   PC9|PC14 ╣2        19╠ PA14|PA15   (SWDIO)\r\n\
\x20       PC15 ╣3        18╠ PA13       (SWDCLK)\r\n\
\x20        Vdd ╣4        17╠ PA12[PA10]         \r\n\
\x20        Vss ╣5        16╠ PA11[PA9]          \r\n\
\x20       nRst ╣6        15╠ PA8|PB0|PB1|PB2    \r\n\
\x20        PA0 ╣7        14╠ PA7           (PWM)\r\n\
\x20 (ADC)  PA1 ╣8        13╠ PA6                \r\n\
\x20 (TX)   PA2 ╣9        12╠ PA5                \r\n\
\x20 (RX)   PA3 ╣10       11╠ PA4                \r\n\
\x20            ╚═══════════╝ \r\n\r\n\
";
