use crate::*;
use core::fmt::Write;
use rtic::Mutex;
use ushell::*;

pub type Uart = serial::Serial<stm32::USART2, serial::BasicConfig>;
pub type Autocomplete = autocomplete::NoAutocomplete;
pub type History = LRUHistory<24, 32>;
pub type Shell = UShell<Uart, Autocomplete, History, 24>;
pub type Env<'a> = app::env::SharedResources<'a>;
pub type EnvResult = SpinResult<Uart, ()>;

pub const BAD_INPUT_ERR: &'static str = "\r\nbad input\r\n";

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
                    "Δ {: >5} | sp: {} m°C | t: {} m°C | {} Δ[{}]\r\n",
                    target_temp as i32 - last_temp as i32,
                    target_temp,
                    last_temp,
                    duty,
                    duty as i32 - last_duty as i32,
                )
                .ok();
                Ok(())
            }
        }
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
                shell.write_str("\r\n")?;
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
                shell.write_str("\r\n")?;
            }
            _ => shell.write_str(BAD_INPUT_ERR)?,
        }
        Ok(())
    }

    fn stop_cmd(&mut self, shell: &mut Shell, _args: &str) -> ShellResult<Uart> {
        self.timer.lock(|timer| timer.reset());
        self.freq.lock(|f| *f = 0);
        shell.write_str("\r\n")?;
        Ok(())
    }

    fn log_cmd(&mut self, shell: &mut Shell, _args: &str) -> ShellResult<Uart> {
        self.log.lock(|log| *log = true);
        shell.write_str("\r\n")?;
        Ok(())
    }

    fn status_cmd(&mut self, shell: &mut Shell, _args: &str) -> ShellResult<Uart> {
        let freq = self.freq.lock(|freq| *freq);
        self.reg.lock(|reg| {
            write!(
                shell,
                "\r\nStatus:\t{}\r\nFreq:\t{} Hz\r\nSP:\t{} m°C\r\nFB:\t{} m°C\r\nkp:\t{}\r\nki:\t{}\r\nkd:\t{}\r\nkf:\t{}\r\nkv:\t{}\r\n",
                if freq > 0 { "On" } else { "Off" },
                freq,
                reg.target_temp,
                reg.last_temp,
                reg.pid.kp,
                reg.pid.ki,
                reg.pid.kd,
                reg.pid.kf,
                reg.pid.kv,
            )
            .ok();
        });

        Ok(())
    }
}

impl Environment<Uart, Autocomplete, History, (), 24> for Env<'_> {
    fn command(&mut self, shell: &mut Shell, cmd: &str, args: &str) -> EnvResult {
        match cmd {
            "clear" => shell.clear()?,
            "start" => self.start_cmd(shell, args)?,
            "stop" => self.stop_cmd(shell, args)?,
            "set" => self.set_cmd(shell, args)?,
            "log" => self.log_cmd(shell, args)?,
            "status" => self.status_cmd(shell, args)?,
            "" => shell.write_str("\r\n")?,
            _ => write!(shell, "\r\nunsupported command: \"{}\"\r\n", cmd)?,
        }
        shell.write_str("» ")?;
        Ok(())
    }

    fn control(&mut self, shell: &mut Shell, code: u8) -> EnvResult {
        match code {
            control::CTRL_C => {
                self.log.lock(|log| *log = false);
                shell.write_str("\r\n» ")?;
            }
            _ => {
                shell.bell()?;
            }
        }
        Ok(())
    }
}
