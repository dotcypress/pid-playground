use dyadic::DF;
use pid_loop::PID;

pub const TEMP_MIN: i32 = 18_000;
pub const TEMP_MAX: i32 = 48_000;

pub const PID_WINDOW: usize = 8;
pub const KP: DF = DF::new(28, 12);
pub const KI: DF = DF::new(2, 12);
pub const KD: DF = DF::new(1, 12);

pub struct SensorCalibration {
    offset: i32,
    gain: DF,
}

impl SensorCalibration {
    pub fn new(offset: i32, gain: DF) -> Self {
        Self { offset, gain }
    }

    pub fn scale(&self, raw_temp: u32) -> i32 {
        let temp = self.gain.scale(raw_temp as i32 - self.offset);
        let temp = TEMP_MAX - (temp * (TEMP_MAX - TEMP_MIN) / 65_535);
        temp.clamp(TEMP_MIN, TEMP_MAX)
    }
}

pub struct Regulator {
    pub duty: DF,
    pub target_temp: i32,
    pub last_temp: i32,
    pub last_duty: u32,
    pub pid: PID<DF, { PID_WINDOW }>,
    max_duty: u32,
}

impl Regulator {
    pub fn new(max_duty: u32) -> Self {
        Self {
            max_duty,
            duty: DF::from(5_100),
            target_temp: 30_000,
            last_temp: 0,
            last_duty: 0,
            pid: PID::new(KP, KI, KD, 0, 0),
        }
    }

    pub fn update(&mut self, fb: i32) -> u32 {
        self.last_temp = fb;
        self.last_duty = self.duty();
        self.duty += self.pid.next(self.target_temp, fb);
        self.duty()
    }

    pub fn duty(&self) -> u32 {
        let duty: i32 = self.duty.into();
        u32::clamp(duty.abs() as u32, 0, self.max_duty)
    }
}
