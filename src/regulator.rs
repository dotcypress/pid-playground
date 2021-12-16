use dyadic::DF;
use pid_loop::PID;

pub const PID_WINDOW: usize = 16;

pub struct Regulator {
    pub pid: PID<DF, { PID_WINDOW }>,
    pub sp: i32,
    pub last: i32,
    pub duty: DF,
    pub last_duty: DF,
    pub max_duty: DF,
}

impl Regulator {
    pub fn new(max_duty: impl Into<DF>) -> Self {
        Self {
            pid: PID::new(DF::new(1, 8), 0, 0, 0, 0),
            duty: DF::zero(),
            last_duty: DF::zero(),
            max_duty: max_duty.into(),
            sp: 32_000,
            last: 0,
        }
    }

    pub fn update(&mut self, fb: i32) -> u32 {
        let duty = self.duty + self.pid.next(self.sp, fb);
        let duty = duty.round(14).clamp(DF::zero(), self.max_duty);
        self.last = fb;
        self.last_duty = self.duty;
        self.duty = duty;
        self.duty.floor() as _
    }

    pub fn reset(&mut self) {
        self.duty = DF::zero();
        self.pid.reset();
    }
}
