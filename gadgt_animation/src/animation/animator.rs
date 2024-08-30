#[derive(Debug, Clone)]
pub struct Animator {
    pub current_time: f32,
    start_time: f32,
    end_time: f32,
    speed: f32,
    instant: iced::time::Instant,
    pub dt: f32,
}

impl Animator {
    pub fn new(start_time: f32, end_time: f32) -> Self {
        Self {
            start_time,
            end_time,
            current_time: start_time,
            speed: 1.0,
            instant: iced::time::Instant::now(),
            dt: 0.0,
        }
    }

    pub fn start(&mut self) {
        self.instant = iced::time::Instant::now();        
    }

    pub fn update(&mut self, instant: iced::time::Instant) {
        self.dt = instant.duration_since(self.instant).as_secs_f32();
        self.current_time += self.speed * self.dt;
        //rollover by default for now;
        if self.current_time > self.end_time {
            self.current_time = self.start_time;
        }
        self.instant = instant;
    }
}

impl Default for Animator {
    fn default() -> Self {
        Self {
            start_time: 0.0,
            end_time: 10.0,
            current_time: 0.0,
            speed: 1.0,
            instant: iced::time::Instant::now(),
            dt: 0.0,
        }
    }
}
