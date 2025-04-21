#[derive(Debug, Clone)]
pub struct Animator {
    pub current_time: f64,
    pub start_time: f64,
    pub end_time: f64,
    pub speed: f64,
    instant: iced::time::Instant,
    pub dt: f64,
    status: AnimatorStatus,
}

impl Animator {
    pub fn new(start_time: f64, end_time: f64) -> Self {
        Self {
            start_time,
            end_time,
            current_time: start_time,
            speed: 1.0,
            instant: iced::time::Instant::now(),
            dt: 0.0,
            status: AnimatorStatus::Playing,
        }
    }

    pub fn start(&mut self) {
        self.status = AnimatorStatus::Playing;
    }

    pub fn stop(&mut self) {
        self.status = AnimatorStatus::Paused;
    }

    pub fn update(&mut self, instant: &iced::time::Instant) {
        match self.status {
            AnimatorStatus::Paused => {}
            AnimatorStatus::Playing => {
                self.dt = instant.duration_since(self.instant).as_secs_f64();
                self.current_time += self.speed * self.dt;
                //rollover by default for now;
                if self.current_time > self.end_time {
                    self.current_time = self.start_time;
                }
            }
        }

        self.instant = instant.clone();
    }
}

#[derive(Debug, Clone)]
enum AnimatorStatus {
    Playing,
    Paused,
}
