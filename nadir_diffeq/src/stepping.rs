#[derive(Copy, Clone)]
pub enum StepMethod {
    Fixed(FixedStepControl),
    Adaptive(StepPIDControl),
}

#[derive(Copy, Clone)]
pub struct FixedStepControl {
    pub dt: f64,
    pub next_time: f64,
}

impl FixedStepControl {
    pub fn new(dt: f64) -> Self {
        Self { dt, next_time: 0.0 }
    }
}

#[derive(Clone, Copy)]
pub struct StepPIDControl {
    pub rel_tol: f64,
    pub abs_tol: f64,
    kp: f64,
    ki: f64,
    kd: f64,
    pub min_dt: Option<f64>,
    pub max_dt: Option<f64>,
    min_growth: Option<f64>,
    max_growth: Option<f64>,
    err_now: f64,
    err_prev: f64,
    err_prevprev: f64,
}

impl Default for StepPIDControl {
    fn default() -> Self {
        Self {
            rel_tol: 1e-3,
            abs_tol: 1e-6,
            kp: 0.075,
            ki: 0.01,
            kd: 0.175,
            min_dt: None,
            max_dt: None,
            min_growth: Some(0.1),
            max_growth: Some(5.0),
            err_now: 1.0, //1.0 ensures small initial steps and no divide by 0
            err_prev: 1.0,
            err_prevprev: 1.0,
        }
    }
}

impl StepPIDControl {
    pub fn new(
        rel_tol: f64,
        abs_tol: f64,
        kp: f64,
        ki: f64,
        kd: f64,
        min_dt: Option<f64>,
        max_dt: Option<f64>,
        min_growth: Option<f64>,
        max_growth: Option<f64>,
    ) -> Self {
        Self {
            rel_tol,
            abs_tol,
            kp,
            ki,
            kd,
            min_dt,
            max_dt,
            min_growth,
            max_growth,
            err_now: 1.0, //1.0 ensures small initial steps and no divide by 0
            err_prev: 1.0,
            err_prevprev: 1.0,
        }
    }

    pub fn with_tolerances(mut self, rel_tol: f64, abs_tol: f64) -> Self {
        self.rel_tol = rel_tol;
        self.abs_tol = abs_tol;
        self
    }

    pub fn step(&mut self, h: f64, err_now: f64) -> f64 {
        self.err_prevprev = self.err_prev;
        self.err_prev = self.err_now;
        self.err_now = err_now;

        // divide by 0 protection
        const EPS: f64 = 1e-14;
        let e0 = self.err_now.max(EPS);
        let e1 = self.err_prev.max(EPS);
        let e2 = self.err_prevprev.max(EPS);

        // calculate the growth factor
        let mut factor = e0.powf(-self.kp) * (e0 / e1).powf(-self.kd) * (e1 / e2).powf(self.ki);

        // limit the growth of the step size
        if let Some(min_growth) = self.min_growth {
            factor = factor.max(min_growth);
        }
        if let Some(max_growth) = self.max_growth {
            factor = factor.min(max_growth);
        }

        let mut new_dt = h * factor;
        // limit the step size
        if let Some(min_dt) = self.min_dt {
            new_dt = new_dt.max(min_dt);
        }
        if let Some(max_dt) = self.max_dt {
            new_dt = new_dt.min(max_dt);
        }
        new_dt
    }
}
