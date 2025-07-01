/// Specifies the type of step size control strategy used by the ODE solver.
///
/// - `Fixed`: Uses a constant step size throughout integration.
/// - `Adaptive`: Dynamically adjusts step size based on local error estimates.
#[derive(Copy, Clone)]
pub enum StepMethods {
    Fixed(FixedStepControl),
    Adaptive(AdaptiveStepControl),
}

impl From<FixedStepControl> for StepMethods {
    fn from(value: FixedStepControl) -> Self {
        Self::Fixed(value)
    }
}

impl From<AdaptiveStepControl> for StepMethods {
    fn from(value: AdaptiveStepControl) -> Self {
        Self::Adaptive(value)
    }
}

/// Fixed-step control configuration.
///
/// This struct specifies the time step `dt` and tracks the next scheduled time
/// (used primarily for internal logic).
#[derive(Copy, Clone)]
pub struct FixedStepControl {
    /// Constant step size.
    pub dt: f64,
    /// Next integration time (managed internally).
    pub next_time: f64,
}

impl FixedStepControl {
    /// Constructs a new fixed-step controller with a given step size.
    pub fn new(dt: f64) -> Self {
        Self { dt, next_time: 0.0 }
    }
}

/// Adaptive step size controller.
///
/// Uses an internal method (basic or PID) to adjust step size based on
/// normalized error. Provides absolute and relative tolerance configuration.
#[derive(Clone, Copy)]
pub struct AdaptiveStepControl {
    method: AdaptiveStepMethods,
    /// Relative tolerance used for error estimation.
    pub rel_tol: f64,
    /// Absolute tolerance used for error estimation.
    pub abs_tol: f64,
    /// Optional minimum allowed step size.
    pub min_dt: Option<f64>,
    /// Optional maximum allowed step size.
    pub max_dt: Option<f64>,
}

impl Default for AdaptiveStepControl {
    fn default() -> Self {
        Self {
            method: AdaptiveStepMethods::Basic,
            rel_tol: 1e-3,
            abs_tol: 1e-6,
            min_dt: None,
            max_dt: None,
        }
    }
}

impl AdaptiveStepControl {
    /// Computes the next step size based on the current step and estimated error.
    ///
    /// - `dt`: current step size
    /// - `error`: normalized RMS error
    /// - `order`: order of the solver
    pub fn step(&mut self, dt: f64, error: f64, order: usize) -> f64 {
        match &mut self.method {
            AdaptiveStepMethods::Basic => 0.9 * dt * (1.0 / error).powf(1.0 / (order as f64 - 1.0)),
            AdaptiveStepMethods::PID(pid) => pid.step(dt, error),
        }
    }

    pub fn with_abs_tol(mut self, abs_tol: f64) -> Self {
        self.abs_tol = abs_tol;
        self
    }

    pub fn with_rel_tol(mut self, rel_tol: f64) -> Self {
        self.rel_tol = rel_tol;
        self
    }

    pub fn with_min_dt(mut self, min_dt: f64) -> Self {
        self.min_dt = Some(min_dt);
        self
    }

    pub fn with_max_dt(mut self, max_dt: f64) -> Self {
        self.max_dt = Some(max_dt);
        self
    }
}

/// Enum for selecting the adaptive step control algorithm.
///
/// - `Basic`: Simple step control using embedded error estimates.
/// - `PID`: PID-controlled step size adaptation.
#[derive(Clone, Copy)]
pub enum AdaptiveStepMethods {
    Basic,
    PID(StepPIDControl),
}

/// PID controller for adaptive step size control.
///
/// Allows fine-grained control over how the step size changes in response
/// to recent error estimates.
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
            kp: 0.6,
            ki: 0.01,
            kd: 0.175,
            min_dt: None,
            max_dt: None,
            min_growth: Some(0.1),
            max_growth: Some(5.0),
            err_now: 1.0,
            err_prev: 1.0,
            err_prevprev: 1.0,
        }
    }
}

impl StepPIDControl {
    /// Creates a new PID step controller with custom gains and limits.
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
            err_now: 1.0,
            err_prev: 1.0,
            err_prevprev: 1.0,
        }
    }

    /// Updates internal tolerances.
    pub fn with_tolerances(mut self, rel_tol: f64, abs_tol: f64) -> Self {
        self.rel_tol = rel_tol;
        self.abs_tol = abs_tol;
        self
    }

    /// Computes the next step size based on PID control of recent error history.
    pub fn step(&mut self, h: f64, err_now: f64) -> f64 {
        self.err_prevprev = self.err_prev;
        self.err_prev = self.err_now;
        self.err_now = err_now;

        const EPS: f64 = 1e-14;
        let e0 = self
            .err_now
            .max(EPS);
        let e1 = self
            .err_prev
            .max(EPS);
        let e2 = self
            .err_prevprev
            .max(EPS);

        let mut factor = e0.powf(self.kp) * (e0 / e1).powf(self.kd) * (e1 / e2).powf(self.ki);

        if let Some(min_growth) = self.min_growth {
            factor = factor.max(min_growth);
        }
        if let Some(max_growth) = self.max_growth {
            factor = factor.min(max_growth);
        }

        let mut new_dt = h * factor;

        if let Some(min_dt) = self.min_dt {
            new_dt = new_dt.max(min_dt);
        }
        if let Some(max_dt) = self.max_dt {
            new_dt = new_dt.min(max_dt);
        }

        new_dt
    }
}
