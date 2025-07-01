use nadir_diffeq::{
    model::OdeModel,
    monte_carlo::{MonteCarloProblem, MonteCarloSolver},
    solvers::RungeKuttaMethods,
    state::state_array::{StateArray, UncertainStateArray},
    stepping::AdaptiveStepControl,
};
use plotting::{QuickPlot, figure::Figure, line::Line, theme::PlotThemes};
use std::error::Error;
use uncertainty::{Normal, UncertainValue, Uncertainty};

#[derive(Debug, Clone)]
struct DampedOscillator {
    spring_constant: f64,
    damping: f64,
}

impl OdeModel for DampedOscillator {
    type State = StateArray<2>;

    fn f(
        &mut self,
        _t: f64,
        x: &StateArray<2>,
        dx: &mut StateArray<2>,
    ) -> Result<(), Box<dyn Error>> {
        dx[0] = x[1];
        dx[1] = -self.spring_constant * x[0] - self.damping * x[1];
        Ok(())
    }
}

#[derive(Clone)]
struct UncertainDampedOscillator {
    spring_constant: UncertainValue,
    damping: UncertainValue,
}

impl Uncertainty for UncertainDampedOscillator {
    type Output = DampedOscillator;
    type Error = ();
    fn sample(
        &self,
        nominal: bool,
        rng: &mut rand::prelude::SmallRng,
    ) -> Result<Self::Output, Self::Error> {
        Ok(DampedOscillator {
            spring_constant: self
                .spring_constant
                .sample(nominal, rng),
            damping: self
                .damping
                .sample(nominal, rng),
        })
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let model = UncertainDampedOscillator {
        spring_constant: UncertainValue::new(1.0)
            .with_distribution(Normal::new(1.0, 0.1 / 3.0)?.into())?,
        damping: UncertainValue::new(0.1).with_distribution(Normal::new(0.1, 0.1 / 3.0)?.into())?,
    };
    let problem = MonteCarloProblem::new(model, 1000);
    let solver = MonteCarloSolver::new(RungeKuttaMethods::Tsit5.into());
    let x0 = UncertainStateArray([
        UncertainValue::new(1.0).with_distribution(Normal::new(1.0, 0.1)?.into())?,
        UncertainValue::new(0.0),
    ]);

    let results = solver.solve_adaptive(
        problem,
        x0,
        (0.0, 10.0),
        AdaptiveStepControl::default().with_max_dt(0.1),
    )?;

    if let Some(results) = results {
        let theme = PlotThemes::Dark.palette();
        let mut f = Figure::new();
        let a = f.get_axes(0)?;

        for i in 0..results.len() {
            let result = &results[i];
            let y: Vec<f64> = result
                .y
                .iter()
                .map(|x| x[0])
                .collect();

            a.add_line(
                Line::new(&result.t, &y)?
                    .with_color(theme.line_colors[0])
                    .with_alpha(0.01)?
                    .with_legend(false),
            )
        }
        QuickPlot::plot(f)?;
    }

    Ok(())
}
