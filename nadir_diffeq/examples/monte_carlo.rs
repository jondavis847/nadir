use nadir_diffeq::{
    OdeModel,
    monte_carlo::{MonteCarloProblem, MonteCarloSolver},
    saving::MemoryResult,
    solvers::RungeKuttaMethods,
    state::state_array::{StateArray, UncertainStateArray},
    stepping::AdaptiveStepControl,
};
use show_image::{ImageInfo, ImageView, run_context};
use std::error::Error;
use uncertainty::{Normal, UncertainValue, Uncertainty};

#[derive(Debug)]
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
            .with_distribution(Normal::new(1.0, 0.1)?.into())?,
        damping: UncertainValue::new(0.1).with_distribution(Normal::new(0.1, 0.1)?.into())?,
    };
    let problem = MonteCarloProblem::new(model, 10);
    let solver = MonteCarloSolver::new(RungeKuttaMethods::Tsit5.into());
    let x0 = UncertainStateArray([
        UncertainValue::new(1.0).with_distribution(Normal::new(1.0, 0.1)?.into())?,
        UncertainValue::new(0.0),
    ]);

    let result = solver.solve_adaptive(
        problem,
        x0,
        (0.0, 10.0),
        AdaptiveStepControl::default().with_max_dt(0.1),
    )?;

    plot(result.unwrap())?;
    Ok(())
}

use plotters::backend::BitMapBackend;
use plotters::prelude::*;
use show_image::create_window;

fn plot(results: Vec<MemoryResult<StateArray<2>>>) -> Result<(), Box<dyn Error>> {
    let width = 1200;
    let height = 800;

    // Create an in-memory bitmap
    let mut buffer = vec![0u8; width * height * 3];
    {
        let root = BitMapBackend::with_buffer(
            &mut buffer,
            (width as u32, height as u32),
        )
        .into_drawing_area();
        root.fill(&WHITE)?;

        // Find the overall min/max across all results
        let mut t_min = f64::INFINITY;
        let mut t_max = f64::NEG_INFINITY;
        let mut pos_min = f64::INFINITY;
        let mut pos_max = f64::NEG_INFINITY;

        // Get the ranges from all results
        for result in &results {
            if let Some(first_t) = result
                .t
                .first()
            {
                t_min = t_min.min(*first_t);
            }
            if let Some(last_t) = result
                .t
                .last()
            {
                t_max = t_max.max(*last_t);
            }

            for state in &result.y {
                pos_min = pos_min.min(state[0]);
                pos_max = pos_max.max(state[0]);
            }
        }

        // Add a small margin to the ranges
        let t_margin = (t_max - t_min) * 0.05;
        let pos_margin = (pos_max - pos_min) * 0.05;

        let t_range = (t_min - t_margin)..(t_max + t_margin);
        let pos_range = (pos_min - pos_margin)..(pos_max + pos_margin);

        let mut chart = ChartBuilder::on(&root)
            .caption(
                "Damped Oscillator - Monte Carlo",
                ("Arial", 30),
            )
            .margin(20)
            .x_label_area_size(40)
            .y_label_area_size(60)
            .build_cartesian_2d(t_range, pos_range)?;

        chart
            .configure_mesh()
            .x_desc("Time")
            .y_desc("Position")
            .draw()?;

        // Define a color palette for the different lines
        let color_palette = [
            &RED,
            &BLUE,
            &GREEN,
            &CYAN,
            &MAGENTA,
            &YELLOW,
            &BLACK,
            &RGBColor(255, 165, 0),
            &RGBColor(128, 0, 128),
            &RGBColor(139, 69, 19),
        ];

        // Draw each result with a different color
        for (i, result) in results
            .iter()
            .enumerate()
        {
            let color_idx = i % color_palette.len();
            let color = color_palette[color_idx].stroke_width(2);

            let times: Vec<f64> = result
                .t
                .iter()
                .copied()
                .collect();
            let positions: Vec<f64> = result
                .y
                .iter()
                .map(|state| state[0])
                .collect();

            let label = if i == 0 {
                "Nominal"
            } else {
                "" // Only label the first line to avoid cluttering the legend
            };

            chart
                .draw_series(LineSeries::new(
                    times
                        .iter()
                        .zip(positions.iter())
                        .map(|(&t, &x)| (t, x)),
                    if i == 0 {
                        // Make nominal trajectory stand out
                        BLACK.stroke_width(3)
                    } else {
                        // Use semi-transparent colors for Monte Carlo runs
                        RGBColor(
                            color
                                .color
                                .0,
                            color
                                .color
                                .1,
                            color
                                .color
                                .2,
                        )
                        .mix(0.4)
                        .stroke_width(1)
                    },
                ))?
                .label(label);
            //.legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], &color));
        }

        // Only show legend if there are labeled series
        if results.len() > 0 {
            chart
                .configure_series_labels()
                .position(SeriesLabelPosition::UpperRight)
                .background_style(&WHITE.mix(0.8))
                .border_style(&BLACK)
                .draw()?;
        }

        root.present()?;
    }

    // Use run_context with a closure that displays the image
    run_context(
        move || -> Result<(), Box<dyn Error>> {
            let window = create_window(
                "Monte Carlo Simulation",
                Default::default(),
            )?;

            window.set_image(
                "plot",
                ImageView::new(
                    ImageInfo::rgb8(width as u32, height as u32),
                    &buffer,
                ),
            )?;

            // Wait for window to be closed
            window.wait_until_destroyed()?;

            Ok(())
        },
    );
}
