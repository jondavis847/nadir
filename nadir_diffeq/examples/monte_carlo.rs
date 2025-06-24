use nadir_diffeq::{
    OdeModel, OdeProblem,
    saving::MemoryResult,
    solvers::{OdeSolver, RungeKuttaMethods},
    state::{OdeState, state_array::StateArray},
    stepping::AdaptiveStepControl,
};
use show_image::{Image, ImageInfo, ImageView, run_context};
use std::error::Error;

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

fn main() -> Result<(), Box<dyn Error>> {
    let model = DampedOscillator { spring_constant: 1.0, damping: 0.1 };
    let problem = OdeProblem::new(model);
    let x0 = StateArray::new([1.0, 0.0]);
    let solver = OdeSolver::new(RungeKuttaMethods::Tsit5.into());

    let result = solver.solve_adaptive(
        problem,
        x0,
        (0.0, 10.0),
        AdaptiveStepControl::default(),
    )?;

    plot(result.unwrap())?;
    Ok(())
}

use plotters::backend::BitMapBackend;
use plotters::prelude::*;
use show_image::create_window;

fn plot(result: MemoryResult<StateArray<2>>) -> Result<(), Box<dyn Error>> {
    let width = 1200;
    let height = 800;

    // Create an in-memory bitmap
    let mut buffer = vec![0u8; width * height * 3];
    {
        let root = BitMapBackend::with_buffer(&mut buffer, (width as u32, height as u32))
            .into_drawing_area();
        root.fill(&WHITE)?;

        // Find the range of your data
        let times: Vec<f64> = result.t.iter().copied().collect();
        let positions: Vec<f64> = result.y.iter().map(|state| state[0]).collect(); // position (x[0])
        let t_min = times.iter().fold(f64::INFINITY, |a, &b| a.min(b));
        let t_max = times
            .iter()
            .fold(f64::NEG_INFINITY, |a, &b| a.max(b));
        let pos_min = positions
            .iter()
            .fold(f64::INFINITY, |a, &b| a.min(b));
        let pos_max = positions
            .iter()
            .fold(f64::NEG_INFINITY, |a, &b| a.max(b));

        let t_range = t_min..t_max;
        let x_range = pos_min..pos_max;

        let mut chart = ChartBuilder::on(&root)
            .caption("Damped Oscillator", ("sans-serif", 30))
            .margin(10)
            .x_label_area_size(40)
            .y_label_area_size(40)
            .build_cartesian_2d(t_range, x_range)?;

        chart
            .configure_mesh()
            .x_desc("Time")
            .y_desc("Position")
            .draw()?;

        // Plot position vs time
        chart
            .draw_series(LineSeries::new(
                times
                    .iter()
                    .zip(positions.iter())
                    .map(|(&t, &x)| (t, x)),
                &RED,
            ))?
            .label("Position")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], &RED));

        chart.configure_series_labels().draw()?;
        root.present()?;
    }

    // Use run_context with a closure that displays the image
    run_context(move || -> Result<(), Box<dyn Error>> {
        let window = create_window("Damped Oscillator", Default::default())?;

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
    });
}
