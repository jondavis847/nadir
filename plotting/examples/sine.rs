use plotting::{QuickPlot, figure::Figure, line::Line, series::Series};
use std::sync::{Arc, Mutex};

fn main() -> iced::Result {
    // Create data for sine wave
    let mut x_data = Vec::new();
    let mut y_data = Vec::new();

    for i in 0..100 {
        let x = i as f64 * 0.1; // 0 to 10 with 0.1 step
        x_data.push(x);
        y_data.push(x.sin());
    }

    // Create a series from the data
    let series = Series::new(&x_data, &y_data).expect("Failed to create series");

    // Create a line from the series
    let line = Arc::new(Mutex::new(Line::new(series)));

    // Create a figure and add the line
    let mut figure = Figure::new();
    let axes = figure
        .get_axes(0)
        .expect("Failed to get axes");
    let mut axes_guard = axes
        .lock()
        .unwrap();
    axes_guard.add_line(line);
    drop(axes_guard); // Release the lock

    // Plot it!
    QuickPlot::plot(figure)
}
