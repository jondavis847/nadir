use plotting::{QuickPlot, figure::Figure, line::Line};
use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    // Create data for sine wave
    let mut t = Vec::new();
    let mut sin = Vec::new();
    let mut cos = Vec::new();

    for i in 0..100 {
        t.push(i as f64 * 0.1);
        sin.push(t[i].sin());
        cos.push(t[i].cos());
    }

    // Create a line
    let sin_line = Line::new(&t, &sin)?.with_yname("sin");
    let cos_line = Line::new(&t, &cos)?.with_yname("cos");

    // Create the figure
    let mut figure = Figure::new();

    // Figure comes with default axes at index 0, grab it
    let axes = figure.get_axes(0)?;
    axes.add_line(sin_line);
    axes.add_line(cos_line);

    // use QuickPlot to immediately open a program blocking window with the plot
    QuickPlot::plot(figure)?;

    Ok(())
}
