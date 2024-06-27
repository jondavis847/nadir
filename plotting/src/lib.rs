use plotters::prelude::*;
use polars::prelude::*;
use std::error::Error;

pub struct ChartTheme<'a> {
    background_color: RGBColor,
    caption_font: FontDesc<'a>,
    axis_label_style: TextStyle<'a>,
    mesh_style: ShapeStyle,
    series_color: RGBColor,
}

impl<'a> Default for ChartTheme<'a> {
    fn default() -> Self {
        Self {
            background_color: WHITE,
            caption_font: ("sans-serif", 50).into_font(),
            axis_label_style: ("sans-serif", 20).into_font().color(&BLACK),
            mesh_style: ShapeStyle {
                color: BLACK.mix(0.2),
                filled: true,
                stroke_width: 1,
            },
            series_color: RED,
        }
    }
}

pub fn plot_column_against_time(df: &DataFrame, column_name: &str, output_file: &str, theme: ChartTheme) -> Result<(), Box<dyn Error>> {
    // Extract the `t` column and the specified column from the DataFrame
    let time_series = df.column("t")?.f64()?;
    let value_series = df.column(column_name)?.f64()?;
    
    let t: Vec<f64> = time_series.into_iter().map(|opt| opt.unwrap()).collect();
    let values: Vec<f64> = value_series.into_iter().map(|opt| opt.unwrap()).collect();
    
    // Create the plot with SVG backend
    let root = SVGBackend::new(output_file, (1024, 768)).into_drawing_area();
    root.fill(&theme.background_color)?;
    let mut chart = ChartBuilder::on(&root)
        .caption(format!("{} vs Time", column_name), theme.caption_font.clone())
        .margin(10)
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(t[0]..t[t.len()-1], *values.iter().min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap()..*values.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap())?;

    chart.configure_mesh()
        .label_style(theme.axis_label_style.clone())
        .bold_line_style(theme.mesh_style.clone())
        .draw()?;

    chart.draw_series(LineSeries::new(
        t.iter().zip(values.iter()).map(|(&x, &y)| (x, y)),
        &theme.series_color,
    ))?
    .label(column_name)
    .legend(|(x, y)| PathElement::new([(x, y), (x + 20, y)], &theme.series_color));

    chart.configure_series_labels()
        .background_style(&theme.background_color.mix(0.8))
        .border_style(&BLACK)
        .draw()?;

    Ok(())
}
