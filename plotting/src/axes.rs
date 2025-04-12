use super::{
    // legend::{Legend, LegendEntry},
    line::Line,
};
use crate::{Series, series::PlotPoint, theme::PlotTheme};
use iced::{
    Color, Font, Point, Rectangle, Size,
    widget::canvas::{Frame, Path, Stroke, Text},
};

use super::axis::Axis;

#[derive(Debug)]
pub struct Axes {
    // legend: Legend,
    //padding: f32,
    pub axis: Axis,
    pub xlim: (f64, f64),
    pub ylim: (f64, f64),
    lines: Vec<Line>,
    pub location: (usize, usize),
    pub click_start: Option<Point>,
    container_bounds: Rectangle,
    left_padding: f32,
    right_padding: f32,
    top_padding: f32,
    bottom_padding: f32,
}

impl Axes {
    pub fn new(location: (usize, usize), container_bounds: Rectangle) -> Self {
        Self {
            axis: Axis::default(),
            xlim: (0.0, 1.0),
            ylim: (-1.0, 1.0),
            lines: Vec::new(),
            location,
            click_start: None,
            container_bounds,
            left_padding: 50.0,
            right_padding: 50.0,
            top_padding: 50.0,
            bottom_padding: 50.0,
        }
    }
    pub fn draw(&mut self, frame: &mut Frame, theme: &PlotTheme) {
        self.draw_background(frame, theme);
        let axis_bounds = Rectangle::new(
            Point::new(self.left_padding, self.top_padding),
            Size::new(
                frame.width() - self.left_padding - self.right_padding,
                frame.height() - self.top_padding - self.bottom_padding,
            ),
        );
        frame.with_clip(axis_bounds, |frame| {
            self.draw_axis(frame, theme);
            self.draw_lines(frame, theme);
        })
    }

    fn draw_axis(&self, frame: &mut Frame, theme: &PlotTheme) {
        self.axis.draw(frame, theme, &self.xlim, &self.ylim);
    }

    fn draw_background(&self, frame: &mut Frame, theme: &PlotTheme) {
        let center = frame.center();
        let size = frame.size();
        let top_left = Point::new(center.x - size.width / 2.0, center.y - size.height / 2.0);
        frame.fill_rectangle(top_left, size, theme.dark_background)
    }

    fn draw_lines(&mut self, frame: &mut Frame, theme: &PlotTheme) {
        // Define axis bounds and scaling factors
        let axis = &self.axis;
        let frame_size = frame.size();
        let left = 0.0;
        let top = 0.0;
        let right = frame_size.width;
        let bottom = frame_size.height;

        let (x_min, x_max) = self.xlim;
        let (y_min, y_max) = self.ylim;
        let x_scale = frame_size.width / (x_max - x_min) as f32;
        let y_scale = frame_size.height / (y_max - y_min) as f32;

        // Helper to transform data points to canvas coordinates
        let update_canvas_point = |p: &mut PlotPoint| {
            p.canvas_position.x =
                (p.data.x - x_min) as f32 * x_scale + left + axis.line_width / 2.0;
            p.canvas_position.y =
                bottom + axis.line_width / 2.0 - (p.data.y - y_min) as f32 * y_scale;
        };

        // Initialize legend tracking
        let legend_y = 8.0;
        let char_width = 8.0; // Approximate width of a character
        let padding = 10.0; // Padding between legend entries        
        let mut current_x = right - axis.x_padding;

        // Iterate over lines in reverse for legend ordering
        for (i, line) in self.lines.iter_mut().enumerate().rev() {
            if line.data.len() <= 1 {
                continue;
            }

            let color = line.color.unwrap_or(theme.line_colors[i]);
            line.data
                .points
                .iter_mut()
                .for_each(|point| update_canvas_point(point));

            // Draw the line
            let path = Path::new(|builder| {
                builder.move_to(line.data.points[0].canvas_position);
                for pt in &line.data.points[1..] {
                    builder.line_to(pt.canvas_position);
                }
            });
            frame.stroke(&path, Stroke::default().with_color(color));

            // Draw legend entry if applicable
            if line.legend {
                let label = &line.label;
                let entry_width = label.len() as f32 * char_width + padding;

                // // Check if the entry fits in the remaining space
                // if current_x + entry_width > frame.width() - axis.x_padding {
                //     break; // Stop rendering if there's no more space
                // }
                current_x -= entry_width;

                let position = Point::new(current_x, legend_y);

                let text = Text {
                    content: label.clone(),
                    position,
                    color,
                    size: iced::Pixels(14.0),
                    font: Font::MONOSPACE,
                    ..Default::default()
                };
                frame.fill_text(text);
            }
        }
    }

    pub fn add_line(&mut self, series: Series, color: Option<Color>) {
        let line = Line::new(series.y_name.clone(), series, color, true);
        self.lines.push(line);

        //update xlim and ylim based on line data
        let (xlim, ylim) = get_global_lims(&self.lines);
        self.xlim = xlim;
        self.ylim = ylim;
    }
}

fn get_global_lims(lines: &Vec<Line>) -> ((f64, f64), (f64, f64)) {
    let mut xmin = if let Some(xmin) = lines
        .iter()
        .map(|line| line.data.xmin)
        .min_by(|a, b| a.total_cmp(b))
    {
        xmin
    } else {
        0.0
    };

    let mut xmax = if let Some(xmax) = lines
        .iter()
        .map(|line| line.data.xmin)
        .max_by(|a, b| a.total_cmp(b))
    {
        xmax
    } else {
        0.0
    };

    let mut ymin = if let Some(ymin) = lines
        .iter()
        .map(|line| line.data.ymin)
        .min_by(|a, b| a.total_cmp(b))
    {
        ymin
    } else {
        0.0
    };

    let mut ymax = if let Some(ymax) = lines
        .iter()
        .map(|line| line.data.ymax)
        .max_by(|a, b| a.total_cmp(b))
    {
        ymax
    } else {
        0.0
    };

    // logic if the values are 0
    let delta_x = xmax - xmin;
    if delta_x.abs() < f64::EPSILON {
        xmax = xmax + 0.5;
        xmin = xmin - 0.5;
    }

    let delta_y = ymax - ymin;
    if (delta_y).abs() < f64::EPSILON {
        ymax = ymax + 0.5;
        ymin = ymin - 0.5;
    } else {
        // add padding so curve isnt exactly at top and bottom of axis
        let padding = delta_y * 0.1;
        ymax += padding;
        ymin -= padding;
    }

    ((xmin, xmax), (ymin, ymax))
}
