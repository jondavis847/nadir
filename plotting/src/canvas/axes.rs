use super::{
    // legend::{Legend, LegendEntry},
    line::Line,
};
use crate::{theme::PlotTheme, Series};
use iced::{
    widget::canvas::{Fill, Frame, Path, Stroke, Text},
    Color, Point, Rectangle, Size, Vector,
};

use super::axis::Axis;

#[derive(Debug)]
pub struct Axes {
    // legend: Legend,
    padding: f32,
    axis: Axis,
    pub xlim: (f32, f32),
    pub ylim: (f32, f32),
    lines: Vec<Line>,
    pub location: (usize, usize),
}

impl Axes {
    pub fn new(location: (usize, usize)) -> Self {
        Self {
            padding: 0.0,
            axis: Axis::default(),
            xlim: (0.0, 1.0),
            ylim: (0.0, 1.0),
            // legend: Legend::default(),
            lines: Vec::new(),
            location: location,
        }
    }
    pub fn draw(&self, frame: &mut Frame, theme: &PlotTheme) {
        let canvas_bounds = self.get_bounds(frame);
        self.draw_background(frame, theme, &canvas_bounds);
        self.draw_axes(frame, theme, &canvas_bounds);
        self.draw_lines(frame, theme, &canvas_bounds);
    }

    fn draw_axes(&self, frame: &mut Frame, theme: &PlotTheme, canvas_bounds: &Rectangle) {
        frame.with_save(|frame| {
            let translation = Vector::new(self.padding, self.padding);
            frame.translate(translation);
            self.axis
                .draw(frame, theme, canvas_bounds, &self.xlim, &self.ylim);
        });
    }

    fn draw_lines(&self, frame: &mut Frame, theme: &PlotTheme, canvas_bounds: &Rectangle) {
        let axes_bounds = Rectangle::new(
            Point::new(self.axis.padding, self.axis.padding),
            Size::new(
                canvas_bounds.width - 2.0 * self.axis.padding,
                canvas_bounds.height - 2.0 * self.axis.padding,
            ),
        );
        let top_left = axes_bounds.position();
        let size = axes_bounds.size();
        let axes_height = size.height;
        let axes_width = size.width;
        let left = top_left.x;
        let right = left + axes_width;
        let top = top_left.y;
        let bottom = top + axes_height;

        let ylim = self.ylim;
        let xlim = self.xlim;

        let canvas_values = |points: &Vec<Point>| -> Vec<Point> {
            let mut canvas_points = points.clone();

            let x_ratio = axes_width / (xlim.1 - xlim.0);
            let y_ratio = axes_height / (ylim.1 - ylim.0);
            for i in 0..points.len() {
                let canvas_x =
                    (points[i].x - xlim.0) * x_ratio + top_left.x + self.axis.line_width / 2.0;
                let canvas_y = top_left.y + axes_height + self.axis.line_width / 2.0
                    - (points[i].y - ylim.0) * y_ratio;

                canvas_points[i] = Point::new(canvas_x, canvas_y);
            }
            canvas_points
        };
        let mut legend_counter = 0;
        for (i, line) in self.lines.iter().enumerate().rev() { // reverse for legend positioning
            if line.data.len() > 1 {
                let line_color = if let Some(color) = line.color {
                    color
                } else {
                    theme.line_colors[i]
                };
                //convert line data to canvas data
                let canvas_data = canvas_values(&line.data);

                // break it up into smaller lines if needed for clipping since with_clip() doesnt seem to work
                let mut sublines = Vec::new();
                let mut current_subline = Vec::new();

                enum Bound {
                    Top,
                    Bottom,
                    Right,
                    Left,
                }
                let is_out_of_bounds = |point: &Point| -> Option<Bound> {
                    let x = if point.x < left {
                        Some((Bound::Left, left - point.x))
                    } else if point.x > right {
                        Some((Bound::Right, point.x - right))
                    } else {
                        None
                    };

                    let y = if point.y < top {
                        Some((Bound::Top, top - point.y))
                    } else if point.y > bottom {
                        Some((Bound::Bottom, point.y - bottom))
                    } else {
                        None
                    };

                    // determine which one clips first if any
                    match (x, y) {
                        (Some(x), Some(y)) => {
                            if x.1 > y.1 {
                                Some(x.0)
                            } else {
                                Some(y.0)
                            }
                        }
                        (Some(x), None) => Some(x.0),
                        (None, Some(y)) => Some(y.0),
                        (None, None) => None,
                    }
                };

                for (i, point) in canvas_data.iter().enumerate() {
                    if let Some(bound) = is_out_of_bounds(&point) {
                        if !current_subline.is_empty() {
                            // previous point was in bound
                            let prev_point = &canvas_data[i - 1];
                            let end_point = match bound {
                                Bound::Bottom => {
                                    let x = prev_point.x
                                        + (bottom - prev_point.y) / (point.y - prev_point.y)
                                            * (point.x - prev_point.x);
                                    Point::new(x, bottom)
                                }
                                Bound::Left => {
                                    let y = prev_point.y
                                        + (left - prev_point.x) / (point.x - prev_point.x)
                                            * (point.y - prev_point.y);
                                    Point::new(left, y)
                                }
                                Bound::Right => {
                                    let y = prev_point.y
                                        + (right - prev_point.x) / (point.x - prev_point.x)
                                            * (point.y - prev_point.y);
                                    Point::new(right, y)
                                }
                                Bound::Top => {
                                    let x = prev_point.x
                                        + (top - prev_point.y) / (point.y - prev_point.y)
                                            * (point.x - prev_point.x);
                                    Point::new(x, top)
                                }
                            };
                            current_subline.push(end_point);
                            sublines.push(current_subline);
                            current_subline = Vec::new();
                        } else if i < canvas_data.len() - 1 {
                            if is_out_of_bounds(&canvas_data[i + 1]).is_none() {
                                // next point is in bounds, get starting point
                                let next_point = &canvas_data[i + 1];
                                let starting_point = match bound {
                                    Bound::Bottom => {
                                        let x = point.x
                                            + (bottom - point.y) / (next_point.y - point.y)
                                                * (next_point.x - point.x);
                                        Point::new(x, bottom)
                                    }
                                    Bound::Left => {
                                        let y = point.y
                                            + (left - point.x) / (next_point.x - point.x)
                                                * (next_point.y - point.y);
                                        Point::new(left, y)
                                    }
                                    Bound::Right => {
                                        let y = point.y
                                            + (right - point.x) / (next_point.x - point.x)
                                                * (next_point.y - point.y);
                                        Point::new(right, y)
                                    }
                                    Bound::Top => {
                                        let x = point.x
                                            + (top - point.y) / (next_point.y - point.y)
                                                * (next_point.x - point.x);
                                        Point::new(x, top)
                                    }
                                };
                                current_subline.push(starting_point);
                            }
                        }
                    } else {
                        current_subline.push(*point);
                    }
                }

                // Add the last collected subline if it's not empty
                if !current_subline.is_empty() {
                    sublines.push(current_subline);
                }

                for subline in sublines {
                    let path = Path::new(|builder| {
                        builder.move_to(subline[0]);

                        for point in &subline[1..] {
                            builder.line_to(*point);
                        }
                    });

                    frame.stroke(&path, Stroke::default().with_color(line_color));                    
                }

                // add a legend entry if there is one
                if line.legend {
                    let legend_bounds = Rectangle::new(
                        canvas_bounds.position(),
                        Size::new(canvas_bounds.width, self.axis.padding),
                    );
                    let legend_entry_width = legend_bounds.width / 5.0;
                    let legend_entry_height = legend_bounds.height / 3.0;

                    frame.with_clip(legend_bounds, |frame| {
                        let x = canvas_bounds.width
                            - legend_entry_width
                            - legend_entry_width * legend_counter as f32; //starts at the right side and goes backwards
                        let y = legend_bounds.height - legend_entry_height;
                        let position = Point::new(x, y);
                        let text = Text {
                            content: line.label.clone(),
                            position,
                            color: line_color,
                            size: iced::Pixels(legend_entry_height * 0.8),
                            ..Default::default()
                        };
                        frame.fill_text(text);
                    });
                    legend_counter += 1;
                };
            }
        }
    }

    fn draw_background(&self, frame: &mut Frame, theme: &PlotTheme, bounds: &Rectangle) {
        let axes_top_left = bounds.position();
        let axes_size = bounds.size();
        let axes_background = Fill::from(theme.light_background);

        // create the axes border
        let axes_border = Path::rectangle(axes_top_left, axes_size);
        frame.stroke(&axes_border, Stroke::default());

        // fill the background
        frame.fill_rectangle(axes_top_left, axes_size, axes_background)
    }

    pub fn get_bounds(&self, frame: &Frame) -> Rectangle {
        let frame_size = frame.size();
        let axes_height = frame_size.height - 2.0 * self.padding;
        let axes_width: f32 = frame_size.width - 2.0 * self.padding;
        let axes_top_left = Point::new(self.padding, self.padding);
        let axes_size = Size::new(axes_width, axes_height);
        let axes_bounds = Rectangle::new(axes_top_left, axes_size);
        axes_bounds
    }

    pub fn add_line(&mut self, series: &Series, color: Option<Color>) {
        let label = format!("{}.{}.{}", series.result, series.component, series.state);
        let line = Line::new(label, series.points.clone(), color, true);

        self.lines.push(line);

        //update xlim and ylim based on line data
        if let Some((x_lim, y_lim)) = get_global_lims(&self.lines) {
            self.xlim = x_lim;
            self.ylim = y_lim;
        };
    }
}

fn get_lims(points: &Vec<Point>) -> Option<((f32, f32), (f32, f32))> {
    if points.is_empty() {
        return None;
    }

    let mut x_min = points[0].x;
    let mut x_max = points[0].x;
    let mut y_min = points[0].y;
    let mut y_max = points[0].y;

    for point in points.iter() {
        if point.x < x_min {
            x_min = point.x;
        }
        if point.x > x_max {
            x_max = point.x;
        }
        if point.y < y_min {
            y_min = point.y;
        }
        if point.y > y_max {
            y_max = point.y;
        }
    }

    Some(((x_min, x_max), (y_min, y_max)))
}

fn get_global_lims(lines: &Vec<Line>) -> Option<((f32, f32), (f32, f32))> {
    let mut x_min = f32::INFINITY;
    let mut x_max = f32::NEG_INFINITY;
    let mut y_min = f32::INFINITY;
    let mut y_max = f32::NEG_INFINITY;

    let mut found_any_points = false;

    for line in lines {
        if let Some(((line_x_min, line_x_max), (line_y_min, line_y_max))) = get_lims(&line.data) {
            found_any_points = true;
            if line_x_min < x_min {
                x_min = line_x_min;
            }
            if line_x_max > x_max {
                x_max = line_x_max;
            }
            if line_y_min < y_min {
                y_min = line_y_min;
            }
            if line_y_max > y_max {
                y_max = line_y_max;
            }
        }
    }

    if (x_max - x_min).abs() < f32::EPSILON {
        x_max = x_max + 0.5;
        x_min = x_min - 0.5;
    }

    if (y_max - y_min).abs() < f32::EPSILON {
        y_max = y_max + 0.5;
        y_min = y_min - 0.5;
    }

    if found_any_points {
        Some(((x_min, x_max), (y_min, y_max)))
    } else {
        None // Return None if no points were found
    }
}
