use super::line::Line;
use crate::ui::theme::Theme;
use iced::{
    widget::canvas::{Fill, Frame, Path, Stroke},
    Point, Rectangle, Size, Vector,
};
use std::collections::HashMap;

pub mod axis;
use axis::Axis;

#[derive(Debug)]
pub struct Axes {
    padding: f32,
    axis: Axis,
    xlim: (f32, f32),
    ylim: (f32, f32),
    lines: HashMap<String, Line>,
}

impl Default for Axes {
    fn default() -> Self {
        Self {
            padding: 0.0,
            axis: Axis::default(),
            xlim: (0.0, 1.0),
            ylim: (0.0, 1.0),
            lines: HashMap::new(),
        }
    }
}

impl Axes {
    pub fn draw(&self, frame: &mut Frame, theme: &Theme) {
        // update the bounds based on the frame
        let axes_bounds = self.get_bounds(frame);
        self.background(frame, theme, &axes_bounds);
        
        self.draw_lines(frame, theme);
        frame.with_save(|frame| {
            let translation = Vector::new(self.padding, self.padding);
            frame.translate(translation);
            self.axis
                .draw(frame, theme, &axes_bounds, &self.xlim, &self.ylim);
        });
    }

    fn draw_lines(&self, frame: &mut Frame, _theme: &Theme) {
        let bounds = self.get_bounds(frame);
        let height = bounds.height;
        let width = bounds.width;
        let top_left = bounds.position();

        let ylim = self.ylim;
        let xlim = self.xlim;
        
        let canvas_values = |points: &Vec<Point>| -> Vec<Point> {
            let mut canvas_points = points.clone();
            let axes_height = height-2.0*self.axis.padding-self.axis.line_width;
            let axes_width = width-2.0*self.axis.padding-self.axis.line_width;

            let x_ratio = axes_width / (xlim.1 - xlim.0);
            let y_ratio = axes_height / (ylim.1 - ylim.0);
            for i in 0..points.len() {
                let canvas_x = (points[i].x - xlim.0) * x_ratio + top_left.x + self.axis.padding + self.axis.line_width/2.0;
                let canvas_y = top_left.y + axes_height  + self.axis.padding + self.axis.line_width/2.0 - (points[i].y - ylim.0) * y_ratio;
                canvas_points[i] = Point::new(canvas_x, canvas_y);
            }
            canvas_points
        };

        for (_, line) in &self.lines {
            if line.data.len() > 1 {
                //convert line data to canvas data
                let canvas_data = canvas_values(&line.data);                

                let path = Path::new(|builder| {
                    builder.move_to(canvas_data[0]);

                    for point in &canvas_data[1..] {
                        builder.line_to(*point);
                    }
                });

                frame.stroke(&path, Stroke::default().with_color(line.color));
            }
        }
    }

    fn background(&self, frame: &mut Frame, theme: &Theme, bounds: &Rectangle) {
        let axes_top_left = bounds.position();
        let axes_size = bounds.size();
        let axes_background = Fill::from(theme.dark_background);

        // create the axes border
        let axes_border = Path::rectangle(axes_top_left, axes_size);
        frame.stroke(&axes_border, Stroke::default());

        // fill the background
        frame.fill_rectangle(axes_top_left, axes_size, axes_background)
    }

    pub fn get_bounds(&self, frame: &Frame) -> Rectangle {
        let frame_size = frame.size();
        let axes_height = frame_size.height - 2.0 * self.padding;
        let axes_width = frame_size.width - 2.0 * self.padding;
        let axes_top_left = Point::new(self.padding, self.padding);
        let axes_size = Size::new(axes_width, axes_height);
        let axes_bounds = Rectangle::new(axes_top_left, axes_size);
        axes_bounds
    }

    pub fn plot(&mut self, line_label: String, points: Vec<Point>) {
        let theme = Theme::ORANGE;
        let line = Line::new(line_label.clone(), points, theme.highlight);
        self.lines.insert(line_label, line);
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

fn get_global_lims(lines: &HashMap<String, Line>) -> Option<((f32, f32), (f32, f32))> {
    let mut x_min = f32::INFINITY;
    let mut x_max = f32::NEG_INFINITY;
    let mut y_min = f32::INFINITY;
    let mut y_max = f32::NEG_INFINITY;

    let mut found_any_points = false;

    for line in lines.values() {
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
