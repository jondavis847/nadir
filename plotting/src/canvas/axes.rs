use super::{
    // legend::{Legend, LegendEntry},
    line::Line,
};
use crate::{Series, theme::PlotTheme};
use iced::{
    Color, Font, Point, Rectangle, Size,
    widget::canvas::{Frame, Path, Stroke, Text},
};

use super::axis::Axis;

// For clipping, define which boundary is violated.
#[derive(Copy, Clone)]
enum Bound {
    Top,
    Bottom,
    Left,
    Right,
}

#[derive(Debug)]
pub struct Axes {
    // legend: Legend,
    //padding: f32,
    pub axis: Axis,
    pub xlim: (f32, f32),
    pub ylim: (f32, f32),
    lines: Vec<Line>,
    pub location: (usize, usize),
    pub click_start: Option<Point>,
    pub bounds: Rectangle,
}

impl Axes {
    pub fn new(location: (usize, usize), bounds: Rectangle) -> Self {
        Self {
            //        padding: 0.0,
            axis: Axis::default(),
            xlim: (0.0, 1.0),
            ylim: (0.0, 1.0),
            // legend: Legend::default(),
            lines: Vec::new(),
            location,
            click_start: None,
            bounds,
        }
    }
    pub fn draw(&self, frame: &mut Frame, theme: &PlotTheme) {
        self.draw_background(frame, theme);
        self.draw_axis(frame, theme);
        self.draw_lines(frame, theme);
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

    fn draw_lines(&self, frame: &mut Frame, theme: &PlotTheme) {
        // Define axis bounds and scaling factors
        let axis = &self.axis;
        let bounds = Rectangle::new(
            Point::new(axis.x_padding, axis.y_padding),
            Size::new(
                frame.width() - 2.0 * axis.x_padding,
                frame.height() - 2.0 * axis.y_padding,
            ),
        );
        let (left, top) = (bounds.x, bounds.y);
        let (width, height) = (bounds.width, bounds.height);
        let right = left + width;
        let bottom = top + height;

        let (x_min, x_max) = self.xlim;
        let (y_min, y_max) = self.ylim;
        let x_scale = width / (x_max - x_min);
        let y_scale = height / (y_max - y_min);

        // Helper to transform data points to canvas coordinates
        let to_canvas = |p: &Point| -> Point {
            Point::new(
                (p.x - x_min) * x_scale + left + axis.line_width / 2.0,
                bottom + axis.line_width / 2.0 - (p.y - y_min) * y_scale,
            )
        };

        // Determine if a point is out of bounds and identify the boundary
        let out_of_bounds = |p: &Point| -> Option<Bound> {
            let mut candidates = Vec::new();
            if p.x < left {
                candidates.push((Bound::Left, left - p.x));
            }
            if p.x > right {
                candidates.push((Bound::Right, p.x - right));
            }
            if p.y < top {
                candidates.push((Bound::Top, top - p.y));
            }
            if p.y > bottom {
                candidates.push((Bound::Bottom, p.y - bottom));
            }
            candidates
                .into_iter()
                .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
                .map(|(bound, _)| bound)
        };

        // Compute intersection point with a boundary
        let intersect = |p0: &Point, p1: &Point, bound: Bound| -> Point {
            match bound {
                Bound::Bottom => {
                    let t = (bottom - p0.y) / (p1.y - p0.y);
                    Point::new(p0.x + t * (p1.x - p0.x), bottom)
                }
                Bound::Top => {
                    let t = (top - p0.y) / (p1.y - p0.y);
                    Point::new(p0.x + t * (p1.x - p0.x), top)
                }
                Bound::Left => {
                    let t = (left - p0.x) / (p1.x - p0.x);
                    Point::new(left, p0.y + t * (p1.y - p0.y))
                }
                Bound::Right => {
                    let t = (right - p0.x) / (p1.x - p0.x);
                    Point::new(right, p0.y + t * (p1.y - p0.y))
                }
            }
        };

        // Initialize legend tracking
        let legend_y = 8.0;
        let char_width = 8.0; // Approximate width of a character
        let padding = 10.0; // Padding between legend entries        
        let mut current_x = self.bounds.width - axis.x_padding;

        // Iterate over lines in reverse for legend ordering
        for (i, line) in self.lines.iter().enumerate().rev() {
            if line.data.len() <= 1 {
                continue;
            }

            let color = line.color.unwrap_or(theme.line_colors[i]);
            let canvas_points: Vec<Point> = line.data.iter().map(to_canvas).collect();

            // Split lines into sublines based on clipping
            let mut sublines = Vec::new();
            let mut current = Vec::new();

            for (idx, &pt) in canvas_points.iter().enumerate() {
                if let Some(bound) = out_of_bounds(&pt) {
                    if !current.is_empty() {
                        let prev = current.last().unwrap();
                        current.push(intersect(prev, &pt, bound));
                        sublines.push(current);
                        current = Vec::new();
                    } else if idx + 1 < canvas_points.len()
                        && out_of_bounds(&canvas_points[idx + 1]).is_none()
                    {
                        current.push(intersect(&pt, &canvas_points[idx + 1], bound));
                    }
                } else {
                    current.push(pt);
                }
            }
            if !current.is_empty() {
                sublines.push(current);
            }

            // Draw each subline
            for subline in sublines {
                if subline.len() < 2 {
                    continue;
                }
                let path = Path::new(|builder| {
                    builder.move_to(subline[0]);
                    for &pt in &subline[1..] {
                        builder.line_to(pt);
                    }
                });
                frame.stroke(&path, Stroke::default().with_color(color));
            }

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

    pub fn add_line(&mut self, series: &Series, color: Option<Color>) {
        let line = Line::new(series.y_name.clone(), series.points.clone(), color, true);
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

    // logic if the values are 0
    let delta_x = x_max - x_min;
    if delta_x.abs() < f32::EPSILON {
        x_max = x_max + 0.5;
        x_min = x_min - 0.5;
    }

    let delta_y = y_max - y_min;
    if (delta_y).abs() < f32::EPSILON {
        y_max = y_max + 0.5;
        y_min = y_min - 0.5;
    } else {
        // add padding so curve isnt exactly at top and bottom of axis
        let padding = delta_y * 0.1;
        y_max += padding;
        y_min -= padding;
    }

    if found_any_points {
        Some(((x_min, x_max), (y_min, y_max)))
    } else {
        None // Return None if no points were found
    }
}
