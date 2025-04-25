use super::{axis::Axis, series::Series, theme::PlotTheme};
use iced::{
    Color, Point, Rectangle,
    widget::canvas::{Frame, Stroke, path::Builder},
};

#[derive(Debug, Clone)]
pub struct Line {
    pub label: Option<String>,
    pub data: Series,
    pub color: Option<Color>,
    pub legend: bool,
    pub width: f32,
}

impl Line {
    pub fn new(data: Series) -> Self {
        Self {
            label: None,
            data,
            color: None,
            legend: true,
            width: 3.0,
        }
    }

    pub fn set_label(&mut self, label: String) {
        self.label = Some(label);
    }

    pub fn set_color(&mut self, color: Color) {
        self.color = Some(color);
    }

    pub fn draw(
        &self,
        frame: &mut Frame,
        theme: &PlotTheme,
        index: usize,
        axis_bounds: &Rectangle,
    ) {
        let mut path = Builder::new();
        let mut last_inbounds = false;
        for i in 0..self.data.points.len() {
            let plotpoint = &self.data.points[i];
            if i == 0 {
                // find the first point in bounds
                if axis_bounds.contains(plotpoint.canvas_position) {
                    path.move_to(plotpoint.canvas_position);
                    last_inbounds = true;
                }
            } else {
                // handle the rest of the points after first
                let current_inbounds = axis_bounds.contains(plotpoint.canvas_position);

                if last_inbounds && current_inbounds {
                    path.line_to(plotpoint.canvas_position)
                } else {
                    // we'll need to find intersections
                    let lastpoint = &self.data.points[i - 1];
                    let boundary_points = calculate_boundary_points(
                        lastpoint.canvas_position,
                        plotpoint.canvas_position,
                        axis_bounds,
                    );

                    match boundary_points {
                        None => {} // no intersections, continue
                        Some((first, None)) => {
                            // one intersection
                            if last_inbounds && !current_inbounds {
                                // was in but is now out, draw line to boundary
                                path.line_to(first);
                                last_inbounds = false;
                            }
                            if !last_inbounds && current_inbounds {
                                // was out but is now in, draw line from boundary
                                path.move_to(first);
                                path.line_to(plotpoint.canvas_position);
                                last_inbounds = true;
                            }
                        }
                        Some((first, Some(second))) => {
                            // intersected twice
                            path.move_to(first);
                            path.line_to(second);
                            last_inbounds = false;
                        }
                    }
                }
            }
        }

        let color = if let Some(color) = &self.color {
            color.clone()
        } else {
            theme.line_colors[index]
        };

        let stroke = Stroke {
            style: color.into(),
            width: self.width,
            ..Stroke::default()
        };

        frame.stroke(&path.build(), stroke);
    }

    pub fn update_scale(&mut self, axis: &Axis) {
        for plotpoint in &mut self.data.points {
            plotpoint.canvas_position.x =
                plotpoint.data.x as f32 * axis.data_to_canvas_scale_x + axis.canvas_origin.x;
            plotpoint.canvas_position.y =
                plotpoint.data.y as f32 * axis.data_to_canvas_scale_y + axis.canvas_origin.y;
        }
    }

    pub fn update_position(&mut self, axis: &Axis) {
        for plotpoint in &mut self.data.points {
            plotpoint.canvas_position.x = plotpoint.data.x as f32 * axis.data_to_canvas_scale_x;
            plotpoint.canvas_position.y = plotpoint.data.y as f32 * axis.data_to_canvas_scale_y;
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
enum Boundary {
    Bottom,
    Inside,
    Left,
    Right,
    Top,
}

/// Calculates the intersection points of a line segment with a rectangle boundary.
/// Returns:
/// - None if there is no intersection
/// - Some((point, None)) if there is one intersection point
/// - Some((point1, Some(point2))) if there are two intersection points
fn calculate_boundary_points(
    p1: Point,
    p2: Point,
    rect: &Rectangle,
) -> Option<(Point, Option<Point>)> {
    // Compute region code as a closure
    let compute_boundary = |p: Point| -> (Boundary, Boundary) {
        // Calculate if the point is to the left, right or inside the x-axis
        let x = if p.x < rect.x {
            Boundary::Left
        } else if p.x > rect.x + rect.width {
            Boundary::Right
        } else {
            Boundary::Inside
        };

        // Calculate if the point is to the top, bottom or inside the y-axis
        let y = if p.y < rect.y {
            Boundary::Bottom
        } else if p.y > rect.y + rect.height {
            Boundary::Top
        } else {
            Boundary::Inside
        };

        (x, y)
    };

    // Get boundary information for both points
    let (p1_x_bound, p1_y_bound) = compute_boundary(p1);
    let (p2_x_bound, p2_y_bound) = compute_boundary(p2);

    // Quick reject: if both points are outside the same boundary, no intersection
    if (p1_x_bound == p2_x_bound && p1_x_bound != Boundary::Inside)
        || (p1_y_bound == p2_y_bound && p1_y_bound != Boundary::Inside)
    {
        return None;
    }

    // If both points are inside, there's no boundary intersection
    if p1_x_bound == Boundary::Inside
        && p1_y_bound == Boundary::Inside
        && p2_x_bound == Boundary::Inside
        && p2_y_bound == Boundary::Inside
    {
        return None;
    }

    // Calculate the intersection with boundaries
    let dx = p2.x - p1.x;
    let dy = p2.y - p1.y;

    // Function to check if a point is actually on the line segment
    let is_on_segment = |p: Point| -> bool {
        if dx == 0.0 && dy == 0.0 {
            return p1 == p; // Zero-length segment case
        }

        let t = if dx.abs() > dy.abs() {
            (p.x - p1.x) / dx
        } else {
            (p.y - p1.y) / dy
        };

        // Allow for floating point errors
        t >= -1e-6 && t <= 1.0 + 1e-6
    };

    // Store up to 2 intersections, sorted by distance from p1
    let mut first_intersection: Option<Point> = None;
    let mut second_intersection: Option<Point> = None;

    // Check each boundary and collect up to 2 intersections
    let mut try_add_intersection = |point: Point| {
        if !is_on_segment(point) {
            return;
        }

        let dist = (point.x - p1.x).powi(2) + (point.y - p1.y).powi(2);

        if let Some(first) = first_intersection {
            let first_dist = (first.x - p1.x).powi(2) + (first.y - p1.y).powi(2);

            // Check if this is a duplicate of the first point
            if (point.x - first.x).abs() < 1e-6 && (point.y - first.y).abs() < 1e-6 {
                return;
            }

            if let Some(second) = second_intersection {
                let second_dist = (second.x - p1.x).powi(2) + (second.y - p1.y).powi(2);

                // Insert the new point if it's closer than any existing point
                if dist < first_dist {
                    second_intersection = Some(first);
                    first_intersection = Some(point);
                } else if dist < second_dist {
                    second_intersection = Some(point);
                }
            } else if dist < first_dist {
                // Put the closer point first
                second_intersection = Some(first);
                first_intersection = Some(point);
            } else {
                second_intersection = Some(point);
            }
        } else {
            first_intersection = Some(point);
        }
    };

    // Left boundary
    if dx != 0.0 {
        let x = rect.x;
        let t = (x - p1.x) / dx;
        let y = p1.y + t * dy;
        if y >= rect.y && y <= rect.y + rect.height {
            try_add_intersection(Point::new(x, y));
        }
    }

    // Right boundary
    if dx != 0.0 {
        let x = rect.x + rect.width;
        let t = (x - p1.x) / dx;
        let y = p1.y + t * dy;
        if y >= rect.y && y <= rect.y + rect.height {
            try_add_intersection(Point::new(x, y));
        }
    }

    // Bottom boundary
    if dy != 0.0 {
        let y = rect.y;
        let t = (y - p1.y) / dy;
        let x = p1.x + t * dx;
        if x >= rect.x && x <= rect.x + rect.width {
            try_add_intersection(Point::new(x, y));
        }
    }

    // Top boundary
    if dy != 0.0 {
        let y = rect.y + rect.height;
        let t = (y - p1.y) / dy;
        let x = p1.x + t * dx;
        if x >= rect.x && x <= rect.x + rect.width {
            try_add_intersection(Point::new(x, y));
        }
    }

    // Return the intersections
    match (first_intersection, second_intersection) {
        (Some(first), second) => Some((first, second)),
        _ => None,
    }
}
