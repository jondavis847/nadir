use std::{
    ops::Deref,
    sync::{Arc, Mutex},
};

use crate::PlotErrors;

use super::{series::Series, theme::PlotTheme};
use iced::{
    Color, Point, Rectangle,
    widget::canvas::{Frame, Stroke, path::Builder},
};

#[derive(Debug, Clone)]
pub struct Line {
    pub data: Series,
    pub color: Option<Color>,
    pub legend: bool,
    pub width: f32,
}

impl Line {
    pub fn new(xdata: &[f64], ydata: &[f64]) -> Result<Self, PlotErrors> {
        let data = Series::new(xdata, ydata)?;
        Ok(Self { data, color: None, legend: true, width: 1.5 })
    }

    pub fn delete_xname(&mut self) {
        self.data
            .xname = None;
    }

    pub fn delete_yname(&mut self) {
        self.data
            .yname = None;
    }

    pub fn set_color(&mut self, color: Color) {
        self.color = Some(color);
    }

    pub fn set_width(&mut self, width: f32) -> Result<(), PlotErrors> {
        if width <= 0.0 {
            return Err(PlotErrors::SmallLineWidth);
        }
        self.width = width;
        Ok(())
    }

    pub fn set_xname(&mut self, name: &str) {
        self.data
            .xname = Some(name.into());
    }

    pub fn set_yname(&mut self, name: &str) {
        self.data
            .yname = Some(name.into());
    }

    pub fn with_color(mut self, color: Color) -> Self {
        self.color = Some(color);
        self
    }

    pub fn with_xname(mut self, name: &str) -> Self {
        self.data
            .xname = Some(name.into());
        self
    }

    pub fn with_yname(mut self, name: &str) -> Self {
        self.data
            .yname = Some(name.into());
        self
    }

    pub fn with_width(mut self, width: f32) -> Result<Self, PlotErrors> {
        if width <= 0.0 {
            return Err(PlotErrors::SmallLineWidth);
        }
        self.width = width;
        Ok(self)
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
        let mut path_started = false;

        // Handle the case when all points are outside but line crosses visible area
        if self
            .data
            .points
            .len()
            < 2
        {
            return; // Nothing to draw
        }

        for i in 0..self
            .data
            .points
            .len()
        {
            let plotpoint = &self
                .data
                .points[i];
            let current_inbounds = point_in_bounds_inclusive(
                plotpoint.canvas_position,
                axis_bounds,
            );

            if i == 0 {
                // Handle the first point
                if current_inbounds {
                    path.move_to(plotpoint.canvas_position);
                    path_started = true;
                    last_inbounds = true;
                }
                // Don't skip first point if out of bounds - we'll need it for intersection
            } else {
                // Handle all subsequent points
                let lastpoint = &self
                    .data
                    .points[i - 1];

                if last_inbounds && current_inbounds {
                    // Simple case: both points in bounds
                    path.line_to(plotpoint.canvas_position);
                } else {
                    // At least one point is out of bounds
                    let boundary_points = calculate_boundary_points(
                        lastpoint.canvas_position,
                        plotpoint.canvas_position,
                        axis_bounds,
                    );

                    match boundary_points {
                        None => {
                            // No intersections with bounds
                            // If both points are outside but on different sides, we might need to draw
                            // through the visible area
                        }
                        Some((entry_point, None)) => {
                            if last_inbounds && !current_inbounds {
                                // Going from inside to outside
                                path.line_to(entry_point);
                            } else if !last_inbounds && current_inbounds {
                                // Going from outside to inside
                                if !path_started {
                                    path.move_to(entry_point);
                                    path_started = true;
                                } else {
                                    path.move_to(entry_point);
                                }
                                path.line_to(plotpoint.canvas_position);
                            }
                        }
                        Some((entry_point, Some(exit_point))) => {
                            // Line crosses through the bounds
                            if !path_started {
                                path.move_to(entry_point);
                                path_started = true;
                            } else {
                                path.move_to(entry_point);
                            }
                            path.line_to(exit_point);
                        }
                    }
                }

                last_inbounds = current_inbounds;
            }
        }

        // Special case for last point being out of bounds but line extends to edge
        if !last_inbounds
            && self
                .data
                .points
                .len()
                >= 2
        {
            let last_idx = self
                .data
                .points
                .len()
                - 1;
            let lastpoint = &self
                .data
                .points[last_idx - 1];
            let endpoint = &self
                .data
                .points[last_idx];

            // Check if the line between the last two points intersects any boundary
            let boundary_points = calculate_boundary_points(
                lastpoint.canvas_position,
                endpoint.canvas_position,
                axis_bounds,
            );

            if let Some((entry_point, _)) = boundary_points {
                // Only draw if the last visible point was the entry point
                // (meaning the line was exiting the visible area)
                if lastpoint
                    .canvas_position
                    .x
                    < entry_point.x
                {
                    path.line_to(entry_point);
                }
            }
        }

        let color = if let Some(color) = &self.color {
            color.clone()
        } else {
            theme.line_colors[index]
        };

        let stroke = Stroke { style: color.into(), width: self.width, ..Stroke::default() };

        frame.stroke(&path.build(), stroke);
    }

    pub fn update_canvas_position(
        &mut self,
        axis_bounds: &Rectangle,
        xlim: &(f32, f32),
        ylim: &(f32, f32),
    ) {
        let axis_bottom = axis_bounds.y + axis_bounds.height;
        for plotpoint in &mut self
            .data
            .points
        {
            let u = (plotpoint
                .data
                .x as f32
                - xlim.0)
                / (xlim.1 - xlim.0);
            let v = (plotpoint
                .data
                .y as f32
                - ylim.0)
                / (ylim.1 - ylim.0);

            plotpoint
                .canvas_position
                .x = u * axis_bounds.width + axis_bounds.x;
            plotpoint
                .canvas_position
                .y = axis_bottom - v * axis_bounds.height;
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

// Helper function to check if point is in bounds (inclusive of all edges) since bounds.contains is exclusive
fn point_in_bounds_inclusive(point: Point, bounds: &Rectangle) -> bool {
    point.x >= bounds.x
        && point.x <= bounds.x + bounds.width
        && point.y >= bounds.y
        && point.y <= bounds.y + bounds.height
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
    match (
        first_intersection,
        second_intersection,
    ) {
        (Some(first), second) => Some((first, second)),
        _ => None,
    }
}

#[derive(Debug, Clone)]
pub struct LineHandle(Arc<Mutex<Line>>);

impl From<Line> for LineHandle {
    fn from(value: Line) -> Self {
        Self(Arc::new(Mutex::new(value)))
    }
}

impl From<Arc<Mutex<Line>>> for LineHandle {
    fn from(value: Arc<Mutex<Line>>) -> Self {
        Self(value)
    }
}

impl Deref for LineHandle {
    type Target = Arc<Mutex<Line>>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
