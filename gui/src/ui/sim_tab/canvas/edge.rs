use super::graph::GraphNode;
use crate::ui::theme::Theme;
use iced::widget::canvas::{
    stroke::{self, Stroke},
    Path,
};
use iced::{Point, Vector};
use lyon_geom::{
    euclid::default::Point2D, CubicBezierSegment, Line, LineSegment, QuadraticBezierSegment,
};
use lyon_path::PathEvent;
use std::collections::HashMap;
use uuid::Uuid;

#[derive(Debug, Clone)]
pub enum EdgeConnection {
    Node(Uuid),
    Point(Point),
}

#[derive(Debug, Clone)]
pub struct Edge {
    pub from: EdgeConnection,
    pub to: EdgeConnection,
    pub control: Option<Point>,
    pub is_selected: bool,
}

impl Edge {
    pub fn new(from: EdgeConnection, to: EdgeConnection) -> Self {
        Self {
            from,
            to,
            control: None,
            is_selected: false,
        }
    }

    pub fn draw(
        &self,
        frame: &mut iced::widget::canvas::Frame,
        nodes: &HashMap<Uuid, GraphNode>,
        theme: &Theme,
        x_offset: f32,
        zoom: f32,
    ) {
        let color = match self.is_selected {
            true => theme.highlight,
            false => theme.primary,
        };

        let offset = Vector::new(x_offset, 0.0);
        let from_point = match self.from {
            EdgeConnection::Node(id) => {
                nodes.get(&id).unwrap().node.rendered_bounds.center() - offset
            }
            EdgeConnection::Point(point) => point - offset,
        };

        let to_point = match self.to {
            EdgeConnection::Node(id) => {
                //TODO: this is redundant, save the path for the node so we dont have to calculate twice
                let graphnode = nodes.get(&id).unwrap();
                let node_path = graphnode.node.calculate_path(x_offset, zoom);

                find_intersection(
                    from_point,
                    graphnode.node.rendered_bounds.center() - offset,
                    &node_path,
                )
            }
            EdgeConnection::Point(point) => point - offset,
        };

        let control_point = Point::new(
            (from_point.x + to_point.x) / 2.0,
            (from_point.y + to_point.y) / 2.0,
        );

        let path = Path::new(|p| {
            p.move_to(from_point);
            p.quadratic_curve_to(control_point, to_point);
        });

        frame.with_save(|frame| {
            frame.stroke(
                &path,
                Stroke {
                    style: stroke::Style::Solid(color),
                    width: 3.0 * zoom,
                    ..Stroke::default()
                },
            );
        });

        // Calculate the direction vector
        let direction = Point::new(to_point.x - from_point.x, to_point.y - from_point.y);
        let length = (direction.x.powi(2) + direction.y.powi(2)).sqrt();
        let unit_direction = Point::new(direction.x / length, direction.y / length);

        // Define the arrowhead size
        let arrowhead_length = 10.0 * zoom;
        let arrowhead_width = 5.0 * zoom;

        // Calculate the points of the arrowhead
        let arrow_point1 = Point::new(
            to_point.x - arrowhead_length * unit_direction.x + arrowhead_width * unit_direction.y,
            to_point.y - arrowhead_length * unit_direction.y - arrowhead_width * unit_direction.x,
        );
        let arrow_point2 = Point::new(
            to_point.x - arrowhead_length * unit_direction.x - arrowhead_width * unit_direction.y,
            to_point.y - arrowhead_length * unit_direction.y + arrowhead_width * unit_direction.x,
        );

        // Draw the arrowhead
        let arrow_path = Path::new(|p| {
            p.move_to(to_point);
            p.line_to(arrow_point1);
            p.line_to(arrow_point2);
            p.close();
        });

        frame.fill(&arrow_path, color);
    }

    pub fn is_clicked(
        &self,
        canvas_cursor_position: Point,
        nodes: &HashMap<Uuid, GraphNode>,
    ) -> bool {
        let from_point = match self.from {
            EdgeConnection::Node(node_id) => {
                let graphnode = nodes.get(&node_id).unwrap();
                graphnode.node.rendered_bounds.center()
            }
            _ => panic!("should not be possible"),
        };
        let to_point = match self.to {
            EdgeConnection::Node(node_id) => {
                let graphnode = nodes.get(&node_id).unwrap();
                graphnode.node.rendered_bounds.center()
            }
            _ => panic!("should not be possible"),
        };

        contains_point(canvas_cursor_position, from_point, to_point)
    }
}

//TODO: Make this a struct in geometry?
fn contains_point(cursor_position: Point, from_point: Point, to_point: Point) -> bool {
    let center_x = (from_point.x + to_point.x) / 2.0;
    let center_y = (from_point.y + to_point.y) / 2.0;

    let direction = from_point - to_point;
    let length = (direction.x * direction.x + direction.y + direction.y).sqrt();
    let angle = direction.y.atan2(direction.x);

    // Translate point to the box's local coordinate system
    let translated_point = Point {
        x: cursor_position.x - center_x,
        y: cursor_position.y - center_y,
    };

    // Rotate the point by the negative of the box's rotation angle
    let cos_angle = angle.cos();
    let sin_angle = angle.sin();
    let local_x = translated_point.x * cos_angle + translated_point.y * sin_angle;
    let local_y = -translated_point.x * sin_angle + translated_point.y * cos_angle;

    // Check if the transformed point is within the bounds of the box
    let half_height = 10.0;
    let half_width = length / 2.0;

    local_x >= -half_width
        && local_x <= half_width
        && local_y >= -half_height
        && local_y <= half_height
}

fn find_intersection(from: Point, to: Point, path: &Path) -> Point {
    let lyon_path = path.raw();

    for event in lyon_path.iter() {
        match event {
            PathEvent::Line {
                from: start,
                to: end,
            } => {
                if let Some(intersection) = line_line_intersection(from, to, start, end) {
                    return intersection;
                }
            }
            PathEvent::Quadratic {
                from: start,
                ctrl,
                to: end,
            } => {
                if let Some(intersection) = line_quadratic_intersection(from, to, start, ctrl, end)
                {
                    return intersection;
                }
            }
            PathEvent::Cubic {
                from: start,
                ctrl1,
                ctrl2,
                to: end,
            } => {
                if let Some(intersection) =
                    line_cubic_intersection(from, to, start, ctrl1, ctrl2, end)
                {
                    return intersection;
                }
            }
            _ => {}
        }
    }

    to // Default to the 'to' point if no intersection is found
}

fn line_line_intersection(
    from: Point,
    to: Point,
    start: Point2D<f32>,
    end: Point2D<f32>,
) -> Option<Point> {
    let line1 = LineSegment {
        from: Point2D::new(from.x, from.y),
        to: Point2D::new(to.x, to.y),
    };

    let line2 = LineSegment {
        from: start,
        to: end,
    };

    if let Some(intersection) = line1.intersection(&line2) {
        Some(Point::new(intersection.x, intersection.y))
    } else {
        None
    }
}

fn line_quadratic_intersection(
    from: Point,
    to: Point,
    start: Point2D<f32>,
    ctrl: Point2D<f32>,
    end: Point2D<f32>,
) -> Option<Point> {
    let line = Line {
        point: Point2D::new(from.x, from.y),
        vector: Point2D::new(to.x - from.x, to.y - from.y).to_vector(),
    };

    let curve = QuadraticBezierSegment {
        from: start,
        ctrl,
        to: end,
    };

    for t in curve.line_intersections_t(&line) {
        let intersection = curve.sample(t);
        return Some(Point::new(intersection.x, intersection.y));
    }

    None
}

fn line_cubic_intersection(
    from: Point,
    to: Point,
    start: Point2D<f32>,
    ctrl1: Point2D<f32>,
    ctrl2: Point2D<f32>,
    end: Point2D<f32>,
) -> Option<Point> {
    let line = Line {
        point: Point2D::new(from.x, from.y),
        vector: Point2D::new(to.x - from.x, to.y - from.y).to_vector(),
    };

    let curve = CubicBezierSegment {
        from: start,
        ctrl1,
        ctrl2,
        to: end,
    };

    for t in curve.line_intersections_t(&line) {
        let intersection = curve.sample(t);
        return Some(Point::new(intersection.x, intersection.y));
    }

    None
}
