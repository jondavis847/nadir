use iced::Point;
use iced::widget::canvas::{Path,stroke::{self,Stroke}};
use uuid::Uuid;
use std::collections::HashMap;
use crate::ui::canvas::graph::GraphNode;
use crate::ui::theme::Theme;
use lyon_geom::{euclid::default::Point2D, CubicBezierSegment, Line, LineSegment, QuadraticBezierSegment};
use lyon_path::PathEvent;

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
}

impl Edge {
    pub fn new(from: EdgeConnection, to: EdgeConnection) -> Self {
        Self {
            from: from,
            to: to,
            control: None,
        }
    }

    pub fn draw(&self, frame: &mut iced::widget::canvas::Frame, nodes: &HashMap<Uuid,GraphNode>, theme: &Theme) {
        let from_point = match self.from {
            EdgeConnection::Node(id) => nodes.get(&id).unwrap().node.bounds.center(),
            EdgeConnection::Point(point) => point,
        };
        
        let to_point = match self.to {
            EdgeConnection::Node(id) => {
                let graphnode = nodes.get(&id).unwrap();
                let node_path = graphnode.node.calculate_path();

                find_intersection(from_point,graphnode.node.bounds.center(), &node_path)
            }
            EdgeConnection::Point(point) => point,
        };

        let control_point = Point::new((from_point.x + to_point.x)/2.0,(from_point.y + to_point.y)/2.0 );

        let path = Path::new(|p| {
            p.move_to(from_point);
            p.quadratic_curve_to(control_point,to_point);
        });

        frame.with_save(|frame| {
            frame.stroke(
                &path,
                Stroke {
                    style: stroke::Style::Solid(theme.primary),
                    width: 3.0,
                    ..Stroke::default()
                },
            );
        });

        // Calculate the direction vector
        let direction = Point::new(to_point.x - from_point.x, to_point.y - from_point.y);
        let length = (direction.x.powi(2) + direction.y.powi(2)).sqrt();
        let unit_direction = Point::new(direction.x / length, direction.y / length);

        // Define the arrowhead size
        let arrowhead_length = 10.0;
        let arrowhead_width = 5.0;

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

        frame.fill(
            &arrow_path,
            theme.primary,
        );
    }
}

fn find_intersection(from: Point, to: Point, path: &Path) -> Point {
    let lyon_path = path.raw();

    for event in lyon_path.iter() {
        match event {
            PathEvent::Line { from: start, to: end } => {
                if let Some(intersection) = line_line_intersection(from, to, start, end) {
                    return intersection;
                }
            }
            PathEvent::Quadratic { from: start, ctrl, to: end } => {
                if let Some(intersection) = line_quadratic_intersection(from, to, start, ctrl, end) {
                    return intersection;
                }
            }
            PathEvent::Cubic { from: start, ctrl1, ctrl2, to: end } => {
                if let Some(intersection) = line_cubic_intersection(from, to, start, ctrl1, ctrl2, end) {
                    return intersection;
                }
            }
            _ => {}
        }
    }

    to // Default to the 'to' point if no intersection is found
}

fn line_line_intersection(from: Point, to: Point, start: Point2D<f32>, end: Point2D<f32>) -> Option<Point> {
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

fn line_quadratic_intersection(from: Point, to: Point, start: Point2D<f32>, ctrl: Point2D<f32>, end: Point2D<f32>) -> Option<Point> {
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

fn line_cubic_intersection(from: Point, to: Point, start: Point2D<f32>, ctrl1: Point2D<f32>, ctrl2: Point2D<f32>, end: Point2D<f32>) -> Option<Point> {
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