use crate::font::Font;
use crate::ui::theme::Theme;

use iced::{
    alignment::{Horizontal, Vertical},
    widget::canvas::{path::Path, stroke, Frame, Stroke, Text},
    Color, Point, Rectangle, Vector,
};

//TODO: Think about using MultibodyMeta instead of the individual Uuid fields
#[derive(Debug, Clone)]
pub struct Node {
    pub label: String,
    pub bounds: Rectangle,
    pub is_left_clicked: bool,
    pub is_middle_clicked: bool,
    pub is_right_clicked: bool,
    pub is_selected: bool,
    //pub label: String,
}

impl Node {
    pub fn new(
        label: String,
        bounds: Rectangle,
        //label: String,
    ) -> Self {
        Self {
            label: label,
            bounds: bounds,
            is_left_clicked: false,
            is_middle_clicked: false,
            is_selected: false,
            is_right_clicked: false,
            //label: label,
        }
    }

    pub fn calculate_path(&self) -> Path {
        let bounds = self.bounds;
        let corner_radius = 3.0;

        let path = Path::new(|p| {
            p.move_to(Point::new(bounds.x + corner_radius, bounds.y));
            p.line_to(Point::new(
                bounds.x + bounds.width - corner_radius,
                bounds.y,
            ));
            p.arc_to(
                Point::new(bounds.x + bounds.width, bounds.y),
                Point::new(bounds.x + bounds.width, bounds.y + corner_radius),
                corner_radius,
            );
            p.line_to(Point::new(
                bounds.x + bounds.width,
                bounds.y + bounds.height - corner_radius,
            ));
            p.arc_to(
                Point::new(bounds.x + bounds.width, bounds.y + bounds.height),
                Point::new(
                    bounds.x + bounds.width - corner_radius,
                    bounds.y + bounds.height,
                ),
                corner_radius,
            );
            p.line_to(Point::new(
                bounds.x + corner_radius,
                bounds.y + bounds.height,
            ));
            p.arc_to(
                Point::new(bounds.x, bounds.y + bounds.height),
                Point::new(bounds.x, bounds.y + bounds.height - corner_radius),
                corner_radius,
            );
            p.line_to(Point::new(bounds.x, bounds.y + corner_radius));
            p.arc_to(
                Point::new(bounds.x, bounds.y),
                Point::new(bounds.x + corner_radius, bounds.y),
                corner_radius,
            );
        });

        path
    }

    pub fn draw(&self, frame: &mut Frame, theme: &Theme) {
        let background = self.calculate_path();

        let node_border_color;
        if self.is_selected {
            node_border_color = theme.highlight;
        } else {
            node_border_color = theme.primary;
        }

        let node_background_color = theme.node_background;

        frame.with_save(|frame| {
            frame.stroke(
                &background,
                Stroke {
                    style: stroke::Style::Solid(node_border_color),
                    width: 5.0,
                    ..Stroke::default()
                },
            );
            frame.fill(&background, node_background_color);
            frame.fill_text(Text {
                content: self.label.clone(),
                color: Color::WHITE, //theme.edge_multibody,
                font: Font::MONOSPACE,
                horizontal_alignment: Horizontal::Center,
                position: self.bounds.center(),
                vertical_alignment: Vertical::Center,
                ..Text::default()
            });
        });
    }

    pub fn translate_by(&mut self, graph_translation: Vector) {
        self.bounds.x = self.bounds.x + graph_translation.x;
        self.bounds.y = self.bounds.y + graph_translation.y;
    }

    pub fn translate_to(&mut self, position: Point) {
        self.bounds.x = position.x - self.bounds.width / 2.0;
        self.bounds.y = position.y - self.bounds.height / 2.0;
    }

    pub fn is_clicked(&mut self, cursor_position: Point, mouse_button: &crate::ui::mouse::MouseButton) {
        let is_inside = self.bounds.contains(cursor_position);

        match mouse_button {
            crate::ui::mouse::MouseButton::Left => self.is_left_clicked = is_inside,
            crate::ui::mouse::MouseButton::Right => self.is_right_clicked = is_inside,
            crate::ui::mouse::MouseButton::Middle => self.is_middle_clicked = is_inside,
        }
    }
}
