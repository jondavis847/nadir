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
    pub rendered_bounds: Rectangle,
    pub is_left_clicked: bool,
    pub is_middle_clicked: bool,
    pub is_right_clicked: bool,
    pub is_selected: bool,
}

impl Node {
    pub fn new(label: String, bounds: Rectangle, zoom: f32) -> Self {
        let mut rendered_bounds = bounds.clone();
        rendered_bounds.height *= zoom;
        rendered_bounds.width *= zoom;

        Self {
            label: label,
            bounds: bounds,
            rendered_bounds: rendered_bounds,
            is_left_clicked: false,
            is_middle_clicked: false,
            is_selected: false,
            is_right_clicked: false,
        }
    }

    pub fn adjust_for_zoom(&mut self, zoom: f32, zoom_delta: f32, canvas_cursor_position: Point) {
        //TODO: just save height and width instead of an entire rectangle in node, since we dont actually use the x,y position of bounds ever
        self.rendered_bounds.height = self.bounds.height * zoom;
        self.rendered_bounds.width = self.bounds.width * zoom;

        let vector_top_left_to_cursor = self.rendered_bounds.position() - canvas_cursor_position;
        let new_vector = vector_top_left_to_cursor * zoom_delta;
        let new_position = canvas_cursor_position + new_vector;
        self.rendered_bounds.x = new_position.x;
        self.rendered_bounds.y = new_position.y;
    }

    pub fn calculate_path(&self, x_offset: f32, zoom: f32) -> Path {
        //TODO: save the path in the node for more efficient calc?
        let mut bounds = self.rendered_bounds;

        // because we create a new frame_with_clip, we need the canvas
        // to subtract off the nodebar width and create the node referenced to the graph
        bounds.x -= x_offset;
        let corner_radius = zoom * 3.0;

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

    pub fn draw(&self, frame: &mut Frame, theme: &Theme, x_offset: f32, zoom: f32) {
        let background = self.calculate_path(x_offset, zoom);

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
                    width: 5.0 * zoom,
                    ..Stroke::default()
                },
            );
            frame.fill(&background, node_background_color);
            let mut text_center = self.rendered_bounds.center();
            text_center.x -= x_offset;

            let mut text = Text {
                content: self.label.clone(),
                color: Color::WHITE, //theme.edge_multibody,
                font: Font::MONOSPACE,
                horizontal_alignment: Horizontal::Center,
                position: text_center,
                vertical_alignment: Vertical::Center,
                ..Text::default()
            };
            text.size = text.size * zoom;
            frame.fill_text(text);
        });
    }

    pub fn translate_by(&mut self, graph_translation: Vector) {
        self.rendered_bounds.x = self.rendered_bounds.x + graph_translation.x;
        self.rendered_bounds.y = self.rendered_bounds.y + graph_translation.y;
    }

    pub fn translate_to(&mut self, position: Point) {
        self.rendered_bounds.x = position.x - self.rendered_bounds.width / 2.0;
        self.rendered_bounds.y = position.y - self.rendered_bounds.height / 2.0;
    }

    pub fn is_clicked(
        &mut self,
        canvas_cursor_position: Point,
        mouse_button: &crate::ui::mouse::MouseButton,
    ) {
        let is_inside = self.rendered_bounds.contains(canvas_cursor_position);

        match mouse_button {
            crate::ui::mouse::MouseButton::Left => self.is_left_clicked = is_inside,
            crate::ui::mouse::MouseButton::Right => self.is_right_clicked = is_inside,
            crate::ui::mouse::MouseButton::Middle => self.is_middle_clicked = is_inside,
        }
    }
}
