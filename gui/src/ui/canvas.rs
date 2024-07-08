use iced::{
    mouse::{self, Cursor},
    widget::canvas::{
        self,
        event::{Event, Status},
        Geometry, Path, Stroke,
    },
    Point, Rectangle, Renderer,
};

pub mod edge;
pub mod graph;
pub mod node;
pub mod nodebar;

use crate::ui::theme::Theme;
use crate::Message;

#[derive(Debug)]
pub struct GraphCanvas<'a> {
    app_state: &'a crate::AppState,
}

impl<'a> GraphCanvas<'a> {
    pub fn new(app_state: &'a crate::AppState) -> Self {
        Self {
            app_state: app_state,
        }
    }
}

impl<'a> canvas::Program<Message, Theme> for GraphCanvas<'a> {
    type State = ();

    fn update(
        &self,
        _state: &mut Self::State,
        event: Event,
        bounds: Rectangle,
        cursor: Cursor,
    ) -> (Status, Option<Message>) {
        if !cursor.is_over(bounds) {
            return (Status::Ignored, None);
        };

        let canvas_cursor_position = cursor.position_in(bounds).unwrap();

        match event {
            Event::Mouse(mouse_event) => match mouse_event {
                mouse::Event::ButtonPressed(mouse::Button::Left) => (
                    Status::Captured,
                    Some(Message::LeftButtonPressed(canvas_cursor_position)),
                ),
                mouse::Event::ButtonReleased(mouse::Button::Left) => (
                    Status::Captured,
                    Some(Message::LeftButtonReleased(canvas_cursor_position)),
                ),
                mouse::Event::ButtonPressed(mouse::Button::Middle) => (
                    Status::Captured,
                    Some(Message::MiddleButtonPressed(canvas_cursor_position)),
                ),
                mouse::Event::ButtonPressed(mouse::Button::Right) => (
                    Status::Captured,
                    Some(Message::RightButtonPressed(canvas_cursor_position)),
                ),
                mouse::Event::ButtonReleased(mouse::Button::Right) => (
                    Status::Captured,
                    Some(Message::RightButtonReleased(canvas_cursor_position)),
                ),
                mouse::Event::CursorMoved { position: _ } => (
                    Status::Captured,
                    Some(Message::CursorMoved(canvas_cursor_position)),
                ),
                mouse::Event::WheelScrolled { delta } => {
                    (Status::Captured, Some(Message::WheelScrolled(delta)))
                }
                _ => (Status::Captured, None),
            },
            _ => (Status::Ignored, None),
        }
    }

    fn draw(
        &self,
        _state: &Self::State,
        renderer: &Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        _cursor: mouse::Cursor,
    ) -> Vec<Geometry> {
        let nodebar_width = self.app_state.nodebar.bounds.width;
        let zoom = self.app_state.graph.zoom;
        let all_content = self.app_state.cache.draw(renderer, bounds.size(), |frame| {
            // node_bar border
            frame.stroke(
                &Path::rectangle(Point::ORIGIN, self.app_state.nodebar.bounds.size()),
                Stroke::default().with_width(2.0),
            );

            // canvas border
            frame.stroke(
                &Path::rectangle(Point::ORIGIN, frame.size()),
                Stroke::default().with_width(2.0),
            );

            // create edges (before nodes so nodes clip the tail)
            frame.with_clip(self.app_state.graph.bounds, |frame| {
                self.app_state.graph.edges.iter().for_each(|(_, edge)| {
                    edge.draw(
                        frame,
                        &self.app_state.graph.nodes,
                        &self.app_state.theme,
                        nodebar_width,zoom
                    )
                });
            });

            // create nodes that are not clipped (nodebar)
            self.app_state
                .nodebar
                .nodes
                .iter()
                .for_each(|(_, nodebarnode)| {
                    nodebarnode.node.draw(frame, &self.app_state.theme, 0.0, 1.0);
                });

            // create nodes that are clipped (graph)

            frame.with_clip(self.app_state.graph.bounds, |frame| {
                self.app_state
                    .graph
                    .nodes
                    .iter()
                    .for_each(|(_, graphnode)| {
                        graphnode
                            .node
                            .draw(frame, &self.app_state.theme, nodebar_width, zoom);
                    });
            });
        });
        vec![all_content]
    }

    fn mouse_interaction(
        &self,
        _state: &Self::State,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> mouse::Interaction {
        if cursor.is_over(bounds) {
            if self.app_state.graph.is_clicked {
                mouse::Interaction::Grabbing
            } else {
                mouse::Interaction::Grab
            }
        } else {
            mouse::Interaction::default()
        }
    }
}
