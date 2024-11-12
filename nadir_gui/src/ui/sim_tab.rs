pub mod canvas;
pub mod sim_div;

use canvas::GraphCanvas;
use sim_div::SimDiv;

use crate::{ui::theme::Theme, AppState, Message};
use iced::{
    widget::{canvas::Canvas, container, Row},
    Element, Length,
};

#[derive(Debug, Default)]
pub struct SimTab {
    pub sim_div: SimDiv,
}

impl SimTab {
    pub fn content<'a>(&'a self, state: &'a AppState) -> Element<Message, Theme> {
        let sim_div = self.sim_div.content();

        let graph_canvas = GraphCanvas::new(state);
        let graph_container = container(
            Canvas::new(graph_canvas)
                .width(Length::Fill)
                .height(Length::Fill),
        )
        .width(Length::FillPortion(4))
        .height(Length::Fill);

        Row::new()
            .push(sim_div)
            .push(graph_container)
            .height(Length::FillPortion(15))
            .width(Length::Fill)
            .into()
    }
}
