use crate::{ui::select_menu::SelectMenu, Message};
use iced::{widget::Row, Element, Length};

pub mod plot_canvas;
use plot_canvas::PlotCanvas;

#[derive(Debug, Default)]
pub struct PlotTab {
    pub sim_menu: SelectMenu,
    pub component_menu: SelectMenu,
    pub state_menu: SelectMenu,
    pub canvas: PlotCanvas,
}

impl PlotTab {
    pub fn content(&self) -> Element<Message, crate::ui::theme::Theme> {
        let sim_menu = self
            .sim_menu
            .content(|string| Message::PlotSimSelected(string));
        let component_menu = self
            .component_menu
            .content(|string| Message::PlotComponentSelected(string));

        let canvas = self.canvas.content();

        Row::new()
            .push(sim_menu)
            .push(component_menu)
            .push(canvas)
            .height(Length::FillPortion(17))
            .width(Length::Fill)
            .into()
    }
}
