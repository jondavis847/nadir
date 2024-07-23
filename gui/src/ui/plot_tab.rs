use crate::{ui::select_menu::SelectMenu, Message};
use iced::{widget::Row, Element, Length, Point};

pub mod plot_canvas;
use plot_canvas::PlotCanvas;

#[derive(Debug)]
pub struct PlotTab {
    pub sim_menu: SelectMenu,
    pub component_menu: SelectMenu,
    pub state_menu: SelectMenu,
    pub selected_components: Vec<String>,
    pub selected_sims: Vec<String>,
    pub selected_states: Vec<String>,
    pub canvas: PlotCanvas,
}

impl Default for PlotTab {
    fn default() -> Self {
        Self {
            sim_menu: SelectMenu::new(Length::FillPortion(1), false),
            component_menu: SelectMenu::new(Length::FillPortion(1), true),
            state_menu: SelectMenu::new(Length::FillPortion(2), true),
            selected_components: Vec::new(),
            selected_sims: Vec::new(),
            selected_states: Vec::new(),
            canvas: PlotCanvas::default(),
        }
    }
}

impl PlotTab {
    pub fn content(&self) -> Element<Message, crate::ui::theme::Theme> {
        let sim_menu = self
            .sim_menu
            .content(|string| Message::PlotSimSelected(string));
        let component_menu = self
            .component_menu
            .content(|string| Message::PlotComponentSelected(string));

        let state_menu = self
            .state_menu
            .content(|string| Message::PlotStateSelected(string));

        let canvas = self.canvas.content();

        Row::new()
            .push(sim_menu)
            .push(component_menu)
            .push(state_menu)
            .push(canvas)
            .height(Length::FillPortion(15))
            .width(Length::Fill)
            .into()
    }

    #[inline]
    pub fn plot(&mut self, line_label: String, points: Vec<Point>) {
        self.canvas.plot(line_label, points);
    }

    pub fn left_button_pressed(&self, _position: Point) {
        //Nothing for now
    }
}
