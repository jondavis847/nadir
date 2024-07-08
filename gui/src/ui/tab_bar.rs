use crate::{
    ui::theme::{ContainerStyles, TextStyles},
    Message,
};
use iced::{
    widget::{button, container, text::Text, Row},
    Element, Length,
};

#[derive(Debug, Clone)]
pub enum AppTabs {
    Simulation,
    Plot,
    Animation,
}

#[derive(Debug, Clone)]
pub struct TabBarState {
    pub current_tab: AppTabs,
}

impl Default for TabBarState {
    fn default() -> Self {
        Self {
            current_tab: AppTabs::Simulation,
        }
    }
}

#[derive(Debug, Clone)]
pub struct TabBar {
    pub state: TabBarState,
}

impl Default for TabBar {
    fn default() -> Self {
        let state = TabBarState::default();
        Self { state }
    }
}

impl TabBar {
    pub fn content(&self) -> Element<Message, crate::ui::theme::Theme> {
        let create_button = |label: &str, message: Message| {
            button(
                Text::new(label.to_string())
                    .height(Length::Fill)
                    .width(150.0)
                    .horizontal_alignment(iced::alignment::Horizontal::Center)
                    .vertical_alignment(iced::alignment::Vertical::Center)
                    .style(TextStyles::Primary),
            )
            .on_press(message)
        };

        let animation_button = create_button("Animation", Message::TabAnimationPressed);
        let plot_button = create_button("Plot", Message::TabPlotPressed);
        let simulation_button = create_button("Simulation", Message::TabSimulationPressed);
        let tab_bar_container = container(
            Row::new()
                .push(simulation_button)
                .push(plot_button)
                .push(animation_button),
        )
        .height(Length::Fill)
        .width(Length::Fill)
        .style(ContainerStyles::TabBarContainer);
        Row::new().push(tab_bar_container).into()
    }
}
