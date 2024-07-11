use iced::{application, Border, Color};

macro_rules! color {
    ($red:expr, $green:expr, $blue:expr) => {
        Color::from_rgb(
            $red as f32 / 255.0,
            $green as f32 / 255.0,
            $blue as f32 / 255.0,
        )
    };
}

#[derive(Debug)]
pub struct Theme {
    pub background: Color,
    pub dark_background: Color,
    pub error: Color,
    pub node_background: Color,
    pub text_background: Color,
    pub text: Color,
    pub greyed: Color,
    pub border: Color,
    pub shadow: Color,
    pub primary: Color,
    pub highlight: Color,
}

impl Theme {
    pub const ORANGE: Self = Self {
        background: color!(37, 37, 38),
        dark_background: color!(27, 27, 28),
        error: color!(255, 0, 0),
        node_background: color!(30, 30, 31),
        text_background: color!(47, 47, 48),
        text: color!(204, 204, 200),
        greyed: color!(100, 100, 100),
        border: color!(10, 10, 10),
        shadow: color!(37, 36, 34),
        //primary: color!(235, 161, 66),
        //highlight: color!(212, 207, 40),
        primary: color!(235 / 2, 161 / 2, 66 / 2),
        highlight: color!(235, 161, 66),
    };
}

impl Default for Theme {
    fn default() -> Self {
        Self::ORANGE
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub enum Application {
    #[default]
    Default,
}

impl iced::application::StyleSheet for Theme {
    type Style = Application;

    fn appearance(&self, style: &Self::Style) -> iced::application::Appearance {
        match style {
            Application::Default => application::Appearance {
                background_color: self.background.into(),
                text_color: self.text,
            },
        }
    }
}

impl iced_aw::style::selection_list::StyleSheet for Theme {
    type Style = ();

    fn style(&self, _style: &Self::Style) -> iced_aw::style::selection_list::Appearance {
        iced_aw::style::selection_list::Appearance {
            background: self.background.into(),
            text_color: self.text,
            ..Default::default()
        }
    }
}

impl iced::widget::scrollable::StyleSheet for Theme {
    type Style = ();
    fn active(&self, _style: &Self::Style) -> iced::widget::scrollable::Appearance {
        let mut border = iced::Border::with_radius(2.0);
        border.color = self.border;
        border.width = 2.0;

        let container = {
            let shadow = iced::Shadow::default();

            iced::widget::container::Appearance {
                text_color: None,
                background: Some(self.background.into()),
                border,
                shadow,
            }
        };

        let scroller = iced::widget::scrollable::Scroller {
            color: self.dark_background.into(),
            border,
        };
        let scrollbar = iced::widget::scrollable::Scrollbar {
            background: Some(iced::Background::Color(self.background)),
            border,
            scroller,
        };
        let gap = None;
        iced::widget::scrollable::Appearance {
            container,
            scrollbar,
            gap,
        }
    }
    fn hovered(
        &self,
        style: &Self::Style,
        _is_mouse_over_scrollber: bool,
    ) -> iced::widget::scrollable::Appearance {
        self.active(style)
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub enum ContainerStyles {
    #[default]
    DefaultContainer,
    TabBarContainer,
}

impl iced::widget::container::StyleSheet for Theme {
    type Style = ContainerStyles;
    fn appearance(&self, style: &Self::Style) -> iced::widget::container::Appearance {
        match style {
            ContainerStyles::DefaultContainer => {
                let mut border = iced::Border::with_radius(2.0);
                border.color = self.border;
                border.width = 2.0;

                let shadow = iced::Shadow {
                    color: self.shadow,
                    offset: iced::Vector::new(3.0, 3.0),
                    blur_radius: 4.0,
                };

                iced::widget::container::Appearance {
                    text_color: None,
                    background: Some(self.background.into()),
                    border,
                    shadow,
                }
            }
            ContainerStyles::TabBarContainer => {
                let mut border = iced::Border::with_radius(2.0);
                border.color = self.border;
                border.width = 2.0;

                let shadow = iced::Shadow {
                    color: self.shadow,
                    offset: iced::Vector::new(3.0, 3.0),
                    blur_radius: 4.0,
                };

                iced::widget::container::Appearance {
                    text_color: None,
                    background: Some(self.dark_background.into()),
                    border,
                    shadow,
                }
            }
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub enum TextStyles {
    #[default]
    Default,
    Primary,
}

impl iced::widget::text::StyleSheet for Theme {
    type Style = TextStyles;

    fn appearance(&self, style: Self::Style) -> iced::widget::text::Appearance {
        match style {
            TextStyles::Default => iced::widget::text::Appearance {
                color: Some(self.text),
            },
            TextStyles::Primary => iced::widget::text::Appearance {
                color: Some(self.highlight),
            },
        }
    }
}

#[derive(Default)]
pub enum ButtonStyles {
    #[default]
    Default,
    Selected,
}

impl iced::widget::button::StyleSheet for Theme {
    type Style = ButtonStyles;

    fn active(&self, style: &Self::Style) -> iced::widget::button::Appearance {
        let border = Border {
            color: self.border,
            width: 2.0,
            radius: 0.0.into(),
        };
        match style {
            ButtonStyles::Default => iced::widget::button::Appearance {
                background: Some(iced::Background::Color(self.node_background)),
                border,
                text_color: self.primary,
                ..Default::default()
            },
            ButtonStyles::Selected => iced::widget::button::Appearance {
                background: Some(iced::Background::Color(self.primary)),
                border,
                text_color: self.primary,
                ..Default::default()
            },
        }
    }
    fn hovered(&self, style: &Self::Style) -> iced::widget::button::Appearance {
        let border = Border {
            color: self.border,
            width: 2.0,
            radius: 0.0.into(),
        };

        match style {
            ButtonStyles::Default => iced::widget::button::Appearance {
                background: Some(iced::Background::Color(self.background)),
                border,
                text_color: self.primary,
                ..Default::default()
            },
            ButtonStyles::Selected => iced::widget::button::Appearance {
                background: Some(iced::Background::Color(self.primary)),
                border,
                text_color: self.primary,
                ..Default::default()
            },
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub enum Card {
    #[default]
    Default,
    Error,
}

impl iced_aw::style::card::StyleSheet for Theme {
    type Style = Card;

    fn active(&self, style: &Self::Style) -> iced_aw::style::card::Appearance {
        match style {
            Card::Default => iced_aw::style::card::Appearance {
                background: iced::Background::Color(self.background),
                head_background: iced::Background::Color(self.node_background),
                head_text_color: self.primary,
                border_color: self.border,
                ..iced_aw::style::card::Appearance::default()
            },
            Card::Error => iced_aw::style::card::Appearance {
                background: iced::Background::Color(self.background),
                head_background: iced::Background::Color(self.error),
                head_text_color: self.error,
                border_color: self.border,
                ..iced_aw::style::card::Appearance::default()
            },
        }
    }
}

impl iced::widget::text_input::StyleSheet for Theme {
    type Style = Application;

    fn active(&self, _style: &Self::Style) -> iced::widget::text_input::Appearance {
        iced::widget::text_input::Appearance {
            background: iced::Background::Color(self.text_background),
            border: iced::Border {
                color: Color::BLACK,
                width: 1.0,
                radius: 0.0.into(),
            },
            icon_color: self.shadow,
        }
    }

    fn focused(&self, _style: &Self::Style) -> iced::widget::text_input::Appearance {
        iced::widget::text_input::Appearance {
            background: iced::Background::Color(self.text_background),
            border: iced::Border {
                color: Color::BLACK,
                width: 1.0,
                radius: 0.0.into(),
            },
            icon_color: self.shadow,
        }
    }

    fn placeholder_color(&self, _style: &Self::Style) -> Color {
        self.greyed
    }

    fn value_color(&self, _style: &Self::Style) -> Color {
        self.text
    }

    fn disabled_color(&self, _style: &Self::Style) -> Color {
        self.greyed
    }

    fn selection_color(&self, _style: &Self::Style) -> Color {
        self.text
    }

    fn disabled(&self, _style: &Self::Style) -> iced::widget::text_input::Appearance {
        let border = iced::Border {
            color: self.border,
            ..Default::default()
        };

        iced::widget::text_input::Appearance {
            background: iced::Background::Color(self.background),
            border: border,
            icon_color: self.shadow,
        }
    }

    fn hovered(&self, _style: &Self::Style) -> iced::widget::text_input::Appearance {
        iced::widget::text_input::Appearance {
            background: iced::Background::Color(self.text_background),
            border: iced::Border {
                color: Color::BLACK,
                width: 1.0,
                radius: 0.0.into(),
            },
            icon_color: self.shadow,
        }
    }
}

impl iced_aw::style::modal::StyleSheet for Theme {
    type Style = ();
    fn active(&self, _style: &Self::Style) -> iced_aw::style::modal::Appearance {
        iced_aw::style::modal::Appearance {
            background: iced::Background::Color(Color::from_rgba(
                self.background.r,
                self.background.g,
                self.background.b,
                0.95,
            )),
        }
    }
}
