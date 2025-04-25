use iced::{Color, color};

#[derive(Debug, Default)]
pub enum PlotThemes {
    #[default]
    Dark,
}

#[derive(Debug)]
pub struct PlotTheme {
    pub axes_border: Color,
    pub axes_background: Color,
    pub axis_background: Color,
    pub axis_border: Color,
    pub figure_background: Color,
    pub grid_color: Color,
    pub text_color: Color,
    pub line_colors: [Color; 10],
}

impl PlotThemes {
    pub fn palette(&self) -> PlotTheme {
        match self {
            PlotThemes::Dark => PlotTheme {
                axes_border: color!(0x000000),
                axes_background: color!(0x111111),
                axis_border: color!(0x000000),
                axis_background: color!(0x131313),
                figure_background: color!(0x121212),
                grid_color: color!(0x191919),
                text_color: color!(0x767676),
                line_colors: [
                    color!(0xf94144), //red
                    color!(0x277da1), //blue
                    color!(0x90be6d), //green
                    color!(0xf9c74f), //yello
                    color!(0x577590), // bluish purple
                    color!(0xf9844a), //salmon
                    color!(0x43aa8b), //teal
                    color!(0xf8961e), //orange
                    color!(0xf3722c), // dark orange
                    color!(0x4d908e), // gray blue
                ],
            },
        }
    }
}
