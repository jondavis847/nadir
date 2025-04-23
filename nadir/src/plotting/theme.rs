use iced::{Color, color};

#[derive(Debug, Default)]
pub enum PlotThemes {
    #[default]
    Dark,
}

#[derive(Debug)]
pub struct PlotTheme {
    pub dark_background: Color,
    pub light_background: Color,
    pub border: Color,
    pub axis_border: Color,
    pub axis_color: Color,
    pub grid_color: Color,
    pub text_color: Color,
    pub line_colors: [Color; 10],
}

impl PlotThemes {
    pub fn palette(&self) -> PlotTheme {
        match self {
            PlotThemes::Dark => PlotTheme {
                dark_background: color!(0x111111),
                light_background: color!(0x202020),
                border: color!(0x000000),
                axis_border: color!(0xffffff),
                axis_color: color!(0x202020),
                grid_color: color!(0x202020),
                text_color: color!(0x565656),
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
