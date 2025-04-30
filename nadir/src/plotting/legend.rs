use super::{line::Line, theme::PlotTheme};
use iced::{
    Background, Border, Color, Point, Rectangle, Size,
    advanced::graphics::text::cosmic_text::BufferLine,
    widget::{
        canvas::{Fill, Frame, Text},
        container::draw_background,
    },
};
#[derive(Debug, Clone)]
pub struct Legend {
    background_color: Option<Color>,
    border: Option<Border>,
    bounds: Rectangle,
    entries: Vec<LegendEntry>,
    text_size: f32,
}

impl Default for Legend {
    fn default() -> Self {
        Self {
            background_color: Some(Color::BLACK),
            border: None,
            bounds: Rectangle::new(Point::new(500.0, 100.0), Size::new(100.0, 50.0)),
            entries: Vec::new(),
            text_size: 10.0,
        }
    }
}

impl Legend {
    pub fn update(&mut self, lines: &Vec<Line>, theme: PlotTheme) {
        self.entries = Vec::new();
        for (i, line) in lines.iter().enumerate() {
            if line.legend {
                let label = if let Some(label) = &line.label {
                    label.clone()
                } else {
                    i.to_string()
                };

                let color = if let Some(color) = &line.color {
                    color.clone()
                } else {
                    theme.line_colors[i]
                };

                self.entries.push(LegendEntry { color, label })
            }
        }
    }
    pub fn draw(&self, frame: &mut Frame, theme: PlotTheme, lines: &Vec<Line>) {
        if let Some(background) = &self.background_color {
            frame.fill_rectangle(
                self.bounds.position(),
                self.bounds.size(),
                Fill::from(background.clone()),
            );
        }
        for entry in &self.entries {
            let mut text = Text::from(entry.label.clone());
            text.color = entry.color;
            frame.fill_text(text);
        }
    }
}

pub enum LegendStyle {
    Inside,
    Top,
    Right,
}

#[derive(Debug, Clone)]
struct LegendEntry {
    color: Color,
    label: String,
}
