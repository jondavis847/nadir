use std::sync::{Arc, Mutex};

use super::{line::Line, theme::PlotTheme};
use iced::{
    Background, Border, Color, Point, Rectangle, Size, Vector,
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
    entry_padding: f32,
    text_size: f32,
}

impl Default for Legend {
    fn default() -> Self {
        Self {
            background_color: Some(Color::BLACK),
            border: None,
            bounds: Rectangle::new(Point::new(500.0, 100.0), Size::new(100.0, 50.0)),
            entries: Vec::new(),
            entry_padding: 5.0,
            text_size: 9.0,
        }
    }
}

impl Legend {
    pub fn update(&mut self, lines: &Vec<Arc<Mutex<Line>>>) {
        self.entries = Vec::new();
        let mut height = self.entry_padding;
        let mut width = self.entry_padding;
        for (i, line) in lines.iter().enumerate() {
            let line = &mut *line.lock().unwrap();
            if line.legend {
                let label = if let Some(label) = &line.data.yname {
                    label.clone()
                } else {
                    i.to_string()
                };
                let entry_width = label.len() as f32 * self.text_size + self.entry_padding;
                if entry_width > width {
                    width = entry_width;
                }

                let color = line.color.clone();
                self.entries.push(LegendEntry { color, label });
                height += 4.0 / 3.0 * self.text_size + self.entry_padding;
            }
        }
        self.bounds.height = height;
        self.bounds.width = width;
    }
    pub fn draw(&self, frame: &mut Frame, theme: &PlotTheme) {
        if let Some(background) = &self.background_color {
            frame.fill_rectangle(
                self.bounds.position(),
                self.bounds.size(),
                Fill::from(background.clone()),
            );
        }
        frame.with_save(|frame| {
            let legend_position = self.bounds.position();
            frame.translate(Vector::new(legend_position.x, legend_position.y));
            for (i, entry) in self.entries.iter().enumerate() {
                let mut text = Text::from(entry.label.clone());
                text.color = if let Some(color) = entry.color {
                    color
                } else {
                    theme.line_colors[i]
                };

                frame.with_save(|frame| {
                    frame.translate(Vector::new(
                        self.entry_padding,
                        i as f32 * (self.text_size + self.entry_padding),
                    ));
                    frame.fill_text(text);
                })
            }
        })
    }
}

#[derive(Debug, Clone)]
struct LegendEntry {
    color: Option<Color>,
    label: String,
}
