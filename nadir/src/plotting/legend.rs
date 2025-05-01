use std::sync::{Arc, Mutex};

use super::{line::Line, theme::PlotTheme};
use iced::{
    Background, Border, Color, Point, Rectangle, Size, Vector,
    advanced::{graphics::text::cosmic_text::BufferLine, text::paragraph::Plain},
    font::Family,
    widget::{
        canvas::{
            Fill, Frame, Stroke, Text,
            path::{Builder, lyon_path::traits::PathBuilder},
        },
        container::draw_background,
        shader::wgpu::hal::MAX_VERTEX_BUFFERS,
    },
};
#[derive(Debug, Clone)]
pub struct Legend {
    background_color: Option<Color>,
    border: Option<Border>,
    bounds: Rectangle,
    entries: Vec<LegendEntry>,
    entry_padding: f32,
    line_height_factor: f32,
    marker_size: f32,
    marker_text_gap: f32,
    padding: f32,
    show_background: bool,
    pub(crate) text_size: f32,
}

impl Default for Legend {
    fn default() -> Self {
        Self {
            background_color: None,
            border: Some(Border::default()),
            bounds: Rectangle::new(Point::new(500.0, 100.0), Size::new(100.0, 50.0)),
            entries: Vec::new(),
            entry_padding: 3.0,
            line_height_factor: 1.0,
            marker_size: 9.0,
            marker_text_gap: 5.0,
            padding: 10.0,
            show_background: true,
            text_size: 12.0,
        }
    }
}

impl Legend {
    pub fn update(&mut self, axis_bounds: &Rectangle, lines: &Vec<Arc<Mutex<Line>>>) {
        self.entries = Vec::new();

        // Calculate the line height based on font size
        let line_height = self.text_size * self.line_height_factor;

        // Start with double padding for top and bottom
        let mut height = self.entry_padding;

        // Start with double padding for left and right
        let mut width = self.entry_padding;

        for (i, line) in lines.iter().enumerate() {
            let line = &mut *line.lock().unwrap();
            if line.legend {
                let label = if let Some(label) = &line.data.yname {
                    label.clone()
                } else {
                    i.to_string()
                };

                // Better text width estimation (still an approximation)
                // Assuming average character width is about 0.6× the font size
                let text_width = label.chars().count() as f32 * (self.text_size * 0.58);

                // Calculate total width for this entry:
                // marker + gap + text
                let entry_width = text_width;

                // Take the maximum width encountered
                width = width.max(entry_width + 2.0 * self.entry_padding);

                let color = line.color.clone();
                self.entries.push(LegendEntry { color, label });

                // Add the height of this entry to the total
                height += line_height;
            }
        }

        self.bounds.height = height + self.entry_padding;
        self.bounds.width = width + self.entry_padding;
        self.bounds.x = axis_bounds.x + axis_bounds.width - self.padding - self.bounds.width;
        self.bounds.y = axis_bounds.y + self.padding;
    }
    pub fn draw(&self, frame: &mut Frame, theme: &PlotTheme) {
        let legend_position = self.bounds.position();
        frame.translate(Vector::new(legend_position.x, legend_position.y));
        if self.show_background {
            let color = if let Some(background) = &self.background_color {
                background.clone()
            } else {
                theme.figure_background
            };

            frame.fill_rectangle(Point::ORIGIN, self.bounds.size(), Fill::from(color));
        }
        let line_height = self.text_size * self.line_height_factor;

        for (i, entry) in self.entries.iter().enumerate() {
            let mut text = Text::from(entry.label.clone());
            text.color = if let Some(color) = entry.color {
                color
            } else {
                theme.line_colors[i]
            };
            text.size = self.text_size.into();
            text.position = Point::new(
                self.entry_padding,
                self.entry_padding + i as f32 * line_height,
            );
            text.font.family = Family::Monospace;
            // text.line_height = line_height.into();

            frame.fill_text(text);
        }

        // draw the border
        let mut border_path = Builder::new();
        border_path.rectangle(Point::ORIGIN, self.bounds.size());

        let border_stroke = Stroke::default();

        frame.stroke(&border_path.build(), border_stroke);
    }
}

#[derive(Debug, Clone)]
struct LegendEntry {
    color: Option<Color>,
    label: String,
}
