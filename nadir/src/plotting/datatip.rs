use std::sync::{Arc, Mutex};

use iced::{
    Point, Rectangle, Size, Vector,
    widget::canvas::{Frame, Path, Stroke, Style, Text},
};

use super::{line::Line, theme::PlotTheme};

#[derive(Debug, Clone)]
pub struct Datatip {
    anchor: Point,
    bounds: Rectangle,
    color: usize,
    text_size: f32,
    x_value: f64,
    y_value: f64,
}

impl Datatip {
    pub fn new(anchor: Point, color: usize, x_value: f64, y_value: f64) -> Self {
        let bounds = Rectangle::new(anchor + Vector::new(30.0, -30.0), Size::new(50.0, 25.0));
        Self {
            anchor,
            bounds,
            color,
            text_size: 9.0,
            x_value,
            y_value,
        }
    }
    pub fn draw(&self, frame: &mut Frame, theme: &PlotTheme, lines: &Vec<Arc<Mutex<Line>>>) {
        let line = lines[self.color].lock().unwrap();
        let line_color = if let Some(color) = line.color {
            color
        } else {
            theme.line_colors[self.color]
        };
        let stroke = Stroke {
            style: Style::Solid(line_color),
            ..Default::default()
        };
        let path = Path::line(self.anchor, self.bounds.center());
        frame.stroke(&path, stroke);
        frame.fill_rectangle(self.bounds.position(), self.bounds.size(), theme.datatip);
        frame.stroke_rectangle(self.bounds.position(), self.bounds.size(), stroke);

        frame.with_save(|frame| {
            frame.translate(Vector::new(
                self.bounds.position().x,
                self.bounds.position().y,
            ));

            let float_format = |value: f64| -> String {
                if value == 0.0 {
                    "0".to_string()
                } else if value.abs() < 0.01 || value.abs() > 1000.0 {
                    format!("{:.3e}", value)
                } else {
                    let formatted = format!("{:.3}", value);
                    if formatted.ends_with(".000") {
                        formatted.trim_end_matches(".000").to_string()
                    } else {
                        formatted
                    }
                }
            };

            let x_string = float_format(self.x_value);
            let y_string = float_format(self.y_value);

            let text = format!("   x: {}\n   y: {}", x_string, y_string);
            frame.fill_text(Text {
                content: text,
                color: theme.text_color,
                position: Point::ORIGIN,
                size: (self.text_size).into(),
                ..Text::default()
            });
        });
    }
}
