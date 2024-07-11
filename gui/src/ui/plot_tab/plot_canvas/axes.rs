use super::line::Line;
use crate::ui::theme::Theme;
use iced::{
    widget::canvas::{Fill, Frame, Path, Stroke},
    Point, Rectangle, Size, Vector,
};
use std::collections::HashMap;

pub mod axis;
use axis::Axis;

#[derive(Debug)]
pub struct Axes {
    padding: f32,
    axis: Axis,
    xlim: (f32, f32),
    ylim: (f32, f32),
    lines: HashMap<String, Line>,
}

impl Default for Axes {
    fn default() -> Self {
        Self {
            padding: 10.0,
            axis: Axis::default(),
            xlim: (0.0, 1.0),
            ylim: (0.0, 1.0),
            lines: HashMap::new(),
        }
    }
}

impl Axes {
    pub fn draw(&self, frame: &mut Frame, theme: &Theme) {
        // update the bounds based on the frame
        let axes_bounds = self.get_bounds(frame);
        self.background(frame, theme, &axes_bounds);
        frame.with_save(|frame| {
            let translation = Vector::new(self.padding, self.padding);
            frame.translate(translation);
            self.axis
                .draw(frame, theme, &axes_bounds, &self.xlim, &self.ylim);
        });
        self.draw_lines(frame, theme);
        dbg!(&self.lines);
    }

    fn draw_lines(&self, frame: &mut Frame, theme: &Theme) {
        for (_, line) in &self.lines {
            if line.data.len() > 1 {
                let path = Path::new(|builder| {
                    builder.move_to(line.data[0]);

                    for point in &line.data[1..] {
                        builder.line_to(*point);
                    }
                });

                frame.stroke(&path, Stroke::default().with_color(line.color));
            }
        }
    }

    fn background(&self, frame: &mut Frame, theme: &Theme, bounds: &Rectangle) {
        let axes_top_left = bounds.position();
        let axes_size = bounds.size();
        let axes_background = Fill::from(theme.dark_background);

        // create the axes border
        let axes_border = Path::rectangle(axes_top_left, axes_size);
        frame.stroke(&axes_border, Stroke::default());

        // fill the background
        frame.fill_rectangle(axes_top_left, axes_size, axes_background)
    }

    pub fn get_bounds(&self, frame: &Frame) -> Rectangle {
        let frame_size = frame.size();
        let axes_height = frame_size.height - 2.0 * self.padding;
        let axes_width = frame_size.width - 2.0 * self.padding;
        let axes_top_left = Point::new(self.padding, self.padding);
        let axes_size = Size::new(axes_width, axes_height);
        let axes_bounds = Rectangle::new(axes_top_left, axes_size);
        axes_bounds
    }

    pub fn plot(&mut self, line_label: String, points: Vec<Point>) {
        let theme = Theme::ORANGE;
        let line = Line::new(line_label.clone(), points, theme.highlight);
        self.lines.insert(line_label, line);
    }
}
