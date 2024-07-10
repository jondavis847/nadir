use super::line::Line;
use crate::ui::theme::Theme;
use iced::{
    widget::canvas::{path::Path, Fill, Frame, Stroke},
    Point, Rectangle, Size, Vector,
};

pub mod axis;
use axis::Axis;

#[derive(Debug)]
pub struct Axes {
    padding: f32,
    axis: Axis,
    xlim: (f32, f32),
    ylim: (f32, f32),
    lines: Vec<Line>,
}

impl Default for Axes {
    fn default() -> Self {
        Self {
            padding: 10.0,
            axis: Axis::default(),
            xlim: (0.0, 1.0),
            ylim: (0.0, 1.0),
            lines: Vec::new(),
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
}
