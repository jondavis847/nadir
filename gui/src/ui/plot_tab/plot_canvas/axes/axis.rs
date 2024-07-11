use crate::ui::theme::Theme;
use iced::{
    alignment::{Horizontal, Vertical},
    widget::canvas::{Frame, Path, Stroke, Text},
    Point, Rectangle, Vector,
};

#[derive(Debug)]
pub struct Axis {
    pub padding: f32,
    pub line_width: f32,
    n_ticks: u32,
    tick_length: f32,
    tick_text_spacing: f32,
}

impl Default for Axis {
    fn default() -> Self {
        Self {
            padding: 50.0,
            line_width: 3.0,
            n_ticks: 5,
            tick_length: 10.0,
            tick_text_spacing: 20.0,
        }
    }
}

impl Axis {
    pub fn draw(
        &self,
        frame: &mut Frame,
        theme: &Theme,
        axes_bounds: &Rectangle,
        xlim: &(f32, f32),
        ylim: &(f32, f32),
    ) {
        //draw the x axis
        let axes_size = axes_bounds.size();

        let axis_top_left = Point::new(self.padding, self.padding);
        let axis_top_right = Point::new(axes_size.width - self.padding, self.padding);

        let axis_bottom_left_x = self.padding;
        let axis_bottom_left_y = axes_size.height - self.padding;
        let axis_bottom_left = Point::new(axis_bottom_left_x, axis_bottom_left_y);

        let axis_bottom_right_x = axes_size.width - self.padding;
        let axis_bottom_right_y = axes_size.height - self.padding;
        let axis_bottom_right = Point::new(axis_bottom_right_x, axis_bottom_right_y);

        let x_axis_line = Path::line(axis_bottom_left, axis_bottom_right);
        let y_axis_line = Path::line(axis_bottom_left, axis_top_left);
        let top_line = Path::line(axis_top_left, axis_top_right);
        let right_line = Path::line(axis_top_right, axis_bottom_right);

        let axis_stroke = Stroke::default().with_width(self.line_width);
        let border_stroke = Stroke::default().with_width(1.0);

        frame.stroke(&x_axis_line, axis_stroke.clone());
        frame.stroke(&y_axis_line, axis_stroke.clone());
        frame.stroke(&top_line, border_stroke.clone());
        frame.stroke(&right_line, border_stroke.clone());

        // calculate x ticks
        let value_span = xlim.1 - xlim.0;
        let canvas_span = axis_bottom_right_x - axis_bottom_left_x;
        let (x_tick_spacing, x_tick_increment) =
            calculate_tick_spacing(value_span, canvas_span, self.n_ticks);

        let mut x_canvas = axis_bottom_left_x as f32;
        let mut x_value = xlim.0;
        let y_canvas = axis_bottom_left.y;
        for _ in 0..self.n_ticks {
            let x_tick_point = Point::new(x_canvas, y_canvas);
            let x_tick_path = Path::line(
                x_tick_point + Vector::new(0.0, self.tick_length / 2.0),
                x_tick_point + Vector::new(0.0, -self.tick_length / 2.0),
            );
            frame.stroke(&x_tick_path, axis_stroke.clone());

            // Automatically decide between fixed-point and scientific notation
            let label = if x_value.abs() < 0.01 || x_value.abs() > 1000.0 {
                format!("{:e}", x_value) // use scientific notation
            } else {
                format!("{:.2}", x_value) // use fixed precision
            };
            let text_center = x_tick_point + Vector::new(0.0, self.tick_text_spacing);

            let text = Text {
                content: label,
                color: theme.primary, //theme.edge_multibody,
                horizontal_alignment: Horizontal::Center,
                position: text_center,
                vertical_alignment: Vertical::Center,
                ..Text::default()
            };
            frame.fill_text(text);

            // increment counters
            x_canvas += x_tick_spacing;
            x_value += x_tick_increment;
        }

        // calculate y ticks
        let value_span = ylim.1 - ylim.0;
        let canvas_span = axis_bottom_left_y - axis_top_left.y;
        let (y_tick_spacing, y_tick_increment) =
            calculate_tick_spacing(value_span, canvas_span, self.n_ticks);

        let mut y_canvas = axis_bottom_left_y as f32;
        let mut y_value = ylim.0;
        let x_canvas = axis_bottom_left.x;
        for _ in 0..self.n_ticks {
            let y_tick_point = Point::new(x_canvas, y_canvas);
            let y_tick_path = Path::line(
                y_tick_point + Vector::new(-self.tick_length / 2.0, 0.0),
                y_tick_point + Vector::new(self.tick_length / 2.0, 0.0),
            );
            frame.stroke(&y_tick_path, axis_stroke.clone());
            // Automatically decide between fixed-point and scientific notation
            let label = if x_value.abs() < 0.01 || x_value.abs() > 1000.0 {
                format!("{:e}", y_value) // use scientific notation
            } else {
                format!("{:.2}", y_value) // use fixed precision
            };
            let text_center = y_tick_point + Vector::new(-self.tick_text_spacing, 0.0);

            let text = Text {
                content: label,
                color: theme.primary, //theme.edge_multibody,
                horizontal_alignment: Horizontal::Center,
                position: text_center,
                vertical_alignment: Vertical::Center,
                ..Text::default()
            };
            frame.fill_text(text);

            // increment counters
            y_canvas -= y_tick_spacing;
            y_value += y_tick_increment;
        }
    }
}

fn calculate_tick_spacing(value_span: f32, canvas_span: f32, n_ticks: u32) -> (f32, f32) {
    // create tick marks
    // default to some kind of whole number, and about 5 ticks per axis
    const BASE: f32 = 10.0;

    let mut ratio = BASE.powf((1.0 / (n_ticks - 1) as f32).log(BASE).floor());

    let mut tick_increment = value_span * ratio;
    // double the ratio until we fill the axes appropriately
    while value_span / tick_increment > n_ticks as f32 {
        ratio *= 2.0;
        tick_increment = value_span * ratio;
    }

    let tick_spacing = canvas_span * ratio;
    (tick_spacing, tick_increment)
}
