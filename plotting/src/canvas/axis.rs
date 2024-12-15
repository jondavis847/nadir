use crate::theme::PlotTheme;
use iced::{
    alignment::{Horizontal, Vertical},
    widget::canvas::{Frame, Path, Stroke, Text},
    Point, Rectangle, Size, Vector,
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
            line_width: 5.0,
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
        theme: &PlotTheme,
        canvas_bounds: &Rectangle,
        xlim: &(f32, f32),
        ylim: &(f32, f32),
    ) {
        // Define constants
        let padding = self.padding;
        let canvas_size = canvas_bounds.size();
        let axes_size = Size::new(
            canvas_size.width - 2.0 * padding,
            canvas_size.height - 2.0 * padding,
        );

        let axis_bottom_left = Point::new(padding, axes_size.height + padding);
        let axis_bottom_right = Point::new(axes_size.width + padding, axes_size.height + padding);
        let axis_top_left = Point::new(padding, padding);
        let axis_top_right = Point::new(axes_size.width + padding, padding);

        // Draw border lines
        let border_stroke = Stroke::default().with_width(1.0).with_color(theme.border);

        let axis_lines = [
            (axis_bottom_left, axis_bottom_right), // x-axis border
            (axis_bottom_left, axis_top_left),     // y-axis border
            (axis_top_left, axis_top_right),       // top border
            (axis_top_right, axis_bottom_right),   // right border
        ];

        for &(start, end) in &axis_lines {
            frame.stroke(&Path::line(start, end), border_stroke.clone());
        }

        // Draw ticks, labels, and grid lines
        let draw_ticks_labels_and_grids =
            |frame: &mut Frame,
             theme: &PlotTheme,
             axis_start: Point,
             axis_end: Point,
             axes_size: Size,
             value_range: (f32, f32),
             canvas_span: f32,
             n_ticks: u32,
             is_x_axis: bool,
             _tick_length: f32,
             tick_text_spacing: f32,
             line_width: f32| {
                let value_span = value_range.1 - value_range.0;
                let (tick_spacing, tick_increment) =
                    calculate_tick_spacing(value_span, canvas_span, n_ticks);

                let grid_stroke = Stroke::default()
                    .with_width(1.0)
                    .with_color(theme.grid_color);

                // Draw ticks in both positive and negative directions from 0.0
                for direction in [-1, 1].iter() {
                    // Determine starting value and canvas position
                    let (mut value, mut canvas_pos) =
                        if value_range.0 <= 0.0 && value_range.1 >= 0.0 {
                            // If 0.0 is within the range, start at 0.0
                            let (v, cp) = (
                                0.0,
                                if is_x_axis {
                                    (0.0 - value_range.0) / value_span * canvas_span + axis_start.x
                                } else {
                                    axis_start.y - (0.0 - value_range.0) / value_span * canvas_span
                                },
                            );

                            // Draw an axis line for 0.0
                            let (zero_axis_start, zero_axis_end) = if is_x_axis {
                                (
                                    Point::new(cp, axis_start.y),
                                    Point::new(cp, axis_start.y - axes_size.height),
                                )
                            } else {
                                (
                                    Point::new(axis_start.x, cp),
                                    Point::new(axis_start.x + axes_size.width, cp),
                                )
                            };

                            let zero_axis = Path::line(zero_axis_start, zero_axis_end);

                            frame.stroke(
                                &zero_axis,
                                Stroke::default()
                                    .with_width(line_width)
                                    .with_color(theme.axis_color),
                            );
                            (v, cp)
                        } else {
                            // Otherwise, start at the range minimum or maximum
                            (
                                value_range.0 * *direction as f32,
                                if is_x_axis {
                                    axis_start.x
                                } else {
                                    axis_start.y
                                },
                            )
                        };

                    // Draw ticks, labels, and grid lines
                    while (is_x_axis && canvas_pos <= axis_end.x && canvas_pos >= axis_start.x)
                        || (!is_x_axis && canvas_pos >= axis_end.y && canvas_pos <= axis_start.y)
                    {
                        let tick_point = if is_x_axis {
                            Point::new(canvas_pos, axis_start.y)
                        } else {
                            Point::new(axis_start.x, canvas_pos)
                        };

                        // Draw grid lines
                        let (grid_start, grid_end) = if is_x_axis {
                            (
                                Point::new(tick_point.x, axis_top_left.y),
                                Point::new(tick_point.x, axis_bottom_left.y),
                            )
                        } else {
                            (
                                Point::new(axis_top_left.x, tick_point.y),
                                Point::new(axis_top_right.x, tick_point.y),
                            )
                        };

                        let grid_line = Path::line(grid_start, grid_end);
                        frame.stroke(&grid_line, grid_stroke.clone());

                        // Draw tick marks
                        // let tick_path = if is_x_axis {
                        //     Path::line(
                        //         tick_point + Vector::new(0.0, tick_length / 2.0),
                        //         tick_point + Vector::new(0.0, -tick_length / 2.0),
                        //     )
                        // } else {
                        //     Path::line(
                        //         tick_point + Vector::new(-tick_length / 2.0, 0.0),
                        //         tick_point + Vector::new(tick_length / 2.0, 0.0),
                        //     )
                        // };

                        // frame.stroke(&tick_path, Stroke::default().with_width(line_width));

                        // Automatically decide between fixed-point and scientific notation
                        let label = if value.abs() < 0.01 || value.abs() > 1000.0 {
                            format!("{:e}", value) // use scientific notation
                        } else {
                            format!("{:.2}", value) // use fixed precision
                        };

                        let text_center = if is_x_axis {
                            tick_point + Vector::new(0.0, tick_text_spacing)
                        } else {
                            tick_point + Vector::new(-tick_text_spacing, 0.0)
                        };

                        let text = Text {
                            content: label,
                            color: theme.text_color,
                            horizontal_alignment: Horizontal::Center,
                            position: text_center,
                            vertical_alignment: Vertical::Center,
                            ..Text::default()
                        };

                        frame.fill_text(text);

                        // Increment counters
                        if is_x_axis {
                            canvas_pos += tick_spacing * *direction as f32;
                        } else {
                            canvas_pos -= tick_spacing * *direction as f32;
                        }
                        value += tick_increment * *direction as f32;
                    }
                }
            };

        draw_ticks_labels_and_grids(
            frame,
            theme,
            axis_bottom_left,
            axis_bottom_right,
            axes_size,
            *xlim,
            axis_bottom_right.x - axis_bottom_left.x,
            self.n_ticks,
            true,
            self.tick_length,
            self.tick_text_spacing,
            self.line_width,
        );

        draw_ticks_labels_and_grids(
            frame,
            theme,
            axis_bottom_left,
            axis_top_left,
            axes_size,
            *ylim,
            axis_bottom_left.y - axis_top_left.y,
            self.n_ticks,
            false,
            self.tick_length,
            self.tick_text_spacing,
            self.line_width,
        );
    }
}

fn calculate_tick_spacing(value_span: f32, canvas_span: f32, n_ticks: u32) -> (f32, f32) {
    // Ensure n_ticks is at least 2 to avoid division by zero
    let n_ticks = n_ticks.max(2);

    // Calculate initial tick increment
    let raw_tick_increment = value_span / n_ticks as f32;

    // Find the nearest power of 10 for the initial tick increment
    let base_10_exponent = raw_tick_increment.log10().floor();
    let mut tick_increment = 10.0f32.powf(base_10_exponent);

    // Scale the tick increment to fit the desired number of ticks
    let mut ratio = 1.0;
    while value_span / (tick_increment * ratio) > n_ticks as f32 {
        ratio *= 2.0;
    }

    // Adjust ratio down if it overshoots the number of ticks
    while value_span / (tick_increment * ratio / 2.0) <= n_ticks as f32 {
        ratio /= 2.0;
    }

    tick_increment *= ratio;

    // Calculate the spacing on the canvas
    let tick_spacing = canvas_span * tick_increment / value_span;

    (tick_spacing, tick_increment)
}
