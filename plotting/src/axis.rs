use super::theme::PlotTheme;
use iced::{
    Padding, Point, Rectangle, Size, Vector,
    alignment::{Horizontal, Vertical},
    widget::canvas::{Frame, Path, Stroke, Text, path::Builder},
};

#[derive(Debug, Default, Clone)]
struct Corners {
    top_left: Point,
    top_right: Point,
    bottom_left: Point,
    bottom_right: Point,
}

#[derive(Debug, Clone)]
pub struct Axis {
    padding: Padding,
    border_width: f32,
    pub bounds: Rectangle,
    n_ticks: u32,
    tick_length: f32,
    tick_text_spacing: f32,
    x_label: Option<String>,
    y_label: Option<String>,
    corners: Corners,
}

impl Default for Axis {
    fn default() -> Self {
        let bounds = Rectangle::default();
        Self {
            padding: Padding { left: 50.0, right: 30.0, top: 30.0, bottom: 40.0 },
            border_width: 1.0,
            bounds,
            n_ticks: 5,
            tick_length: 10.0,
            tick_text_spacing: 20.0,
            y_label: None,
            x_label: None,
            corners: Corners::default(), // updated in update_bounds
        }
    }
}

impl Axis {
    pub fn draw_border(&self, frame: &mut Frame, theme: &PlotTheme) {
        let mut border_path = Builder::new();
        border_path.rectangle(
            self.bounds
                .position(),
            self.bounds
                .size(),
        );

        // Draw border lines
        let border_stroke = Stroke::default()
            .with_width(self.border_width)
            .with_color(theme.axis_border);

        frame.stroke(
            &border_path.build(),
            border_stroke,
        );
    }

    pub fn draw_grid(
        &self,
        frame: &mut Frame,
        theme: &PlotTheme,
        xlim: &(f32, f32),
        ylim: &(f32, f32),
    ) {
        // Draw axis background
        frame.fill_rectangle(
            self.corners
                .top_left,
            self.bounds
                .size(),
            theme.axis_background,
        );

        // Draw horizontal grid lines and y-axis ticks
        self.draw_axis_grid(
            frame, theme, ylim, false, // y-axis
        );

        // Draw vertical grid lines and x-axis ticks
        self.draw_axis_grid(
            frame, theme, xlim, true, // x-axis
        );

        // Draw x-axis label
        self.draw_x_label(frame, theme);

        // Draw y-axis label
        self.draw_y_label(frame, theme);
    }

    // Helper method to draw grid lines and tick marks for an axis
    fn draw_axis_grid(
        &self,
        frame: &mut Frame,
        theme: &PlotTheme,
        value_range: &(f32, f32),
        is_x_axis: bool,
    ) {
        let (axis_start, axis_end) = if is_x_axis {
            (
                self.corners
                    .bottom_left,
                self.corners
                    .bottom_right,
            )
        } else {
            (
                self.corners
                    .bottom_left,
                self.corners
                    .top_left,
            )
        };

        let canvas_span = if is_x_axis {
            axis_end.x - axis_start.x
        } else {
            axis_start.y - axis_end.y
        };

        let value_span = value_range.1 - value_range.0;
        let (_, tick_increment) = calculate_tick_spacing(
            value_span,
            canvas_span,
            self.n_ticks,
        );

        let ticks = generate_ticks(
            value_range.0,
            value_range.1,
            tick_increment,
        );
        let grid_stroke = Stroke::default()
            .with_width(1.0)
            .with_color(theme.grid_color);

        for &value in &ticks {
            // Calculate position on canvas
            let tick_point = self.value_to_canvas_point(
                value,
                is_x_axis,
                value_range,
                canvas_span,
                axis_start,
            );

            // Draw grid line
            let grid_line = self.create_grid_line(tick_point, is_x_axis);
            frame.stroke(
                &grid_line,
                grid_stroke.clone(),
            );

            // Draw tick label
            let text = self.create_tick_label(
                value, tick_point, is_x_axis, theme,
            );
            frame.fill_text(text);
        }
    }

    // Helper to convert a value to canvas coordinates
    fn value_to_canvas_point(
        &self,
        value: f32,
        is_x_axis: bool,
        value_range: &(f32, f32),
        canvas_span: f32,
        axis_start: Point,
    ) -> Point {
        let value_span = value_range.1 - value_range.0;

        if is_x_axis {
            let x = (value - value_range.0) / value_span * canvas_span + axis_start.x;
            Point::new(x, axis_start.y)
        } else {
            let y = axis_start.y - (value - value_range.0) / value_span * canvas_span;
            Point::new(axis_start.x, y)
        }
    }

    // Create a grid line path based on tick position
    fn create_grid_line(&self, tick_point: Point, is_x_axis: bool) -> Path {
        let (start, end) = if is_x_axis {
            (
                Point::new(
                    tick_point.x,
                    self.corners
                        .top_left
                        .y,
                ),
                Point::new(
                    tick_point.x,
                    self.corners
                        .bottom_left
                        .y,
                ),
            )
        } else {
            (
                Point::new(
                    self.corners
                        .top_left
                        .x,
                    tick_point.y,
                ),
                Point::new(
                    self.corners
                        .top_right
                        .x,
                    tick_point.y,
                ),
            )
        };

        Path::line(start, end)
    }

    // Create formatted tick label
    fn create_tick_label(
        &self,
        value: f32,
        tick_point: Point,
        is_x_axis: bool,
        theme: &PlotTheme,
    ) -> Text {
        let label = Self::format_tick_value(value);

        let text_position = if is_x_axis {
            tick_point + Vector::new(0.0, self.tick_text_spacing)
        } else {
            tick_point + Vector::new(-self.tick_text_spacing, 0.0)
        };

        Text {
            content: label,
            color: theme.text_color,
            horizontal_alignment: Horizontal::Center,
            position: text_position,
            vertical_alignment: Vertical::Center,
            size: (14.0).into(),
            ..Text::default()
        }
    }

    // Helper function to format tick values nicely
    fn format_tick_value(value: f32) -> String {
        if value == 0.0 {
            "0".to_string()
        } else if value.abs() < 0.01 || value.abs() > 1000.0 {
            format!("{:.2e}", value)
        } else {
            let formatted = format!("{:.2}", value);
            if formatted.ends_with(".00") {
                formatted
                    .trim_end_matches(".00")
                    .to_string()
            } else {
                formatted
            }
        }
    }

    // Draw the x-axis label
    fn draw_x_label(&self, frame: &mut Frame, theme: &PlotTheme) {
        if let Some(x_label) = &self.x_label {
            let center_x = (self
                .corners
                .bottom_left
                .x
                + self
                    .corners
                    .bottom_right
                    .x)
                / 2.0;
            let position = Point::new(
                center_x,
                self.corners
                    .bottom_left
                    .y
                    + self.tick_text_spacing * 2.0,
            );

            let text = Text {
                content: x_label.clone(),
                color: theme.text_color,
                horizontal_alignment: Horizontal::Center,
                vertical_alignment: Vertical::Top,
                position,
                size: (16.0).into(),
                ..Text::default()
            };

            frame.fill_text(text);
        }
    }

    // Draw the y-axis label
    fn draw_y_label(&self, frame: &mut Frame, theme: &PlotTheme) {
        if let Some(y_label) = &self.y_label {
            let center_y = (self
                .corners
                .bottom_left
                .y
                + self
                    .corners
                    .top_left
                    .y)
                / 2.0;
            let position = Point::new(
                self.corners
                    .top_left
                    .x
                    - self.tick_text_spacing * 4.0,
                center_y,
            );

            frame.with_save(|frame| {
                frame.translate(position - Point::ORIGIN);
                frame.rotate(std::f32::consts::FRAC_PI_2);

                let text = Text {
                    content: y_label.clone(),
                    color: theme.text_color,
                    horizontal_alignment: Horizontal::Center,
                    vertical_alignment: Vertical::Center,
                    position,
                    size: (16.0).into(),
                    ..Text::default()
                };

                frame.fill_text(text);
            });
        }
    }

    pub fn set_x_label(&mut self, label: String) {
        self.x_label = Some(label);
        self.update_corners();
    }
    pub fn set_y_label(&mut self, label: String) {
        self.y_label = Some(label);
        self.update_corners();
    }

    pub fn update_bounds(&mut self, axes_bounds: &Rectangle) {
        // dynamic sizing based on labels
        let bottom = if self
            .x_label
            .is_some()
        {
            axes_bounds.y + axes_bounds.height
                - 2.0
                    * self
                        .padding
                        .bottom
        } else {
            axes_bounds.y + axes_bounds.height
                - self
                    .padding
                    .bottom
        };

        // dynamic sizing based on labels
        let left = if self
            .y_label
            .is_some()
        {
            2.0 * self
                .padding
                .left
        } else {
            self.padding
                .left
        };

        self.bounds
            .x = axes_bounds.x + left;
        self.bounds
            .y = axes_bounds.y
            + self
                .padding
                .top;
        self.bounds
            .width = axes_bounds.width
            - left
            - self
                .padding
                .right;
        self.bounds
            .height = bottom
            - self
                .bounds
                .y;
        self.update_corners();
    }

    pub fn update_corners(&mut self) {
        let left = self
            .bounds
            .x;
        let bottom = self
            .bounds
            .y
            + self
                .bounds
                .height;
        let top = self
            .bounds
            .y;
        let right = self
            .bounds
            .x
            + self
                .bounds
                .width;

        self.corners
            .bottom_left = Point::new(left, bottom);
        self.corners
            .bottom_right = Point::new(right, bottom);
        self.corners
            .top_left = Point::new(left, top);
        self.corners
            .top_right = Point::new(right, top);
    }
}

fn calculate_tick_spacing(value_span: f32, canvas_span: f32, n_ticks: u32) -> (f32, f32) {
    let n_ticks = n_ticks.max(2);
    let raw_tick_increment = value_span / n_ticks as f32;
    let exponent = raw_tick_increment
        .log10()
        .floor();
    let base = 10f32.powf(exponent);
    let multiples = [1.0, 2.0, 5.0, 10.0];

    let mut tick_increment = base;
    for &m in &multiples {
        if base * m >= raw_tick_increment {
            tick_increment = base * m;
            break;
        }
    }

    let tick_spacing = canvas_span * tick_increment / value_span;
    (tick_spacing, tick_increment)
}

fn generate_ticks(start: f32, end: f32, tick_increment: f32) -> Vec<f32> {
    let mut ticks = Vec::new();
    let mut tick = (start / tick_increment).ceil() * tick_increment;
    while tick <= end {
        ticks.push(tick);
        tick += tick_increment;
    }
    ticks
}
