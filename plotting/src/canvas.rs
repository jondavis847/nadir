use crate::{
    SeriesMap,
    figure::{Figure, Message},
    theme::{PlotTheme, PlotThemes},
};

use iced::{
    Point, Rectangle, Renderer, Size, Theme, Vector,
    advanced::mouse,
    event::Status,
    mouse::{Button, Cursor, ScrollDelta},
    widget::canvas::{Cache, Event, Geometry, Program},
};

#[derive(Debug)]
pub struct CanvasState {
    theme: PlotTheme,
    //text_size: f32,
}

impl Default for CanvasState {
    fn default() -> Self {
        let theme = PlotThemes::Dark.palette();

        Self {
            theme,
            //text_size: 0.1,
        }
    }
}

#[derive(Debug, Default)]
pub struct PlotCanvas {
    pub cache: Cache,
    canvas_bounds: Rectangle,
    figure: Figure,
}

impl PlotCanvas {
    pub fn new(window_size: Size) -> Self {
        let mut canvas = Self::default();
        canvas.canvas_bounds = Rectangle::new(Point::ORIGIN, window_size);

        // get the number of axes columns and rows for the layout
        let mut nrows = 0;
        let mut ncols = 0;
        for entry in series_map {
            if entry.axes.0 > nrows {
                nrows = entry.axes.0;
            }
            if entry.axes.1 > ncols {
                ncols = entry.axes.1;
            }
        }
        let row_height = canvas.canvas_bounds.height / (nrows as f32 + 1.0);
        let col_width = canvas.canvas_bounds.width / (ncols as f32 + 1.0);
        let axes_size = Size::new(col_width, row_height);

        for entry in series_map {
            let (row, col) = entry.axes;
            let top = row_height * row as f32;
            let left = col_width * col as f32;
            let top_left = Point::new(left, top);
            let axes_bounds = Rectangle::new(top_left, axes_size);
            let mut axes = Axes::new(entry.axes, axes_bounds);

            for series in &entry.map {
                axes.add_line(series, None);
            }
            canvas.axes.push(axes);
        }
        canvas
    }

    pub fn mouse_left_clicked(&mut self, point: Point) {
        for axes in &mut self.axes {
            let axis_bounds = Rectangle::new(
                Point::new(
                    axes.bounds.x + axes.axis.x_padding,
                    axes.bounds.y + axes.axis.y_padding,
                ),
                Size::new(
                    axes.bounds.width - axes.axis.x_padding * 2.0,
                    axes.bounds.height - axes.axis.y_padding * 2.0,
                ),
            );
            if axis_bounds.contains(point) {
                axes.click_start = Some(point);
            }
        }
    }

    pub fn mouse_left_released(&mut self, point: Point) {
        for axes in &mut self.axes {
            let axis_bounds = Rectangle::new(
                Point::new(
                    axes.bounds.x + axes.axis.x_padding,
                    axes.bounds.y + axes.axis.y_padding,
                ),
                Size::new(
                    axes.bounds.width - axes.axis.x_padding * 2.0,
                    axes.bounds.height - axes.axis.y_padding * 2.0,
                ),
            );
            if axis_bounds.contains(point) {
                if let Some(start_point) = axes.click_start {
                    // determine the value at the start point
                    let sx_start = (start_point.x - axis_bounds.x) / axis_bounds.width;
                    let new_xlim_0 = sx_start * (axes.xlim.1 - axes.xlim.0) + axes.xlim.0;

                    let sx_end = (point.x - axis_bounds.x) / axis_bounds.width;
                    let new_xlim_1 = sx_end * (axes.xlim.1 - axes.xlim.0) + axes.xlim.0;
                    axes.xlim = if new_xlim_1 > new_xlim_0 {
                        (new_xlim_0, new_xlim_1)
                    } else {
                        (new_xlim_1, new_xlim_0)
                    };

                    // remeber point.y start from the top, but ylim is from bottom
                    let sy_start =
                        ((axis_bounds.y + axis_bounds.height) - start_point.y) / axis_bounds.height;
                    let new_ylim_1 = sy_start * (axes.ylim.1 - axes.ylim.0) + axes.ylim.0;

                    let sy_end =
                        ((axis_bounds.y + axis_bounds.height) - point.y) / axis_bounds.height;
                    let new_ylim_0 = sy_end * (axes.ylim.1 - axes.ylim.0) + axes.ylim.0;
                    axes.ylim = if new_ylim_1 > new_ylim_0 {
                        (new_ylim_0, new_ylim_1)
                    } else {
                        (new_ylim_1, new_ylim_0)
                    };
                }
            }
            axes.click_start = None;
        }
        self.cache.clear();
    }

    pub fn window_resized(&mut self, window_size: Size) {
        let x_scale = window_size.width / self.canvas_bounds.width;
        let y_scale = window_size.height / self.canvas_bounds.height;

        for axes in &mut self.axes {
            axes.bounds.x *= x_scale;
            axes.bounds.y *= y_scale;
            axes.bounds.width *= x_scale;
            axes.bounds.height *= y_scale;
        }
        self.canvas_bounds.width = window_size.width;
        self.canvas_bounds.height = window_size.height;
        self.cache.clear();
    }

    pub fn wheel_scrolled(&mut self, _point: Point, delta: ScrollDelta) {
        const SPEED: f32 = 0.1;
        let delta = match delta {
            ScrollDelta::Lines { x: _, y } => y,
            ScrollDelta::Pixels { x: _, y } => y,
        };

        for axes in &mut self.axes {
            let width = axes.xlim.1 - axes.xlim.0;
            let height = axes.ylim.1 - axes.ylim.0;
            axes.xlim.0 += width * SPEED * delta;
            axes.xlim.1 += -width * SPEED * delta;
            axes.ylim.0 += height * SPEED * delta;
            axes.ylim.1 += -height * SPEED * delta;
        }

        self.cache.clear();
    }
}

impl Program<Message> for PlotCanvas {
    type State = CanvasState;

    fn draw(
        &self,
        state: &Self::State,
        renderer: &Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        _cursor: Cursor,
    ) -> Vec<Geometry> {
        let content = self.cache.draw(renderer, bounds.size(), |frame| {
            // background
            frame.fill_rectangle(Point::ORIGIN, frame.size(), state.theme.dark_background);

            // axes
            for axes in &self.axes {
                frame.with_save(|frame| {
                    // move to axis location
                    frame.translate(axes.bounds.position() - Point::ORIGIN);
                    // scale down to axes size
                    let x_scale = axes.bounds.width / frame.width();
                    let y_scale = axes.bounds.height / frame.height();
                    frame.scale_nonuniform(Vector::new(x_scale, y_scale));
                    axes.draw(frame, &state.theme);
                });
            }
        });
        vec![content]
    }

    fn update(
        &self,
        _state: &mut Self::State,
        event: Event,
        _bounds: Rectangle,
        cursor: Cursor,
    ) -> (Status, Option<Message>) {
        match event {
            Event::Mouse(event) => match event {
                mouse::Event::WheelScrolled { delta } => {
                    if let Some(point) = cursor.position() {
                        (Status::Captured, Some(Message::WheelScrolled(point, delta)))
                    } else {
                        (Status::Captured, None)
                    }
                }
                mouse::Event::ButtonPressed(button) => match button {
                    Button::Left => {
                        if let Some(point) = cursor.position() {
                            (Status::Captured, Some(Message::MouseLeftPressed(point)))
                        } else {
                            (Status::Captured, None)
                        }
                    }
                    _ => (Status::Ignored, None),
                },
                mouse::Event::ButtonReleased(button) => match button {
                    Button::Left => {
                        if let Some(point) = cursor.position() {
                            (Status::Captured, Some(Message::MouseLeftReleased(point)))
                        } else {
                            (Status::Captured, None)
                        }
                    }
                    _ => (Status::Captured, None),
                },
                // mouse::Event::CursorMoved { position } => {
                //     (Status::Captured, Some(Message::CursorMoved(position)))
                // }
                _ => (Status::Captured, None),
            },
            _ => (Status::Ignored, None),
        }
    }
}
