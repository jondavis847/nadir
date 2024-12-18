use crate::{
    application::Message,
    theme::{PlotTheme, PlotThemes},
    SeriesMap,
};
use axes::Axes;
use iced::{
    advanced::mouse,
    event::Status,
    mouse::{Cursor, ScrollDelta},
    widget::canvas::{Cache, Event, Geometry, Program},
    Point, Rectangle, Renderer, Size, Theme, Vector,
};

mod axes;
mod axis;
mod line;

#[derive(Debug)]
pub struct CanvasState {
    theme: PlotTheme,
    text_size: f32,
}

impl Default for CanvasState {
    fn default() -> Self {
        let theme = PlotThemes::Dark.palette();

        Self {
            theme,
            text_size: 0.1,
        }
    }
}

#[derive(Debug, Default)]
pub struct PlotCanvas {
    pub cache: Cache,
    axes: Vec<Axes>,
    canvas_bounds: Rectangle,
}

impl PlotCanvas {
    pub fn new(series_map: &Vec<SeriesMap>, window_size: Size) -> Self {
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

    pub fn window_resized(&mut self, window_size: Size) {
        let x_scale = window_size.width / self.canvas_bounds.width;
        let y_scale = window_size.height / self.canvas_bounds.height;

        for axes in &mut self.axes {
            axes.bounds.x *= x_scale;
            axes.bounds.width *= x_scale;
            axes.bounds.y *= y_scale;
            axes.bounds.height *= y_scale;
            axes.axis.x_padding *= x_scale;
            axes.axis.y_padding *= y_scale;
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
                // mouse::Event::ButtonPressed(button) => {
                //     match button {
                //         Button::Left =>
                //     }
                // }
                // mouse::Event::ButtonReleased(button) => {

                // }
                // mouse::Event::CursorMoved(position) => {

                // }
                _ => (Status::Captured, None),
            },
            _ => (Status::Ignored, None),
        }
    }
}
