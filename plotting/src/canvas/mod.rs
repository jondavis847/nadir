use crate::{
    application::Message,
    theme::{PlotTheme, PlotThemes},
    SeriesMap,
};
use axes::Axes;
use iced::{
    advanced::{graphics::text::cosmic_text::Scroll, mouse},
    event::Status,
    mouse::{Cursor, ScrollDelta},
    widget::canvas::{Cache, Event, Geometry, Program},
    Point, Rectangle, Renderer, Theme,
};

mod axes;
mod axis;
mod line;

#[derive(Debug)]
pub struct CanvasState {
    theme: PlotTheme,
}

impl Default for CanvasState {
    fn default() -> Self {
        let theme = PlotThemes::Dark.palette();
        Self { theme }
    }
}

#[derive(Debug, Default)]
pub struct PlotCanvas {
    cache: Cache,
    axes: Vec<Axes>,
}

impl PlotCanvas {
    pub fn new(series_map: &SeriesMap) -> Self {
        let mut canvas = Self::default();

        for series in &series_map.map {
            // Check if an Axes with the desired location already exists
            if let Some(axes) = canvas
                .axes
                .iter_mut()
                .find(|axes| axes.location == series.axes)
            {
                // If found, add the line to the existing Axes
                axes.add_line(series);
            } else {
                // If not found, create a new Axes with the specified location
                let mut new_axes = Axes::new(series.axes);
                new_axes.add_line(series);
                canvas.axes.push(new_axes);
            }
        }
        canvas
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
            axes.xlim.0 += -width * SPEED * delta;
            axes.xlim.1 += width * SPEED * delta;
            axes.ylim.0 += -height * SPEED * delta;
            axes.ylim.1 += height * SPEED * delta;
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
                axes.draw(frame, &state.theme);
            }

            // // lines
            // for (i, series) in &self.series.map {
            //     if series.points.len() > 1 {
            //         let mut builder = Builder::new();
            //         builder.move_to(self.series.get_relative_point(&series.points[0], &frame));
            //         for point in &series.points {
            //             let point = self.series.get_relative_point(&point, &frame);
            //             builder.line_to(point);
            //             builder.move_to(point);
            //         }
            //         let path = builder.build();
            //         frame.stroke(
            //             &path,
            //             Stroke {
            //                 width: 2.0,
            //                 style: Style::Solid(state.theme.line_colors[*i as usize]),
            //                 ..Stroke::default()
            //             },
            //         )
            //     };
            // }
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
                _ => (Status::Captured, None),
            },
            _ => (Status::Ignored, None),
        }
    }
}
