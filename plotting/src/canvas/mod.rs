use std::collections::HashMap;

use iced::{
    mouse::Cursor,
    widget::canvas::{path::Builder, Cache, Geometry, Path, Program, Stroke, Style},
    Color, Point, Rectangle, Renderer, Theme,
};

use crate::{Series, SeriesMap};

#[derive(Debug, Default)]
pub struct CanvasState {}

#[derive(Debug, Default)]
pub struct PlotCanvas {
    series: SeriesMap,
    state: CanvasState,
    cache: Cache,
}

impl PlotCanvas {
    pub fn new(series: SeriesMap) -> Self {
        Self {
            series,
            ..Default::default()
        }
    }
}

impl<Message> Program<Message> for PlotCanvas {
    type State = CanvasState;

    fn draw(
        &self,
        _state: &Self::State,
        renderer: &Renderer,
        theme: &Theme,
        bounds: Rectangle,
        _cursor: Cursor,
    ) -> Vec<Geometry> {
        let content = self.cache.draw(renderer, bounds.size(), |frame| {
            frame.fill_rectangle(Point::ORIGIN, frame.size(), theme.palette().background);

            for (_, series) in &self.series.map {
                if series.points.len() > 1 {
                    let mut builder = Builder::new();
                    builder.move_to(self.series.get_relative_point(&series.points[0], &frame));
                    for point in &series.points {
                        let point = self.series.get_relative_point(&point, &frame);
                        builder.line_to(point);
                        builder.move_to(point);
                    }
                    let path = builder.build();
                    frame.stroke(
                        &path,
                        Stroke {
                            width: 2.0,
                            style: Style::Solid(theme.palette().primary),
                            ..Stroke::default()
                        },
                    )
                };
            }
        });
        vec![content]
    }
}
