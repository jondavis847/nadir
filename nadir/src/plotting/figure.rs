use std::collections::HashMap;

use color::Color;
use iced::{Point, Rectangle, Size, Vector, mouse::ScrollDelta, widget::canvas::Frame};

use super::{axes::Axes, note_bar::NoteBar, theme::PlotTheme, title_bar::TitleBar};

#[derive(Debug, Clone)]
pub struct Figure {
    size: Size,
    axes: Vec<Axes>,
    background_color: Color,
    title_bar: Option<TitleBar>,
    note_bar: Option<NoteBar>,
    nrows: usize,
    ncols: usize,
}

impl Figure {
    pub fn add_axes(&mut self, position: (usize, usize)) {
        // update # of rows and cols
        if position.0 > self.nrows {
            self.nrows = position.0
        }
        if position.1 > self.ncols {
            self.ncols = position.1
        }

        let mut axes = Axes::new(position);
        axes.update_bounds(self.size, self.nrows, self.ncols);
        self.axes.push(axes);
    }

    pub fn draw(&self, frame: &mut Frame, theme: &PlotTheme) {
        // axes
        for axes in &self.axes {
            frame.with_save(|frame| {
                frame.with_clip(axes.bounds, |frame| {
                    axes.draw(frame, theme);
                });
            });
        }
    }
    pub fn new() -> Self {
        let mut fig = Self {
            axes: vec![],
            size: Size::new(720.0, 480.0),
            background_color: Color::new(0.9, 0.9, 0.9, 1.0),
            title_bar: None,
            note_bar: None,
            nrows: 1,
            ncols: 1,
        };

        fig.add_axes((0, 0));

        fig
    }

    pub fn mouse_left_clicked(&mut self, point: Point) {
        for axes in &mut self.axes {
            axes.mouse_left_clicked(point);
        }
    }

    pub fn mouse_left_released(&mut self, point: Point) {
        for axes in &mut self.axes {
            axes.mouse_left_released(point);
        }
    }

    pub fn set_background_color(&mut self, color: Color) {
        self.background_color = color;
    }

    pub fn set_height(&mut self, height: f32) {
        self.size.height = height;
    }

    pub fn set_width(&mut self, width: f32) {
        self.size.width = width;
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
    }

    pub fn window_resized(&mut self, window_size: Size) {
        self.size.width = window_size.width;
        self.size.height = window_size.height;

        self.axes
            .iter_mut()
            .for_each(|axes| axes.update_bounds(self.size, self.nrows, self.ncols));
    }
}
