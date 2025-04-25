use iced::{Point, Size, mouse::ScrollDelta, widget::canvas::Frame, window::Id};

use super::{
    axes::{self, Axes},
    note_bar::NoteBar,
    theme::PlotTheme,
    title_bar::TitleBar,
};

#[derive(Debug, Clone)]
pub struct Figure {
    pub id: Option<Id>,
    size: Size,
    axes: Vec<Axes>,
    title_bar: Option<TitleBar>,
    note_bar: Option<NoteBar>,
    nrows: usize,
    ncols: usize,
    current_axes: usize,
}

impl Figure {
    pub fn add_axes(&mut self, row: usize, col: usize) {
        // update # of rows and cols
        // +1 because 0 based indexing
        if row + 1 > self.nrows {
            self.nrows = row + 1
        }
        if col + 1 > self.ncols {
            self.ncols = col + 1
        }

        let axes = Axes::new((row, col));
        self.axes.push(axes);
        for axes in &mut self.axes {
            axes.update_bounds(self.size, self.nrows, self.ncols);
        }
    }

    pub fn delete_axes(&mut self, i: usize) {
        if i >= self.axes.len() {
            return;
        }

        // update # of rows and cols
        self.axes.remove(i);
        self.nrows = 1;
        self.ncols = 1;
        for axes in &self.axes {
            if axes.location.0 > self.nrows {
                self.nrows = axes.location.0;
            }
            if axes.location.1 > self.ncols {
                self.ncols = axes.location.1;
            }
        }
        for axes in &mut self.axes {
            axes.update_bounds(self.size, self.nrows, self.ncols);
        }
    }

    pub fn draw(&self, frame: &mut Frame, theme: &PlotTheme) {
        // draw background
        // frame.with_save(|frame| {
        //     frame.fill_rectangle(Point::ORIGIN, frame.size(), Color::from_rgb(1.0, 0.0, 0.0)); // theme.figure_background);
        // });

        for axes in &self.axes {
            frame.with_clip(axes.bounds, |frame| {
                axes.draw(frame, theme);
            });
        }
    }

    pub fn new() -> Self {
        let mut fig = Self {
            id: None,
            axes: vec![],
            size: Size::new(720.0, 480.0),
            title_bar: None,
            note_bar: None,
            nrows: 1,
            ncols: 1,
            current_axes: 0,
        };

        fig.add_axes(0, 0);

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
