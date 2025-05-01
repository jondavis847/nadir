use core::fmt;
use std::sync::{Arc, Mutex};

use iced::{Point, Size, mouse::ScrollDelta, widget::canvas::Frame, window::Id};

use super::{PlotErrors, axes::Axes, note_bar::NoteBar, theme::PlotTheme, title_bar::TitleBar};

#[derive(Clone)]
pub struct Figure {
    id: Option<Id>,
    size: Size,
    pub axes: Vec<Arc<Mutex<Axes>>>,
    title_bar: Option<TitleBar>,
    note_bar: Option<NoteBar>,
    nrows: usize,
    ncols: usize,
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

        let axes = Arc::new(Mutex::new(Axes::new((row, col), None)));
        self.axes.push(axes);
        for axes in &mut self.axes {
            let axes = &mut *axes.lock().unwrap();
            axes.update_bounds(self.size, self.nrows, self.ncols);
        }
    }

    pub fn cursor_moved(&mut self, point: Point) {
        for axes in &mut self.axes {
            let axes = &mut *axes.lock().unwrap();
            axes.cursor_moved(point);
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
            let axes = &*axes.lock().unwrap();
            if axes.location.0 > self.nrows {
                self.nrows = axes.location.0;
            }
            if axes.location.1 > self.ncols {
                self.ncols = axes.location.1;
            }
        }
        for axes in &mut self.axes {
            let axes = &mut *axes.lock().unwrap();
            axes.update_bounds(self.size, self.nrows, self.ncols);
        }
    }

    pub fn draw(&self, frame: &mut Frame, theme: &PlotTheme) {
        // draw background
        // frame.with_save(|frame| {
        //     frame.fill_rectangle(Point::ORIGIN, frame.size(), Color::from_rgb(1.0, 0.0, 0.0)); // theme.figure_background);
        // });

        for axes in &self.axes {
            let axes = &*axes.lock().unwrap();
            axes.draw(frame, theme);
        }
    }

    pub fn get_axes(&mut self, index: usize) -> Result<Arc<Mutex<Axes>>, PlotErrors> {
        if index > self.axes.len() - 1 {
            return Err(PlotErrors::AxesIndexOOB);
        }
        Ok(self.axes[index].clone())
    }

    pub fn get_id(&self) -> Option<Id> {
        self.id
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
        };

        fig.add_axes(0, 0);

        fig
    }

    pub fn mouse_left_clicked(&mut self, point: Point) {
        for axes in &mut self.axes {
            let axes = &mut *axes.lock().unwrap();
            axes.mouse_left_clicked(point);
        }
    }

    pub fn mouse_left_released(&mut self, point: Point) {
        for axes in &mut self.axes {
            let axes = &mut *axes.lock().unwrap();
            axes.mouse_left_released(point);
        }
    }

    pub fn mouse_middle_clicked(&mut self, point: Point) {
        for axes in &mut self.axes {
            let axes = &mut *axes.lock().unwrap();
            axes.mouse_middle_clicked(point);
        }
    }

    pub fn mouse_middle_released(&mut self, point: Point) {
        for axes in &mut self.axes {
            let axes = &mut *axes.lock().unwrap();
            axes.mouse_middle_released(point);
        }
    }

    pub fn set_height(&mut self, height: f32) {
        self.size.height = height;
    }

    pub fn set_width(&mut self, width: f32) {
        self.size.width = width;
    }

    pub fn set_window_id(&mut self, id: Id) {
        self.id = Some(id);
        for axes in &self.axes {
            let axes = &mut *axes.lock().unwrap();
            axes.set_figure_id(id);
        }
    }

    pub fn wheel_scrolled(&mut self, point: Point, delta: ScrollDelta) {
        const SPEED: f32 = 0.1;
        let delta = match delta {
            ScrollDelta::Lines { x: _, y } => y,
            ScrollDelta::Pixels { x: _, y } => y,
        };

        for axes in &self.axes {
            let axes = &mut *axes.lock().unwrap();
            if axes.axis.bounds.contains(point) {
                let width = axes.xlim.1 - axes.xlim.0;
                let height = axes.ylim.1 - axes.ylim.0;
                axes.xlim.0 += width * SPEED * delta;
                axes.xlim.1 += -width * SPEED * delta;
                axes.ylim.0 += height * SPEED * delta;
                axes.ylim.1 += -height * SPEED * delta;

                for line in &mut axes.lines {
                    let line = &mut *line.lock().unwrap();
                    line.update_canvas_position(&axes.axis.bounds, &axes.xlim, &axes.ylim);
                }
            }
        }
    }

    pub fn window_resized(&mut self, window_size: Size) {
        self.size.width = window_size.width;
        self.size.height = window_size.height;

        for axes in &self.axes {
            let axes = &mut *axes.lock().unwrap();
            axes.update_bounds(self.size, self.nrows, self.ncols);
        }
    }
}

impl fmt::Debug for Figure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "Figure")?;

        let id = match self.get_id() {
            Some(id) => id.to_string(),
            None => "None".to_string(),
        };
        writeln!(f, "id: {},", id)?;

        if self.axes.is_empty() {
            writeln!(f, "axes: [],")?;
        } else {
            writeln!(f, "axes: [")?;
            for (i, axes) in self.axes.iter().enumerate() {
                let axes = axes.lock().unwrap();
                writeln!(
                    f,
                    "     {} Axes({},{}),",
                    &i.to_string(),
                    axes.location.0,
                    axes.location.1
                )?;
            }
            writeln!(f, "   ],")?;
        }

        Ok(())
    }
}
