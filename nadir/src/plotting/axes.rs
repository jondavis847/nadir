use std::sync::{Arc, Mutex};

use super::legend::Legend;
use super::theme::PlotTheme;
use super::{
    // legend::{Legend, LegendEntry},
    line::Line,
};
use iced::Padding;
use iced::window::Id;
use iced::{Point, Rectangle, Size, widget::canvas::Frame};

use super::axis::Axis;

#[derive(Debug, Clone)]
pub struct Axes {
    pub axis: Axis,
    pub bounds: Rectangle,
    pub figure_id: Option<Id>,
    pub legend: Legend,
    pub lines: Vec<Arc<Mutex<Line>>>,
    pub location: (usize, usize),
    pub padding: Padding,
    pub xlim: (f32, f32),
    pub ylim: (f32, f32),
    click_start: Option<Point>,
}

impl Axes {
    pub fn add_line(&mut self, line: Arc<Mutex<Line>>) {
        self.lines.push(line);

        //update xlim and ylim based on line data
        let (xlim, ylim) = get_global_lims(&self.lines);
        self.xlim = xlim;
        self.ylim = ylim;

        self.axis
            .update_bounds(&self.bounds, &self.xlim, &self.ylim);

        for line in &self.lines {
            let line = &mut *line.lock().unwrap();
            line.update_canvas_position(&self.axis.bounds, &self.xlim, &self.ylim);
        }
    }

    pub fn draw(&self, frame: &mut Frame, theme: &PlotTheme) {
        self.draw_background(frame, theme);
        self.axis.draw_grid(frame, theme, &self.xlim, &self.ylim);

        for (i, line) in self.lines.iter().enumerate() {
            let line = &*line.lock().unwrap();
            line.draw(frame, theme, i, &self.axis.bounds);
        }
        self.axis.draw_border(frame, theme);
    }

    fn draw_background(&self, frame: &mut Frame, theme: &PlotTheme) {
        frame.fill_rectangle(Point::ORIGIN, frame.size(), theme.axes_background)
    }

    pub fn get_figure_id(&self) -> Option<Id> {
        self.figure_id
    }

    pub fn mouse_left_clicked(&mut self, point: Point) {
        if self.axis.bounds.contains(point) {
            self.click_start = Some(point);
        }
    }

    pub fn mouse_left_released(&mut self, point: Point) {
        if self.axis.bounds.contains(point) {
            if let Some(start_point) = self.click_start {
                // determine the value at the start point
                let sx_start = (start_point.x - self.axis.bounds.x) / self.axis.bounds.width;
                let new_xlim_0 = sx_start * (self.xlim.1 - self.xlim.0) + self.xlim.0;

                let sx_end = (point.x - self.axis.bounds.x) / self.axis.bounds.width;
                let new_xlim_1 = sx_end * (self.xlim.1 - self.xlim.0) + self.xlim.0;
                self.xlim = if new_xlim_1 > new_xlim_0 {
                    (new_xlim_0, new_xlim_1)
                } else {
                    (new_xlim_1, new_xlim_0)
                };

                // remeber point.y start from the top, but ylim is from bottom
                let sy_start = ((self.axis.bounds.y + self.axis.bounds.height) - start_point.y)
                    / self.axis.bounds.height;
                let new_ylim_1 = sy_start * (self.ylim.1 - self.ylim.0) + self.ylim.0;

                let sy_end = ((self.axis.bounds.y + self.axis.bounds.height) - point.y)
                    / self.axis.bounds.height;
                let new_ylim_0 = sy_end * (self.ylim.1 - self.ylim.0) + self.ylim.0;
                self.ylim = if new_ylim_1 > new_ylim_0 {
                    (new_ylim_0, new_ylim_1)
                } else {
                    (new_ylim_1, new_ylim_0)
                };
            }
        }
        self.click_start = None;
    }

    pub fn new(location: (usize, usize), figure_id: Option<Id>) -> Self {
        Self {
            figure_id,
            axis: Axis::default(),
            xlim: (0.0, 1.0),
            ylim: (-1.0, 1.0),
            legend: Legend {},
            lines: Vec::new(),
            location,
            click_start: None,
            bounds: Rectangle::default(), // to be updated later
            padding: Padding {
                left: 0.0,
                right: 0.0,
                top: 0.0,
                bottom: 0.0,
            },
        }
    }

    pub fn set_figure_id(&mut self, id: Id) {
        self.figure_id = Some(id);
    }

    pub fn update_bounds(&mut self, fig_size: Size, nrows: usize, ncols: usize) {
        self.bounds.height =
            fig_size.height / nrows as f32 - self.padding.top - self.padding.bottom;
        self.bounds.width = fig_size.width / ncols as f32 - self.padding.left - self.padding.right;
        self.bounds.x = self.bounds.width * self.location.1 as f32 + self.padding.left;
        self.bounds.y = self.bounds.height * self.location.0 as f32 + self.padding.top;

        self.axis
            .update_bounds(&self.bounds, &self.xlim, &self.ylim);
        for line in &self.lines {
            let line = &mut *line.lock().unwrap();
            line.update_canvas_position(&self.axis.bounds, &self.xlim, &self.ylim);
        }
    }
}

fn get_global_lims(lines: &Vec<Arc<Mutex<Line>>>) -> ((f32, f32), (f32, f32)) {
    let mut xmin = if let Some(xmin) = lines
        .iter()
        .map(|line| {
            let line = &*line.lock().unwrap();
            line.data.xmin
        })
        .min_by(|a, b| a.total_cmp(b))
    {
        xmin
    } else {
        0.0
    };

    let mut xmax = if let Some(xmax) = lines
        .iter()
        .map(|line| {
            let line = &*line.lock().unwrap();
            line.data.xmax
        })
        .max_by(|a, b| a.total_cmp(b))
    {
        xmax
    } else {
        0.0
    };

    let mut ymin = if let Some(ymin) = lines
        .iter()
        .map(|line| {
            let line = &*line.lock().unwrap();
            line.data.ymin
        })
        .min_by(|a, b| a.total_cmp(b))
    {
        ymin
    } else {
        0.0
    };

    let mut ymax = if let Some(ymax) = lines
        .iter()
        .map(|line| {
            let line = &*line.lock().unwrap();
            line.data.ymax
        })
        .max_by(|a, b| a.total_cmp(b))
    {
        ymax
    } else {
        0.0
    };

    // logic if the values are 0
    let delta_x = xmax - xmin;
    if delta_x.abs() < f64::EPSILON {
        xmax = xmax + 0.5;
        xmin = xmin - 0.5;
    }

    let delta_y = ymax - ymin;
    if (delta_y).abs() < f64::EPSILON {
        ymax = ymax + 0.5;
        ymin = ymin - 0.5;
    } else {
        // add padding so curve isnt exactly at top and bottom of axis
        let padding = delta_y * 0.1;
        ymax += padding;
        ymin -= padding;
    }

    ((xmin as f32, xmax as f32), (ymin as f32, ymax as f32))
}
