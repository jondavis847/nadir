use std::sync::{Arc, Mutex};

use super::legend::Legend;
use super::theme::PlotTheme;
use super::{
    // legend::{Legend, LegendEntry},
    line::Line,
};
use iced::Padding;
use iced::mouse::ScrollDelta;
use iced::window::Id;
use iced::{Point, Rectangle, Size, widget::canvas::Frame};

use super::axis::Axis;

#[derive(Debug, Clone)]
pub struct Axes {
    pub axis: Axis,
    pub bounds: Rectangle,
    pub figure_id: Option<Id>,
    pub legend: Option<Legend>,
    pub lines: Vec<Arc<Mutex<Line>>>,
    pub location: (usize, usize),
    pub padding: Padding,
    pub xlim: (f32, f32),
    pub ylim: (f32, f32),
    click_start: Option<Point>,
    is_panning: bool,
    target_zoom: f32,
    current_zoom: f32,
    zoom_speed: f32,
    zoom_center: Option<Point>,
    initial_xlim: (f32, f32),
    initial_ylim: (f32, f32),
}

impl Axes {
    pub fn add_line(&mut self, line: Arc<Mutex<Line>>) {
        self.lines.push(line);

        //update xlim and ylim based on line data
        let (xlim, ylim) = get_global_lims(&self.lines);
        self.xlim = xlim;
        self.ylim = ylim;

        self.initial_xlim = xlim;
        self.initial_ylim = ylim;

        self.axis.update_bounds(&self.bounds);

        for line in &self.lines {
            let line = &mut *line.lock().unwrap();
            line.update_canvas_position(&self.axis.bounds, &self.xlim, &self.ylim);
        }
        if let Some(legend) = &mut self.legend {
            legend.update(&self.axis.bounds, &self.lines);
        }
    }

    pub fn cursor_moved(&mut self, point: Point) {
        if self.is_panning {
            if let Some(start_point) = self.click_start {
                let dx = point.x - start_point.x;
                let dy = point.y - start_point.y;

                let xrange = self.xlim.1 - self.xlim.0;
                let yrange = self.ylim.1 - self.ylim.0;

                // fraction of canvas moved
                let frac_x = dx / self.axis.bounds.width;
                let frac_y = dy / self.axis.bounds.height;

                // convert to data space shift
                let data_dx = -frac_x * xrange;
                let data_dy = frac_y * yrange; // careful: Y axis is typically flipped

                // shift limits
                self.xlim.0 += data_dx;
                self.xlim.1 += data_dx;
                self.ylim.0 += data_dy;
                self.ylim.1 += data_dy;

                // update lines
                for line in &self.lines {
                    let line = &mut *line.lock().unwrap();
                    line.update_canvas_position(&self.axis.bounds, &self.xlim, &self.ylim);
                }
                // update click start point
                self.click_start = Some(point);
            }
        }
    }

    pub fn draw(&self, frame: &mut Frame, theme: &PlotTheme) {
        // background
        frame.fill_rectangle(self.bounds.position(), frame.size(), theme.axes_background);

        self.axis.draw_grid(frame, theme, &self.xlim, &self.ylim);

        for (i, line) in self.lines.iter().enumerate() {
            let line = &*line.lock().unwrap();
            line.draw(frame, theme, i, &self.axis.bounds);
        }
        self.axis.draw_border(frame, theme);

        //draw legend
        if let Some(legend) = &self.legend {
            legend.draw(frame, theme);
        }
    }

    pub fn get_figure_id(&self) -> Option<Id> {
        self.figure_id
    }

    pub fn mouse_double_clicked(&mut self, point: Point) {
        if self.axis.bounds.contains(point) {
            self.reset_axis();
        }
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

    pub fn mouse_middle_clicked(&mut self, point: Point) {
        if self.axis.bounds.contains(point) {
            self.click_start = Some(point);
            self.is_panning = true;
        }
    }

    pub fn mouse_middle_released(&mut self, _point: Point) {
        self.is_panning = false;
    }

    pub fn new(location: (usize, usize), figure_id: Option<Id>) -> Self {
        Self {
            figure_id,
            axis: Axis::default(),
            xlim: (0.0, 1.0),
            ylim: (-1.0, 1.0),
            legend: Some(Legend::default()),
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
            is_panning: false,
            current_zoom: 1.0,
            target_zoom: 1.0,
            zoom_speed: 5.0,
            zoom_center: None,
            initial_xlim: (-1.0, 1.0),
            initial_ylim: (-1.0, 1.0),
        }
    }
    pub fn reset_axis(&mut self) {
        self.xlim = self.initial_xlim;
        self.ylim = self.initial_ylim;
        for line in &self.lines {
            let mut line = line.lock().unwrap();
            line.update_canvas_position(&self.axis.bounds, &self.xlim, &self.ylim);
        }
        self.target_zoom = 1.0;
        self.current_zoom = 1.0;
    }
    pub fn set_figure_id(&mut self, id: Id) {
        self.figure_id = Some(id);
    }

    pub fn set_x_label(&mut self, label: String) {
        self.axis.set_x_label(label);
    }

    pub fn set_y_label(&mut self, label: String) {
        self.axis.set_y_label(label);
    }

    pub fn animation_tick(&mut self, dt: f32) -> bool {
        // Clamp dt to prevent huge time steps from causing visual jumps
        let dt = dt.min(0.1); // Maximum 100ms time step

        // Calculate how much more we need to zoom
        let zoom_delta = self.target_zoom - self.current_zoom;
        let mut request_clear = false;

        // If we're very close to target, snap to it and end animation
        if zoom_delta.abs() < 0.001 || zoom_delta.abs() / self.target_zoom.abs() < 0.01 {
            if zoom_delta != 0.0 {
                // Only if we actually need to change
                self.current_zoom = self.target_zoom;
                self.update_plot_for_zoom(1.0); // No scaling, just update lines
                request_clear = true;
            }
            return request_clear;
        }

        // Calculate zoom step based on dt and speed
        let zoom_step = zoom_delta * (1.0 - (-self.zoom_speed * dt).exp());

        // Calculate the zoom ratio (how much the view will change this frame)
        let zoom_ratio = (self.current_zoom + zoom_step) / self.current_zoom;

        // Update current zoom
        self.current_zoom += zoom_step;

        // Apply the zoom at the appropriate center point
        self.update_plot_for_zoom(zoom_ratio);

        request_clear = true;
        request_clear
    }
    pub fn update_bounds(&mut self, fig_size: Size, nrows: usize, ncols: usize) {
        self.bounds.height =
            fig_size.height / nrows as f32 - self.padding.top - self.padding.bottom;
        self.bounds.width = fig_size.width / ncols as f32 - self.padding.left - self.padding.right;
        self.bounds.x = self.bounds.width * self.location.1 as f32 + self.padding.left;
        self.bounds.y = self.bounds.height * self.location.0 as f32 + self.padding.top;
        self.axis.update_bounds(&self.bounds);
        for line in &self.lines {
            let line = &mut *line.lock().unwrap();
            line.update_canvas_position(&self.axis.bounds, &self.xlim, &self.ylim);
        }

        if let Some(legend) = &mut self.legend {
            legend.update(&self.axis.bounds, &self.lines);
        }
    }

    // Helper method to update plot limits based on zoom ratio and center point
    fn update_plot_for_zoom(&mut self, zoom_ratio: f32) {
        // Use zoom center if available, otherwise use plot center
        if let Some(center) = self.zoom_center {
            // Calculate new limits, scaling around the zoom center
            self.xlim.0 = center.x - (center.x - self.xlim.0) * zoom_ratio;
            self.xlim.1 = center.x + (self.xlim.1 - center.x) * zoom_ratio;
            self.ylim.0 = center.y - (center.y - self.ylim.0) * zoom_ratio;
            self.ylim.1 = center.y + (self.ylim.1 - center.y) * zoom_ratio;
        } else {
            // Calculate center point of the current view
            let center_x = (self.xlim.0 + self.xlim.1) / 2.0;
            let center_y = (self.ylim.0 + self.ylim.1) / 2.0;

            // Calculate current width and height
            let width = self.xlim.1 - self.xlim.0;
            let height = self.ylim.1 - self.ylim.0;

            // Calculate new width and height based on zoom ratio
            let new_width = width * zoom_ratio;
            let new_height = height * zoom_ratio;

            // Calculate new limits, keeping the center point fixed
            self.xlim.0 = center_x - (new_width / 2.0);
            self.xlim.1 = center_x + (new_width / 2.0);
            self.ylim.0 = center_y - (new_height / 2.0);
            self.ylim.1 = center_y + (new_height / 2.0);
        }

        // Update the line positions
        for line in &mut self.lines {
            let mut line = line.lock().unwrap();
            line.update_canvas_position(&self.axis.bounds, &self.xlim, &self.ylim);
        }
    }
    pub fn wheel_scrolled(&mut self, delta: ScrollDelta, point: Point) {
        // Only apply zoom if mouse is over the axis
        if !self.axis.bounds.contains(point) {
            return;
        }

        // Convert different scroll inputs to a consistent zoom factor
        let zoom_factor = match delta {
            ScrollDelta::Lines { x: _, y } => {
                // Lines tend to be larger movements, so scale appropriately
                // Negative y means scroll up (zoom in), positive means scroll down (zoom out)
                1.0 - (y * 0.2) // 20% change per line
            }
            ScrollDelta::Pixels { x: _, y } => {
                // Pixels need a smaller factor since there are more of them
                // Typical mouse wheel might generate 20-100 pixels per notch
                1.0 - (y * 0.005) // 0.5% change per pixel
            }
        };

        // Update target zoom based on wheel direction
        // zoom_factor < 1.0 = zoom in, zoom_factor > 1.0 = zoom out
        self.target_zoom *= zoom_factor;

        // clamp zoom to reasonable limits
        let min_zoom = 0.01; // Maximum zoom in (smaller values = more zoomed in)
        let max_zoom = 10.0; // Maximum zoom out
        self.target_zoom = self.target_zoom.clamp(min_zoom, max_zoom);

        // This makes the zoom feel more natural (commented out for simplicity)
        // Convert screen coordinates to plot coordinates
        let plot_x = self.xlim.0
            + (point.x - self.axis.bounds.x) / self.axis.bounds.width * (self.xlim.1 - self.xlim.0);
        let plot_y = self.ylim.1
            - (point.y - self.axis.bounds.y) / self.axis.bounds.height
                * (self.ylim.1 - self.ylim.0);

        // Store zoom center for use in animation_tick
        self.zoom_center = Some(Point::new(plot_x, plot_y));
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
