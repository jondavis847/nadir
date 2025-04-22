use std::collections::HashMap;

use color::Color;
use iced::{Point, Rectangle, Size, mouse::ScrollDelta};

use super::{axes::Axes, note_bar::NoteBar, title_bar::TitleBar};

#[derive(Debug, Clone)]
pub struct Figure {
    size: Size,
    axes: HashMap<u32, Axes>,
    background_color: Color,
    title_bar: Option<TitleBar>,
    note_bar: Option<NoteBar>,
}

impl Figure {
    pub fn new() -> Self {
        Self {
            axes: HashMap::new(),
            size: Size::new(720.0, 480.0),
            background_color: Color::new(0.9, 0.9, 0.9, 1.0),
            title_bar: None,
            note_bar: None,
        }
    }

    pub fn mouse_left_clicked(&mut self, point: Point) {
        for (_, axes) in &mut self.axes {
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
        for (_, axes) in &mut self.axes {
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

        for (_, axes) in &mut self.axes {
            let width = axes.xlim.1 - axes.xlim.0;
            let height = axes.ylim.1 - axes.ylim.0;
            axes.xlim.0 += width * SPEED * delta;
            axes.xlim.1 += -width * SPEED * delta;
            axes.ylim.0 += height * SPEED * delta;
            axes.ylim.1 += -height * SPEED * delta;
        }
    }

    pub fn window_resized(&mut self, window_size: Size) {
        let x_scale = window_size.width / self.size.width;
        let y_scale = window_size.height / self.size.height;

        for (_, axes) in &mut self.axes {
            axes.bounds.x *= x_scale;
            axes.bounds.y *= y_scale;
            axes.bounds.width *= x_scale;
            axes.bounds.height *= y_scale;
        }
        self.size.width = window_size.width;
        self.size.height = window_size.height;
    }
}
