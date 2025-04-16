use color::Color;
use serde::{Deserialize, Serialize};

//use crate::axes::Axes;
use std::collections::HashMap;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Figure {
    id: u32,
    height: f32,
    width: f32,
    //axes: HashMap<u32, Axes>,
    background_color: Color,
}

impl Figure {
    pub fn new(id: u32) -> Self {
        Self {
            id,
            //axes: HashMap::new(),
            width: 800.0,
            height: 400.0,
            background_color: Color::new(0.9, 0.9, 0.9, 1.0),
        }
    }

    pub fn set_background_color(&mut self, color: Color) {
        self.background_color = color;
    }

    pub fn set_height(&mut self, height: f32) {
        self.height = height;
    }

    pub fn set_width(&mut self, width: f32) {
        self.width = width;
    }
}
