use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum ColorMode {
    Preset,
    Rgba,
}
#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
pub struct Color {
    r: f32,
    g: f32,
    b: f32,
    a: f32,
}

impl Color {
    pub const RED: Self = Self { r: 1.0, g: 0.0, b: 0.0, a: 1.0 };

    pub const GREEN: Self = Self { r: 0.0, g: 1.0, b: 0.0, a: 1.0 };

    pub const BLUE: Self = Self { r: 0.0, g: 0.0, b: 1.0, a: 1.0 };

    pub const BLACK: Self = Self { r: 0.0, g: 0.0, b: 0.0, a: 1.0 };

    pub const YELLOW: Self = Self { r: 1.0, g: 1.0, b: 0.0, a: 1.0 };

    pub const ORANGE: Self = Self { r: 1.0, g: 0.65, b: 0.0, a: 1.0 };

    pub const WHITE: Self = Self { r: 1.0, g: 1.0, b: 1.0, a: 1.0 };

    pub fn new(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self { r, g, b, a }
    }
}

impl From<&Color> for [f32; 4] {
    fn from(color: &Color) -> Self {
        [color.r, color.g, color.b, color.a]
    }
}
