use iced::{Color, Point};
#[derive(Debug, Clone)]
pub struct Line {
    pub label: String,
    pub data: Vec<Point>,
    pub color: Option<Color>,
}

impl Line {
    pub fn new(label: String, data: Vec<Point>, color: Option<Color>) -> Self {
        Self { label, data, color }
    }
}
