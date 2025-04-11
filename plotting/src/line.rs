use iced::Color;

use crate::series::Series;
#[derive(Debug)]
pub struct Line {
    pub label: String,
    pub data: Series,
    pub color: Option<Color>,
    pub legend: bool,
}

impl Line {
    pub fn new(label: String, data: Series, color: Option<Color>, legend: bool) -> Self {
        Self {
            label,
            data,
            color,
            legend,
        }
    }
}
