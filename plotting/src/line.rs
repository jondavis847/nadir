use iced::Color;
use serde::{Deserialize, Serialize};

use crate::series::Series;
#[derive(Debug, Clone, Serialize, Deserialize)]
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
