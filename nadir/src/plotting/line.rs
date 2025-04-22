use super::series::Series;
use iced::Color;
#[derive(Debug, Clone)]
pub struct Line {
    pub label: Option<String>,
    pub data: Series,
    pub color: Option<Color>,
    pub legend: bool,
}

impl Line {
    pub fn new(data: Series) -> Self {
        Self {
            label: None,
            data,
            color: None,
            legend: true,
        }
    }

    pub fn set_label(&mut self, label: String) {
        self.label = Some(label);
    }

    pub fn set_color(&mut self, color: Color) {
        self.color = Some(color);
    }
}
