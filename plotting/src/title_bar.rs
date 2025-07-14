#[derive(Debug, Clone)]
pub struct TitleBar {
    _title: String,
}

impl TitleBar {
    pub fn new(title: &str) -> Self {
        Self { _title: title.to_string() }
    }
}
