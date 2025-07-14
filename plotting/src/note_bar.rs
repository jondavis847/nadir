#[derive(Debug, Clone)]
pub struct NoteBar {
    _note: String,
}

impl NoteBar {
    pub fn new(note: &str) -> Self {
        Self { _note: note.to_string() }
    }
}
