#[derive(Debug, Clone, Copy)]
pub enum Errors {
    TooManyBases,
}

impl Errors {
    pub fn get_error_message(&self) -> &str {
        match self {
            Errors::TooManyBases => "Cannot have more than one base. Delete the old one first if this is intended."
        }
    }
}
