#[derive(Debug,Clone,Copy)]
pub enum MouseButton {
    Left,
    Right,
    Middle,
}

#[derive(Debug,Clone,Copy)]
pub enum MouseButtonReleaseEvents {
    SingleClick,
    DoubleClick,
    Held,
    Nothing,
}

