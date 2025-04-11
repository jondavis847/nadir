use crate::figure::Figure;

pub struct Identifier {
    current: usize,
}

impl Identifier {
    fn next(&mut self) -> usize {
        let value = self.current;
        self.current += 1;
        value
    }
}

pub struct PlotApp {
    identifier: Identifier,
    figures: Vec<Figure>,
}
