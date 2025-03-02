pub trait Uncertainty {
    fn sample_uncertainty(&mut self) -> Self;
}
