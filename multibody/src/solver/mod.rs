pub mod rk4;

pub struct SimulationConfig {
    pub name: String,
    pub start: f64,
    pub stop: f64,
    pub dt: f64,
}