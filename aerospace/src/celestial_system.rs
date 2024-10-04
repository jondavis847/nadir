use nalgebra::Vector3;
use time::EphemerisTime;
use rotations::Rotation;
use crate::earth::Earth;
use serde::{Deserialize, Serialize};

#[derive(Debug,Clone,Deserialize,Serialize)]
pub struct CelestialSystem {
    epoch: EphemerisTime,
    bodies: Vec<CelestialBody>,
}

#[derive(Debug,Clone,Deserialize,Serialize)]
pub struct CelestialBody {
    position: Vector3<f64>,
    orientation: Rotation,
}

#[derive(Debug,Clone,Deserialize,Serialize)]
pub enum CelestialBodies {
    Earth(Earth),
    Jupiter(CelestialBody),
    Mercury(CelestialBody),
    Mars(CelestialBody),
    Moon(CelestialBody),
    Neptune(CelestialBody),
    Pluto(CelestialBody),
    Saturn(CelestialBody),
    Sun(CelestialBody),
    Uranus(CelestialBody),
    Venus(CelestialBody),
}
