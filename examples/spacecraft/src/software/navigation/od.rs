use aerospace::{celestial_system::CelestialBodies, orbit::KeplerianElements};
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use time::{Time, TimeSystem};

use crate::software::sensors::gps::GpsFsw;

#[derive(Clone, Debug, Serialize, Deserialize, Default)]
pub struct OrbitDetermination {
    pub state: State,
    parameters: Parameters,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct State {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    orbit: KeplerianElements,
    pub sun_position: Vector3<f64>,
}

impl Default for State {
    fn default() -> Self {
        let orbit = KeplerianElements::new(
            1e8,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            Time::from_sec_j2k(0.0, time::TimeSystem::TAI),
            CelestialBodies::Earth,
        );
        let (position, velocity) = orbit.get_rv();
        Self {
            position,
            velocity,
            orbit,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, Default)]
struct Parameters {
    //propagator:
}

impl OrbitDetermination {
    pub fn run(&mut self, gps: &GpsFsw) {
        if gps.state.valid {
            self.state.position = gps.state.position;
            self.state.velocity = gps.state.velocity;
            self.state.orbit = KeplerianElements::from_rv(
                gps.state.position,
                gps.state.velocity,
                gps.state.time.to_system(TimeSystem::TAI),
                CelestialBodies::Earth,
            );
        } else {
            unimplemented!("need to implement orbit propagator")
        }
    }
}
