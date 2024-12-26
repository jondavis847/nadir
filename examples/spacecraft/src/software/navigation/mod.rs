use serde::{Deserialize, Serialize};

use super::sensors::SensorFsw;

mod od;
mod ad;
use od::OrbitDetermination;
use ad::AttitudeDetermination;


#[derive(Clone,Debug,Serialize,Deserialize,Default)]
pub struct NavigationFsw {
    pub od: OrbitDetermination,
    pub ad: AttitudeDetermination,
}

impl NavigationFsw {
    pub fn run(&mut self, sensors: &SensorFsw) {
      self.od.run(&sensors.gps);
      self.ad.run(&sensors.st);
    }
}