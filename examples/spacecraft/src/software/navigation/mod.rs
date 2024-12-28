use nadir_result::{NadirResult,ResultManager};
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
      self.ad.run(&sensors.st, &sensors.imu);
    }

    pub fn write_results(&self, results: &mut ResultManager) {
      self.ad.write_result(results);
      self.od.write_result(results);      
  }

  pub fn initialize_results(&mut self, results: &mut ResultManager) {
      self.ad.new_result(results);
      self.od.new_result(results);      
  }
}
