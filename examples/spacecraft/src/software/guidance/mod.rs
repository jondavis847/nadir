use rotations::prelude::{Quaternion, RotationMatrix};
use serde::{Deserialize, Serialize};

use super::navigation::NavigationFsw;

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
enum TargetMode {
    #[default]
    Nadir,
    Sun,
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
pub struct GuidanceFsw {
    parameters: Parameters,
    state: State,
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
struct Parameters {
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
struct State {
    target_attitude: Quaternion,
    target_mode: TargetMode,
}

impl GuidanceFsw {
    pub fn run(&mut self, nav: &NavigationFsw) {
        self.state.target_attitude = match self.state.target_mode {
            TargetMode::Nadir => {
                // create frame for x in velocity vector, z nadir, y completes
                let x = nav.od.state.velocity.normalize();
                let z = -nav.od.state.position.normalize();
                let y = z.cross(&x);
                let m = match RotationMatrix::from_cols(x,y,z) {
                        Ok(m) => m,
                        Err(_) => unimplemented!("better error handling for guidance")
                };
                Quaternion::from(&m)
            }
            _ => unimplemented!("implement other targeting modes")
        }
    }
}