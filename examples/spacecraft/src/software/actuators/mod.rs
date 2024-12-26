use serde::{Deserialize, Serialize};


#[derive(Clone,Debug,Serialize,Deserialize,Default)]
pub struct ActuatorFsw {
    rw: ReactionWheelFsw,
}