use super::quaternion::Quaternion;
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub enum EulerSequence {
    #[default]
    ZYX,
    XYZ,
    XZY,
    YXZ,
    YZX,
    ZXY,
    XYX,
    XZX,
    YXY,
    YZY,
    ZXZ,
    ZYZ,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct EulerAngles {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub sequence: EulerSequence,
}

impl EulerAngles {
    pub fn new(x: f64, y: f64, z: f64, sequence: EulerSequence) -> Self {
        Self { x, y, z, sequence }
    }
}
impl From<Quaternion> for EulerAngles {
    fn from(quat: Quaternion) -> Self {
        EulerAngles {
            x: quat.x,
            y: quat.y,
            z: quat.z,
            sequence: EulerSequence::default(),
        }
    }
}
