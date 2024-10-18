use std::fmt;

use crate::body::BodyErrors;

#[derive(Debug, Clone, Copy)]
pub enum JointErrors {
    BodyErrors(BodyErrors),
    InnerBodyExists,
    NoMassProperties,
    OuterBodyExists,
}

impl fmt::Display for JointErrors {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            JointErrors::BodyErrors(e) => writeln!(f, "{:?}",e),
            JointErrors::InnerBodyExists => writeln!(f, "ERROR: inner body exists!"),
            JointErrors::NoMassProperties => writeln!(f, "ERROR: no joint mass properties found!"),
            JointErrors::OuterBodyExists => writeln!(f, "ERROR: outer body exists!"),
        }
    }
}

impl From<BodyErrors> for JointErrors {
    fn from(value: BodyErrors) -> Self {
        JointErrors::BodyErrors(value)
    }
}