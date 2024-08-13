use std::fmt;

#[derive(Debug)]
pub enum JointErrors {
    InnerBodyExists,
    NoMassProperties,
    OuterBodyExists,
}

impl fmt::Display for JointErrors {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            JointErrors::InnerBodyExists => writeln!(f, "ERROR: inner body exists!"),
            JointErrors::NoMassProperties => writeln!(f, "ERROR: no joint mass properties found!"),
            JointErrors::OuterBodyExists => writeln!(f, "ERROR: outer body exists!"),
        }
    }
}