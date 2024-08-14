use std::ops::{AddAssign, MulAssign};
use differential_equations::Integrable;

use super::{prismatic::PrismaticState,revolute::RevoluteState};

#[derive(Clone, Copy, Debug)]
pub enum JointState {
    Prismatic(PrismaticState), // Spherical(SphericalState)
    Revolute(RevoluteState),   // Spherical(SphericalState)
}

impl<'a> AddAssign<&'a Self> for JointState {
    fn add_assign(&mut self, rhs: &'a Self) {
        match (self, rhs) {
            (JointState::Revolute(lhs), JointState::Revolute(rhs)) => *lhs += rhs,
            (JointState::Prismatic(lhs), JointState::Prismatic(rhs)) => *lhs += rhs,
            // Handle other variants here if they are added
            // (JointState::Spherical(lhs), JointState::Spherical(rhs)) => JointState::Spherical(lhs + rhs),
            _ => panic!("Cannot add different JointState variants"),
        }
    }
}

impl MulAssign<f64> for JointState {
    fn mul_assign(&mut self, rhs: f64) {
        match self {
            JointState::Revolute(lhs) => *lhs *= rhs,
            JointState::Prismatic(lhs) => *lhs *= rhs,
        }
    }
}