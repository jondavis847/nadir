use std::ops::{AddAssign, MulAssign};
use super::{floating::FloatingState, prismatic::PrismaticState,revolute::RevoluteState};

#[derive(Clone, Copy, Debug)]
pub enum JointState {
    Floating(FloatingState),
    Prismatic(PrismaticState), // Spherical(SphericalState)
    Revolute(RevoluteState),   // Spherical(SphericalState)    
}

impl<'a> AddAssign<&'a Self> for JointState {
    fn add_assign(&mut self, rhs: &'a Self) {
        match (self, rhs) {
            (JointState::Floating(lhs), JointState::Floating(rhs)) => *lhs += rhs,
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
            JointState::Floating(lhs) => *lhs *= rhs,
            JointState::Revolute(lhs) => *lhs *= rhs,
            JointState::Prismatic(lhs) => *lhs *= rhs,
        }
    }
}


//thought about making this a type JointStates = Vec<Joints> but it needs to be integrable
#[derive(Clone, Debug)]
pub struct JointStates(pub Vec<JointState>);

impl<'a> AddAssign<&'a Self> for JointStates {
    fn add_assign(&mut self, rhs: &'a Self) {
        assert_eq!(
            self.0.len(),
            rhs.0.len(),
            "Joint vectors must have the same length"
        );
        for (a, b) in self.0.iter_mut().zip(rhs.0.iter()) {
            *a += b;
        }
    }
}

impl MulAssign<f64> for JointStates {
    fn mul_assign(&mut self, rhs: f64) {
        for joint in &mut self.0 {
            *joint *= rhs;
        }
    }
}