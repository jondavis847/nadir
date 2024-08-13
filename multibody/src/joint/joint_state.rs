use std::ops::{Add, AddAssign, Mul, Div};
use super::{prismatic::PrismaticState,revolute::RevoluteState};

#[derive(Clone, Copy, Debug)]
pub enum JointState {
    Prismatic(PrismaticState), // Spherical(SphericalState)
    Revolute(RevoluteState),   // Spherical(SphericalState)
}

impl Add for JointState {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        match (self, rhs) {
            (JointState::Revolute(lhs), JointState::Revolute(rhs)) => {
                JointState::Revolute(lhs + rhs)
            }
            (JointState::Prismatic(lhs), JointState::Prismatic(rhs)) => {
                JointState::Prismatic(lhs + rhs)
            } // Handle other variants here if they are added
            // (JointState::Spherical(lhs), JointState::Spherical(rhs)) => JointState::Spherical(lhs + rhs),
            _ => panic!("Cannot add different JointState variants"), // shouldnt be possible, just used to integrate systems
        }
    }
}

impl AddAssign for JointState {
    fn add_assign(&mut self, rhs: Self) {
        match (self, rhs) {
            (JointState::Revolute(lhs), JointState::Revolute(rhs)) => *lhs += rhs,
            (JointState::Prismatic(lhs), JointState::Prismatic(rhs)) => *lhs += rhs,
            // Handle other variants here if they are added
            // (JointState::Spherical(lhs), JointState::Spherical(rhs)) => JointState::Spherical(lhs + rhs),
            _ => panic!("Cannot add different JointState variants"),
        }
    }
}

impl Mul<f64> for JointState {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self {
        match self {
            JointState::Prismatic(state) => JointState::Prismatic(state * rhs),
            JointState::Revolute(state) => JointState::Revolute(state * rhs),
        }
    }
}

impl Div<f64> for JointState {
    type Output = Self;

    fn div(self, rhs: f64) -> Self {
        match self {
            JointState::Prismatic(state) => JointState::Prismatic(state / rhs),
            JointState::Revolute(state) => JointState::Revolute(state / rhs),
        }
    }
}