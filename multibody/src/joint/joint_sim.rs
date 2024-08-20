use super::{
    floating::FloatingSim, joint_state::JointState, joint_transforms::JointTransforms, prismatic::PrismaticSim, revolute::RevoluteSim, Joint
};
use crate::algorithms::{
    articulated_body_algorithm::ArticulatedBodyAlgorithm, composite_rigid_body::CompositeRigidBody,
    recursive_newton_euler::RecursiveNewtonEuler, MultibodyAlgorithm,
};
use nalgebra::{DMatrix, DVector};
use spatial_algebra::{Acceleration, Force, SpatialInertia, SpatialTransform, Velocity};
use uuid::Uuid;

#[derive(Debug, Clone)]
pub enum JointSim {
    Floating(FloatingSim),
    Prismatic(PrismaticSim),
    Revolute(RevoluteSim),
}

impl From<Joint> for JointSim {
    fn from(joint: Joint) -> Self {
        match joint {
            Joint::Floating(joint) => JointSim::Floating(joint.into()),
            Joint::Prismatic(joint) => JointSim::Prismatic(joint.into()),
            Joint::Revolute(joint) => JointSim::Revolute(joint.into()),
        }
    }
}

pub trait JointSimTrait {
    fn calculate_tau(&mut self);
    fn calculate_vj(&mut self);
    fn get_a(&self) -> &Acceleration;
    fn get_derivative(&self) -> JointState;
    fn get_id(&self) -> &Uuid;
    fn get_inertia(&self) -> SpatialInertia;
    fn get_ndof(&self) -> usize;    
    fn get_state(&self) -> JointState;
    fn get_v(&self) -> &Velocity;
    fn set_inertia(&mut self, inertia: Option<SpatialInertia>);
    fn set_force(&mut self, force: Force);
    fn set_result(&mut self);
    fn set_state(&mut self, state: JointState);
    fn get_transforms(&self) -> &JointTransforms;
    fn get_transforms_mut(&mut self) -> &mut JointTransforms;
    fn update_transforms(&mut self, ij_transforms: Option<(SpatialTransform, SpatialTransform)>);
    fn with_algorithm(self, algorithm: MultibodyAlgorithm) -> Self;
}

impl JointSimTrait for JointSim {
    #[inline]
    fn calculate_tau(&mut self) {
        match self {
            JointSim::Floating(joint) => joint.calculate_tau(),
            JointSim::Prismatic(joint) => joint.calculate_tau(),
            JointSim::Revolute(joint) => joint.calculate_tau(),
        }
    }

    #[inline]
    fn calculate_vj(&mut self) {
        match self {
            JointSim::Floating(joint) => joint.calculate_vj(),
            JointSim::Prismatic(joint) => joint.calculate_vj(),
            JointSim::Revolute(joint) => joint.calculate_vj(),
        }
    }

    #[inline]
    fn get_a(&self) -> &Acceleration {
        match self {
            JointSim::Floating(joint) => joint.get_a(),
            JointSim::Prismatic(joint) => joint.get_a(),
            JointSim::Revolute(joint) => joint.get_a(),
        }
    }

    #[inline]
    fn get_derivative(&self) -> JointState {
        match self {
            JointSim::Floating(joint) => joint.get_derivative(),
            JointSim::Prismatic(joint) => joint.get_derivative(),
            JointSim::Revolute(joint) => joint.get_derivative(),
        }
    }

    #[inline]
    fn get_id(&self) -> &Uuid {
        match self {
            JointSim::Floating(joint) => joint.get_id(),
            JointSim::Prismatic(joint) => joint.get_id(),
            JointSim::Revolute(joint) => joint.get_id(),
        }
    }
    fn get_inertia(&self) -> SpatialInertia {
        match self {
            JointSim::Floating(joint) => joint.get_inertia(),
            JointSim::Prismatic(joint) => joint.get_inertia(),
            JointSim::Revolute(joint) => joint.get_inertia(),
        }
    }

    fn get_ndof(&self) -> usize {
        match self {
            JointSim::Floating(joint) => joint.get_ndof(),
            JointSim::Prismatic(joint) => joint.get_ndof(),
            JointSim::Revolute(joint) => joint.get_ndof(),
        }
    }

    #[inline]
    fn get_state(&self) -> JointState {
        match self {
            JointSim::Floating(joint) => joint.get_state(),
            JointSim::Prismatic(joint) => joint.get_state(),
            JointSim::Revolute(joint) => joint.get_state(),
        }
    }

    #[inline]
    fn get_v(&self) -> &Velocity {
        match self {
            JointSim::Floating(joint) => joint.get_v(),
            JointSim::Prismatic(joint) => joint.get_v(),
            JointSim::Revolute(joint) => joint.get_v(),
        }
    }

    #[inline]
    fn set_inertia(&mut self, inertia: Option<SpatialInertia>) {
        match self {
            JointSim::Floating(joint) => joint.set_inertia(inertia),
            JointSim::Prismatic(joint) => joint.set_inertia(inertia),
            JointSim::Revolute(joint) => joint.set_inertia(inertia),
        }
    }

    #[inline]
    fn set_force(&mut self, force: Force) {
        match self {
            JointSim::Floating(joint) => joint.set_force(force),
            JointSim::Prismatic(joint) => joint.set_force(force),
            JointSim::Revolute(joint) => joint.set_force(force),
        }
    }

    fn set_result(&mut self) {
        match self {
            JointSim::Floating(joint) => joint.set_result(),
            JointSim::Prismatic(joint) => joint.set_result(),
            JointSim::Revolute(joint) => joint.set_result(),
        }
    }

    #[inline]
    fn set_state(&mut self, state: JointState) {
        match self {
            JointSim::Floating(joint) => joint.set_state(state),
            JointSim::Prismatic(joint) => joint.set_state(state),
            JointSim::Revolute(joint) => joint.set_state(state),
        }
    }

    #[inline]
    fn get_transforms(&self) -> &JointTransforms {
        match self {
            JointSim::Floating(joint) => joint.get_transforms(),
            JointSim::Prismatic(joint) => joint.get_transforms(),
            JointSim::Revolute(joint) => joint.get_transforms(),
        }
    }

    #[inline]
    fn get_transforms_mut(&mut self) -> &mut JointTransforms {
        match self {
            JointSim::Floating(joint) => joint.get_transforms_mut(),
            JointSim::Prismatic(joint) => joint.get_transforms_mut(),
            JointSim::Revolute(joint) => joint.get_transforms_mut(),
        }
    }

    #[inline]
    fn update_transforms(&mut self, ij_transforms: Option<(SpatialTransform, SpatialTransform)>) {
        match self {
            JointSim::Floating(joint) => joint.update_transforms(ij_transforms),
            JointSim::Prismatic(joint) => joint.update_transforms(ij_transforms),
            JointSim::Revolute(joint) => joint.update_transforms(ij_transforms),
        }
    }

    #[inline]
    fn with_algorithm(self, algorithm: MultibodyAlgorithm) -> Self {
        match self {
            JointSim::Floating(joint) => JointSim::Floating(joint.with_algorithm(algorithm)),
            JointSim::Prismatic(joint) => JointSim::Prismatic(joint.with_algorithm(algorithm)),
            JointSim::Revolute(joint) => JointSim::Revolute(joint.with_algorithm(algorithm)),
        }
    }
}

impl ArticulatedBodyAlgorithm for JointSim {
    fn aba_first_pass(&mut self, v_ij: Velocity) {
        match self {
            JointSim::Floating(joint) => joint.aba_first_pass(v_ij),
            JointSim::Prismatic(joint) => joint.aba_first_pass(v_ij),
            JointSim::Revolute(joint) => joint.aba_first_pass(v_ij),
        }
    }
    fn aba_second_pass(&mut self, inner_is_base: bool) -> Option<(SpatialInertia, Force)> {
        match self {
            JointSim::Floating(joint) => joint.aba_second_pass(inner_is_base),
            JointSim::Prismatic(joint) => joint.aba_second_pass(inner_is_base),
            JointSim::Revolute(joint) => joint.aba_second_pass(inner_is_base),
        }
    }
    fn aba_third_pass(&mut self, a_ij: Acceleration) {
        match self {
            JointSim::Floating(joint) => joint.aba_third_pass(a_ij),
            JointSim::Prismatic(joint) => joint.aba_third_pass(a_ij),
            JointSim::Revolute(joint) => joint.aba_third_pass(a_ij),
        }
    }

    fn get_p_big_a(&self) -> Force {
        match self {
            JointSim::Floating(joint) => joint.get_p_big_a(),
            JointSim::Prismatic(joint) => joint.get_p_big_a(),
            JointSim::Revolute(joint) => joint.get_p_big_a(),
        }
    }

    fn add_inertia_articulated(&mut self, inertia: SpatialInertia) {
        match self {
            JointSim::Floating(joint) => joint.add_inertia_articulated(inertia),
            JointSim::Prismatic(joint) => joint.add_inertia_articulated(inertia),
            JointSim::Revolute(joint) => joint.add_inertia_articulated(inertia),
        }
    }

    fn add_p_big_a(&mut self, force: Force) {
        match self {
            JointSim::Floating(joint) => joint.add_p_big_a(force),
            JointSim::Prismatic(joint) => joint.add_p_big_a(force),
            JointSim::Revolute(joint) => joint.add_p_big_a(force),
        }
    }
}

impl RecursiveNewtonEuler for JointSim {
    fn rne_first_pass(&mut self, a_ij: Acceleration, v_ij: Velocity, use_qddot: bool) {
        match self {
            JointSim::Floating(joint) => joint.rne_first_pass(a_ij, v_ij, use_qddot),
            JointSim::Prismatic(joint) => joint.rne_first_pass(a_ij, v_ij, use_qddot),
            JointSim::Revolute(joint) => joint.rne_first_pass(a_ij, v_ij, use_qddot),
        }
    }

    fn rne_second_pass(&mut self) {
        match self {
            JointSim::Floating(joint) => joint.rne_second_pass(),
            JointSim::Prismatic(joint) => joint.rne_second_pass(),
            JointSim::Revolute(joint) => joint.rne_second_pass(),
        }
    }

    fn rne_add_force(&mut self, force: Force) {
        match self {
            JointSim::Floating(joint) => joint.rne_add_force(force),
            JointSim::Prismatic(joint) => joint.rne_add_force(force),
            JointSim::Revolute(joint) => joint.rne_add_force(force),
        }
    }

    fn rne_get_force(&self) -> Force {
        match self {
            JointSim::Floating(joint) => joint.rne_get_force(),
            JointSim::Prismatic(joint) => joint.rne_get_force(),
            JointSim::Revolute(joint) => joint.rne_get_force(),
        }
    }

    fn rne_set_tau(&mut self) {
        match self {
            JointSim::Floating(joint) => joint.rne_set_tau(),
            JointSim::Prismatic(joint) => joint.rne_set_tau(),
            JointSim::Revolute(joint) => joint.rne_set_tau(),
        }
    }
}

impl CompositeRigidBody for JointSim {
    fn add_ic(&mut self, new_ic: SpatialInertia) {
        match self {
            JointSim::Floating(joint) => joint.add_ic(new_ic),
            JointSim::Prismatic(joint) => joint.add_ic(new_ic),
            JointSim::Revolute(joint) => joint.add_ic(new_ic),
        }
    }

    fn get_crb_index(&self) -> usize {
        match self {
            JointSim::Floating(joint) => joint.get_crb_index(),
            JointSim::Prismatic(joint) => joint.get_crb_index(),
            JointSim::Revolute(joint) => joint.get_crb_index(),
        }
    }

    fn get_ic(&self) -> SpatialInertia {
        match self {
            JointSim::Floating(joint) => joint.get_ic(),
            JointSim::Prismatic(joint) => joint.get_ic(),
            JointSim::Revolute(joint) => joint.get_ic(),
        }
    }

    fn reset_ic(&mut self) {
        match self {
            JointSim::Floating(joint) => joint.reset_ic(),
            JointSim::Prismatic(joint) => joint.reset_ic(),
            JointSim::Revolute(joint) => joint.reset_ic(),
        }
    }
    fn set_crb_index(&mut self, n: usize) {
        match self {
            JointSim::Floating(joint) => joint.set_crb_index(n),
            JointSim::Prismatic(joint) => joint.set_crb_index(n),
            JointSim::Revolute(joint) => joint.set_crb_index(n),
        }
    }

    fn set_c(&self, c: &mut DVector<f64>) {
        match self {
            JointSim::Floating(joint) => joint.set_c(c),
            JointSim::Prismatic(joint) => joint.set_c(c),
            JointSim::Revolute(joint) => joint.set_c(c),
        }
    }

    fn set_h(&self, h: &mut DMatrix<f64>) {
        match self {
            JointSim::Floating(joint) => joint.set_h(h),
            JointSim::Prismatic(joint) => joint.set_h(h),
            JointSim::Revolute(joint) => joint.set_h(h),
        }
    }
}

#[derive(Debug)]
pub struct JointSimCommon {
    pub transforms: JointTransforms,
    pub cache: JointCache,
}

/// vj is the velocity across a joint (velocity of jof with respect to jif)
/// v is the velocity of the jof in the jof frame (inner joint's jof + vj)
#[derive(Debug, Default, Clone)]
pub struct JointCache {
    pub a: Acceleration,
    pub f: Force,
    pub v: Velocity,
    pub vj: Velocity,
}
