use super::{
    errors::JointErrors, joint_state::JointState, joint_transforms::JointTransforms,
    prismatic::PrismaticSim, revolute::RevoluteSim, Joint, JointParameters,
};
use crate::algorithms::{
    articulated_body_algorithm::ArticulatedBodyAlgorithm, composite_rigid_body::CompositeRigidBody,
    recursive_newton_euler::RecursiveNewtonEuler, MultibodyAlgorithm,
};
use spatial_algebra::{Acceleration, Force, SpatialInertia, SpatialTransform, Velocity};
use uuid::Uuid;

#[derive(Debug)]
pub enum JointSim {
    Prismatic(PrismaticSim),
    Revolute(RevoluteSim),
}

impl From<Joint> for JointSim {
    fn from(joint: Joint) -> Self {
        match joint {
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
    fn get_inertia(&self) -> &SpatialInertia;
    fn get_ndof(&self) -> usize;
    fn get_state(&self) -> JointState;
    fn get_v(&self) -> &Velocity;
    fn set_inertia(&mut self, inertia: SpatialInertia);
    fn set_state(&mut self, state: JointState);
    fn get_transforms(&self) -> &JointTransforms;
    fn get_transforms_mut(&mut self) -> &mut JointTransforms;
    fn update_transforms(&mut self, ij_transforms: Option<(SpatialTransform, SpatialTransform)>);
    fn with_algorithm(self, algorithm: MultibodyAlgorithm) -> Self;
}

impl JointSimTrait for JointSim {
    fn calculate_tau(&mut self) {
        match self {
            JointSim::Prismatic(joint) => joint.calculate_tau(),
            JointSim::Revolute(joint) => joint.calculate_tau(),
        }
    }

    fn calculate_vj(&mut self) {
        match self {
            JointSim::Prismatic(joint) => joint.calculate_vj(),
            JointSim::Revolute(joint) => joint.calculate_vj(),
        }
    }

    #[inline]
    fn get_a(&self) -> &Acceleration {
        match self {
            JointSim::Prismatic(joint) => joint.get_a(),
            JointSim::Revolute(joint) => joint.get_a(),
        }
    }

    #[inline]
    fn get_derivative(&self) -> JointState {
        match self {
            JointSim::Prismatic(joint) => joint.get_derivative(),
            JointSim::Revolute(joint) => joint.get_derivative(),
        }
    }

    #[inline]
    fn get_id(&self) -> &Uuid {
        match self {
            JointSim::Prismatic(joint) => joint.get_id(),
            JointSim::Revolute(joint) => joint.get_id(),
        }
    }
    fn get_inertia(&self) -> &SpatialInertia {
        match self {
            JointSim::Prismatic(joint) => joint.get_inertia(),
            JointSim::Revolute(joint) => joint.get_inertia(),
        }
    }

    fn get_ndof(&self) -> usize {
        match self {
            JointSim::Prismatic(joint) => joint.get_ndof(),
            JointSim::Revolute(joint) => joint.get_ndof(),
        }
    }

    #[inline]
    fn get_state(&self) -> JointState {
        match self {
            JointSim::Prismatic(joint) => joint.get_state(),
            JointSim::Revolute(joint) => joint.get_state(),
        }
    }

    #[inline]
    fn get_v(&self) -> &Velocity {
        match self {
            JointSim::Prismatic(joint) => joint.get_v(),
            JointSim::Revolute(joint) => joint.get_v(),
        }
    }

    #[inline]
    fn set_inertia(&mut self, inertia: SpatialInertia) {
        match self {
            JointSim::Prismatic(joint) => joint.set_inertia(inertia),
            JointSim::Revolute(joint) => joint.set_inertia(inertia),
        }
    }

    #[inline]
    fn set_state(&mut self, state: JointState) {
        match self {
            JointSim::Prismatic(joint) => joint.set_state(state),
            JointSim::Revolute(joint) => joint.set_state(state),
        }
    }

    #[inline]
    fn get_transforms(&self) -> &JointTransforms {
        match self {
            JointSim::Prismatic(joint) => joint.get_transforms(),
            JointSim::Revolute(joint) => joint.get_transforms(),
        }
    }

    #[inline]
    fn get_transforms_mut(&mut self) -> &mut JointTransforms {
        match self {
            JointSim::Prismatic(joint) => joint.get_transforms_mut(),
            JointSim::Revolute(joint) => joint.get_transforms_mut(),
        }
    }

    #[inline]
    fn update_transforms(&mut self, ij_transforms: Option<(SpatialTransform, SpatialTransform)>) {
        match self {
            JointSim::Prismatic(joint) => joint.update_transforms(ij_transforms),
            JointSim::Revolute(joint) => joint.update_transforms(ij_transforms),
        }
    }

    #[inline]
    fn with_algorithm(self, algorithm: MultibodyAlgorithm) -> Self {
        match self {
            JointSim::Prismatic(joint) => JointSim::Prismatic(joint.with_algorithm(algorithm)),
            JointSim::Revolute(joint) => JointSim::Revolute(joint.with_algorithm(algorithm)),
        }
    }
}

impl ArticulatedBodyAlgorithm for JointSim {
    fn aba_first_pass(&mut self, v_ij: Velocity, f_ob: &Force) {
        match self {
            JointSim::Prismatic(joint) => joint.aba_first_pass(v_ij, f_ob),
            JointSim::Revolute(joint) => joint.aba_first_pass(v_ij, f_ob),
        }
    }
    fn aba_second_pass(&mut self, inner_is_base: bool) -> Option<(SpatialInertia, Force)> {
        match self {
            JointSim::Prismatic(joint) => joint.aba_second_pass(inner_is_base),
            JointSim::Revolute(joint) => joint.aba_second_pass(inner_is_base),
        }
    }
    fn aba_third_pass(&mut self, a_ij: Acceleration) {
        match self {
            JointSim::Prismatic(joint) => joint.aba_third_pass(a_ij),
            JointSim::Revolute(joint) => joint.aba_third_pass(a_ij),
        }
    }

    fn get_p_big_a(&self) -> Force {
        match self {
            JointSim::Prismatic(joint) => joint.get_p_big_a(),
            JointSim::Revolute(joint) => joint.get_p_big_a(),
        }
    }

    fn add_inertia_articulated(&mut self, inertia: SpatialInertia) {
        match self {
            JointSim::Prismatic(joint) => joint.add_inertia_articulated(inertia),
            JointSim::Revolute(joint) => joint.add_inertia_articulated(inertia),
        }
    }

    fn add_p_big_a(&mut self, force: Force) {
        match self {
            JointSim::Prismatic(joint) => joint.add_p_big_a(force),
            JointSim::Revolute(joint) => joint.add_p_big_a(force),
        }
    }
}

impl RecursiveNewtonEuler for JointSim {
    fn rne_first_pass(
        &mut self,
        a_ij: Acceleration,
        v_ij: Velocity,
        f_ob: &Force,
        use_qddot: bool,
    ) {
        match self {
            JointSim::Prismatic(joint) => joint.rne_first_pass(a_ij, v_ij, f_ob, use_qddot),
            JointSim::Revolute(joint) => joint.rne_first_pass(a_ij, v_ij, f_ob, use_qddot),
        }
    }

    fn rne_second_pass(&mut self) {
        match self {
            JointSim::Prismatic(joint) => joint.rne_second_pass(),
            JointSim::Revolute(joint) => joint.rne_second_pass(),
        }
    }
}

impl CompositeRigidBody for JointSim {
    fn get_crb_index(&self) -> usize {
        match self {
            JointSim::Prismatic(joint) => joint.get_crb_index(),
            JointSim::Revolute(joint) => joint.get_crb_index(),
        }
    }
    fn set_crb_index(&mut self, n: usize) {
        match self {
            JointSim::Prismatic(joint) => joint.set_crb_index(n),
            JointSim::Revolute(joint) => joint.set_crb_index(n),
        }
    }
}

#[derive(Debug)]
pub struct JointSimCommon {
    pub transforms: JointTransforms,
}

#[derive(Debug)]
pub struct JointSimParameters {
    pub constant_force: f64,
    pub damping: f64,
    pub mass_properties: SpatialInertia,
    pub spring_constant: f64,
}

// This just unwraps the mass_properties
impl TryFrom<JointParameters> for JointSimParameters {
    type Error = JointErrors;
    fn try_from(jp: JointParameters) -> Result<Self, JointErrors> {
        if let Some(mass_properties) = &jp.mass_properties {
            Ok(Self {
                constant_force: jp.constant_force,
                damping: jp.damping,
                mass_properties: *mass_properties,
                spring_constant: jp.spring_constant,
            })
        } else {
            Err(JointErrors::NoMassProperties)
        }
    }
}
