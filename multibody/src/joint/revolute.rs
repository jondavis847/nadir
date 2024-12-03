use crate::{
    algorithms::articulated_body_algorithm::ArticulatedBodyAlgorithm,
    joint::{joint_transforms::JointTransforms, JointModel, JointParameters},
    result::{MultibodyResultTrait, ResultEntry},
};
use coordinate_systems::CoordinateSystem;
use mass_properties::MassProperties;
use nalgebra::{Matrix6x1, Vector6};
use rotations::{
    euler_angles::{EulerAngles, EulerSequence},
    Rotation,
};
use serde::{Deserialize, Serialize};
use spatial_algebra::{Acceleration, Force, SpatialInertia, SpatialTransform, Velocity};
use std::ops::{AddAssign, MulAssign};
use std::{collections::HashMap, mem::take};
use thiserror::Error;
use transforms::Transform;

use super::{JointCache, JointRef, JointStateVector};
#[derive(Debug, Copy, Clone, Error)]
pub enum RevoluteErrors {}

#[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
pub struct RevoluteState {
    pub theta: f64,
    pub omega: f64,
}

impl RevoluteState {
    pub fn new(theta: f64, omega: f64) -> Self {
        // assume this is about Z until we add more axes
        Self { theta, omega }
    }
}

impl AddAssign<&Self> for RevoluteState {
    fn add_assign(&mut self, rhs: &Self) {
        self.theta += rhs.theta;
        self.omega += rhs.omega;
    }
}

impl MulAssign<f64> for RevoluteState {
    fn mul_assign(&mut self, rhs: f64) {
        self.theta *= rhs;
        self.omega *= rhs;
    }
}
#[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
pub struct RevoluteParameters(JointParameters);

#[derive(Debug, Default, Clone)]
struct RevoluteResult {
    theta: Vec<f64>,
    omega: Vec<f64>,
    alpha: Vec<f64>,
    tau: Vec<f64>,
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct Revolute {
    pub parameters: RevoluteParameters,
    pub state: RevoluteState,
    #[serde(skip)]
    cache: RevoluteCache,
    #[serde(skip)]
    result: RevoluteResult,
}

impl Revolute {
    pub fn new(parameters: RevoluteParameters, state: RevoluteState) -> Self {
        Self {
            parameters,
            state,
            ..Default::default()
        }
    }
}

#[typetag::serde]
impl JointModel for Revolute {
    fn calculate_joint_inertia(
        &mut self,
        inertia: &MassProperties,
        transforms: &JointTransforms,
    ) -> SpatialInertia {
        transforms.jof_from_ob * SpatialInertia::from(*inertia)
    }

    fn calculate_tau(&mut self) {
        let JointParameters {
            constant_force,
            damping,
            equilibrium,
            spring_constant,
        } = self.parameters.0;
        self.cache.tau = constant_force + spring_constant * (equilibrium - self.state.theta)
            - damping * self.state.omega;
    }

    fn calculate_vj(&self) -> Velocity {
        Velocity::from(Vector6::new(self.state.omega, 0.0, 0.0, 0.0, 0.0, 0.0))
    }

    fn ndof(&self) -> u32 {
        1
    }

    fn state_derivative(&self, derivative: &mut JointStateVector) {
        derivative.0[0] = self.state.omega;
        derivative.0[1] = self.cache.q_ddot;
    }

    fn state_vector_init(&self) -> JointStateVector {
        JointStateVector(vec![self.state.theta, self.state.omega])
    }

    fn state_vector_read(&mut self, state: &JointStateVector) {
        self.state.theta = state.0[0];
        self.state.omega = state.0[1];
    }

    fn state_vector_write(&self, state: &mut JointStateVector) {
        state.0[0] = self.state.omega;
        state.0[1] = self.state.theta;
    }

    fn update_transforms(
        &mut self,
        transforms: &mut JointTransforms,
        inner_joint: &Option<JointRef>,
    ) {
        let euler_angles = EulerAngles::new(0.0, 0.0, self.state.theta, EulerSequence::ZYX);
        let rotation = Rotation::EulerAngles(euler_angles);
        let translation = CoordinateSystem::ZERO;
        let transform = Transform::new(rotation, translation);

        transforms.jof_from_jif = SpatialTransform(transform);
        transforms.jif_from_jof = transforms.jof_from_jif.inv();
        transforms.update(inner_joint)
    }
}

impl MultibodyResultTrait for Revolute {
    fn get_result_entry(&mut self) -> ResultEntry {
        let mut result = HashMap::new();
        result.insert("theta".to_string(), take(&mut self.result.theta));
        result.insert("omega".to_string(), take(&mut self.result.omega));
        result.insert("alpha".to_string(), take(&mut self.result.alpha));
        result.insert("tau".to_string(), take(&mut self.result.tau));
        ResultEntry::new(result)
    }

    fn initialize_result(&mut self, capacity: usize) {
        self.result.theta = Vec::with_capacity(capacity);
        self.result.omega = Vec::with_capacity(capacity);
        self.result.alpha = Vec::with_capacity(capacity);
        self.result.tau = Vec::with_capacity(capacity);
    }

    fn update_result(&mut self) {
        self.result.theta.push(self.state.theta);
        self.result.omega.push(self.state.omega);
        self.result.alpha.push(self.cache.q_ddot);
        self.result.tau.push(self.cache.tau);
    }
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
struct RevoluteAbaCache {
    lil_u: f64,
    big_d_inv: f64,
    big_u: Matrix6x1<f64>,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
struct RevoluteCrbCache {
    cache_index: usize,
    ic: SpatialInertia,
}

#[derive(Debug, Default, Clone, Copy)]
struct RevoluteCache {
    aba: RevoluteAbaCache,
    //crb: RevoluteCrbCache,
    q_ddot: f64,
    //rne: RneCache,
    tau: f64,
}

impl ArticulatedBodyAlgorithm for Revolute {
    fn aba_second_pass(&mut self, joint_cache: &mut JointCache, inner_joint: &Option<JointRef>) {
        let aba = &mut self.cache.aba;
        let inertia_articulated_matrix = joint_cache.aba.inertia_articulated.matrix();

        // use the most efficient method for creating these. Indexing is much faster than 6x6 matrix mul
        aba.big_u = inertia_articulated_matrix.column(0).into();
        aba.big_d_inv = 1.0 / aba.big_u[0];
        aba.lil_u = self.cache.tau - (joint_cache.aba.p_big_a.get_index(1).unwrap()); //note force is 1 indexed, so

        if let Some(inner_joint_ref) = inner_joint {
            let big_u_times_big_d_inv = aba.big_u * aba.big_d_inv;
            let i_lil_a = SpatialInertia(
                inertia_articulated_matrix - big_u_times_big_d_inv * aba.big_u.transpose(),
            );

            joint_cache.aba.p_lil_a = joint_cache.aba.p_big_a
                + Force::from(i_lil_a * joint_cache.aba.c)
                + Force::from(big_u_times_big_d_inv * aba.lil_u);

            let mut inner_joint = inner_joint_ref.borrow_mut();
            inner_joint.cache.aba.inertia_articulated +=
                joint_cache.transforms.ij_jof_from_jof * i_lil_a;
            inner_joint.cache.aba.p_big_a +=
                joint_cache.transforms.ij_jof_from_jof * joint_cache.aba.p_lil_a;
        }
    }

    fn aba_third_pass(&mut self, joint_cache: &mut JointCache, inner_joint: &Option<JointRef>) {
        let a_ij = if let Some(inner_joint_ref) = inner_joint {
            inner_joint_ref.borrow().cache.a
        } else {
            Acceleration::zeros()
        };
        let a_prime = joint_cache.transforms.jof_from_ij_jof * a_ij + joint_cache.aba.c;
        self.cache.q_ddot = self.cache.aba.big_d_inv
            * (self.cache.aba.lil_u - (self.cache.aba.big_u.transpose() * a_prime.vector())[0]);
        joint_cache.a =
            a_prime + Acceleration::from(Vector6::new(self.cache.q_ddot, 0.0, 0.0, 0.0, 0.0, 0.0));
    }
}

// impl RecursiveNewtonEuler for Revolute {
//     fn rne_first_pass(&mut self, a_ij: Acceleration, v_ij: Velocity, use_qddot: bool) {
//         let a = &mut self.cache.common.a;
//         let v = &mut self.cache.common.v;
//         let vj = &mut self.cache.common.vj;
//         let q_ddot = &mut self.cache.q_ddot;
//         let f = &mut self.cache.rne.as_mut().unwrap().f;
//         let f_b = &mut self.cache.common.f;

//         let jof_from_ij_jof = &self.transforms.jof_from_ij_jof;
//         let joint_inertia = &self.mass_properties.unwrap();

//         *v = *jof_from_ij_jof * v_ij + *vj;

//         let a_new = match use_qddot {
//             true => {
//                 *jof_from_ij_jof * a_ij
//                     + Acceleration::from(Vector6::new(*q_ddot, 0.0, 0.0, 0.0, 0.0, 0.0))
//                     + v.cross_motion(*vj)
//             }
//             false => *jof_from_ij_jof * a_ij + v.cross_motion(*vj),
//         };

//         *a = a_new;

//         *f = *joint_inertia * *a + v.cross_force(*joint_inertia * *v) - *f_b;
//     }

//     fn rne_second_pass(&mut self) {
//         self.cache.tau = self.cache.rne.unwrap().f.rotation()[0];
//     }

//     fn rne_add_force(&mut self, force: Force) {
//         self.cache.rne.as_mut().unwrap().f = self.cache.rne.as_mut().unwrap().f + force;
//     }

//     fn rne_get_force(&self) -> Force {
//         self.cache.rne.unwrap().f
//     }

//     fn rne_set_tau(&mut self) {
//         self.cache.tau = self.cache.rne.unwrap().f.vector()[0];
//     }
// }

// impl CompositeRigidBody for Revolute {
//     fn add_ic(&mut self, new_ic: SpatialInertia) {
//         let ic = &mut self.cache.crb.as_mut().unwrap().ic;
//         *ic += new_ic;
//     }

//     fn get_crb_index(&self) -> usize {
//         self.cache.crb.unwrap().cache_index
//     }

//     fn get_ic(&self) -> SpatialInertia {
//         self.cache.crb.unwrap().ic
//     }

//     fn reset_ic(&mut self) {
//         let ic = &mut self.cache.crb.as_mut().unwrap().ic;
//         *ic = self.mass_properties.unwrap();
//     }

//     fn set_crb_index(&mut self, n: usize) {
//         if let Some(crb) = &mut self.cache.crb {
//             crb.cache_index = n;
//         }
//     }

//     fn set_c(&self, c: &mut DVector<f64>) {
//         let index = self.cache.crb.unwrap().cache_index;
//         let tau = Vector1::new(self.cache.tau);
//         c.set_row(index, &tau);
//     }

//     fn set_h(&self, h: &mut DMatrix<f64>) {
//         let crb = &self.cache.crb.unwrap();
//         let index = crb.cache_index;
//         let ic = &crb.ic;

//         let mut view = h.fixed_view_mut::<1, 1>(index, index);
//         view[0] = ic.matrix()[0];
//     }
// }
