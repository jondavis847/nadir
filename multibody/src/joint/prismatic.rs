use crate::{
    algorithms::{
        articulated_body_algorithm::ArticulatedBodyAlgorithm, recursive_newton_euler::RneCache,
    },
    joint::{joint_transforms::JointTransforms, JointParameters},
    result::{MultibodyResultTrait, ResultEntry},
};
use coordinate_systems::{cartesian::Cartesian, CoordinateSystem};
use mass_properties::MassProperties;
use nalgebra::{Matrix6x1, Vector6};
use rotations::{Rotation, RotationTrait};
use serde::{Deserialize, Serialize};
use spatial_algebra::{Acceleration, Force, SpatialInertia, SpatialTransform, Velocity};
use std::ops::{AddAssign, MulAssign};
use std::{collections::HashMap, mem::take};
use transforms::Transform;

use super::{JointCache, JointModel, JointRef, JointStateVector};

#[derive(Debug, Copy, Clone)]
pub enum PrismaticErrors {}

#[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
pub struct PrismaticState {
    pub position: f64,
    pub velocity: f64,
}

impl PrismaticState {
    pub fn new(position: f64, velocity: f64) -> Self {
        // assume this is about Z until we add more axes
        Self { position, velocity }
    }
}

impl<'a> AddAssign<&'a Self> for PrismaticState {
    fn add_assign(&mut self, rhs: &'a Self) {
        self.position += rhs.position;
        self.velocity += rhs.velocity;
    }
}

impl MulAssign<f64> for PrismaticState {
    fn mul_assign(&mut self, rhs: f64) {
        self.position *= rhs;
        self.velocity *= rhs;
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PrismaticParameters(JointParameters);

#[derive(Debug, Default, Clone)]
struct PrismaticResult {
    position: Vec<f64>,
    velocity: Vec<f64>,
    acceleration: Vec<f64>,
    tau: Vec<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Prismatic {
    pub parameters: PrismaticParameters,
    pub state: PrismaticState,
    #[serde(skip)]
    cache: PrismaticCache,
    #[serde(skip)]
    result: PrismaticResult,
}

impl Prismatic {
    pub fn new(parameters: PrismaticParameters, state: PrismaticState) -> Self {
        Self {
            parameters,
            state,
            cache: PrismaticCache::default(),
            result: PrismaticResult::default(),
        }
    }
}

#[typetag::serde]
impl JointModel for Prismatic {
    fn calculate_joint_inertia(
        &mut self,
        mass_properties: &MassProperties,
        transforms: &JointTransforms,
    ) -> SpatialInertia {
        transforms.jof_from_ob * SpatialInertia::from(*mass_properties)
    }

    fn calculate_tau(&mut self) {
        let JointParameters {
            constant_force,
            damping,
            equilibrium,
            spring_constant,
            ..
        } = self.parameters.0;
        self.cache.tau = constant_force
            - spring_constant * (equilibrium - self.state.position)
            - damping * self.state.velocity;
    }

    fn calculate_vj(&self, _transforms: &JointTransforms) -> Velocity {
        Velocity::from(Vector6::new(0.0, 0.0, 0.0, self.state.velocity, 0.0, 0.0))
    }

    fn ndof(&self) -> u32 {
        1
    }

    fn state_derivative(&self, derivative: &mut JointStateVector, _transforms: &JointTransforms) {
        derivative.0[0] = self.state.velocity;
        derivative.0[1] = self.cache.q_ddot;
    }

    fn state_vector_init(&self) -> JointStateVector {
        JointStateVector(vec![self.state.position, self.state.velocity])
    }

    fn state_vector_read(&mut self, state: &JointStateVector) {
        self.state.position = state.0[0];
        self.state.velocity = state.0[1];
    }

    fn state_vector_write(&self, state: &mut JointStateVector) {
        state.0[0] = self.state.position;
        state.0[1] = self.state.velocity;
    }

    fn update_transforms(
        &mut self,
        transforms: &mut JointTransforms,
        inner_joint: &Option<JointRef>,
    ) {
        let rotation = Rotation::identity();
        let translation = CoordinateSystem::from(Cartesian::new(self.state.position, 0.0, 0.0));
        let transform = Transform::new(rotation, translation);

        transforms.jof_from_jif = SpatialTransform(transform);
        transforms.jif_from_jof = transforms.jof_from_jif.inv();
        transforms.update(inner_joint)
    }
}

impl MultibodyResultTrait for Prismatic {
    fn get_result_entry(&mut self) -> ResultEntry {
        let mut result = HashMap::new();
        result.insert("position".to_string(), take(&mut self.result.position));
        result.insert("velocity".to_string(), take(&mut self.result.velocity));
        result.insert(
            "acceleration".to_string(),
            take(&mut self.result.acceleration),
        );
        result.insert("tau".to_string(), take(&mut self.result.tau));
        ResultEntry::new(result)
    }

    fn initialize_result(&mut self, capacity: usize) {
        self.result.position = Vec::with_capacity(capacity);
        self.result.velocity = Vec::with_capacity(capacity);
        self.result.acceleration = Vec::with_capacity(capacity);
        self.result.tau = Vec::with_capacity(capacity);
    }

    fn update_result(&mut self) {
        self.result.position.push(self.state.position);
        self.result.velocity.push(self.state.velocity);
        self.result.acceleration.push(self.cache.q_ddot);
        self.result.tau.push(self.cache.tau);
    }
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
struct PrismaticAbaCache {
    lil_u: f64,
    big_d_inv: f64,
    big_u: Matrix6x1<f64>,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
struct PrismaticCrbCache {
    cache_index: usize,
    ic: SpatialInertia,
}

#[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
struct PrismaticCache {
    aba: PrismaticAbaCache,
    crb: PrismaticCrbCache,
    q_ddot: f64,
    rne: RneCache,
    tau: f64,
}

impl ArticulatedBodyAlgorithm for Prismatic {
    fn aba_second_pass(&mut self, joint_cache: &mut JointCache, inner_joint: &Option<JointRef>) {
        let aba = &mut self.cache.aba;
        let inertia_articulated_matrix = joint_cache.aba.inertia_articulated.matrix();

        // use the most efficient method for creating these. Indexing is much faster than 6x6 matrix mul
        aba.big_u = inertia_articulated_matrix.column(3).into();
        aba.big_d_inv = 1.0 / aba.big_u[3];
        aba.lil_u = self.cache.tau - (joint_cache.aba.p_big_a.get_index(4).unwrap()); //note force is 1 indexed, so

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
            * (self.cache.aba.lil_u - (self.cache.aba.big_u.transpose() * a_prime.vector())[3]);
        joint_cache.a =
            a_prime + Acceleration::from(Vector6::new(0.0, 0.0, 0.0, self.cache.q_ddot, 0.0, 0.0));
    }
}

// impl RecursiveNewtonEuler for Prismatic {
//     fn rne_first_pass(&mut self, a_ij: Acceleration, v_ij: Velocity, use_qddot: bool) {
//         let a = &mut self.cache.common.a;
//         let f = &mut self.cache.rne.as_mut().unwrap().f;
//         let q_ddot = &mut self.cache.q_ddot;
//         let v = &mut self.cache.common.v;
//         let vj = &self.cache.common.vj;
//         let f_b = &self.cache.common.f;

//         let jof_from_ij_jof = &self.transforms.jof_from_ij_jof;
//         let joint_inertia = &self.mass_properties.unwrap();

//         *v = *jof_from_ij_jof * v_ij + *vj;

//         let a_new = match use_qddot {
//             true => {
//                 *jof_from_ij_jof * a_ij
//                     + Acceleration::from(Vector6::new(0.0, 0.0, 0.0, *q_ddot, 0.0, 0.0))
//                     + v.cross_motion(*vj)
//             }
//             false => *jof_from_ij_jof * a_ij + v.cross_motion(*vj),
//         };

//         *a = a_new;

//         *f = *joint_inertia * *a + v.cross_force(*joint_inertia * *v) - *f_b;
//     }

//     fn rne_second_pass(&mut self) {
//         self.cache.tau = self.cache.rne.unwrap().f.translation()[0];
//     }

//     fn rne_add_force(&mut self, force: Force) {
//         self.cache.rne.as_mut().unwrap().f = self.cache.rne.as_mut().unwrap().f + force;
//     }

//     fn rne_get_force(&self) -> Force {
//         self.cache.rne.unwrap().f
//     }

//     fn rne_set_tau(&mut self) {
//         self.cache.tau = self.cache.rne.unwrap().f.vector()[3];
//     }
// }

// impl CompositeRigidBody for Prismatic {
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
//         let crb = self.cache.crb.unwrap();
//         let index = crb.cache_index;
//         let ic = &crb.ic;

//         let mut view = h.fixed_view_mut::<1, 1>(index, index);
//         view[0] = ic.matrix()[(3, 3)];
//     }
// }
