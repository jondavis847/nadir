use crate::{
    algorithms::{
        articulated_body_algorithm::{AbaCache, ArticulatedBodyAlgorithm},
        composite_rigid_body::CompositeRigidBody,
        recursive_newton_euler::{RecursiveNewtonEuler, RneCache},
        MultibodyAlgorithm,
    },
    body::{Body, BodyConnection, BodyTrait},
    joint::{
        joint_sim::{JointCache, JointSimTrait},
        joint_state::JointState,
        joint_transforms::JointTransforms,
        JointCommon, JointConnection, JointErrors, JointParameters, JointTrait,
    },
    MultibodyTrait,
};
use coordinate_systems::CoordinateSystem;
use nalgebra::{DMatrix, DVector, Matrix6x1, Vector1, Vector6};
use rotations::{
    euler_angles::{EulerAngles, EulerSequence},
    Rotation,
};
use serde::{Deserialize, Serialize};
use spatial_algebra::{Acceleration, Force, SpatialInertia, SpatialTransform, Velocity};
use std::ops::{AddAssign, MulAssign};
use transforms::Transform;
use uuid::Uuid;

#[derive(Debug, Copy, Clone)]
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

impl<'a> AddAssign<&'a Self> for RevoluteState {
    fn add_assign(&mut self, rhs: &'a Self) {
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

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct RevoluteResult {
    pub theta: Vec<f64>,
    pub omega: Vec<f64>,
    pub angular_accel: Vec<f64>,
    pub internal_torque: Vec<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Revolute {
    pub common: JointCommon,
    pub parameters: JointParameters,
    pub state: RevoluteState,
}

impl Revolute {
    pub fn new(name: &str, parameters: JointParameters, state: RevoluteState) -> Self {
        let common = JointCommon::new(name);

        Self {
            common,
            parameters,
            state,
        }
    }
}

impl JointTrait for Revolute {
    fn connect_inner_body<T: BodyTrait>(
        &mut self,
        body: &mut T,
        transform: Transform,
    ) -> Result<(), JointErrors> {
        if self.common.connection.inner_body.is_some() {
            return Err(JointErrors::InnerBodyExists);
        }
        body.connect_outer_joint(self).unwrap();
        let connection = BodyConnection::new(*body.get_id(), transform);
        self.common.connection.inner_body = Some(connection);
        Ok(())
    }

    fn connect_outer_body(
        &mut self,
        body: &mut Body,
        transform: Transform,
    ) -> Result<(), JointErrors> {
        if self.common.connection.outer_body.is_some() {
            return Err(JointErrors::OuterBodyExists);
        }
        //let body_mass_properties = body.get_mass_properties();
        //let spatial_transform = SpatialTransform::from(transform);
        //let spatial_inertia = SpatialInertia::from(*body_mass_properties);
        //let joint_mass_properties = spatial_transform * spatial_inertia;
        //self.parameters.mass_properties = Some(joint_mass_properties); lets calculate only when we sim now, leave as None
        body.connect_inner_joint(self).unwrap();
        let connection = BodyConnection::new(*body.get_id(), transform);
        self.common.connection.outer_body = Some(connection);
        Ok(())
    }

    fn delete_inner_body_id(&mut self) {
        if self.common.connection.inner_body.is_some() {
            self.common.connection.inner_body = None;
        }
    }

    fn delete_outer_body_id(&mut self) {
        if self.common.connection.outer_body.is_some() {
            self.common.connection.outer_body = None;
        }
        self.common.mass_properties = None;
    }

    fn get_connections(&self) -> &JointConnection {
        &self.common.connection
    }

    fn get_connections_mut(&mut self) -> &mut JointConnection {
        &mut self.common.connection
    }

    fn get_inner_body_id(&self) -> Option<&Uuid> {
        match &self.common.connection.inner_body {
            Some(connection) => Some(&connection.body_id),
            None => None,
        }
    }
    fn get_outer_body_id(&self) -> Option<&Uuid> {
        match &self.common.connection.outer_body {
            Some(connection) => Some(&connection.body_id),
            None => None,
        }
    }
}

impl MultibodyTrait for Revolute {
    fn get_id(&self) -> &Uuid {
        &self.common.id
    }
    fn get_name(&self) -> &str {
        &self.common.name
    }

    fn set_name(&mut self, name: String) {
        self.common.name = name;
    }
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
struct RevoluteAbaCache {
    common: AbaCache,
    lil_u: f64,
    big_d_inv: f64,
    big_u: Matrix6x1<f64>,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
struct RevoluteCrbCache {
    cache_index: usize,
    ic: SpatialInertia,
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
struct RevoluteCache {
    common: JointCache,
    aba: Option<RevoluteAbaCache>,
    crb: Option<RevoluteCrbCache>,
    q_ddot: f64,
    rne: Option<RneCache>,
    tau: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RevoluteSim {
    cache: RevoluteCache,
    id: Uuid,
    mass_properties: Option<SpatialInertia>,
    parameters: JointParameters,
    pub result: RevoluteResult,
    state: RevoluteState,
    transforms: JointTransforms,
}

impl RevoluteSim {
    pub const DOF: usize = 1;
}

impl From<Revolute> for RevoluteSim {
    fn from(revolute: Revolute) -> Self {
        // update the joints to body transforms
        let mut transforms = JointTransforms::default();
        if let Some(inner_body) = &revolute.common.connection.inner_body {
            transforms.jif_from_ib = inner_body.transform.into();
            transforms.ib_from_jif = inner_body.transform.inv().into();
        } else {
            panic!("should always be an inner body connected")
        }

        if let Some(outer_body) = &revolute.common.connection.outer_body {
            transforms.jof_from_ob = outer_body.transform.into();
            transforms.ob_from_jof = outer_body.transform.inv().into();
        } else {
            panic!("should always be an inner body connected")
        }

        RevoluteSim {
            cache: RevoluteCache::default(),
            id: *revolute.get_id(),
            mass_properties: None,
            parameters: revolute.parameters,
            result: RevoluteResult::default(),
            state: revolute.state,
            transforms,
        }
    }
}

impl ArticulatedBodyAlgorithm for RevoluteSim {
    fn aba_first_pass(&mut self, v_ij: Velocity) {
        let aba = self.cache.aba.as_mut().unwrap();
        let transforms = &self.transforms;
        let joint_inertia = &self.mass_properties.unwrap();
        let v = &mut self.cache.common.v;
        let vj = &self.cache.common.vj;
        let f = &self.cache.common.f;

        *v = transforms.jof_from_ij_jof * v_ij + *vj;
        aba.common.c = v.cross_motion(*vj); // + cj
        aba.common.inertia_articulated = *joint_inertia;
        aba.common.p_big_a = v.cross_force(*joint_inertia * *v) - *f;
    }

    fn aba_second_pass(&mut self, inner_is_base: bool) -> Option<(SpatialInertia, Force)> {
        let aba = self.cache.aba.as_mut().unwrap();
        let inertia_articulated_matrix = aba.common.inertia_articulated.matrix();

        // use the most efficient method for creating these. Indexing is much faster than 6x6 matrix mul
        aba.big_u = inertia_articulated_matrix.column(0).into();
        aba.big_d_inv = 1.0 / aba.big_u[0];
        aba.lil_u = self.cache.tau - (aba.common.p_big_a.get_index(1).unwrap()); //note force is 1 indexed, so 1

        if !inner_is_base {
            let big_u_times_big_d_inv = aba.big_u * aba.big_d_inv;
            let i_lil_a = SpatialInertia(
                inertia_articulated_matrix - big_u_times_big_d_inv * aba.big_u.transpose(),
            );

            aba.common.p_lil_a = aba.common.p_big_a
                + Force::from(i_lil_a * aba.common.c)
                + Force::from(big_u_times_big_d_inv * aba.lil_u);

            let parent_inertia_articulated_contribution = self.transforms.ij_jof_from_jof * i_lil_a;
            let parent_p_big_a = self.transforms.ij_jof_from_jof * aba.common.p_lil_a;
            Some((parent_inertia_articulated_contribution, parent_p_big_a))
        } else {
            None
        }
    }

    fn aba_third_pass(&mut self, a_ij: Acceleration) {
        let aba = self.cache.aba.as_mut().unwrap();
        let a = &mut self.cache.common.a;
        let q_ddot = &mut self.cache.q_ddot;

        aba.common.a_prime = self.transforms.jof_from_ij_jof * a_ij + aba.common.c;

        *q_ddot =
            aba.big_d_inv * (aba.lil_u - (aba.big_u.transpose() * aba.common.a_prime.vector())[0]);
        *a =
            aba.common.a_prime + Acceleration::from(Vector6::new(*q_ddot, 0.0, 0.0, 0.0, 0.0, 0.0));
    }

    fn get_p_big_a(&self) -> Force {
        self.cache.aba.unwrap().common.p_big_a
    }

    fn add_inertia_articulated(&mut self, inertia: SpatialInertia) {
        let aba = self.cache.aba.as_mut().unwrap();
        let ia = &mut aba.common.inertia_articulated;
        *ia = *ia + inertia;
    }

    fn add_p_big_a(&mut self, p_big_a: Force) {
        let aba = self.cache.aba.as_mut().unwrap();
        let pa = &mut aba.common.p_big_a;
        *pa = *pa + p_big_a;
    }
}

impl RecursiveNewtonEuler for RevoluteSim {
    fn rne_first_pass(&mut self, a_ij: Acceleration, v_ij: Velocity, use_qddot: bool) {
        let a = &mut self.cache.common.a;
        let v = &mut self.cache.common.v;
        let vj = &mut self.cache.common.vj;
        let q_ddot = &mut self.cache.q_ddot;
        let f = &mut self.cache.rne.as_mut().unwrap().f;
        let f_b = &mut self.cache.common.f;

        let jof_from_ij_jof = &self.transforms.jof_from_ij_jof;
        let joint_inertia = &self.mass_properties.unwrap();

        *v = *jof_from_ij_jof * v_ij + *vj;

        let a_new = match use_qddot {
            true => {
                *jof_from_ij_jof * a_ij
                    + Acceleration::from(Vector6::new(*q_ddot, 0.0, 0.0, 0.0, 0.0, 0.0))
                    + v.cross_motion(*vj)
            }
            false => *jof_from_ij_jof * a_ij + v.cross_motion(*vj),
        };

        *a = a_new;

        *f = *joint_inertia * *a + v.cross_force(*joint_inertia * *v) - *f_b;
    }

    fn rne_second_pass(&mut self) {
        self.cache.tau = self.cache.rne.unwrap().f.rotation()[0];
    }

    fn rne_add_force(&mut self, force: Force) {
        self.cache.rne.as_mut().unwrap().f = self.cache.rne.as_mut().unwrap().f + force;
    }

    fn rne_get_force(&self) -> Force {
        self.cache.rne.unwrap().f
    }

    fn rne_set_tau(&mut self) {
        self.cache.tau = self.cache.rne.unwrap().f.vector()[0];
    }
}

impl JointSimTrait for RevoluteSim {
    fn calculate_tau(&mut self) {
        let JointParameters {
            constant_force,
            damping,
            spring_constant,
            ..
        } = self.parameters;
        self.cache.tau =
            constant_force - spring_constant * self.state.theta - damping * self.state.omega;
    }

    fn calculate_vj(&mut self) {
        self.cache.common.vj =
            Velocity::from(Vector6::new(self.state.omega, 0.0, 0.0, 0.0, 0.0, 0.0));
    }

    fn get_a_jof(&self) -> &Acceleration {
        &self.cache.common.a
    }

    fn get_derivative(&self) -> JointState {
        let derivative = RevoluteState::new(self.state.omega, self.cache.q_ddot);
        JointState::Revolute(derivative)
    }

    fn get_id(&self) -> &Uuid {
        &self.id
    }
    fn get_inertia(&self) -> SpatialInertia {
        self.mass_properties.unwrap()
    }

    fn get_ndof(&self) -> usize {
        RevoluteSim::DOF
    }

    fn get_state(&self) -> JointState {
        JointState::Revolute(self.state)
    }

    fn get_v(&self) -> &Velocity {
        &self.cache.common.v
    }

    fn set_inertia(&mut self, inertia: Option<SpatialInertia>) {
        self.mass_properties = inertia;
    }

    fn set_force(&mut self, force: Force) {
        self.cache.common.f = force;
    }

    fn set_result(&mut self) {
        self.result.omega.push(self.state.omega);
        self.result.theta.push(self.state.theta);
        self.result.angular_accel.push(self.cache.q_ddot);
        self.result.internal_torque.push(self.cache.tau);
    }

    fn set_state(&mut self, state: JointState) {
        if let JointState::Revolute(revolute_state) = state {
            self.state = revolute_state
        }
    }
    fn get_transforms(&self) -> &JointTransforms {
        &self.transforms
    }

    fn get_transforms_mut(&mut self) -> &mut JointTransforms {
        &mut self.transforms
    }
    fn update_transforms(&mut self, ij_transforms: Option<(SpatialTransform, SpatialTransform)>) {
        let euler_angles = EulerAngles::new(0.0, 0.0, self.state.theta, EulerSequence::ZYX);
        let rotation = Rotation::EulerAngles(euler_angles);
        let translation = CoordinateSystem::default();
        let transform = Transform::new(rotation, translation);

        self.transforms.jof_from_jif = SpatialTransform(transform);
        self.transforms.jif_from_jof = self.transforms.jof_from_jif.inv();
        self.transforms.update(ij_transforms)
    }

    fn with_algorithm(mut self, algorithm: MultibodyAlgorithm) -> Self {
        match algorithm {
            MultibodyAlgorithm::ArticulatedBody => {
                self.cache.aba = Some(RevoluteAbaCache::default());
                self.cache.crb = None;
                self.cache.rne = None;
            }
            MultibodyAlgorithm::CompositeRigidBody => {
                self.cache.aba = None;
                self.cache.crb = Some(RevoluteCrbCache::default());
                self.cache.rne = Some(RneCache::default());
            }
        }
        self
    }
}

impl CompositeRigidBody for RevoluteSim {
    fn add_ic(&mut self, new_ic: SpatialInertia) {
        let ic = &mut self.cache.crb.as_mut().unwrap().ic;
        *ic += new_ic;
    }

    fn get_crb_index(&self) -> usize {
        self.cache.crb.unwrap().cache_index
    }

    fn get_ic(&self) -> SpatialInertia {
        self.cache.crb.unwrap().ic
    }

    fn reset_ic(&mut self) {
        let ic = &mut self.cache.crb.as_mut().unwrap().ic;
        *ic = self.mass_properties.unwrap();
    }

    fn set_crb_index(&mut self, n: usize) {
        if let Some(crb) = &mut self.cache.crb {
            crb.cache_index = n;
        }
    }

    fn set_c(&self, c: &mut DVector<f64>) {
        let index = self.cache.crb.unwrap().cache_index;
        let tau = Vector1::new(self.cache.tau);
        c.set_row(index, &tau);
    }

    fn set_h(&self, h: &mut DMatrix<f64>) {
        let crb = &self.cache.crb.unwrap();
        let index = crb.cache_index;
        let ic = &crb.ic;

        let mut view = h.fixed_view_mut::<1, 1>(index, index);
        view[0] = ic.matrix()[0];
    }
}
