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
use coordinate_systems::cartesian::Cartesian;
use nalgebra::{DMatrix, DVector, Matrix4x3, Matrix6, Vector3, Vector6};
use rotations::{euler_angles::EulerAngles, quaternion::Quaternion, RotationTrait};
use serde::{Deserialize, Serialize};
use spatial_algebra::{Acceleration, Force, SpatialInertia, SpatialTransform, Velocity};
use std::ops::{AddAssign, MulAssign};
use transforms::Transform;
use uuid::Uuid;

#[derive(Debug, Copy, Clone)]
pub enum FloatingErrors {}

/// IMPORTANT: State values are in the JOF
/// This just makes sense for applying angular quantities
/// translation frame is tied to rotation frame due to the use of spatial vectors (since they both live in the same vector)
/// i.e. r (position) is the JIFs position in the JOF!
#[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
pub struct FloatingState {
    pub q: Quaternion,
    pub w: Vector3<f64>,
    pub r: Vector3<f64>,
    pub v: Vector3<f64>,
}

impl FloatingState {
    pub fn new(q: Quaternion, w: Vector3<f64>, r: Vector3<f64>, v: Vector3<f64>) -> Self {
        // assume this is about Z until we add more axes
        Self { q, w, r, v }
    }
}

impl<'a> AddAssign<&'a Self> for FloatingState {
    fn add_assign(&mut self, rhs: &'a Self) {
        self.q += &rhs.q; //note this should only be used for adding quaternion derivatives in an ODE
        self.w += &rhs.w;
        self.r += &rhs.r;
        self.v += &rhs.v;
    }
}

impl MulAssign<f64> for FloatingState {
    fn mul_assign(&mut self, rhs: f64) {
        self.q *= rhs;
        self.w *= rhs;
        self.r *= rhs;
        self.v *= rhs;
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct FloatingResult {
    pub q: Vec<Quaternion>,
    pub w: Vec<Vector3<f64>>,
    pub aa: Vec<Vector3<f64>>,
    pub r: Vec<Vector3<f64>>,
    pub v: Vec<Vector3<f64>>,
    pub a: Vec<Vector3<f64>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Floating {
    pub common: JointCommon,
    pub parameters: [JointParameters; 6],
    pub state: FloatingState,
}

impl Floating {
    pub fn new(name: &str, parameters: [JointParameters; 6], state: FloatingState) -> Self {
        let common = JointCommon::new(name);

        Self {
            common,
            parameters,
            state,
        }
    }
}

impl JointTrait for Floating {
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

impl MultibodyTrait for Floating {
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
struct FloatingAbaCache {
    common: AbaCache,
    lil_u: Vector6<f64>,
    big_d_inv: Matrix6<f64>,
    big_u: Matrix6<f64>,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
struct FloatingCrbCache {
    cache_index: usize,
    ic: SpatialInertia,
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
struct FloatingCache {
    common: JointCache,
    aba: Option<FloatingAbaCache>,
    crb: Option<FloatingCrbCache>,
    q_ddot: Vector6<f64>,
    rne: Option<RneCache>,
    tau: Vector6<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FloatingSim {
    cache: FloatingCache,
    id: Uuid,
    parameters: [JointParameters; 6],
    mass_properties: Option<SpatialInertia>,
    pub result: FloatingResult,
    state: FloatingState,
    transforms: JointTransforms,
}

impl FloatingSim {
    pub const DOF: usize = 6;
}

impl From<Floating> for FloatingSim {
    fn from(floating: Floating) -> Self {
        // update the joints to body transforms
        let mut transforms = JointTransforms::default();
        if let Some(inner_body) = &floating.common.connection.inner_body {
            transforms.jif_from_ib = inner_body.transform.into();
            transforms.ib_from_jif = inner_body.transform.inv().into();
        } else {
            panic!("should always be an inner body connected")
        }

        if let Some(outer_body) = &floating.common.connection.outer_body {
            transforms.jof_from_ob = outer_body.transform.into();
            transforms.ob_from_jof = outer_body.transform.inv().into();
        } else {
            panic!("should always be an inner body connected")
        }

        FloatingSim {
            cache: FloatingCache::default(),
            id: *floating.get_id(),
            mass_properties: floating.common.mass_properties,
            parameters: floating.parameters,
            result: FloatingResult::default(),
            state: floating.state,
            transforms,
        }
    }
}

impl ArticulatedBodyAlgorithm for FloatingSim {
    fn aba_first_pass(&mut self, v_ij: Velocity) {
        let aba = self.cache.aba.as_mut().unwrap();
        let transforms = &self.transforms;
        let joint_inertia = &self.mass_properties.unwrap();
        let v = &mut self.cache.common.v;
        let vj = &self.cache.common.vj;

        *v = transforms.jof_from_ij_jof * v_ij + *vj;
        aba.common.c = v.cross_motion(*vj); // + cj
        aba.common.inertia_articulated = *joint_inertia;
        aba.common.p_big_a = v.cross_force(*joint_inertia * *v) - self.cache.common.f;
    }

    fn aba_second_pass(&mut self, inner_is_base: bool) -> Option<(SpatialInertia, Force)> {
        let aba = self.cache.aba.as_mut().unwrap();
        let inertia_articulated_matrix = aba.common.inertia_articulated.matrix();

        // use the most efficient method for creating these. Indexing is much faster than 6x6 matrix mul
        aba.big_u = inertia_articulated_matrix; // S is just identity for floating joint
        aba.big_d_inv = aba.big_u.try_inverse().unwrap();
        aba.lil_u = self.cache.tau - aba.common.p_big_a.vector(); //note force is 1 indexed, so 1

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
        *q_ddot = aba.big_d_inv * (aba.lil_u - aba.big_u.transpose() * aba.common.a_prime.vector());
        *a = aba.common.a_prime + Acceleration::from(*q_ddot);
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

impl RecursiveNewtonEuler for FloatingSim {
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
            true => *jof_from_ij_jof * a_ij + Acceleration::from(*q_ddot) + v.cross_motion(*vj),
            false => *jof_from_ij_jof * a_ij + v.cross_motion(*vj),
        };

        *a = a_new;

        *f = *joint_inertia * *a + v.cross_force(*joint_inertia * *v) - *f_b;
    }

    fn rne_second_pass(&mut self) {
        self.cache.tau = self.cache.rne.unwrap().f.vector();
    }

    fn rne_add_force(&mut self, force: Force) {
        self.cache.rne.as_mut().unwrap().f = self.cache.rne.as_mut().unwrap().f + force;
    }

    fn rne_get_force(&self) -> Force {
        self.cache.rne.unwrap().f
    }

    fn rne_set_tau(&mut self) {
        self.cache.tau = self.cache.rne.unwrap().f.vector(); //TODO: duplicate of second pass?
    }
}

impl JointSimTrait for FloatingSim {
    fn calculate_tau(&mut self) {
        let p = &self.parameters;
        // this assume tait-bryan euler angle sequence (ZYX)

        let angles = EulerAngles::from(self.state.q);

        self.cache.tau[0] = p[0].constant_force
            - p[0].spring_constant * angles.psi
            - p[0].damping * self.state.w[0];
        self.cache.tau[1] = p[1].constant_force
            - p[1].spring_constant * angles.theta
            - p[1].damping * self.state.w[1];
        self.cache.tau[2] = p[2].constant_force
            - p[2].spring_constant * angles.phi
            - p[2].damping * self.state.w[2];

        // translation uantities are + instead of - since they are expressed in JOF
        // i.e. if the JOF is +1 units in the x direction represented in the JIF frame,
        // then r would be -1 in the JOF frame, and the spring force would be K*r instead of -K*r
        // to get back to a JIF equilibrium of 0
        self.cache.tau[3] = p[3].constant_force
            - p[3].spring_constant * self.state.r[0]
            - p[3].damping * self.state.v[0];
        self.cache.tau[4] = p[4].constant_force
            - p[4].spring_constant * self.state.r[1]
            - p[4].damping * self.state.v[1];
        self.cache.tau[5] = p[5].constant_force
            - p[5].spring_constant * self.state.r[2]
            - p[5].damping * self.state.v[2];
    }

    fn calculate_vj(&mut self) {
        // self.state.velocity is in the JIF frame
        // ABA assumes velocities in the JOF frame so S is constant
        // transform to JOF velocity
        self.cache.common.vj = Velocity::from(Vector6::new(
            self.state.w[0],
            self.state.w[1],
            self.state.w[2],
            self.state.v[0],
            self.state.v[1],
            self.state.v[2],
        ));
    }

    fn get_a_jof(&self) -> &Acceleration {
        &self.cache.common.a
    }

    fn get_derivative(&self) -> JointState {
        // Quaternion is from the body to base, or the body's orientation in the base frame
        //due to quaternion kinematic equations
        let q = self.state.q;
        // Markley eq 3.20 & 2.88
        let tmp = Matrix4x3::new(
            q.s, -q.z, q.y, q.z, q.s, -q.x, -q.y, q.x, q.s, -q.x, -q.y, -q.z,
        );
        let q_dot = Quaternion::from(0.5 * tmp * self.state.w);

        // transform v and jif frame so it makes sense for position translation integration
        let v_jof = self.cache.common.v.translation();
        let v_jif = self.transforms.jif_from_jof.0.rotation.transform(*v_jof);

        let derivative = FloatingState::new(
            q_dot,
            *self.cache.common.a.rotation(),
            v_jif,
            *self.cache.common.a.translation(),
        );
        JointState::Floating(derivative)
    }

    fn get_id(&self) -> &Uuid {
        &self.id
    }
    fn get_inertia(&self) -> SpatialInertia {
        self.mass_properties.unwrap()
    }

    fn get_ndof(&self) -> usize {
        FloatingSim::DOF
    }

    fn get_state(&self) -> JointState {
        JointState::Floating(self.state)
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
        self.result.q.push(self.state.q);
        self.result.w.push(self.state.w);
        self.result.r.push(self.state.r);
        self.result.v.push(self.state.v);
        self.result
            .aa
            .push(self.cache.q_ddot.fixed_rows::<3>(0).clone_owned());
        self.result
            .a
            .push(self.cache.q_ddot.fixed_rows::<3>(3).clone_owned());
    }

    fn set_state(&mut self, state: JointState) {
        if let JointState::Floating(floating_state) = state {
            self.state = floating_state
        }
    }
    fn get_transforms(&self) -> &JointTransforms {
        &self.transforms
    }

    fn get_transforms_mut(&mut self) -> &mut JointTransforms {
        &mut self.transforms
    }
    fn update_transforms(&mut self, ij_transforms: Option<(SpatialTransform, SpatialTransform)>) {
        let rotation = self.state.q;
        let translation = Cartesian::from(self.state.r); // r is already jif to jof
        let transform = Transform::new(rotation.into(), translation.into());

        self.transforms.jof_from_jif = SpatialTransform(transform);
        self.transforms.jif_from_jof = self.transforms.jof_from_jif.inv();
        self.transforms.update(ij_transforms)
    }

    fn with_algorithm(mut self, algorithm: MultibodyAlgorithm) -> Self {
        match algorithm {
            MultibodyAlgorithm::ArticulatedBody => {
                self.cache.aba = Some(FloatingAbaCache::default());
                self.cache.crb = None;
                self.cache.rne = None;
            }
            MultibodyAlgorithm::CompositeRigidBody => {
                self.cache.aba = None;
                self.cache.crb = Some(FloatingCrbCache::default());
                self.cache.rne = Some(RneCache::default());
            }
        }
        self
    }
}

impl CompositeRigidBody for FloatingSim {
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
        let mut my_space = c.fixed_rows_mut::<6>(index);
        my_space.copy_from(&self.cache.tau);
    }

    fn set_h(&self, h: &mut DMatrix<f64>) {
        let crb = &self.cache.crb.unwrap();
        let index = crb.cache_index;
        let ic = &crb.ic;

        let mut view = h.fixed_view_mut::<1, 1>(index, index);
        view[0] = ic.matrix()[0];
    }
}
