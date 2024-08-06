use crate::{
    algorithms::articulated_body_algorithm::{AbaCache, ArticulatedBodyAlgorithm},
    body::{Body, BodyTrait},
    joint::{
        Connection, JointCommon, JointConnection, JointErrors, JointParameters, JointSimTrait,
        JointState, JointTrait, JointTransforms,
    },
    MultibodyTrait,
};
use coordinate_systems::{cartesian::Cartesian, CoordinateSystem};
use nalgebra::{Matrix6x1, Vector6};
use rotations::{Rotation, RotationTrait};
use spatial_algebra::{Acceleration, Force, SpatialInertia, SpatialTransform, Velocity};
use std::ops::{Add, AddAssign, Div, Mul};
use transforms::Transform;
use uuid::Uuid;
use serde::{Serialize, Deserialize};

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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Prismatic {
    pub common: JointCommon,
    pub parameters: JointParameters,
    pub state: PrismaticState,
}

impl Prismatic {
    pub fn new(name: &str, parameters: JointParameters, state: PrismaticState) -> Self {
        let common = JointCommon::new(name);

        Self {
            common,
            parameters,
            state,
        }
    }
}

impl JointTrait for Prismatic {
    fn connect_inner_body<T: BodyTrait>(
        &mut self,
        body: &mut T,
        transform: Transform,
    ) -> Result<(), JointErrors> {
        if self.common.connection.inner_body.is_some() {
            return Err(JointErrors::InnerBodyExists);
        }
        body.connect_outer_joint(self).unwrap();
        let connection = Connection::new(*body.get_id(), transform);
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
        // we now only do this when we sim incase the transform gets updated
        //let body_mass_properties = body.get_mass_properties();
        //let spatial_transform = SpatialTransform::from(transform);
        //let spatial_inertia = SpatialInertia::from(*body_mass_properties);
        //let joint_mass_properties = spatial_transform * spatial_inertia;
        //self.parameters.mass_properties = Some(joint_mass_properties);
        body.connect_inner_joint(self).unwrap();
        let connection = Connection::new(*body.get_id(), transform);
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
        self.parameters.mass_properties = None;
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

impl MultibodyTrait for Prismatic {
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

#[derive(Clone, Copy, Debug, Default)]
struct PrismaticAbaCache {
    common: AbaCache,
    lil_u: f64,
    big_d_inv: f64,
    big_u: Matrix6x1<f64>,
    q_ddot: f64,
    tau: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PrismaticSim {
    aba: PrismaticAbaCache,
    id: Uuid,
    parameters: JointParameters,
    state: PrismaticState,
    transforms: JointTransforms,
}

impl From<Prismatic> for PrismaticSim {
    fn from(prismatic: Prismatic) -> Self {
        // update the joints to body transforms
        let mut transforms = JointTransforms::default();
        if let Some(inner_body) = &prismatic.common.connection.inner_body {
            transforms.jif_from_ib = inner_body.transform.into();
            transforms.ib_from_jif = inner_body.transform.inv().into();
        } else {
            panic!("should always be an inner body connected")
        }

        if let Some(outer_body) = &prismatic.common.connection.outer_body {
            transforms.jof_from_ob = outer_body.transform.into();
            transforms.ob_from_jof = outer_body.transform.inv().into();
        } else {
            panic!("should always be an inner body connected")
        }

        PrismaticSim {
            aba: PrismaticAbaCache::default(),
            id: *prismatic.get_id(),
            parameters: prismatic.parameters,
            state: prismatic.state,
            transforms: JointTransforms::default(),
        }
    }
}

impl ArticulatedBodyAlgorithm for PrismaticSim {
    fn first_pass(&mut self, v_ij: Velocity, f_ob: &Force) {
        let transforms = &self.transforms;
        let aba = &mut self.aba;
        let joint_inertia = self.parameters.mass_properties.unwrap();

        aba.common.v = transforms.jof_from_ij_jof * v_ij + aba.common.vj;
        aba.common.c = aba.common.v.cross_motion(aba.common.vj); // + cj
        aba.common.inertia_articulated = joint_inertia;
        aba.common.p_big_a =
            aba.common.v.cross_force(joint_inertia * aba.common.v) - transforms.jof_from_ob * *f_ob;
    }

    fn second_pass(&mut self, inner_is_base: bool) -> Option<(SpatialInertia, Force)> {
        let aba = &mut self.aba;
        let inertia_articulated_matrix = aba.common.inertia_articulated.matrix();

        // use the most efficient method for creating these. Indexing is much faster than 6x6 matrix mul
        // assum prismatic is in x for now
        aba.big_u = inertia_articulated_matrix.column(3).into();
        aba.big_d_inv = 1.0 / aba.big_u[3];
        aba.lil_u = aba.tau - (aba.common.p_big_a.get_index(4).unwrap()); //note force is 1 indexed, so 4
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

    fn third_pass(&mut self, a_ij: Acceleration) {
        let aba = &mut self.aba;

        aba.common.a_prime = self.transforms.jof_from_ij_jof * a_ij + aba.common.c;
        aba.q_ddot =
            aba.big_d_inv * (aba.lil_u - (aba.big_u.transpose() * aba.common.a_prime.vector())[0]);
        aba.common.a = aba.common.a_prime
            + Acceleration::from(Vector6::new(0.0, 0.0, 0.0, aba.q_ddot, 0.0, 0.0));
    }

    fn get_aba_derivative(&self) -> JointState {
        let derivative = PrismaticState::new(self.state.velocity, self.aba.q_ddot);
        JointState::Prismatic(derivative)
    }

    fn get_v(&self) -> &Velocity {
        &self.aba.common.v
    }

    fn get_p_big_a(&self) -> &Force {
        &self.aba.common.p_big_a
    }

    fn get_a(&self) -> &Acceleration {
        &self.aba.common.a
    }

    fn add_inertia_articulated(&mut self, inertia: SpatialInertia) {
        self.aba.common.inertia_articulated = self.aba.common.inertia_articulated + inertia;
    }

    fn add_p_big_a(&mut self, p_big_a: Force) {
        self.aba.common.p_big_a = self.aba.common.p_big_a + p_big_a;
    }
}

impl JointSimTrait for PrismaticSim {
    fn calculate_tau(&mut self) {
        let JointParameters {
            constant_force,
            damping,
            spring_constant,
            ..
        } = self.parameters;
        self.aba.tau =
            constant_force - spring_constant * self.state.position - damping * self.state.velocity;
    }

    fn calculate_vj(&mut self) {
        self.aba.common.vj =
            Velocity::from(Vector6::new(0.0, 0.0, 0.0, self.state.velocity, 0.0, 0.0));
    }
    fn get_id(&self) -> &Uuid {
        &self.id
    }

    fn get_inertia(&self) -> &Option<SpatialInertia> {
        &self.parameters.mass_properties
    }
    fn get_state(&self) -> JointState {
        JointState::Prismatic(self.state)
    }
    fn set_state(&mut self, state: JointState) {
        match state {
            JointState::Prismatic(prismatic_state) => self.state = prismatic_state,
            _ => panic!("Can't set a different joints state to Prismatic"),
        }
    }
    fn set_inertia(&mut self, inertia: Option<SpatialInertia>) {
        self.parameters.mass_properties = inertia;
    }
    fn get_transforms(&self) -> &JointTransforms {
        &self.transforms
    }

    fn get_transforms_mut(&mut self) -> &mut JointTransforms {
        &mut self.transforms
    }
    fn update_transforms(&mut self, ij_transforms: Option<(SpatialTransform, SpatialTransform)>) {
        let rotation = Rotation::identity();
        let translation = CoordinateSystem::from(Cartesian::new(self.state.position, 0.0, 0.0));
        let transform = Transform::new(rotation, translation);

        self.transforms.jof_from_jif = SpatialTransform(transform);
        self.transforms.jif_from_jof = self.transforms.jof_from_jif.inv();
        self.transforms.update(ij_transforms)
    }
}

impl Add for PrismaticState {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        PrismaticState {
            position: self.position + rhs.position,
            velocity: self.velocity + rhs.velocity,
        }
    }
}

impl AddAssign for PrismaticState {
    fn add_assign(&mut self, rhs: Self) {
        self.position += rhs.position;
        self.velocity += rhs.velocity;
    }
}

impl Mul<f64> for PrismaticState {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self {
        PrismaticState {
            position: self.position * rhs,
            velocity: self.velocity * rhs,
        }
    }
}

impl Div<f64> for PrismaticState {
    type Output = Self;

    fn div(self, rhs: f64) -> Self {
        PrismaticState {
            position: self.position / rhs,
            velocity: self.velocity / rhs,
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct PrismaticResult {
    pub position: Vec<f64>,
    pub velocity: Vec<f64>,
}
