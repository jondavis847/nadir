use crate::{
    algorithms::articulated_body_algorithm::{AbaCache, ArticulatedBodyAlgorithm},
    body::{Body, BodyTrait},
    joint::{
        Connection, JointCommon, JointErrors, JointParameters, JointSimTrait, JointTrait,
        JointTransforms,
    },
    MultibodyTrait,
};
use linear_algebra::{matrix6x1::Matrix6x1, vector6::Vector6};
use spatial_algebra::{Acceleration, Force, SpatialInertia, SpatialTransform, Velocity};
use transforms::Transform;
use uuid::Uuid;

pub enum RevoluteErrors {}

#[derive(Debug, Default, Clone, Copy)]
pub struct RevoluteState {
    theta: f64,
    omega: f64,
    q_ddot: f64,
}

impl RevoluteState {
    pub fn new(theta: f64, omega: f64) -> Self {
        // assume this is about Z until we add more axes
        let q_ddot = 0.0;
        Self {
            theta,
            omega,
            q_ddot,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Revolute {
    common: JointCommon,
    parameters: JointParameters,
    state: RevoluteState,
    aba: RevoluteAbaCache,
}

impl Revolute {
    pub fn new(name: &str, parameters: JointParameters, state: RevoluteState) -> Self {
        let common = JointCommon::new(name);
        let aba = RevoluteAbaCache::default();

        Self {
            common,
            parameters,
            state,
            aba,
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
        let body_mass_properties = body.get_mass_properties();
        let spatial_transform = SpatialTransform::from(transform);
        let spatial_inertia = SpatialInertia::from(*body_mass_properties);
        let joint_mass_properties = spatial_transform * spatial_inertia;
        self.parameters.mass_properties = Some(joint_mass_properties);
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

#[derive(Clone, Copy, Debug, Default)]
struct RevoluteAbaCache {
    common: AbaCache,
    lil_u: f64,
    big_d_inv: f64,
    big_u: Matrix6x1,    
}

#[derive(Clone, Copy, Debug, Default)]
pub struct RevoluteSim {
    aba: RevoluteAbaCache,
    parameters: JointParameters,    
    state: RevoluteState,
    transforms: JointTransforms,
}

impl RevoluteSim {
    pub fn set_state(&mut self, state: &RevoluteState) {
        self.state = *state;
    }

    pub fn get_state(&self) -> &RevoluteState {
        &self.state
    }
}

impl From<Revolute> for RevoluteSim {
    fn from(revolute: Revolute) -> Self {
        RevoluteSim {
            aba: RevoluteAbaCache::default(),
            parameters: revolute.parameters,
            state: revolute.state,
            transforms: JointTransforms::default(),
        }
    }
}

impl ArticulatedBodyAlgorithm for RevoluteSim {
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
        aba.big_u = inertia_articulated_matrix.get_column(3).unwrap();
        aba.big_d_inv = 1.0 / aba.big_u.e31;
        aba.lil_u = -(aba.common.p_big_a.get_index(3).unwrap());

        if !inner_is_base {
            let big_u_times_big_d_inv = aba.big_u * aba.big_d_inv;
            let i_lil_a = SpatialInertia(
                inertia_articulated_matrix - big_u_times_big_d_inv * aba.big_u.transpose(),
            );            
            aba.common.p_lil_a = aba.common.p_big_a
                + Force::from(i_lil_a * aba.common.c)
                + Force::from(big_u_times_big_d_inv * aba.lil_u);

            let parent_inertia_articulated_contribution = self.transforms.ij_jof_from_jof * i_lil_a;

            let parent_p_big_a = self.transforms.ij_jof_from_jof * aba.common.p_big_a;
            Some((parent_inertia_articulated_contribution, parent_p_big_a))
        } else {
            None
        }
    }

    fn third_pass(&mut self, a_ij: Acceleration) {
        let aba = &mut self.aba;

        aba.common.a_prime = self.transforms.jof_from_ij_jof * a_ij + aba.common.c;
        self.state.q_ddot =
            aba.big_d_inv * (aba.lil_u - aba.big_u.transpose() * aba.common.a_prime.vector());
        aba.common.a = aba.common.a_prime
            + Acceleration::from(Vector6::new(0.0, 0.0, self.state.q_ddot, 0.0, 0.0, 0.0));
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

impl JointSimTrait for RevoluteSim {
    fn get_transforms(&self) -> &JointTransforms {
        &self.transforms
    }

    fn get_transforms_mut(&mut self) -> &mut JointTransforms {
        &mut self.transforms
    }
    fn update_transforms(&mut self, ij_transforms: Option<(SpatialTransform, SpatialTransform)>) {
        self.transforms.update(ij_transforms)
    }
}
