use crate::{
    algorithms::articulated_body_algorithm::{AbaCache, ArticulatedBodyAlgorithm},
    body::{body_enum::BodyEnum, BodyTrait},
    joint::{
        Connection, JointCommon, JointEnum, JointErrors, JointParameters, JointTrait,
        JointTransforms,
    },
    MultibodyTrait,
};
use linear_algebra::{matrix6x1::Matrix6x1, vector6::Vector6};
use spatial_algebra::{Acceleration, Force, SpatialInertia, Velocity};
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

#[derive(Debug, Default, Clone, Copy)]

struct RevoluteRk4Cache([RevoluteState; 4]);

#[derive(Debug, Clone)]
pub struct Revolute {
    common: JointCommon,
    parameters: JointParameters,
    state: RevoluteState,
    aba: RevoluteAbaCache,
    rk4: RevoluteRk4Cache,
}

impl Revolute {
    pub fn new(name: &str, parameters: JointParameters, state: RevoluteState) -> Revolute {
        let common = JointCommon::new(name);
        let aba = RevoluteAbaCache::default();
        let rk4 = RevoluteRk4Cache::default();

        Self {
            common,
            parameters,
            state,
            aba,
            rk4,
        }
    }
}

impl JointTrait for Revolute {
    fn connect_inner_body(
        &mut self,
        body: &BodyEnum,
        transform: Transform,
    ) -> Result<(), JointErrors> {
        if self.common.connection.inner_body.is_some() {
            return Err(JointErrors::InnerBodyExists);
        }
        let connection = Connection::new(*body.get_id(), transform);
        self.common.connection.inner_body = Some(connection);
        Ok(())
    }

    fn connect_outer_body(
        &mut self,
        body: &BodyEnum,
        transform: Transform,
    ) -> Result<(), JointErrors> {
        if self.common.connection.outer_body.is_some() {
            return Err(JointErrors::OuterBodyExists);
        }
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
    fn get_transforms(&self) -> &JointTransforms {
        &self.common.transforms
    }

    fn update_transforms(&mut self, inner_joint: Option<&JointEnum>) {
        self.common.update_transforms(inner_joint);
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
    q: f64,
    q_dot: f64,
    q_ddot: f64,
    inertia_lil_a: f64,
}

pub struct RevoluteSim {
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
            parameters: revolute.parameters,
            state: revolute.state,
            transforms: JointTransforms::default(),
        }
    }
}
