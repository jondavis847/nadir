use crate::{
    articulated_body_algorithm::{AbaCache, ArticulatedBodyAlgorithm},
    body::{BodyRef, BodyTrait},
    joint::{
        Connection, JointCommon, JointEnum, JointErrors, JointParameters, JointRef, JointTrait,
        JointTransforms,
    },
    MultibodyTrait,
};
use coordinate_systems::CoordinateSystem;
use rotations::euler_angles::{Angles, EulerAngles};
use spatial_algebra::{Acceleration, Force, Velocity, SpatialInertia};
use std::cell::{Ref, RefCell};
use std::rc::Rc;
use transforms::Transform;

pub enum RevoluteErrors {}

#[derive(Debug, Default, Clone, Copy)]
pub struct RevoluteState {
    theta: f64,
    omega: f64,
    transform: Transform,
    aba: RevoluteAbaCache,
}

impl RevoluteState {
    pub fn new(theta: f64, omega: f64) -> Self {
        let rotation = EulerAngles::XYZ(Angles::new(0.0, 0.0, theta));
        // assume this is about Z until we add more axes
        let transform = Transform::new(rotation.into(), CoordinateSystem::default());
        let aba = RevoluteAbaCache::default();
        Self {
            theta,
            omega,
            transform,
            aba,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Revolute {
    common: JointCommon,
    parameters: JointParameters,
    state: RevoluteState,
}

impl Revolute {
    pub fn new(name: &str, parameters: JointParameters, state: RevoluteState) -> JointRef {
        let common = JointCommon::new(name);

        Rc::new(RefCell::new(JointEnum::Revolute(Self {
            common,
            parameters,
            state,
        })))
    }
}

impl JointTrait for Revolute {
    fn connect_inner_body(
        &mut self,
        body: BodyRef,
        transform: Transform,
    ) -> Result<(), JointErrors> {
        if self.common.connection.inner_body.is_some() {
            return Err(JointErrors::InnerBodyExists);
        }
        let connection = Connection::new(body, transform);
        self.common.connection.inner_body = Some(connection);
        Ok(())
    }

    fn connect_outer_body(
        &mut self,
        body: BodyRef,
        transform: Transform,
    ) -> Result<(), JointErrors> {
        if self.common.connection.outer_body.is_some() {
            return Err(JointErrors::OuterBodyExists);
        }
        let connection = Connection::new(body.clone(), transform);
        self.common.connection.outer_body = Some(connection);
        self.common.mass_properties = Some(SpatialInertia(transform * body.borrow().get_mass_properties()));
        Ok(())
    }

    fn delete_inner_body(&mut self) {
        if self.common.connection.inner_body.is_some() {
            self.common.connection.inner_body = None;
        }
    }

    fn delete_outer_body(&mut self) {
        if self.common.connection.outer_body.is_some() {
            self.common.connection.outer_body = None;            
        }
        self.common.mass_properties = None;
    }

    fn get_inner_body(&self) -> Option<Connection> {
        self.common.connection.inner_body.clone()
    }
    fn get_outer_body(&self) -> Option<Connection> {
        self.common.connection.outer_body.clone()
    }
    fn get_transforms(&self) -> JointTransforms {
        self.common.transforms
    }

    fn update_transforms(&mut self) {
        //update the joint transform (transform from joint inner frame(jif) to joint outer frame (jof))
        let rotation = EulerAngles::XYZ(Angles::new(0.0, 0.0, self.state.theta));
        // assume this is about Z until we add more axes
        self.state.transform.rotation = rotation.into();

        //update all of the transforms now that we have updated the joint transform
        self.common.update_transforms();
    }
}

impl MultibodyTrait for Revolute {
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
    u: f64,
    D: f64,
    U: f64,
    q: f64,
    q_dot: f64,
    q_ddot: f64,
}

impl ArticulatedBodyAlgorithm for Revolute {
    fn first_pass(&mut self) {
        let parent_ref = self.common.get_inner_joint();
        let parent = parent_ref.borrow();
        let transforms = &self.common.transforms;
        let body = self.get_outer_body().unwrap().body;

        let aba = &mut self.state.aba.common;

        aba.v = transforms.jof_from_ij_jof * parent.get_v() + aba.vj;
        aba.c = aba.v.cross_motion(aba.vj); // + cj
        
        let joint_mass_properties = self.common.mass_properties.unwrap();

        aba.inertia_articulated = joint_mass_properties;
        aba.p_big_a = aba.v.cross_force(joint_mass_properties * aba.v) - transforms.jof_from_ob * body.get_external_force();
    }
    fn second_pass(&mut self) {}
    fn third_pass(&mut self) {}

    fn get_v(&self) -> Velocity {
        self.state.aba.common.v
    }

    fn get_p_big_a(&self) -> Force {
        self.state.aba.common.p_big_a
    }

    fn get_a(&self) -> Acceleration {
        self.state.aba.common.a
    }
}
