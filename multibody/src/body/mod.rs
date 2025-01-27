use crate::joint::{Joint, JointRef};
use celestial::CelestialSystem;
use gravity::Gravity;

use mass_properties::MassProperties;
use nadir_3d::mesh::Mesh;
use nadir_result::{NadirResult, ResultManager};
use nalgebra::{Vector3, Vector6};
use ron::ser::{to_string_pretty, PrettyConfig};
use rotations::{prelude::UnitQuaternion, RotationTrait};
use serde::{Deserialize, Serialize};
use spatial_algebra::{Force, SpatialTransform};
use std::{
    cell::RefCell,
    fs::File,
    io::Write,
    rc::{Rc, Weak},
};
use thiserror::Error;
use transforms::Transform;

#[derive(Clone, Debug, Error)]
pub enum BodyErrors {
    #[error("name cannot be empty for body")]
    EmptyName,
    #[error("attempted to connect inner joint to body '{0}', but it already has an inner joint")]
    InnerJointExists(String),
    #[error("joint '{0}' already connected to {1} as an outer joint")]
    OuterJointExists(String, String),
}

pub trait BodyTrait {
    fn get_name(&self) -> String;
    fn connect_outer_joint(&mut self, joint: &JointRef) -> Result<(), BodyErrors>;
}

pub type BodyRef = Rc<RefCell<Body>>;
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Body {
    #[serde(skip)]
    pub inner_joint: Option<Weak<RefCell<Joint>>>,
    pub mass_properties: MassProperties,
    pub mesh: Option<Mesh>,
    pub name: String,
    #[serde(skip)]
    pub outer_joints: Vec<Weak<RefCell<Joint>>>, // id of joint in system.joints, joint contains the transform information
    #[serde(skip)]
    pub state: BodyState,
    result_id: Option<u32>,
}

impl Body {
    pub fn connect_inner_joint(&mut self, joint: &JointRef) -> Result<(), BodyErrors> {
        match self.inner_joint {
            Some(_) => return Err(BodyErrors::InnerJointExists(self.name.to_string())),
            None => self.inner_joint = Some(Rc::downgrade(&joint.clone())),
        }
        Ok(())
    }

    pub fn new(name: &str, mass_properties: MassProperties) -> Result<Self, BodyErrors> {
        if name.is_empty() {
            return Err(BodyErrors::EmptyName);
        }
        Ok(Self {
            mesh: None,
            inner_joint: None,
            mass_properties: mass_properties,
            name: name.to_string(),
            outer_joints: Vec::new(),
            state: BodyState::default(),
            result_id: None,
        })
    }

    pub fn with_mesh(mut self, mesh: Mesh) -> Self {
        self.mesh = Some(mesh);
        self
    }

    pub fn calculate_gravity(&mut self, body_from_base: &SpatialTransform, gravity: &mut Gravity) {
        let g_vec = gravity.calculate(&self.state.position_base).unwrap();

        // convert g_vec to a force by multiplying by mass
        // note that we just calculate gravity as translation of the cm
        // any torque applied via gravity and it's joints is handled by
        // the conversion to a joint force through spatial algebra
        let g_vec = g_vec * self.mass_properties.mass;

        self.state.gravity_force_base = g_vec;
        // transform to the body frame
        self.state.gravity_force_body = body_from_base.0.rotation.transform(g_vec);
    }

    //TODO: combine this with calculate_gravity?
    pub fn calculate_gravity_celestial(
        &mut self,
        body_from_base: &SpatialTransform,
        celestial: &mut CelestialSystem,
    ) {
        let g_vec = celestial.calculate_gravity(&self.state.position_base);

        // convert g_vec to a force by multiplying by mass
        // note that we just calculate gravity as translation of the cm
        // any torque applied via gravity and it's joints is handled by
        // the conversion to a joint force through spatial algebra
        let g_vec = g_vec * self.mass_properties.mass;

        self.state.gravity_force_base = g_vec;
        // transform to the body frame
        self.state.gravity_force_body = body_from_base.0.rotation.transform(g_vec);
    }

    pub fn calculate_external_force(&mut self) {
        // convert gravity to spatial force
        let gravity_force_body = Force::from(Vector6::new(
            0.0,
            0.0,
            0.0,
            self.state.gravity_force_body[0],
            self.state.gravity_force_body[1],
            self.state.gravity_force_body[2],
        ));

        // sum the forces
        self.state.external_spatial_force_body = gravity_force_body
            + self.state.actuator_force_body
            + self.state.environments_force_body;

        // populate values for reporting
        self.state.external_force_body = *self.state.external_spatial_force_body.translation();
        self.state.external_torque_body = *self.state.external_spatial_force_body.rotation();
    }

    pub fn update_acceleration(&mut self) {
        let inner_joint_weak = self.inner_joint.as_ref().unwrap();
        let inner_joint_rc = inner_joint_weak.upgrade().unwrap();
        let inner_joint = inner_joint_rc.borrow();

        let transforms = &inner_joint.cache.transforms;
        let body_from_joint = transforms.ob_from_jof;
        let base_from_body = &(transforms.base_from_jof * transforms.jof_from_ob)
            .0
            .rotation;
        let joint_a = inner_joint.cache.a;
        let body_a = body_from_joint * joint_a;
        self.state.acceleration_body = *body_a.translation();
        self.state.angular_accel_body = *body_a.rotation();
        // need to apply kinematic transport theorem to get acceleration of the body in the base
        // r is zero since the position of the frame in the frame is just 0
        // v however is non zero in the body frame
        // using transport theorem, a_base = R_base_from_body * (a_body + w_body x v_body)
        self.state.acceleration_base = base_from_body.transform(
            body_a.translation()
                + self
                    .state
                    .angular_rate_body
                    .cross(&self.state.velocity_body),
        );
    }

    pub fn update_state(&mut self) {
        let inner_joint_weak = self.inner_joint.as_ref().unwrap();
        let inner_joint_rc = inner_joint_weak.upgrade().unwrap();
        let inner_joint = inner_joint_rc.borrow();

        let transforms = &inner_joint.cache.transforms;
        let body_from_joint = transforms.ob_from_jof;
        let base_from_body = transforms.base_from_jof * transforms.jof_from_ob;
        let joint_v = inner_joint.cache.v;
        let body_v = body_from_joint * joint_v;
        // need to apply kinematic transport theorem to get velocity of the body in the base
        // r is technically 0 since the body is coincident iwth its own frame
        // there would be a non-zero r if we were looking for motion of body frame w.r.t jof, but that motion
        // is already accounted for in the spatial algebra when converting from jof to body.
        let body_v_in_base_translation = base_from_body.0.rotation.transform(*body_v.translation());
        self.state.velocity_body = *body_v.translation();
        self.state.velocity_base = body_v_in_base_translation;
        self.state.angular_rate_body = *body_v.rotation();
        let body_from_base = base_from_body.0.inv();
        self.state.position_base = body_from_base.translation.vec();
        self.state.attitude_base = UnitQuaternion::from(&body_from_base.rotation);

        // reset actuator force and environment force to be updated later
        self.state.actuator_force_body = Force::zeros();
        self.state.environments_force_body = Force::zeros();
        self.state.internal_momentum_body = Vector3::zeros();
        //self.state.internal_torque_body = Vector3::zeros();
    }
}

impl NadirResult for Body {
    fn new_result(&mut self, results: &mut ResultManager) {
        let bodies_folder = results.result_path.join("bodies");

        // Check if the folder exists, if not, create it
        if !bodies_folder.exists() {
            std::fs::create_dir_all(&bodies_folder).expect("Failed to create bodies folder");
        }

        let id = results.new_writer(
            &self.name,
            &bodies_folder,
            &[
                "acceleration(base)[x]",
                "acceleration(base)[y]",
                "acceleration(base)[z]",
                "acceleration(body)[x]",
                "acceleration(body)[y]",
                "acceleration(body)[z]",
                "angular_accel(body)[x]",
                "angular_accel(body)[y]",
                "angular_accel(body)[z]",
                "angular_rate(body)[x]",
                "angular_rate(body)[y]",
                "angular_rate(body)[z]",
                "attitude(base)[x]",
                "attitude(base)[y]",
                "attitude(base)[z]",
                "attitude(base)[w]",
                "external_force(body)[x]",
                "external_force(body)[y]",
                "external_force(body)[z]",
                "external_torque(body)[x]",
                "external_torque(body)[y]",
                "external_torque(body)[z]",
                "position(base)[x]",
                "position(base)[y]",
                "position(base)[z]",
                "velocity(base)[x]",
                "velocity(base)[y]",
                "velocity(base)[z]",
                "velocity(body)[x]",
                "velocity(body)[y]",
                "velocity(body)[z]",
                "actuator_torque(body)[x]",
                "actuator_torque(body)[y]",
                "actuator_torque(body)[z]",
                "actuator_force(body)[x]",
                "actuator_force(body)[y]",
                "actuator_force(body)[z]",
            ],
        );
        self.result_id = Some(id);

        // also need to write the meshes for animation
        if let Some(mesh) = &self.mesh {
            let mesh_file_path = bodies_folder.join(self.name.clone() + ".mesh");
            let mut mesh_file = File::create(mesh_file_path).expect("could not create file");
            let ron_string = to_string_pretty(mesh, PrettyConfig::default()).unwrap();
            mesh_file.write_all(ron_string.as_bytes()).unwrap();
        }
    }
    fn write_result(&self, results: &mut ResultManager) {
        if let Some(id) = self.result_id {
            results.write_record(
                id,
                &[
                    self.state.acceleration_base[0].to_string(),
                    self.state.acceleration_base[1].to_string(),
                    self.state.acceleration_base[2].to_string(),
                    self.state.acceleration_body[0].to_string(),
                    self.state.acceleration_body[1].to_string(),
                    self.state.acceleration_body[2].to_string(),
                    self.state.angular_accel_body[0].to_string(),
                    self.state.angular_accel_body[1].to_string(),
                    self.state.angular_accel_body[2].to_string(),
                    self.state.angular_rate_body[0].to_string(),
                    self.state.angular_rate_body[1].to_string(),
                    self.state.angular_rate_body[2].to_string(),
                    self.state.attitude_base.x.to_string(),
                    self.state.attitude_base.y.to_string(),
                    self.state.attitude_base.z.to_string(),
                    self.state.attitude_base.w.to_string(),
                    self.state.external_force_body[0].to_string(),
                    self.state.external_force_body[1].to_string(),
                    self.state.external_force_body[2].to_string(),
                    self.state.external_torque_body[0].to_string(),
                    self.state.external_torque_body[1].to_string(),
                    self.state.external_torque_body[2].to_string(),
                    self.state.position_base[0].to_string(),
                    self.state.position_base[1].to_string(),
                    self.state.position_base[2].to_string(),
                    self.state.velocity_base[0].to_string(),
                    self.state.velocity_base[1].to_string(),
                    self.state.velocity_base[2].to_string(),
                    self.state.velocity_body[0].to_string(),
                    self.state.velocity_body[1].to_string(),
                    self.state.velocity_body[2].to_string(),
                    self.state.actuator_force_body.rotation()[0].to_string(),
                    self.state.actuator_force_body.rotation()[1].to_string(),
                    self.state.actuator_force_body.rotation()[2].to_string(),
                    self.state.actuator_force_body.translation()[0].to_string(),
                    self.state.actuator_force_body.translation()[1].to_string(),
                    self.state.actuator_force_body.translation()[2].to_string(),
                ],
            );
        }
    }
}

impl BodyTrait for Body {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn connect_outer_joint(&mut self, joint: &JointRef) -> Result<(), BodyErrors> {
        // Check if the joint already exists in outer_joints
        let name = joint.borrow().name.clone();
        for jointref in &self.outer_joints {
            if let Some(joint) = jointref.upgrade() {
                if joint.borrow().name == name {
                    return Err(BodyErrors::OuterJointExists(name, self.name.clone()));
                }
            }
        }

        // Push the new joint connection
        self.outer_joints.push(Rc::downgrade(&joint.clone()));
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct BodyState {
    pub position_base: Vector3<f64>,
    pub velocity_base: Vector3<f64>,
    pub velocity_body: Vector3<f64>,
    pub acceleration_base: Vector3<f64>,
    pub acceleration_body: Vector3<f64>,
    pub attitude_base: UnitQuaternion,
    pub angular_rate_body: Vector3<f64>,
    pub angular_accel_body: Vector3<f64>,
    pub actuator_force_body: Force,
    pub environments_force_body: Force,
    pub gravity_force_base: Vector3<f64>,
    pub gravity_force_body: Vector3<f64>,
    pub external_spatial_force_body: Force, //used for calculations
    pub external_force_body: Vector3<f64>,
    pub external_torque_body: Vector3<f64>,
    //pub internal_torque_body: Vector3<f64>,
    pub internal_momentum_body: Vector3<f64>,
    pub angular_momentum_body: Vector3<f64>,
    pub angular_momentum_system_body: Vector3<f64>,
    pub angular_momentum_base: Vector3<f64>,
    pub angular_momentum_system_base: Vector3<f64>,
    pub linear_momentum_body: Vector3<f64>,
    pub linear_momentum_base: Vector3<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BodyConnection {
    pub body: BodyRef,
    pub transform: Transform,
}
impl BodyConnection {
    pub fn new(body: BodyRef, transform: Transform) -> Self {
        Self { body, transform }
    }
}
