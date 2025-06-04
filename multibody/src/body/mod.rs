use crate::{
    base::{Base, BaseRef},
    joint::{Joint, JointBuilder, JointErrors, JointRef},
    system::Id,
};
use celestial::CelestialSystem;
use color::Color;
use gravity::Gravity;

use mass_properties::{MassProperties, MassPropertiesBuilder, MassPropertiesErrors};
use nadir_3d::{
    geometry::{
        Geometry,
        cuboid::{Cuboid, CuboidErrors},
        ellipsoid::{Ellipsoid16, Ellipsoid32, Ellipsoid64, EllipsoidErrors},
    },
    material::Material,
    mesh::Mesh,
};
use nadir_diffeq::saving::{WriterId, WriterManager};
use nadir_result::{NadirResult, ResultManager};
use nalgebra::{Vector3, Vector6};
use rand::rngs::SmallRng;
use ron::ser::{PrettyConfig, to_string_pretty};
use rotations::{RotationTrait, prelude::UnitQuaternion};
use serde::{Deserialize, Serialize};
use spatial_algebra::Force;
use std::{
    cell::RefCell,
    fs::File,
    io::Write,
    rc::{Rc, Weak},
};
use thiserror::Error;
use transforms::Transform;
use uncertainty::Uncertainty;

#[derive(Debug, Error)]
pub enum BodyErrors {
    #[error("{0}")]
    Cuboid(#[from] CuboidErrors),
    #[error("{0}")]
    Ellipsoid(#[from] EllipsoidErrors),
    #[error("name cannot be empty for body")]
    EmptyName,
    #[error("attempted to connect inner joint to body '{0}', but it already has an inner joint")]
    InnerJointExists(String),
    #[error("{0}")]
    Joint(#[from] JointErrors),
    #[error("no inner joint found for body '{0}'")]
    NoInnerJoint(String),
    #[error("no mass properties found for body '{0}'")]
    NoMassProperties(String),
    #[error("joint '{0}' already connected to {1} as an outer joint")]
    OuterJointExists(String, String),
    #[error("{0}")]
    MassPropertiesError(#[from] MassPropertiesErrors),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BodyBuilder {
    pub actuators: Vec<Id>,
    pub id: Id,
    pub inner_joint: Option<Id>,
    pub mass_properties: Option<MassPropertiesBuilder>,
    pub mesh: Option<Mesh>,
    pub name: String,
    pub outer_joints: Vec<Id>, // id of joint in system.joints, joint contains the transform information
    pub sensors: Vec<Id>,
}

impl BodyBuilder {
    pub fn new(name: &str, id: Id) -> Result<Self, BodyErrors> {
        if name.is_empty() {
            return Err(BodyErrors::EmptyName);
        }
        Ok(Self {
            actuators: Vec::new(),
            id,
            mesh: None,
            inner_joint: None,
            mass_properties: None,
            name: name.to_string(),
            outer_joints: Vec::new(),
            sensors: Vec::new(),
        })
    }

    pub fn connect_inner_joint(
        &mut self,
        inner_joint: &mut JointBuilder,
        transform: Transform,
    ) -> Result<(), BodyErrors> {
        if self.inner_joint.is_some() {
            return Err(BodyErrors::InnerJointExists(self.name.clone()));
        }
        if inner_joint.connections.outer_body.is_some() {
            return Err(JointErrors::OuterBodyExists(inner_joint.name.clone()).into());
        }
        self.inner_joint = Some(inner_joint.id);
        inner_joint.connections.outer_body = Some(BodyConnectionBuilder::new(self.id, transform));
        Ok(())
    }

    pub fn connect_outer_joint(
        &mut self,
        outer_joint: &mut JointBuilder,
        transform: Transform,
    ) -> Result<(), BodyErrors> {
        if self.outer_joints.contains(&outer_joint.id) {
            return Err(BodyErrors::OuterJointExists(
                self.name.clone(),
                outer_joint.name.clone(),
            ));
        }
        if outer_joint.connections.inner_body.is_some() {
            return Err(JointErrors::InnerBodyExists(outer_joint.name.clone()).into());
        }
        self.outer_joints.push(outer_joint.id);
        outer_joint.connections.inner_body = Some(BodyConnectionBuilder::new(self.id, transform));
        Ok(())
    }

    pub fn sample(
        &self,
        inner_joint: JointRef,
        nominal: bool,
        rng: &mut SmallRng,
    ) -> Result<Body, BodyErrors> {
        let mass_properties = if let Some(mp_builder) = &self.mass_properties {
            mp_builder.sample(nominal, rng)?
        } else {
            return Err(BodyErrors::NoMassProperties(self.name.clone()));
        };

        let body = Body {
            inner_joint: Rc::downgrade(&inner_joint),
            mass_properties,
            mesh: self.mesh.clone(),
            name: self.name.clone(),
            outer_joints: Vec::new(),
            state: BodyState::default(),
            result_id: None,
        };
        Ok(body)
    }

    /// Setter method for adding an optional 3d mesh
    pub fn set_mesh(&mut self, mesh: Mesh) {
        self.mesh = Some(mesh);
    }

    pub fn set_geometry_cuboid(&mut self, x: f64, y: f64, z: f64) -> Result<(), BodyErrors> {
        let geometry = Geometry::Cuboid(Cuboid::new(x, y, z)?);
        if let Some(mesh) = &mut self.mesh {
            mesh.geometry = geometry;
        } else {
            let mut mesh = Mesh::new(&self.name);
            mesh.geometry = geometry;
            self.mesh = Some(mesh);
        }
        Ok(())
    }

    pub fn set_geometry_ellipsoid16(&mut self, x: f64, y: f64, z: f64) -> Result<(), BodyErrors> {
        let geometry = Geometry::Ellipsoid16(Ellipsoid16::new(x, y, z)?);
        if let Some(mesh) = &mut self.mesh {
            mesh.geometry = geometry;
        } else {
            let mut mesh = Mesh::new(&self.name);
            mesh.geometry = geometry;
            self.mesh = Some(mesh);
        }
        Ok(())
    }

    pub fn set_geometry_ellipsoid32(&mut self, x: f64, y: f64, z: f64) -> Result<(), BodyErrors> {
        let geometry = Geometry::Ellipsoid32(Ellipsoid32::new(x, y, z)?);
        if let Some(mesh) = &mut self.mesh {
            mesh.geometry = geometry;
        } else {
            let mut mesh = Mesh::new(&self.name);
            mesh.geometry = geometry;
            self.mesh = Some(mesh);
        }
        Ok(())
    }

    pub fn set_geometry_ellipsoid64(&mut self, x: f64, y: f64, z: f64) -> Result<(), BodyErrors> {
        let geometry = Geometry::Ellipsoid64(Ellipsoid64::new(x, y, z)?);
        if let Some(mesh) = &mut self.mesh {
            mesh.geometry = geometry;
        } else {
            let mut mesh = Mesh::new(&self.name);
            mesh.geometry = geometry;
            self.mesh = Some(mesh);
        }
        Ok(())
    }

    pub fn set_material_basic(&mut self, color: Color) {
        if let Some(mesh) = &mut self.mesh {
            mesh.material = Material::Basic { color };
        } else {
            let mut mesh = Mesh::new(&self.name);
            mesh.material = Material::Basic { color };
            self.mesh = Some(mesh);
        }
    }

    pub fn set_material_phong(&mut self, color: Color, specular_power: f32) {
        if let Some(mesh) = &mut self.mesh {
            mesh.material = Material::Phong {
                color,
                specular_power,
            };
        } else {
            let mut mesh = Mesh::new(&self.name);
            mesh.material = Material::Phong {
                color,
                specular_power,
            };
            self.mesh = Some(mesh);
        }
    }

    /// Setter method for mass properties
    pub fn set_mass_properties(&mut self, mass_properties: MassPropertiesBuilder) {
        self.mass_properties = Some(mass_properties);
    }
}

#[derive(Debug, Clone)]
pub struct Body {
    pub inner_joint: Weak<RefCell<Joint>>,
    pub mass_properties: MassProperties,
    pub mesh: Option<Mesh>,
    pub name: String,
    pub outer_joints: Vec<Weak<RefCell<Joint>>>,
    pub state: BodyState,
    writer_id: Option<WriterId>,
}

impl Body {
    pub fn calculate_gravity(&mut self, gravity: &mut Gravity) {
        // get inner joint transforms
        let inner_joint = self
            .inner_joint
            .upgrade()
            .expect("validation should catch this");
        let inner_joint = inner_joint.borrow();
        let body_from_base = &inner_joint.cache.transforms.ob_from_base;
        let g_vec = gravity.calculate(&self.state.position_base).unwrap();

        // convert g_vec to a force by multiplying by mass
        // note that we just calculate gravity as translation of the cm
        // any torque applied via gravity and it's joints is handled by
        // the conversion to a joint force through spatial algebra
        let g_vec = g_vec * self.mass_properties.mass;

        self.state.gravity_force_base = g_vec;
        // transform to the body frame
        self.state.gravity_force_body = body_from_base.0.rotation.transform(&g_vec);
    }

    //TODO: combine this with calculate_gravity?
    pub fn calculate_gravity_celestial(&mut self, celestial: &mut CelestialSystem) {
        let g_vec = celestial.calculate_gravity(&self.state.position_base);

        // convert g_vec to a force by multiplying by mass
        // note that we just calculate gravity as translation of the cm
        // any torque applied via gravity and it's joints is handled by
        // the conversion to a joint force through spatial algebra
        let g_vec = g_vec * self.mass_properties.mass;

        self.state.gravity_force_base = g_vec;
        // transform to the body frame
        let q = self.state.attitude_base;
        self.state.gravity_force_body = q.transform(&g_vec);
    }

    pub fn calculate_magnetic_field(&mut self, celestial: &mut CelestialSystem) {
        let b_vec = celestial.calculate_magnetic_field(&self.state.position_base);
        self.state.magnetic_field_base = b_vec;
        // transform to the body frame
        let q = &self.state.attitude_base;
        self.state.magnetic_field_body = q.transform(&b_vec);
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

        // write values for reporting
        self.state.external_force_body = *self.state.external_spatial_force_body.translation();
        self.state.external_torque_body = *self.state.external_spatial_force_body.rotation();
    }

    pub fn update_acceleration(&mut self) {
        let inner_joint = self
            .inner_joint
            .upgrade()
            .expect("validation should catch this");
        let inner_joint = inner_joint.borrow();
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
            &(body_a.translation()
                + self
                    .state
                    .angular_rate_body
                    .cross(&self.state.velocity_body)),
        );
    }

    /// Updates the body's state after integration of the joint states
    /// Forces, torques, and accelerations are updated elsewhere
    pub fn update_state(&mut self) {
        let inner_joint = self
            .inner_joint
            .upgrade()
            .expect("validation should catch this");
        let inner_joint = inner_joint.borrow();
        let transforms = &inner_joint.cache.transforms;
        let body_from_joint = transforms.ob_from_jof;
        let base_from_body = transforms.base_from_jof * transforms.jof_from_ob;
        let joint_v = inner_joint.cache.v;
        let body_v = body_from_joint * joint_v;
        // need to apply kinematic transport theorem to get velocity of the body in the base
        // r is technically 0 since the body is coincident with its own frame
        // there would be a non-zero r if we were looking for motion of body frame w.r.t jof, but that motion
        // is already accounted for in the spatial algebra when converting from jof to body.
        let body_v_in_base_translation = base_from_body.0.rotation.transform(body_v.translation());
        self.state.velocity_body = *body_v.translation();
        self.state.velocity_base = body_v_in_base_translation;
        self.state.angular_rate_body = *body_v.rotation();
        let body_from_base = base_from_body.0.inv();
        self.state.position_base = body_from_base.translation.vec();
        self.state.attitude_base = UnitQuaternion::from(&body_from_base.rotation);

        // reset actuator force and environment force to be updated later
        self.state.actuator_force_body *= 0.0;
        self.state.environments_force_body *= 0.0;
        self.state.internal_momentum_body *= 0.0;

        self.state.kinetic_energy = 0.5
            * self.mass_properties.mass
            * self.state.velocity_base.dot(&self.state.velocity_base)
            + 0.5
                * (self.state.angular_rate_body.transpose()
                    * self.mass_properties.inertia()
                    * self.state.angular_rate_body)[0];

        //TODO: calculate potential energy
    }

    pub fn ncols() -> usize {
        Self::headers().len()
    }

    pub fn headers() -> Vec<String> {
        vec![
            "acceleration(base)[x]".into(),
            "acceleration(base)[y]".into(),
            "acceleration(base)[z]".into(),
            "acceleration(body)[x]".into(),
            "acceleration(body)[y]".into(),
            "acceleration(body)[z]".into(),
            "angular_accel(body)[x]".into(),
            "angular_accel(body)[y]".into(),
            "angular_accel(body)[z]".into(),
            "angular_rate(body)[x]".into(),
            "angular_rate(body)[y]".into(),
            "angular_rate(body)[z]".into(),
            "attitude(base)[x]".into(),
            "attitude(base)[y]".into(),
            "attitude(base)[z]".into(),
            "attitude(base)[w]".into(),
            "external_force(body)[x]".into(),
            "external_force(body)[y]".into(),
            "external_force(body)[z]".into(),
            "external_torque(body)[x]".into(),
            "external_torque(body)[y]".into(),
            "external_torque(body)[z]".into(),
            "position(base)[x]".into(),
            "position(base)[y]".into(),
            "position(base)[z]".into(),
            "velocity(base)[x]".into(),
            "velocity(base)[y]".into(),
            "velocity(base)[z]".into(),
            "velocity(body)[x]".into(),
            "velocity(body)[y]".into(),
            "velocity(body)[z]".into(),
            "actuator_torque(body)[x]".into(),
            "actuator_torque(body)[y]".into(),
            "actuator_torque(body)[z]".into(),
            "actuator_force(body)[x]".into(),
            "actuator_force(body)[y]".into(),
            "actuator_force(body)[z]".into(),
            "magnetic_field(base)[x]".into(),
            "magnetic_field(base)[y]".into(),
            "magnetic_field(base)[z]".into(),
            "magnetic_field(body)[x]".into(),
            "magnetic_field(body)[y]".into(),
            "magnetic_field(body)[z]".into(),
            "kinetic_energy".into(),
            "potential_energy".into(),
            "total_energy".into(),
        ]
    }

    pub fn write_record(&self, manager: &mut WriterManager) {
        if let Some(id) = &self.writer_id {
            if let Some(writer) = manager.writers.get_mut(id) {
                writer.float_buffer[0] = self.state.acceleration_base[0];
                writer.float_buffer[1] = self.state.acceleration_base[1];
                writer.float_buffer[2] = self.state.acceleration_base[2];
                writer.float_buffer[3] = self.state.acceleration_body[0];
                writer.float_buffer[4] = self.state.acceleration_body[1];
                writer.float_buffer[5] = self.state.acceleration_body[2];
                writer.float_buffer[6] = self.state.angular_accel_body[0];
                writer.float_buffer[7] = self.state.angular_accel_body[1];
                writer.float_buffer[8] = self.state.angular_accel_body[2];
                writer.float_buffer[9] = self.state.angular_rate_body[0];
                writer.float_buffer[10] = self.state.angular_rate_body[1];
                writer.float_buffer[11] = self.state.angular_rate_body[2];
                writer.float_buffer[12] = self.state.attitude_base.0.x;
                writer.float_buffer[13] = self.state.attitude_base.0.y;
                writer.float_buffer[14] = self.state.attitude_base.0.z;
                writer.float_buffer[15] = self.state.attitude_base.0.w;
                writer.float_buffer[16] = self.state.external_force_body[0];
                writer.float_buffer[17] = self.state.external_force_body[1];
                writer.float_buffer[18] = self.state.external_force_body[2];
                writer.float_buffer[19] = self.state.external_torque_body[0];
                writer.float_buffer[20] = self.state.external_torque_body[1];
                writer.float_buffer[21] = self.state.external_torque_body[2];
                writer.float_buffer[22] = self.state.position_base[0];
                writer.float_buffer[23] = self.state.position_base[1];
                writer.float_buffer[24] = self.state.position_base[2];
                writer.float_buffer[25] = self.state.velocity_base[0];
                writer.float_buffer[26] = self.state.velocity_base[1];
                writer.float_buffer[27] = self.state.velocity_base[2];
                writer.float_buffer[28] = self.state.velocity_body[0];
                writer.float_buffer[29] = self.state.velocity_body[1];
                writer.float_buffer[30] = self.state.velocity_body[2];
                writer.float_buffer[31] = self.state.actuator_force_body.rotation()[0];
                writer.float_buffer[32] = self.state.actuator_force_body.rotation()[1];
                writer.float_buffer[33] = self.state.actuator_force_body.rotation()[2];
                writer.float_buffer[34] = self.state.actuator_force_body.translation()[0];
                writer.float_buffer[35] = self.state.actuator_force_body.translation()[1];
                writer.float_buffer[36] = self.state.actuator_force_body.translation()[2];
                writer.float_buffer[37] = self.state.magnetic_field_base[0];
                writer.float_buffer[38] = self.state.magnetic_field_base[1];
                writer.float_buffer[39] = self.state.magnetic_field_base[2];
                writer.float_buffer[40] = self.state.magnetic_field_body[0];
                writer.float_buffer[41] = self.state.magnetic_field_body[1];
                writer.float_buffer[42] = self.state.magnetic_field_body[2];
                writer.float_buffer[43] = self.state.kinetic_energy;
                writer.float_buffer[44] = self.state.potential_energy;
                writer.float_buffer[45] = self.state.total_energy;
            }
        }
    }

    // also need to write the meshes for animation
    //        if let Some(mesh) = &self.mesh {
    //          let mesh_file_path = bodies_folder.join(self.name.clone() + ".mesh");
    //        let mut mesh_file = File::create(mesh_file_path).expect("could not create file");
    //      let ron_string = to_string_pretty(mesh, PrettyConfig::default()).unwrap();
    //    mesh_file.write_all(ron_string.as_bytes()).unwrap();
    // }
    // }
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
                    self.state.attitude_base.0.x.to_string(),
                    self.state.attitude_base.0.y.to_string(),
                    self.state.attitude_base.0.z.to_string(),
                    self.state.attitude_base.0.w.to_string(),
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
                    self.state.magnetic_field_base[0].to_string(),
                    self.state.magnetic_field_base[1].to_string(),
                    self.state.magnetic_field_base[2].to_string(),
                    self.state.magnetic_field_body[0].to_string(),
                    self.state.magnetic_field_body[1].to_string(),
                    self.state.magnetic_field_body[2].to_string(),
                    self.state.kinetic_energy.to_string(),
                    self.state.potential_energy.to_string(),
                    self.state.total_energy.to_string(),
                ],
            );
        }
    }
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct BodyState {
    pub acceleration_base: Vector3<f64>,
    pub acceleration_body: Vector3<f64>,
    pub actuator_force_body: Force,
    pub angular_accel_body: Vector3<f64>,
    pub angular_rate_body: Vector3<f64>,
    pub angular_momentum_body: Vector3<f64>,
    pub angular_momentum_system_body: Vector3<f64>,
    pub angular_momentum_base: Vector3<f64>,
    pub angular_momentum_system_base: Vector3<f64>,
    pub attitude_base: UnitQuaternion,
    pub environments_force_body: Force,
    pub external_spatial_force_body: Force, //used for calculations
    pub external_force_body: Vector3<f64>,
    pub external_torque_body: Vector3<f64>,
    pub internal_momentum_body: Vector3<f64>,
    pub gravity_force_base: Vector3<f64>,
    pub gravity_force_body: Vector3<f64>,
    pub kinetic_energy: f64,
    pub linear_momentum_body: Vector3<f64>,
    pub linear_momentum_base: Vector3<f64>,
    pub magnetic_field_base: Vector3<f64>,
    pub magnetic_field_body: Vector3<f64>,
    pub position_base: Vector3<f64>,
    pub potential_energy: f64,
    pub total_energy: f64,
    pub velocity_base: Vector3<f64>,
    pub velocity_body: Vector3<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BodyConnectionBuilder {
    pub body_id: Id,
    pub transform: Transform,
}
impl BodyConnectionBuilder {
    pub fn new(body_id: Id, transform: Transform) -> Self {
        Self { body_id, transform }
    }
}

#[derive(Debug, Clone)]
pub struct BodyConnection {
    pub body: BodyRef,
    pub transform: Transform,
}

#[derive(Debug, Clone)]
pub struct BodyRef(BaseOrBody);

impl BodyRef {
    pub fn is_base(&self) -> bool {
        match &self.0 {
            BaseOrBody::Base(_) => true,
            BaseOrBody::Body(_) => false,
        }
    }
    pub fn is_body(&self) -> bool {
        match &self.0 {
            BaseOrBody::Base(_) => false,
            BaseOrBody::Body(_) => true,
        }
    }

    pub fn new(body: Body) -> Self {
        Self(BaseOrBody::Body(Rc::new(RefCell::new(body))))
    }

    pub fn borrow(&self) -> std::cell::Ref<Body> {
        match &self.0 {
            BaseOrBody::Body(body) => body.borrow(),
            BaseOrBody::Base(_) => panic!("tried to borrow a base as a body"),
        }
    }

    pub fn borrow_mut(&self) -> std::cell::RefMut<Body> {
        match &self.0 {
            BaseOrBody::Body(body) => body.borrow_mut(),
            BaseOrBody::Base(_) => panic!("tried to borrow a base as a body"),
        }
    }

    // cloning a Vec<Rc<RefCell<Joint>> should be very cheap to do
    pub fn get_outer_joints(&self) -> Vec<Weak<RefCell<Joint>>> {
        match &self.0 {
            BaseOrBody::Body(body) => body.borrow().outer_joints.clone(),
            BaseOrBody::Base(base) => base.borrow().outer_joints.clone(),
        }
    }
}

#[derive(Debug, Clone)]
pub enum BaseOrBody {
    Base(Rc<RefCell<Base>>),
    Body(Rc<RefCell<Body>>),
}

impl From<BaseRef> for BodyRef {
    fn from(base: BaseRef) -> Self {
        Self(BaseOrBody::Base(base))
    }
}

impl From<Rc<RefCell<Body>>> for BodyRef {
    fn from(body: Rc<RefCell<Body>>) -> Self {
        Self(BaseOrBody::Body(body))
    }
}

impl From<Body> for BodyRef {
    fn from(body: Body) -> Self {
        Self(BaseOrBody::Body(Rc::new(RefCell::new(body))))
    }
}
