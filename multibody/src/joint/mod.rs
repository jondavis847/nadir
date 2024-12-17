pub mod floating;
pub mod joint_transforms;
pub mod prismatic;
pub mod revolute;

use super::body::BodyErrors;
use crate::{
    algorithms::articulated_body_algorithm::{AbaCache, ArticulatedBodyAlgorithm},
    base::{BaseConnection, BaseRef},
    body::{BodyConnection, BodyRef}, MultibodyResult,
};
use csv::Writer;
use joint_transforms::JointTransforms;
use mass_properties::MassProperties;
use serde::{Deserialize, Serialize};
use spatial_algebra::{Acceleration, Force, SpatialInertia, SpatialTransform, Velocity};
use utilities::initialize_writer;
use std::{
    cell::RefCell,
    fmt::Debug,
    fs::File,
    io::BufWriter,
    ops::{AddAssign, MulAssign},
    path::PathBuf,
    rc::Rc,
};
use thiserror::Error;
use transforms::Transform;

#[derive(Debug, Clone, Error)]
pub enum JointErrors {
    #[error("{0}")]
    BodyErrors(#[from] BodyErrors),
    #[error("name cannot be empty for joint")]
    EmptyName,
    #[error("inner body already exists for joint '{0}'")]
    InnerBodyExists(String),
    #[error("no joint mass properties found for joint '{0}'")]
    NoMassProperties(String),
    #[error("outer body already exists for joint '{0}'")]
    OuterBodyExists(String),
}

pub type JointRef = Rc<RefCell<Joint>>;

#[typetag::serde]
pub trait JointModel:
    CloneJointModel + Debug + MultibodyResult + ArticulatedBodyAlgorithm
{
    fn calculate_joint_inertia(
        &mut self,
        mass_properties: &MassProperties,
        transforms: &JointTransforms,
    ) -> SpatialInertia;
    /// Calculates internal force tau based on model joint parameters
    fn calculate_tau(&mut self);
    /// Calculates velocity across the joint based on model state
    fn calculate_vj(&self, transforms: &JointTransforms) -> Velocity;
    /// Returns the number of degrees of freedom for the joint
    /// Used to populate elements in the mass matrix and state arrays        
    fn ndof(&self) -> u32;
    /// Populates derivative with the appropriate values for the joint state derivative
    fn state_derivative(&self, derivative: &mut JointStateVector, transforms: &JointTransforms);
    /// Initializes a vector of f64 values representing state vector for the ODE integration
    fn state_vector_init(&self) -> JointStateVector;
    /// Reads a state vector into the joint state
    fn state_vector_read(&mut self, state: &JointStateVector);
    /// Updates the joint transforms based on model specific state
    /// Depends on the inner joint transforms as well
    fn update_transforms(
        &mut self,
        transforms: &mut JointTransforms,
        inner_joint: &Option<JointRef>,
    );
}
pub trait CloneJointModel {
    fn clone_model(&self) -> Box<dyn JointModel>;
}
impl<T> CloneJointModel for T
where
    T: JointModel + Clone + 'static,
{
    fn clone_model(&self) -> Box<dyn JointModel> {
        Box::new(self.clone())
    }
}

impl Clone for Box<dyn JointModel> {
    fn clone(&self) -> Self {
        self.clone_model()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Joint {
    pub name: String,
    pub model: Box<dyn JointModel>,
    pub connections: JointConnection,
    #[serde(skip)]
    pub cache: JointCache,
    #[serde(skip)]
    pub inner_joint: Option<JointRef>, //index of the parent joint
}

impl Joint {
    pub fn aba_first_pass(&mut self) {
        // get the inner joint velocity
        let v_ij = if let Some(inner_joint_ref) = &self.inner_joint {
            inner_joint_ref.borrow().cache.v
        } else {
            Velocity::zeros()
        };

        let c = &mut self.cache;
        c.v = c.transforms.jof_from_ij_jof * v_ij + c.vj;
        c.aba.c = c.v.cross_motion(c.vj); // + cj
        c.aba.inertia_articulated = c.inertia;
        c.aba.p_big_a = c.v.cross_force(c.inertia * c.v) - c.f;
    }

    pub fn aba_second_pass(&mut self) {
        self.model
            .aba_second_pass(&mut self.cache, &self.inner_joint);
    }

    pub fn aba_third_pass(&mut self) {
        self.model
            .aba_third_pass(&mut self.cache, &self.inner_joint);
    }

    pub fn new(name: &str, model: impl JointModel + 'static) -> Result<Self, JointErrors> {
        if name.is_empty() {
            return Err(JointErrors::EmptyName);
        }
        Ok(Self {
            name: name.to_string(),
            model: Box::new(model),
            connections: JointConnection::default(),
            cache: JointCache::default(),
            inner_joint: None,
        })
    }
    /// Calculates the mass properties about the joint given mass properties at the outer body
    pub fn calculate_joint_inertia(&mut self) {
        let outer_body_connection = self.connections.outer_body.as_ref().unwrap();
        let mass_properties = &outer_body_connection.body.borrow().mass_properties;

        self.cache.inertia = self
            .model
            .calculate_joint_inertia(mass_properties, &self.cache.transforms);
    }

    pub fn calculate_vj(&mut self) {
        self.cache.vj = self.model.calculate_vj(&self.cache.transforms);
    }

    pub fn connect_base(&mut self, base: BaseRef, transform: Transform) -> Result<(), JointErrors> {
        if let Some(_) = &self.connections.inner_body {
            return Err(JointErrors::InnerBodyExists(self.name.to_string()));
        } else {
            self.connections.inner_body =
                Some(InnerBody::Base(BaseConnection::new(base, transform)));
            self.cache.transforms.jif_from_ib = SpatialTransform(transform);
            self.cache.transforms.ib_from_jif = SpatialTransform(transform.inv());
        }
        Ok(())
    }

    pub fn connect_inner_body(
        &mut self,
        body: &BodyRef,
        transform: Transform,
    ) -> Result<(), JointErrors> {
        if let Some(_) = &self.connections.inner_body {
            return Err(JointErrors::InnerBodyExists(self.name.to_string()));
        } else {
            self.connections.inner_body = Some(InnerBody::Body(BodyConnection::new(
                Rc::clone(body),
                transform,
            )));
            self.cache.transforms.jif_from_ib = SpatialTransform(transform);
            self.cache.transforms.ib_from_jif = SpatialTransform(transform.inv());
        }
        Ok(())
    }

    pub fn connect_outer_body(
        &mut self,
        bodyref: &BodyRef,
        transform: Transform,
    ) -> Result<(), JointErrors> {
        if let Some(_) = &self.connections.outer_body {
            return Err(JointErrors::OuterBodyExists(self.name.clone()));
        } else {
            {
                let mass_properties = bodyref.borrow().mass_properties;
                self.cache.transforms.jof_from_ob = SpatialTransform(transform);
                self.cache.transforms.ob_from_jof = SpatialTransform(transform.inv());
                // calculate inertia about the joint
                // some joints make model specific assumptions about this
                self.cache.inertia = self
                    .model
                    .calculate_joint_inertia(&mass_properties, &self.cache.transforms);
            }
            self.connections.outer_body = Some(BodyConnection::new(Rc::clone(bodyref), transform));
        }
        Ok(())
    }

    pub fn state_derivative(&self, derivative: &mut JointStateVector) {
        self.model
            .state_derivative(derivative, &self.cache.transforms);
    }

    pub fn update_transforms(&mut self) {
        self.model
            .update_transforms(&mut self.cache.transforms, &self.inner_joint);
    }

    pub fn initialize_writer(&self, result_folder_path: &PathBuf) -> Writer<BufWriter<File>> {
        // Define the joints subfolder folder path
        let joints_folder_path = result_folder_path.join("joints");

        // Check if the folder exists, if not, create it
        if !joints_folder_path.exists() {
            std::fs::create_dir_all(&joints_folder_path).expect("Failed to create bodies folder");
        }

        // Initialize writer using the updated path
        initialize_writer(self.name.clone(), &joints_folder_path)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum InnerBody {
    Base(BaseConnection),
    Body(BodyConnection),
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct JointConnection {
    pub inner_body: Option<InnerBody>,
    pub outer_body: Option<BodyConnection>,
}

#[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
pub struct JointParameters {
    pub constant_force: f64,
    pub damping: f64,
    pub equilibrium: f64,
    pub spring_constant: f64,
}

impl JointParameters {
    //pub fn with_constant_force
}

#[derive(Debug, Clone)]
pub struct JointStateVector(Vec<f64>);
impl MulAssign<f64> for JointStateVector {
    fn mul_assign(&mut self, rhs: f64) {
        self.0.iter_mut().for_each(|state| *state *= rhs);
    }
}
impl AddAssign<&Self> for JointStateVector {
    fn add_assign(&mut self, rhs: &Self) {
        self.0
            .iter_mut()
            .zip(rhs.0.iter()) // Use `iter()` to iterate immutably over `rhs`
            .for_each(|(left, right)| *left += right);
    }
}

#[derive(Debug, Clone)]
pub struct JointStates(pub Vec<JointStateVector>);

impl MulAssign<f64> for JointStates {
    fn mul_assign(&mut self, rhs: f64) {
        self.0.iter_mut().for_each(|state| *state *= rhs);
    }
}
impl AddAssign<&Self> for JointStates {
    fn add_assign(&mut self, rhs: &Self) {
        self.0
            .iter_mut()
            .zip(rhs.0.iter())
            .for_each(|(left, right)| *left += right);
    }
}

#[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
pub struct JointCache {
    /// acceleration of the jof in the jof, includes acceleration of the inner joint
    pub a: Acceleration,
    /// force of jof in the jof
    pub f: Force,
    /// velocity of the jof in the jof, includes velocity of the inner joint
    pub v: Velocity,
    /// velocity across the joint (velocity of jof with respect to jif, does not include velocity of inner joint)
    pub vj: Velocity,
    /// spatial inertia of the joint based on outer body mass properties transformed from the body frame to the jof
    pub inertia: SpatialInertia,
    pub transforms: JointTransforms,
    pub aba: AbaCache,
}
