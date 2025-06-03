pub mod floating;
pub mod joint_transforms;
pub mod prismatic;
pub mod revolute;

use crate::{
    algorithms::articulated_body_algorithm::{AbaCache, ArticulatedBodyAlgorithm},
    body::{BodyConnection, BodyConnectionBuilder},
    joint::{prismatic::PrismaticState, revolute::RevoluteState},
    system::Id,
};
use floating::{Floating, FloatingBuilder, FloatingErrors, FloatingState};
use joint_transforms::JointTransforms;
use mass_properties::MassProperties;
use nadir_diffeq::state::state_vector::StateVector;
use nadir_result::{NadirResult, ResultManager};
use nalgebra::Vector6;
use prismatic::{Prismatic, PrismaticBuilder, PrismaticErrors};
use rand::rngs::SmallRng;
use revolute::{Revolute, RevoluteBuilder, RevoluteErrors};
use rotations::RotationTrait;
use serde::{Deserialize, Serialize};
use spatial_algebra::{Acceleration, Force, Momentum, SpatialInertia, Velocity};
use std::{
    cell::RefCell,
    fmt::Debug,
    ops::{AddAssign, MulAssign},
    rc::Rc,
};
use thiserror::Error;
use transforms::Transform;
use uncertainty::{SimValue, Uncertainty};

#[derive(Debug, Error)]
pub enum JointErrors {
    #[error("name cannot be empty for joint")]
    EmptyName,
    #[error("{0}")]
    FloatingError(#[from] FloatingErrors),
    #[error("inner body already exists for joint '{0}'")]
    InnerBodyExists(String),
    #[error("no joint mass properties found for joint '{0}'")]
    NoMassProperties(String),
    #[error("outer body already exists for joint '{0}'")]
    OuterBodyExists(String),
    #[error("{0}")]
    PrismaticError(#[from] PrismaticErrors),
    #[error("{0}")]
    RevoluteError(#[from] RevoluteErrors),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum JointModelBuilders {
    Floating(FloatingBuilder),
    Revolute(RevoluteBuilder),
    Prismatic(PrismaticBuilder),
}

impl Uncertainty for JointModelBuilders {
    type Error = JointErrors;
    type Output = JointModels;
    fn sample(&self, nominal: bool, rng: &mut SmallRng) -> Result<Self::Output, Self::Error> {
        match self {
            JointModelBuilders::Floating(builder) => {
                Ok(JointModels::Floating(builder.sample(nominal, rng)?))
            }
            JointModelBuilders::Revolute(builder) => {
                Ok(JointModels::Revolute(builder.sample(nominal, rng)?))
            }
            JointModelBuilders::Prismatic(builder) => {
                Ok(JointModels::Prismatic(builder.sample(nominal, rng)?))
            }
        }
    }
}

impl From<FloatingBuilder> for JointModelBuilders {
    fn from(value: FloatingBuilder) -> Self {
        JointModelBuilders::Floating(value)
    }
}
impl From<RevoluteBuilder> for JointModelBuilders {
    fn from(value: RevoluteBuilder) -> Self {
        JointModelBuilders::Revolute(value)
    }
}
impl From<PrismaticBuilder> for JointModelBuilders {
    fn from(value: PrismaticBuilder) -> Self {
        JointModelBuilders::Prismatic(value)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JointBuilder {
    pub id: Id,
    pub name: String,
    pub model: JointModelBuilders,
    pub connections: JointConnectionBuilder,
}

impl JointBuilder {
    pub fn new(id: Id, name: &str, model: JointModelBuilders) -> Result<Self, JointErrors> {
        if name.is_empty() {
            return Err(JointErrors::EmptyName);
        }
        Ok(Self {
            id,
            name: name.to_string(),
            model,
            connections: JointConnectionBuilder::default(),
        })
    }

    pub fn connect_inner_body(
        &mut self,
        body: Id,
        transform: Transform,
    ) -> Result<(), JointErrors> {
        if let Some(_) = &self.connections.inner_body {
            return Err(JointErrors::InnerBodyExists(self.name.to_string()));
        } else {
            self.connections.inner_body = Some(BodyConnectionBuilder::new(body, transform));
        }
        Ok(())
    }

    pub fn connect_outer_body(
        &mut self,
        body: Id,
        transform: Transform,
    ) -> Result<(), JointErrors> {
        if let Some(_) = &self.connections.outer_body {
            return Err(JointErrors::OuterBodyExists(self.name.clone()));
        } else {
            self.connections.outer_body = Some(BodyConnectionBuilder::new(body, transform));
        }
        Ok(())
    }

    pub fn sample(
        &self,
        connections: JointConnection,
        nominal: bool,
        rng: &mut SmallRng,
    ) -> Result<Joint, JointErrors> {
        let model = self.model.sample(nominal, rng)?;
        Ok(Joint {
            name: self.name.clone(),
            model,
            connections,
            result_id: None,
            state_start: 0,
            state_end: 0,
            cache: JointCache::default(),
            inner_joint: None,
        })
    }
}

pub trait JointModel: ArticulatedBodyAlgorithm {
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
    fn state_derivative(&self, derivative: &mut [f64], transforms: &JointTransforms);
    /// Initializes a vector of f64 values representing state vector for the ODE integration
    fn state_vector_init(&self) -> StateVector;
    /// Reads a state vector into the joint state
    fn state_vector_read(&mut self, state: &[f64]);
    /// Updates the joint transforms based on model specific state
    /// Depends on the inner joint transforms as well
    fn update_transforms(
        &mut self,
        transforms: &mut JointTransforms,
        inner_joint: &Option<JointRef>,
    );
    fn result_headers(&self) -> &[&str];
    fn result_content(&self, id: u32, results: &mut ResultManager);
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum JointModels {
    Floating(Floating),
    Prismatic(Prismatic),
    Revolute(Revolute),
}

impl JointModel for JointModels {
    fn calculate_joint_inertia(
        &mut self,
        mass_properties: &MassProperties,
        transforms: &JointTransforms,
    ) -> SpatialInertia {
        match self {
            JointModels::Floating(model) => {
                model.calculate_joint_inertia(mass_properties, transforms)
            }
            JointModels::Prismatic(model) => {
                model.calculate_joint_inertia(mass_properties, transforms)
            }
            JointModels::Revolute(model) => {
                model.calculate_joint_inertia(mass_properties, transforms)
            }
        }
    }

    fn calculate_tau(&mut self) {
        match self {
            JointModels::Floating(model) => model.calculate_tau(),
            JointModels::Prismatic(model) => model.calculate_tau(),
            JointModels::Revolute(model) => model.calculate_tau(),
        }
    }

    fn calculate_vj(&self, transforms: &JointTransforms) -> Velocity {
        match self {
            JointModels::Floating(model) => model.calculate_vj(transforms),
            JointModels::Prismatic(model) => model.calculate_vj(transforms),
            JointModels::Revolute(model) => model.calculate_vj(transforms),
        }
    }

    fn ndof(&self) -> u32 {
        match self {
            JointModels::Floating(model) => model.ndof(),
            JointModels::Prismatic(model) => model.ndof(),
            JointModels::Revolute(model) => model.ndof(),
        }
    }

    fn result_content(&self, id: u32, results: &mut ResultManager) {
        match self {
            JointModels::Floating(model) => model.result_content(id, results),
            JointModels::Prismatic(model) => model.result_content(id, results),
            JointModels::Revolute(model) => model.result_content(id, results),
        }
    }

    fn result_headers(&self) -> &[&str] {
        match self {
            JointModels::Floating(model) => model.result_headers(),
            JointModels::Prismatic(model) => model.result_headers(),
            JointModels::Revolute(model) => model.result_headers(),
        }
    }

    fn state_derivative(&self, derivative: &mut [f64], transforms: &JointTransforms) {
        match self {
            JointModels::Floating(model) => model.state_derivative(derivative, transforms),
            JointModels::Prismatic(model) => model.state_derivative(derivative, transforms),
            JointModels::Revolute(model) => model.state_derivative(derivative, transforms),
        }
    }

    fn state_vector_init(&self) -> StateVector {
        match self {
            JointModels::Floating(model) => model.state_vector_init(),
            JointModels::Prismatic(model) => model.state_vector_init(),
            JointModels::Revolute(model) => model.state_vector_init(),
        }
    }

    fn state_vector_read(&mut self, state: &[f64]) {
        match self {
            JointModels::Floating(model) => model.state_vector_read(state),
            JointModels::Prismatic(model) => model.state_vector_read(state),
            JointModels::Revolute(model) => model.state_vector_read(state),
        }
    }
    fn update_transforms(
        &mut self,
        transforms: &mut JointTransforms,
        inner_joint: &Option<JointRef>,
    ) {
        match self {
            JointModels::Floating(model) => model.update_transforms(transforms, inner_joint),
            JointModels::Prismatic(model) => model.update_transforms(transforms, inner_joint),
            JointModels::Revolute(model) => model.update_transforms(transforms, inner_joint),
        }
    }
}

impl ArticulatedBodyAlgorithm for JointModels {
    fn aba_second_pass(&mut self, joint_cache: &mut JointCache, inner_joint: &Option<JointRef>) {
        match self {
            JointModels::Floating(model) => model.aba_second_pass(joint_cache, inner_joint),
            JointModels::Prismatic(model) => model.aba_second_pass(joint_cache, inner_joint),
            JointModels::Revolute(model) => model.aba_second_pass(joint_cache, inner_joint),
        }
    }

    fn aba_third_pass(&mut self, joint_cache: &mut JointCache, inner_joint: &Option<JointRef>) {
        match self {
            JointModels::Floating(model) => model.aba_third_pass(joint_cache, inner_joint),
            JointModels::Prismatic(model) => model.aba_third_pass(joint_cache, inner_joint),
            JointModels::Revolute(model) => model.aba_third_pass(joint_cache, inner_joint),
        }
    }
}

#[derive(Debug)]
pub struct Joint {
    pub name: String,
    pub model: JointModels,
    pub connections: JointConnection,
    result_id: Option<u32>,
    state_start: usize,
    state_end: usize,
    pub cache: JointCache,
    pub inner_joint: Option<JointRef>,
}

impl Joint {
    pub fn aba_first_pass(&mut self) {
        // get the inner joint velocity
        let v_ij = if let Some(inner_joint) = &self.inner_joint {
            inner_joint.borrow().cache.v
        } else {
            // no inner joint, inner body is base, velocity is 0
            Velocity::zeros()
        };

        let c = &mut self.cache;
        c.v = c.transforms.jof_from_ij_jof * v_ij + c.vj;
        c.aba.c = c.v.cross_motion(c.vj); // + cj
        c.aba.inertia_articulated = c.inertia;

        // from eulers equation, p_big_a is w x H - T,
        // but H in the joint is H in the body transformed to the joint.
        // H in the body is Hs = Hb + Hi, where Hb is wI of the body.
        // need to add in Hi, which is momentum of internally rotating components
        let h_i = {
            let outer_body = &self
                .connections
                .outer_body
                .as_ref()
                .expect("validation shuold catch this")
                .body
                .borrow();
            let h = c
                .transforms
                .jof_from_ob
                .0
                .rotation
                .transform(&outer_body.state.angular_momentum_body);
            Momentum::from(Vector6::new(h[0], h[1], h[2], 0.0, 0.0, 0.0))
        };

        c.aba.p_big_a = c.v.cross_force(c.inertia * c.v + h_i) - c.f;
    }

    pub fn aba_second_pass(&mut self) {
        self.model
            .aba_second_pass(&mut self.cache, &self.inner_joint);
    }

    pub fn aba_third_pass(&mut self) {
        self.model
            .aba_third_pass(&mut self.cache, &self.inner_joint);
    }

    /// Calculates the mass properties about the joint given mass properties at the outer body
    pub fn calculate_joint_inertia(&mut self) {
        let outer_body = self
            .connections
            .outer_body
            .as_ref()
            .expect("validation should catch this")
            .body
            .borrow();
        self.cache.inertia = self
            .model
            .calculate_joint_inertia(&outer_body.mass_properties, &self.cache.transforms);
    }

    pub fn calculate_vj(&mut self) {
        self.cache.vj = self.model.calculate_vj(&self.cache.transforms);
    }

    pub fn set_inertia(&mut self, mass_properties: &MassProperties) {
        self.cache.inertia = self
            .model
            .calculate_joint_inertia(mass_properties, &self.cache.transforms);
    }

    pub fn state_derivative(&self, derivatives: &mut StateVector) {
        let derivative = &mut derivatives[self.state_start..self.state_end];
        self.model
            .state_derivative(derivative, &self.cache.transforms);
    }

    pub fn update_transforms(&mut self) {
        self.model
            .update_transforms(&mut self.cache.transforms, &self.inner_joint);
    }

    pub fn with_inner_joint(mut self, inner_joint: JointRef) -> Self {
        self.inner_joint = Some(inner_joint);
        self
    }

    pub fn state_vector_init(&mut self, x0: &mut StateVector) {
        self.state_start = x0.len();
        let state = self.model.state_vector_init();
        self.state_end = self.state_start + state.len();
        x0.extend(&self.model.state_vector_init());
    }

    pub fn state_vector_read(&mut self, x0: &StateVector) {
        let state = &x0[self.state_start..self.state_end];
        self.model.state_vector_read(state);
    }
}

impl NadirResult for Joint {
    fn new_result(&mut self, results: &mut ResultManager) {
        // Define the joints subfolder folder path
        let joints_folder_path = results.result_path.join("joints");
        // Check if the folder exists, if not, create it
        if !joints_folder_path.exists() {
            std::fs::create_dir_all(&joints_folder_path).expect("Failed to create bodies folder");
        }
        // Get the headers from the joint model
        let headers = self.model.result_headers();
        // Create the writer and assign its id
        let id = results.new_writer(&self.name.clone(), &joints_folder_path, headers);
        self.result_id = Some(id);
    }

    fn write_result(&self, results: &mut ResultManager) {
        if let Some(id) = self.result_id {
            self.model.result_content(id, results);
        }
    }
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct JointConnectionBuilder {
    pub inner_body: Option<BodyConnectionBuilder>,
    pub outer_body: Option<BodyConnectionBuilder>,
}

#[derive(Debug)]
pub struct JointConnection {
    pub inner_body: BodyConnection,
    pub outer_body: Option<BodyConnection>, // needs to be option just because the body isnt created yet when system is recursively built
}

#[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
pub struct JointParameters {
    constant_force: f64,
    damping: f64,
    equilibrium: f64,
    spring_constant: f64,
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct JointParametersBuilder {
    constant_force: SimValue,
    damping: SimValue,
    equilibrium: SimValue,
    spring_constant: SimValue,
}

impl JointParametersBuilder {
    //TODO: builder pattern?
    //pub fn with_constant_force
}

impl Uncertainty for JointParametersBuilder {
    type Output = JointParameters;
    type Error = JointErrors;

    fn sample(
        &self,
        nominal: bool,
        rng: &mut rand::rngs::SmallRng,
    ) -> Result<Self::Output, Self::Error> {
        Ok(JointParameters {
            constant_force: self.constant_force.sample(nominal, rng),
            damping: self.damping.sample(nominal, rng),
            equilibrium: self.equilibrium.sample(nominal, rng),
            spring_constant: self.spring_constant.sample(nominal, rng),
        })
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

pub type JointRef = Rc<RefCell<Joint>>;

pub struct JointStates {
    floating: Vec<FloatingState>,
    prismatic: Vec<PrismaticState>,
    revolute: Vec<RevoluteState>,
}

impl AddAssign<&Self> for JointStates {
    fn add_assign(&mut self, rhs: &Self) {
        for (mut lhs, rhs) in self.floating.iter_mut().zip(rhs) {
            lhs += rhs;
        }
        for (mut lhs, rhs) in self.prismatic.iter_mut().zip(rhs) {
            lhs += rhs;
        }
        for (mut lhs, rhs) in self.revolute.iter_mut().zip(rhs) {
            lhs += rhs;
        }
    }
}

impl MulAssign<f64> for JointStates {
    fn mul_assign(&mut self, rhs: &Self) {
        for x in &mut self.floating {
            x *= rhs;
        }
        for x in &mut self.prismatic {
            x *= rhs;
        }
        for x in &mut self.revolute {
            x *= rhs;
        }
    }
}
