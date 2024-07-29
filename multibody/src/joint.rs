use super::{
    algorithms::articulated_body_algorithm::ArticulatedBodyAlgorithm, body::Body, MultibodyTrait,
};
use std::ops::{Add, AddAssign, Div, Mul};

use spatial_algebra::{Acceleration, Force, SpatialInertia, SpatialTransform, Velocity};

use super::body::BodyTrait;

use transforms::Transform;
use uuid::Uuid;

pub mod revolute;
use revolute::{Revolute, RevoluteResult, RevoluteSim, RevoluteState};
pub mod prismatic;
use prismatic::{Prismatic, PrismaticResult, PrismaticSim, PrismaticState};
pub trait JointTrait: MultibodyTrait {
    fn connect_inner_body<T: BodyTrait>(
        &mut self,
        body: &mut T,
        transform: Transform,
    ) -> Result<(), JointErrors>;

    fn connect_outer_body(
        &mut self,
        body: &mut Body,
        transform: Transform,
    ) -> Result<(), JointErrors>;

    fn delete_inner_body_id(&mut self);
    fn delete_outer_body_id(&mut self);
    fn get_connections(&self) -> &JointConnection;
    fn get_connections_mut(&mut self) -> &mut JointConnection;    
    fn get_inner_body_id(&self) -> Option<&Uuid>;
    fn get_outer_body_id(&self) -> Option<&Uuid>;
}

#[derive(Debug)]
pub enum JointErrors {
    InnerBodyExists,
    OuterBodyExists,
}

#[derive(Debug, Clone)]
pub struct JointCommon {
    pub id: Uuid,
    pub name: String,
    pub connection: JointConnection,
    pub transforms: JointTransforms,
}

impl JointCommon {
    pub fn new(name: &str) -> Self {
        Self {
            id: Uuid::new_v4(),
            name: name.to_string(),
            connection: JointConnection::default(),
            transforms: JointTransforms::default(),
        }
    }
}

#[derive(Debug, Clone)]
pub enum Joint {
    //Floating,
    Prismatic(Prismatic),
    Revolute(Revolute),
    //Spherical,
}

impl From<Revolute> for Joint {
    fn from(revolute: Revolute) -> Self {
        Joint::Revolute(revolute)
    }
}
impl From<Prismatic> for Joint {
    fn from(prismatic: Prismatic) -> Self {
        Joint::Prismatic(prismatic)
    }
}

impl MultibodyTrait for Joint {
    fn get_id(&self) -> &Uuid {
        match self {
            Joint::Prismatic(joint) => joint.get_id(),
            Joint::Revolute(joint) => joint.get_id(),
        }
    }
    fn get_name(&self) -> &str {
        match self {
            Joint::Prismatic(joint) => joint.get_name(),
            Joint::Revolute(joint) => joint.get_name(),
        }
    }

    fn set_name(&mut self, name: String) {
        match self {
            Joint::Prismatic(joint) => joint.set_name(name),
            Joint::Revolute(joint) => joint.set_name(name),
        }
    }
}

impl JointTrait for Joint {
    fn connect_inner_body<T: BodyTrait>(
        &mut self,
        body: &mut T,
        transform: Transform,
    ) -> Result<(), JointErrors> {
        match self {
            Joint::Prismatic(joint) => joint.connect_inner_body(body, transform),
            Joint::Revolute(joint) => joint.connect_inner_body(body, transform),
        }
    }

    fn connect_outer_body(
        &mut self,
        body: &mut Body,
        transform: Transform,
    ) -> Result<(), JointErrors> {
        match self {
            Joint::Prismatic(joint) => joint.connect_outer_body(body, transform),
            Joint::Revolute(joint) => joint.connect_outer_body(body, transform),
        }
    }

    fn delete_inner_body_id(&mut self) {
        match self {
            Joint::Prismatic(joint) => joint.delete_inner_body_id(),
            Joint::Revolute(joint) => joint.delete_inner_body_id(),
        }
    }
    fn delete_outer_body_id(&mut self) {
        match self {
            Joint::Prismatic(joint) => joint.delete_outer_body_id(),
            Joint::Revolute(joint) => joint.delete_outer_body_id(),
        }
    }

    fn get_connections(&self) -> &JointConnection {
        match self {
            Joint::Prismatic(joint) => joint.get_connections(),
            Joint::Revolute(joint) => joint.get_connections(),
        }
    }

    fn get_connections_mut(&mut self) -> &mut JointConnection {
        match self {
            Joint::Prismatic(joint) => joint.get_connections_mut(),
            Joint::Revolute(joint) => joint.get_connections_mut(),
        }
    }

    fn get_inner_body_id(&self) -> Option<&Uuid> {
        match self {
            Joint::Prismatic(joint) => joint.get_inner_body_id(),
            Joint::Revolute(joint) => joint.get_inner_body_id(),
        }
    }
    fn get_outer_body_id(&self) -> Option<&Uuid> {
        match self {
            Joint::Prismatic(joint) => joint.get_outer_body_id(),
            Joint::Revolute(joint) => joint.get_outer_body_id(),
        }
    }    
}

impl ArticulatedBodyAlgorithm for JointSim {
    fn first_pass(&mut self, v_ij: Velocity, f_ob: &Force) {
        match self {
            JointSim::Prismatic(joint) => joint.first_pass(v_ij, f_ob),
            JointSim::Revolute(joint) => joint.first_pass(v_ij, f_ob),
        }
    }
    fn second_pass(&mut self, inner_is_base: bool) -> Option<(SpatialInertia, Force)> {
        match self {
            JointSim::Prismatic(joint) => joint.second_pass(inner_is_base),
            JointSim::Revolute(joint) => joint.second_pass(inner_is_base),
        }
    }
    fn third_pass(&mut self, a_ij: Acceleration) {
        match self {
            JointSim::Prismatic(joint) => joint.third_pass(a_ij),
            JointSim::Revolute(joint) => joint.third_pass(a_ij),
        }
    }

    fn get_aba_derivative(&self) -> JointState {
        match self {
            JointSim::Prismatic(joint) => joint.get_aba_derivative(),
            JointSim::Revolute(joint) => joint.get_aba_derivative(),
        }
    }

    fn get_v(&self) -> &Velocity {
        match self {
            JointSim::Prismatic(joint) => joint.get_v(),
            JointSim::Revolute(joint) => joint.get_v(),
        }
    }
    fn get_p_big_a(&self) -> &Force {
        match self {
            JointSim::Prismatic(joint) => joint.get_p_big_a(),
            JointSim::Revolute(joint) => joint.get_p_big_a(),
        }
    }

    fn get_a(&self) -> &Acceleration {
        match self {
            JointSim::Prismatic(joint) => joint.get_a(),
            JointSim::Revolute(joint) => joint.get_a(),
        }
    }

    fn add_inertia_articulated(&mut self, inertia: SpatialInertia) {
        match self {
            JointSim::Prismatic(joint) => joint.add_inertia_articulated(inertia),
            JointSim::Revolute(joint) => joint.add_inertia_articulated(inertia),
        }
    }

    fn add_p_big_a(&mut self, force: Force) {
        match self {
            JointSim::Prismatic(joint) => joint.add_p_big_a(force),
            JointSim::Revolute(joint) => joint.add_p_big_a(force),
        }
    }
}

// We choose to keep the transform information with the joint rather than the body.
// This is because bodies can have multiple outer joints where as a joint only has one inner or outer body.
// If it was in the body, and I want the the transform from parent body to this body,
// then I need the transform from parent body to this inner joint.
// To get that transform with that info in the parent body, we would need to search it's
// outer joints for this inner joint and apply that transform.
// If the transform is already in the joint, then no searching.
// We also choose to make the transform be from the body frame to the joint frame.
// This is because the location of things like actuators or other components are typically expressed in the body frame.

#[derive(Debug, Clone)]
pub struct Connection {
    pub body_id: Uuid,
    pub transform: Transform,
}
impl Connection {
    pub fn new(body_id: Uuid, transform: Transform) -> Self {
        Self { body_id, transform }
    }
}

#[derive(Debug, Default, Clone)]
pub struct JointConnection {
    pub inner_body: Option<Connection>,
    pub outer_body: Option<Connection>,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct JointParameters {
    pub constant_force: f64,
    pub damping: f64,
    pub mass_properties: Option<SpatialInertia>,
    pub spring_constant: f64,
}

impl JointParameters {
    pub fn new(constant_force: f64, damping: f64, spring_constant: f64) -> Self {
        let mass_properties = None;
        Self {
            constant_force,
            damping,
            mass_properties,
            spring_constant,
        }
    }
}

/// We use the terminology B_from_A rather than A_to_B so that notation matches matrix multiplication
/// i.e. v_C = C_from_B * B_from_A * v_A instead of
///      v_C = (A_to_B * B_to_C) * v_A
/// base: the reference frame that is the base
/// inner_body: the "body frame" of the body on the base side of the joint
/// outer_body: the "body frame" of the body on the tip side of the joint
/// jif: the "joint inner frame"
/// jof: the "joint outer frame"
#[derive(Clone, Copy, Debug, Default)]
pub struct JointTransforms {
    // joint frame
    pub jif_from_jof: SpatialTransform, // my-joint-inner-frame from my-joint-outer-frame
    pub jof_from_jif: SpatialTransform, // my-joint-outer-frame from my-joint-inner-frame

    // body to joint frames
    pub jif_from_ib: SpatialTransform, // my-joint-inner-frame from my-inner-body-frame
    pub ib_from_jif: SpatialTransform, // my-inner-body-frame from my-joint-inner-frame

    pub jof_from_ob: SpatialTransform, // my-joint-outer-frame from my-outer-body-frame
    pub ob_from_jof: SpatialTransform, // my-outer-body-frame from my-joint-outer-frame

    // joint to joint frames
    pub jof_from_ij_jof: SpatialTransform, // my-joint-outer-frame from inner-joint-outer-frame
    pub ij_jof_from_jof: SpatialTransform, // inner-joint-outer-frame from my-joint-outer-frame

    // base to joint frames - only need outer really
    pub jof_from_base: SpatialTransform,
    pub base_from_jof: SpatialTransform,

    //base to outer body
    pub base_from_ob: SpatialTransform,
    pub ob_from_base: SpatialTransform,
}

impl JointTransforms {
    pub fn update(&mut self, ij_transforms: Option<(SpatialTransform, SpatialTransform)>) {
        // transforms are multiplied like matrices from right to left.
        // i.e. if you want to express v from frame A in frame C
        // you would use vC = C_to_B * B_to_A * vA
        // this means that the transform outer_body_to_inner_body is actually a
        // transform from the inner body to the outer body
        // I just like this notation better

        let jof_from_ij_jof;
        let ij_jof_from_jof;
        let jof_from_base;

        // get relevant transforms from the parent for calculations to the base, if the inner body is not the base
        if let Some((ij_ob_from_ij_jof, ij_jof_from_base)) = ij_transforms {
            // this joints inner body is the parent joints outer body
            jof_from_ij_jof = self.jof_from_jif * self.jif_from_ib * ij_ob_from_ij_jof;
            ij_jof_from_jof = jof_from_ij_jof.inv();
            jof_from_base = jof_from_ij_jof * ij_jof_from_base;
        } else {
            // inner joint is the base, so base transform is the inner joint transform
            // note that the base to outer joint transform is still accounted for
            jof_from_ij_jof = self.jof_from_jif * self.jif_from_ib;
            ij_jof_from_jof = jof_from_ij_jof.inv();
            jof_from_base = jof_from_ij_jof;
        }
        self.jof_from_ij_jof = jof_from_ij_jof;
        self.ij_jof_from_jof = ij_jof_from_jof;
        self.jof_from_base = jof_from_base;
        self.base_from_jof = jof_from_base.inv();
        self.ob_from_base = self.ob_from_jof * jof_from_base;
        self.base_from_ob = self.ob_from_base.inv();
    }
}

#[derive(Debug, Copy, Clone)]
pub enum JointSim {
    Prismatic(PrismaticSim),
    Revolute(RevoluteSim),
}

impl From<Joint> for JointSim {
    fn from(joint: Joint) -> Self {
        match joint {
            Joint::Prismatic(joint) => JointSim::Prismatic(joint.into()),
            Joint::Revolute(joint) => JointSim::Revolute(joint.into()),
        }
    }
}

impl JointSimTrait for JointSim {
    fn calculate_tau(&mut self) {
        match self {
            JointSim::Prismatic(joint) => joint.calculate_tau(),
            JointSim::Revolute(joint) => joint.calculate_tau(),
        }
    }

    fn calculate_vj(&mut self) {
        match self {
            JointSim::Prismatic(joint) => joint.calculate_vj(),
            JointSim::Revolute(joint) => joint.calculate_vj(),
        }
    }

    #[inline]
    fn get_id(&self) -> &Uuid {
        match self {
            JointSim::Prismatic(joint) => joint.get_id(),
            JointSim::Revolute(joint) => joint.get_id(),
        }
    }
    fn get_inertia(&self) -> &Option<SpatialInertia> {
        match self {
            JointSim::Prismatic(joint) => joint.get_inertia(),
            JointSim::Revolute(joint) => joint.get_inertia(),
        }
    }
    #[inline]
    fn get_state(&self) -> JointState {
        match self {
            JointSim::Prismatic(joint) => joint.get_state(),
            JointSim::Revolute(joint) => joint.get_state(),
        }
    }
    #[inline]
    fn set_state(&mut self, state: JointState) {
        match self {
            JointSim::Prismatic(joint) => joint.set_state(state),
            JointSim::Revolute(joint) => joint.set_state(state),
        }
    }

    #[inline]
    fn get_transforms(&self) -> &JointTransforms {
        match self {
            JointSim::Prismatic(joint) => joint.get_transforms(),
            JointSim::Revolute(joint) => joint.get_transforms(),
        }
    }

    #[inline]
    fn get_transforms_mut(&mut self) -> &mut JointTransforms {
        match self {
            JointSim::Prismatic(joint) => joint.get_transforms_mut(),
            JointSim::Revolute(joint) => joint.get_transforms_mut(),
        }
    }

    #[inline]
    fn update_transforms(&mut self, ij_transforms: Option<(SpatialTransform, SpatialTransform)>) {
        match self {
            JointSim::Prismatic(joint) => joint.update_transforms(ij_transforms),
            JointSim::Revolute(joint) => joint.update_transforms(ij_transforms),
        }
    }
}

pub trait JointSimTrait {
    fn calculate_tau(&mut self);
    fn calculate_vj(&mut self);
    fn get_id(&self) -> &Uuid;
    fn get_inertia(&self) -> &Option<SpatialInertia>;
    fn get_state(&self) -> JointState;
    fn set_state(&mut self, state: JointState);
    fn get_transforms(&self) -> &JointTransforms;
    fn get_transforms_mut(&mut self) -> &mut JointTransforms;
    fn update_transforms(&mut self, ij_transforms: Option<(SpatialTransform, SpatialTransform)>);
}

#[derive(Clone, Copy, Debug)]
pub enum JointState {
    Prismatic(PrismaticState), // Spherical(SphericalState)
    Revolute(RevoluteState),   // Spherical(SphericalState)
}

impl Add for JointState {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        match (self, rhs) {
            (JointState::Revolute(lhs), JointState::Revolute(rhs)) => {
                JointState::Revolute(lhs + rhs)
            }
            (JointState::Prismatic(lhs), JointState::Prismatic(rhs)) => {
                JointState::Prismatic(lhs + rhs)
            } // Handle other variants here if they are added
            // (JointState::Spherical(lhs), JointState::Spherical(rhs)) => JointState::Spherical(lhs + rhs),
            _ => panic!("Cannot add different JointState variants"), // shouldnt be possible, just used to integrate systems
        }
    }
}

impl AddAssign for JointState {
    fn add_assign(&mut self, rhs: Self) {
        match (self, rhs) {
            (JointState::Revolute(lhs), JointState::Revolute(rhs)) => *lhs += rhs,
            (JointState::Prismatic(lhs), JointState::Prismatic(rhs)) => *lhs += rhs,
            // Handle other variants here if they are added
            // (JointState::Spherical(lhs), JointState::Spherical(rhs)) => JointState::Spherical(lhs + rhs),
            _ => panic!("Cannot add different JointState variants"),
        }
    }
}

impl Mul<f64> for JointState {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self {
        match self {
            JointState::Prismatic(state) => JointState::Prismatic(state * rhs),
            JointState::Revolute(state) => JointState::Revolute(state * rhs),
        }
    }
}

impl Div<f64> for JointState {
    type Output = Self;

    fn div(self, rhs: f64) -> Self {
        match self {
            JointState::Prismatic(state) => JointState::Prismatic(state / rhs),
            JointState::Revolute(state) => JointState::Revolute(state / rhs),
        }
    }
}

#[derive(Debug, Clone)]
pub enum JointResult {
    Prismatic(PrismaticResult),
    Revolute(RevoluteResult),
}
