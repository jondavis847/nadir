use super::{
    algorithms::articulated_body_algorithm::ArticulatedBodyAlgorithm, body::BodySim, MultibodyTrait,
};

use spatial_algebra::{Acceleration, Force, SpatialInertia, SpatialTransform, Velocity};

use super::body::body_enum::BodyEnum;

use transforms::Transform;
use uuid::Uuid;

pub mod revolute;
use revolute::{Revolute, RevoluteSim};
pub trait JointTrait {
    fn connect_inner_body(
        &mut self,
        body: &BodyEnum,
        transform: Transform,
    ) -> Result<(), JointErrors>;
    fn connect_outer_body(
        &mut self,
        body: &BodyEnum,
        transform: Transform,
    ) -> Result<(), JointErrors>;
    fn delete_inner_body_id(&mut self);
    fn delete_outer_body_id(&mut self);
    fn get_inner_body_id(&self) -> Option<&Uuid>;
    fn get_outer_body_id(&self) -> Option<&Uuid>;    
}
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
pub enum JointEnum {
    //Floating,
    //Prismatic,
    Revolute(Revolute),
    //Spherical,
}

impl MultibodyTrait for JointEnum {
    fn get_id(&self) -> &Uuid {
        match self {
            JointEnum::Revolute(joint) => joint.get_id(),
        }
    }
    fn get_name(&self) -> &str {
        match self {
            JointEnum::Revolute(revolute) => revolute.get_name(),
        }
    }

    fn set_name(&mut self, name: String) {
        match self {
            JointEnum::Revolute(revolute) => revolute.set_name(name),
        }
    }
}

impl JointTrait for JointEnum {
    fn connect_inner_body(
        &mut self,
        body: &BodyEnum,
        transform: Transform,
    ) -> Result<(), JointErrors> {
        match self {
            JointEnum::Revolute(joint) => joint.connect_inner_body(body, transform),
        }
    }
    fn connect_outer_body(
        &mut self,
        body: &BodyEnum,
        transform: Transform,
    ) -> Result<(), JointErrors> {
        match self {
            JointEnum::Revolute(joint) => joint.connect_outer_body(body, transform),
        }
    }

    fn delete_inner_body_id(&mut self) {
        match self {
            JointEnum::Revolute(joint) => joint.delete_inner_body_id(),
        }
    }
    fn delete_outer_body_id(&mut self) {
        match self {
            JointEnum::Revolute(joint) => joint.delete_outer_body_id(),
        }
    }

    fn get_inner_body_id(&self) -> Option<&Uuid> {
        match self {
            JointEnum::Revolute(joint) => joint.get_inner_body_id(),
        }
    }
    fn get_outer_body_id(&self) -> Option<&Uuid> {
        match self {
            JointEnum::Revolute(joint) => joint.get_outer_body_id(),
        }
    }
}

impl ArticulatedBodyAlgorithm for JointSim {
    fn first_pass(&mut self, v_ij: Velocity, f_ob: &Force) {
        match self {
            JointSim::Revolute(joint) => joint.first_pass(v_ij, f_ob),
        }
    }
    fn second_pass(&mut self, inner_is_base: bool) -> Option<(SpatialInertia, Force)> {
        match self {
            JointSim::Revolute(joint) => joint.second_pass(inner_is_base),
        }
    }
    fn third_pass(&mut self, a_ij: Acceleration) {
        match self {
            JointSim::Revolute(joint) => joint.third_pass(a_ij),
        }
    }
    fn get_v(&self) -> &Velocity {
        match self {
            JointSim::Revolute(joint) => joint.get_v(),
        }
    }
    fn get_p_big_a(&self) -> &Force {
        match self {
            JointSim::Revolute(joint) => joint.get_p_big_a(),
        }
    }

    fn get_a(&self) -> &Acceleration {
        match self {
            JointSim::Revolute(joint) => joint.get_a(),
        }
    }

    fn add_inertia_articulated(&mut self, inertia: SpatialInertia) {
        match self {
            JointSim::Revolute(joint) => joint.add_inertia_articulated(inertia),
        }
    }

    fn add_p_big_a(&mut self, force: Force) {
        match self {
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
    inner_body: Option<Connection>,
    outer_body: Option<Connection>,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct JointParameters {
    constant_force: f64,
    dampening: f64,
    mass_properties: Option<SpatialInertia>,
    spring_constant: f64,
}

impl JointParameters {
    pub fn new(constant_force: f64, dampening: f64, spring_constant: f64) -> Self {
        let mass_properties = None;
        Self {
            constant_force,
            dampening,
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
            jof_from_ij_jof = self.jif_from_ib * ij_ob_from_ij_jof;
            ij_jof_from_jof = jof_from_ij_jof.inv();
            jof_from_base = jof_from_ij_jof * ij_jof_from_base;
        } else {
            // inner joint is the base, so base transform is the inner joint transform
            // note that the base to outer joint transform is still accounted for
            jof_from_ij_jof = self.jif_from_ib;
            ij_jof_from_jof = jof_from_ij_jof.inv();
            jof_from_base = jof_from_ij_jof;
        }
        self.jof_from_ij_jof = jof_from_ij_jof;
        self.ij_jof_from_jof = ij_jof_from_jof;
        self.jof_from_base = jof_from_base;
        self.base_from_jof = jof_from_base.inv();
    }
}

pub enum JointSim {
    Revolute(RevoluteSim),
}

impl From<JointEnum> for JointSim {
    fn from(joint: JointEnum) -> Self {
        match joint {
            JointEnum::Revolute(revolute) => JointSim::Revolute(revolute.into()),
        }
    }
}

impl JointSimTrait for JointSim {
    #[inline]
    fn get_transforms(&self) -> &JointTransforms {
        match self {
            JointSim::Revolute(joint) => joint.get_transforms(),
        }
    }

    #[inline]
    fn get_transforms_mut(&mut self) -> &mut JointTransforms {
        match self {
            JointSim::Revolute(joint) => joint.get_transforms_mut(),
        }
    }

    #[inline]
    fn update_transforms(&mut self, ij_transforms: Option<(SpatialTransform, SpatialTransform)>) {
        match self {
            JointSim::Revolute(joint) => joint.update_transforms(ij_transforms),
        }
    }
}

pub trait JointSimTrait {
    fn get_transforms(&self) -> &JointTransforms;
    fn get_transforms_mut(&mut self) -> &mut JointTransforms;
    fn update_transforms(&mut self, ij_transforms: Option<(SpatialTransform, SpatialTransform)>);
}
