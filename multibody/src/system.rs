use super::{
    algorithms::{articulated_body_algorithm::ArticulatedBodyAlgorithm, MultibodyAlgorithm},
    body::{BodyEnum, BodyRef, BodyTrait},
    joint::{JointRef, JointTrait},
    MultibodyErrors, MultibodyTrait,
};

#[derive(Debug, Clone)]
pub struct MultibodySystem {
    algorithm: MultibodyAlgorithm,
    bodies: Vec<BodyRef>,
    joints: Vec<JointRef>,
}

impl MultibodySystem {
    pub fn new() -> Self {
        Self {
            algorithm: MultibodyAlgorithm::ArticulatedBody, // for now, default to this
            bodies: Vec::new(),
            joints: Vec::new(),
        }
    }

    pub fn add_body(&mut self, bodyref: BodyRef) -> Result<(), MultibodyErrors> {
        // return if this is a Base and we already have a Base
        if matches!(*bodyref.borrow(), BodyEnum::Base(_)) {
            if let Some(_) = self.bodies.iter().find(|body| {
                let body = body.borrow();
                matches!(*body, BodyEnum::Base(_))
            }) {
                return Err(MultibodyErrors::BaseAlreadyExists);
            }
        }

        // Return if a component with this name already exists
        let name = bodyref.borrow().get_name().to_string(); // Clone the name to avoid multiple borrow

        if self
            .bodies
            .iter()
            .any(|body| body.borrow().get_name() == name)
        {
            return Err(MultibodyErrors::NameTaken);
        }

        if self
            .joints
            .iter()
            .any(|joint| joint.borrow().get_name() == name)
        {
            return Err(MultibodyErrors::NameTaken);
        }

        self.bodies.push(bodyref);
        Ok(())
    }

    pub fn add_joint(&mut self, jointref: JointRef) -> Result<(), MultibodyErrors> {
        // Return if a component with this name already exists
        let name = jointref.borrow().get_name().to_string(); // Clone the name to avoid multiple borrow

        if self
            .bodies
            .iter()
            .any(|body| body.borrow().get_name() == name)
        {
            return Err(MultibodyErrors::NameTaken);
        }

        if self
            .joints
            .iter()
            .any(|joint| joint.borrow().get_name() == name)
        {
            return Err(MultibodyErrors::NameTaken);
        }
        self.joints.push(jointref);
        Ok(())
    }

    fn find_base(&self) -> BodyRef {
        self.bodies
            .iter()
            .find(|bodyref| {
                let body = bodyref.borrow();
                match &*body {
                    BodyEnum::Base(_) => true,
                    BodyEnum::Body(_) => false,
                }
            })
            .unwrap()
            .clone()
    }

    pub fn find_body_by_name(&self, name: &str) -> Option<BodyRef> {
        self.bodies
            .iter()
            .find(|joint| joint.borrow().get_name() == name)
            .cloned()
    }

    pub fn find_joint_by_name(&self, name: &str) -> Option<JointRef> {
        self.joints
            .iter()
            .find(|joint| joint.borrow().get_name() == name)
            .cloned()
    }

    pub fn initialize(&mut self) -> Result<(), MultibodyErrors> {
        //gets validated in sort
        match self.sort() {
            Ok(()) => {}
            Err(error) => return Err(error),
        }

        Ok(())
    }

    pub fn run(&mut self) {
        self.update_joints();
        self.update_bodies();
        match self.algorithm {
            MultibodyAlgorithm::ArticulatedBody => {
                self.joints.iter_mut().for_each(|joint| joint.first_pass());
                self.joints
                    .iter_mut()
                    .rev()
                    .for_each(|joint| joint.second_pass());
                self.joints.iter_mut().for_each(|joint| joint.third_pass());
            }
            _ => {} //TODO: nothing for now
        }
    }

    pub fn sort(&mut self) -> Result<(), MultibodyErrors> {
        //must be valid in order to sort
        match self.validate() {
            Ok(()) => {}
            Err(error) => return Err(error),
        }

        let mut new_bodies = Vec::new();
        let mut new_joints = Vec::new();

        let base = self.find_base();
        new_bodies.push(base.clone());

        // recursive loop to identify all multibody elements
        find_joints_for_sort(base.clone(), &mut new_bodies, &mut new_joints);

        self.bodies = new_bodies;
        self.joints = new_joints;
        Ok(())
    }

    pub fn update_bodies(&mut self) {
        //TODO update body states based on joint states
        //calculate external forces
    }

    pub fn update_joints(&mut self) {
        // update the states based on new ode state
        // TODO
        // update the joint transforms based on generalized coords from new ode state
        self.joints
            .iter_mut()
            .for_each(|joint| joint.update_transforms());
    }

    pub fn validate(&mut self) -> Result<(), MultibodyErrors> {
        // check that there's exactly 1 base
        let num_bases = self
            .bodies
            .iter()
            .filter(|body_ref| match &*body_ref.borrow() {
                BodyEnum::Base(_) => true,
                _ => false,
            })
            .count();

        if num_bases == 0 {
            return Err(MultibodyErrors::NoBaseFound);
        };
        if num_bases > 1 {
            return Err(MultibodyErrors::TooManyBasesFound);
        };

        println!("Found exactly 1 base.");

        // check that the base has an outer joint
        if self
            .bodies
            .iter()
            .find(|bodyref| {
                let body = bodyref.borrow();
                match &*body {
                    BodyEnum::Base(base) => base.get_outer_joints().is_empty(),
                    BodyEnum::Body(_) => false,
                }
            })
            .is_some()
        {
            return Err(MultibodyErrors::BaseMissingOuterJoint);
        }

        println!("Base has at least 1 outer joint.");

        // check that every body has an inner joint
        if let Some(body) = self.bodies.iter().find(|bodyref| {
            let body = bodyref.borrow();
            match &*body {
                BodyEnum::Base(_) => false,
                BodyEnum::Body(body) => body.get_inner_joint().is_none(),
            }
        }) {
            return Err(MultibodyErrors::BodyMissingInnerJoint(body.clone()));
        }

        println!("All bodies have an inner joint.");

        // check that every joint has an inner and outer body connection
        if let Some(joint) = self
            .joints
            .iter()
            .find(|joint| joint.get_inner_body().is_none())
        {
            return Err(MultibodyErrors::JointMissingInnerBody(joint.clone()));
        }

        println!("All joints have an inner body.");

        if let Some(joint) = self
            .joints
            .iter()
            .find(|joint| joint.get_outer_body().is_none())
        {
            return Err(MultibodyErrors::JointMissingOuterBody(joint.clone()));
        }

        println!("All joints have an outer body.");

        Ok(())
    }
}

fn find_body_for_sort(
    joint: JointRef,
    new_bodies: &mut Vec<BodyRef>,
    new_joints: &mut Vec<JointRef>,
) {
    new_joints.push(joint.clone());
    find_joints_for_sort(joint.get_outer_body().unwrap().body, new_bodies, new_joints);
}

fn find_joints_for_sort(
    body: BodyRef,
    new_bodies: &mut Vec<BodyRef>,
    new_joints: &mut Vec<JointRef>,
) {
    new_bodies.push(body.clone());
    for joint_connection in body.get_outer_joints() {
        find_body_for_sort(joint_connection.joint, new_bodies, new_joints);
    }
}
