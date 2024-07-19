use gravity::{ConstantGravity, GravityEnum};

use super::{
    algorithms::MultibodyAlgorithm,
    base::Base,
    body::{Body, BodyTrait},
    joint::{Joint, JointTrait},
    result::MultibodyResult,
    system_sim::MultibodySystemSim,
    MultibodyErrors, MultibodyTrait,
};

use std::collections::HashMap;
use uuid::Uuid;

#[derive(Debug, Clone)]

pub struct MultibodySystem{

    pub algorithm: MultibodyAlgorithm,
    pub base: Option<Base>,
    pub bodies: HashMap<Uuid, Body>,
    pub gravity: Option<GravityEnum>, 
    pub joints: HashMap<Uuid, Joint>,
}

impl MultibodySystem {
    pub fn new() -> Self {
        Self {
            algorithm: MultibodyAlgorithm::ArticulatedBody, // for now, default to this
            base: None,
            bodies: HashMap::new(),
            gravity: None,
            joints: HashMap::new(),
        }
    }

    pub fn add_base(&mut self, base: Base) -> Result<(), MultibodyErrors> {
        // return if this is a Base and we already have a Base
        if self.base.is_some() {
            return Err(MultibodyErrors::BaseAlreadyExists);
        } else {
            self.base = Some(base)
        }
        Ok(())
    }

    pub fn add_body(&mut self, body: Body) -> Result<(), MultibodyErrors> {
        // Return if a component with this name already exists
        if self.check_name_taken(body.get_name()) {
            return Err(MultibodyErrors::NameTaken);
        }

        self.bodies.insert(*body.get_id(), body);
        Ok(())
    }

    pub fn add_joint(&mut self, joint: Joint) -> Result<(), MultibodyErrors> {
        // Return if a component with this name already exists
        if self.check_name_taken(joint.get_name()) {
            return Err(MultibodyErrors::NameTaken);
        }

        self.joints.insert(*joint.get_id(), joint);
        Ok(())
    }

    fn check_name_taken(&self, name: &str) -> bool {
        if let Some(base) = &self.base {
            if base.get_name() == name {
                return true;
            }
        }

        if self.bodies.iter().any(|(_, body)| body.get_name() == name) {
            return true;
        }

        if self
            .joints
            .iter()
            .any(|(_, joint)| joint.get_name() == name)
        {
            return true;
        }

        false
    }

    pub fn simulate(&self, name: String, tstart: f64, tstop: f64, dt: f64) -> MultibodyResult {
        let mut sim = MultibodySystemSim::from(self.clone());
        sim.simulate(name, tstart, tstop, dt)
    }

    pub fn validate(&self) {
        // check that there's a base
        let base = &self.base;

        if base.is_none() {
            panic!("No base found.")
        };

        println!("Found exactly 1 base.");

        // check that the base has an outer joint
        if let Some(base) = base {
            let base_outer_joints = base.get_outer_joints();
            if base_outer_joints.is_empty() {
                panic!("Base missing any outer joints.")
            }

            println!("Base has at least 1 outer joint.");

            // check that all base outer joints exist
            for id in base_outer_joints {
                if !self.joints.contains_key(id) {
                    panic!("Invalid base outer joint ID: {}", id);
                }
            }
            println!("All base outer joint IDs exist in the map.");
        }

        // check that every body has an inner joint
        for (id, body) in &self.bodies {
            if body.get_inner_joint_id().is_none() {
                panic!("Body ({}) does not have an inner joint.", id);
            }
        }
        println!("All bodies have an inner joint ID.");

        // check that all body inner joints exist
        for (body_id, body) in &self.bodies {
            if let Some(joint_id) = body.get_inner_joint_id() {
                if !self.joints.contains_key(joint_id) {
                    panic!(
                        "Invalid inner joint ID ({}) for body ({}).",
                        joint_id, body_id
                    );
                }
            }
        }
        println!("All inner joint IDs exist in the map.");

        // check that every joint has an inner and outer body connection
        for (id, joint) in &self.joints {
            if joint.get_inner_body_id().is_none() {
                panic!("Joint ({}) does not have an inner body.", id);
            }
            if joint.get_outer_body_id().is_none() {
                panic!("Joint ({}) does not have an outer body.", id);
            }
        }
        println!("All joints have an inner and outer body.");

        // check that every joint inner and outer body exists
        for (joint_id, joint) in &self.joints {
            if let Some(body_id) = joint.get_inner_body_id() {
                if !self.bodies.contains_key(body_id) {
                    if let Some(base) = &self.base {
                        if base.get_id() != body_id {
                            panic!(
                                "Invalid inner body ID ({}) for joint ({}).",
                                body_id, joint_id
                            );
                        }
                    }
                }
            }
            if let Some(body_id) = joint.get_outer_body_id() {
                if !self.bodies.contains_key(body_id) {
                    panic!(
                        "Invalid outer body ID ({}) for joint ({}).",
                        body_id, joint_id
                    );
                }
            }
        }
        println!("All joints inner and outer bodies exist in the map.");
        println!("System validation complete!");
    }
}
