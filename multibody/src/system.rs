use crate::{component::MultibodyComponent, sensor::Sensor};

use super::{
    aerospace::MultibodyGravity,
    algorithms::MultibodyAlgorithm,
    base::Base,
    body::{Body, BodyTrait},
    joint::{Joint, JointTrait},
    result::MultibodyResult,
    system_sim::MultibodySystemSim,
    MultibodyErrors, MultibodyTrait,
};
use serde::{Serialize, Deserialize};
use std::collections::HashMap;
use transforms::Transform;
use uuid::Uuid;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MultibodySystem {
    pub algorithm: MultibodyAlgorithm,
    pub base: Option<Base>,
    pub bodies: HashMap<Uuid, Body>,
    pub gravities: HashMap<Uuid, MultibodyGravity>,
    pub joints: HashMap<Uuid, Joint>,
    pub sensors: HashMap<Uuid, Sensor>,
}

impl MultibodySystem {
    pub fn new() -> Self {
        Self {
            algorithm: MultibodyAlgorithm::ArticulatedBody, // for now, default to this
            base: None,
            bodies: HashMap::new(),
            gravities: HashMap::new(),
            joints: HashMap::new(),
            sensors: HashMap::new(),
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

    pub fn add_gravity(&mut self, gravity: MultibodyGravity) -> Result<(), MultibodyErrors>{
         // Return if a component with this name already exists
         if self.check_name_taken(gravity.get_name()) {
            return Err(MultibodyErrors::NameTaken);
        }
        self.gravities.insert(*gravity.get_id(), gravity);
        Ok(())
    }

    pub fn add_sensor(&mut self, sensor: Sensor) -> Result<(), MultibodyErrors> {
        // Return if a component with this name already exists
        if self.check_name_taken(sensor.get_name()) {
            return Err(MultibodyErrors::NameTaken);
        }

        self.sensors.insert(*sensor.get_id(), sensor);
        Ok(())
    }
    pub fn connect_gravity(
        &mut self,
        gravity_id: &Uuid,
        body_id: &Uuid,
    ) -> Result<(), MultibodyErrors> {
        let mut result = Err(MultibodyErrors::BodyNotFound);
        if let Some(base) = &mut self.base {
            if base.get_id() == body_id {
                base.gravity.push(*gravity_id);
                result = Ok(());
            }
        }
        if let Some(body) = self.bodies.get_mut(body_id) {
            body.gravity.push(*gravity_id);
            result = Ok(());
        }
        result
    }

    pub fn connect(
        &mut self,
        from_name: &str,
        to_name: &str,
        transform: Option<Transform>,
    ) -> Result<(), MultibodyErrors> {
        // logic for the base
        if let Some(base) = &mut self.base {
            // if the base is the 'from' component
            if base.get_name() == from_name {
                // look for valid 'to' components (only joints for now)
                for (_, joint) in &mut self.joints {
                    if joint.get_name() == to_name {
                        if transform.is_none() {
                            return Err(MultibodyErrors::NoTransformFound);
                        }
                        joint.connect_inner_body(base, transform.unwrap())?; // unwrap should be safe since we early returned
                        return Ok(());
                    }
                }
                // if no joint found, return TODO: InvalidConnection or ComponentNotFound?
                return Err(MultibodyErrors::InvalidConnection);
            }

            // if the base is the 'to' component
            if base.get_name() == to_name {
                // look for valid 'to' components (only gravity for now)
                for (_, gravity) in &mut self.gravities {
                    if gravity.get_name() == from_name {
                        base.connect_gravity(gravity);
                        return Ok(());
                    }
                }

                // if no gravity found, return TODO: InvalidConnection or ComponentNotFound?
                return Err(MultibodyErrors::InvalidConnection);
            }
        }

        // logic for bodies
        for (_, body) in &mut self.bodies {
            // if the body is the 'from' component
            if body.get_name() == from_name {
                if transform.is_none() {
                    return Err(MultibodyErrors::NoTransformFound);
                }

                // look for valid 'to' components (only joints for now)
                for (_, joint) in &mut self.joints {
                    if joint.get_name() == to_name {
                        joint.connect_inner_body(body, transform.unwrap())?; //unwrap should be safe since we early returned
                        return Ok(());
                    }
                }
                // valid connections found
                return Err(MultibodyErrors::InvalidConnection);
            }

            // if the body is the 'to' component
            if body.get_name() == to_name {
                // look for valid 'to' components (only joints, gravity for now, will need sensor and actuators too)

                // joints
                for (_, joint) in &mut self.joints {
                    if joint.get_name() == from_name {
                        // transform is required
                        if transform.is_none() {
                            return Err(MultibodyErrors::NoTransformFound);
                        }
                        joint.connect_outer_body(body, transform.unwrap())?; //unwrap should be safe since we early returned
                        return Ok(());
                    }
                }

                // body specific gravity
                for (_, gravity) in &mut self.gravities {
                    if gravity.get_name() == from_name {
                        body.connect_gravity(gravity);
                        return Ok(());
                    }
                }

                // no valid connections found
                return Err(MultibodyErrors::InvalidConnection);
            }
        }

        // components were not found
        return Err(MultibodyErrors::InvalidConnection);
    }

    pub fn delete(&mut self, name: &str) -> Result<(), MultibodyErrors> {
        if let Some((component,id)) = self.get_from_name(name) {
            match component {
                MultibodyComponent::Base => {
                    // remove the base from any connected components 
                    self.joints.iter_mut().for_each(|(id,joint)| {
                        if let Some(inner_connection) = &joint.get_connections().inner_body {
                            if inner_connection.body_id == *id {
                                joint.delete_inner_body_id();
                            }
                        }
                    });
                    self.base = None;
                    Ok(())
                }
                MultibodyComponent::Body => {
                    // remove the body from any connected components 
                    self.joints.iter_mut().for_each(|(id,joint)| {
                        if let Some(connection) = &joint.get_connections().inner_body {
                            if connection.body_id == *id {
                                joint.delete_inner_body_id();
                            }
                        }
                        if let Some(connection) = &joint.get_connections().outer_body {
                            if connection.body_id == *id {
                                joint.delete_outer_body_id();
                            }
                        }
                    });
                    self.bodies.remove(&id);
                    Ok(())
                }
                MultibodyComponent::Joint => {
                    // remove the joint from any connected components 
                    if let Some(joint) = self.joints.get_mut(&id) {
                        if let Some(connection) = &joint.get_connections().inner_body {
                            self.bodies.iter_mut().for_each(|(body_id,body)| {                                
                                if connection.body_id == *body_id {
                                    body.delete_outer_joint(&id);
                                }
                                });
                            }
                        
                        if let Some(connection) = &joint.get_connections().outer_body {
                            self.bodies.iter_mut().for_each(|(body_id,body)| {                                
                                if connection.body_id == *body_id {
                                    body.delete_inner_joint();
                                }
                            });
                        }
                    }
                    self.joints.remove(&id);
                    Ok(())
                }
                MultibodyComponent::Gravity => {
                    self.delete_gravity(&id);
                    Ok(())
                }
                MultibodyComponent::Sensor => {
                    self.delete_sensor(&id);
                    Ok(())
                }
            }
        } else {
            Err(MultibodyErrors::ComponentNotFound(name.to_string()))
        }
    }

    pub fn get_from_name(&self, name: &str) -> Option<(MultibodyComponent, Uuid)> {
        if let Some(base) = &self.base {
            if base.get_name() == name {
                return Some((MultibodyComponent::Base, *base.get_id()));
            }
        }
        for (_, body) in &self.bodies {
            if body.get_name() == name {
                return Some((MultibodyComponent::Body, *body.get_id()));
            }
        }
        for (_, joint) in &self.joints {
            if joint.get_name() == name {
                return Some((MultibodyComponent::Joint, *joint.get_id()));
            }
        }
        for (_, gravity) in &self.gravities {
            if gravity.get_name() == name {
                return Some((MultibodyComponent::Gravity, *gravity.get_id()));
            }
        }
        for (_, sensor) in &self.sensors {
            if sensor.get_name() == name {
                return Some((MultibodyComponent::Sensor, *sensor.get_id()));
            }
        }
        None
    }

    pub fn delete_gravity(&mut self, gravity_id: &Uuid) {
        if let Some(base) = &mut self.base {
            base.gravity.retain(|&id| id != *gravity_id);
        }
        for (_, body) in &mut self.bodies {
            body.gravity.retain(|&id| id != *gravity_id);
        }
        self.gravities.remove(gravity_id);
    }

    pub fn delete_sensor(&mut self, sensor_id: &Uuid) {        
        for (_, body) in &mut self.bodies {
            body.sensors.retain(|&id| id != *sensor_id);
        }
        self.sensors.remove(sensor_id);
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

        if self.gravities.iter().any(|(_, g)| g.get_name() == name) {
            return true;
        }

        if self.sensors.iter().any(|(_, sensor)| sensor.get_name() == name) {
            return true;
        }

        false
    }

    pub fn simulate(&self, name: String, tstart: f64, tstop: f64, dt: f64) -> Result<MultibodyResult, MultibodyErrors> {
        let mut sim = MultibodySystemSim::try_from(self.clone())?;
        sim.simulate(name, tstart, tstop, dt)
    }

    pub fn validate(&self) -> Result<(), MultibodyErrors> {
        // check that there's a base
        let base = &self.base;

        if base.is_none() {
            return Err(MultibodyErrors::NoBaseFound);
        };

        // check that the base has an outer joint
        if let Some(base) = base {
            let base_outer_joints = base.get_outer_joints();
            if base_outer_joints.is_empty() {
                return Err(MultibodyErrors::BaseMissingOuterJoint);
            }

            // check that all base outer joints exist
            for id in base_outer_joints {
                if !self.joints.contains_key(id) {
                    return Err(MultibodyErrors::JointNotFound);
                }
            }
        }

        // check that every body has an inner joint
        for (id, body) in &self.bodies {
            if body.get_inner_joint_id().is_none() {
                return Err(MultibodyErrors::BodyMissingInnerJoint(*id));
            }
        }

        // check that all body inner joints exist
        for (_, body) in &self.bodies {
            if let Some(joint_id) = body.get_inner_joint_id() {
                if !self.joints.contains_key(joint_id) {
                    return Err(MultibodyErrors::JointNotFound);
                }
            }
        }

        // check that every joint has an inner and outer body connection
        for (id, joint) in &self.joints {
            if joint.get_inner_body_id().is_none() {
                return Err(MultibodyErrors::JointMissingInnerBody(*id));
            }
            if joint.get_outer_body_id().is_none() {
                return Err(MultibodyErrors::JointMissingOuterBody(*id));
            }
        }

        // check that every joint inner and outer body exists
        for (_, joint) in &self.joints {
            if let Some(body_id) = joint.get_inner_body_id() {
                if !self.bodies.contains_key(body_id) {
                    if let Some(base) = &self.base {
                        if base.get_id() != body_id {
                            return Err(MultibodyErrors::BodyNotFound);
                        }
                    }
                }
            }
            if let Some(body_id) = joint.get_outer_body_id() {
                if !self.bodies.contains_key(body_id) {
                    return Err(MultibodyErrors::BodyNotFound);
                }
            }
        }        
        Ok(())
    }
}
