use crate::{
    base::BaseSystems,
    joint::JointStates,
    result::MultibodyResult,
    solver::rk4::solve_fixed_rk4,
};

use super::{
    actuator::Actuator,
    algorithms::MultibodyAlgorithm,
    base::Base,
    body::{Body, BodyTrait},
    joint::Joint,
    sensor::Sensor,
    MultibodyErrors,
};

use aerospace::{celestial_system::CelestialSystem, gravity::Gravity};
use ron::ser::{to_string_pretty, PrettyConfig};
use rotations::{prelude::Quaternion, RotationTrait};
use serde::{Deserialize, Serialize};
use spatial_algebra::{Acceleration, MotionVector, SpatialVector, Velocity};
use spice::Spice;
use std::{
    collections::HashMap,
    fs::File,
    io::Write,
    path::Path,
    time::{Instant, SystemTime},
};
use transforms::Transform;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MultibodySystem {
    pub actuators: HashMap<String, Actuator>,
    pub algorithm: MultibodyAlgorithm,
    pub base: Base,
    pub bodies: Vec<Body>,
    pub joints: Vec<Joint>,
    pub sensors: HashMap<String, Sensor>,
}

impl MultibodySystem {
    pub fn new() -> Self {
        Self {
            actuators: HashMap::new(),
            algorithm: MultibodyAlgorithm::ArticulatedBody, // for now, default to this
            base: Base::default(),
            bodies: Vec::new(),
            joints: Vec::new(),
            sensors: HashMap::new(),
        }
    }

    pub fn add_body(&mut self, body: Body) -> Result<(), MultibodyErrors> {
        // Return if a component with this name already exists
        if self.check_name_taken(&body.name) {
            return Err(MultibodyErrors::NameTaken(body.name));
        }

        self.bodies.push(body);
        Ok(())
    }

    pub fn add_celestial_system(&mut self, celestial: CelestialSystem) {
        self.base.add_celestial_system(celestial);
    }

    pub fn add_joint(&mut self, joint: Joint) -> Result<(), MultibodyErrors> {
        // Return if a component with this name already exists
        if self.check_name_taken(&joint.name) {
            return Err(MultibodyErrors::NameTaken(joint.name));
        }
        
        self.joints.push(joint);
        Ok(())
    }

    pub fn add_sensor(&mut self, sensor: Sensor) -> Result<(), MultibodyErrors> {
        // Return if a component with this name already exists
        if self.check_name_taken(&sensor.name) {
            return Err(MultibodyErrors::NameTaken(sensor.name));
        }

        self.sensors.insert(sensor.name.clone(), sensor);
        Ok(())
    }
    pub fn add_gravity(&mut self, gravity: Gravity) -> Result<(), MultibodyErrors> {
        Ok(self.base.add_basic_gravity(gravity)?)
    }

    fn write_derivative(&self, dx: &mut JointStates) {
        for (i, joint) in self.joints.iter().enumerate() {
            joint.model.state_derivative(&mut dx.0[i]);
        }
    }

    pub fn connect(
        &mut self,
        from_name: &str,
        to_name: &str,
        transform: Transform,
    ) -> Result<(), MultibodyErrors> {
        // if the base is the 'from' component
        if self.base.get_name() == from_name {
            // look for valid 'to' components (only joints for now)
            for joint in &mut self.joints {
                if joint.name == to_name {
                    joint.connect_inner_body(&mut self.base, transform)?;
                    return Ok(());
                }
                // if no joint found, return TODO: InvalidConnection or ComponentNotFound?
                return Err(MultibodyErrors::InvalidConnection);
            }

            // if the base is the 'to' component
            if self.base.get_name() == to_name {
                /*
                // look for valid 'to' components (only gravity for now)
                for (_, gravity) in &mut self.gravities {
                    if gravity.get_name() == from_name {
                        self.base.connect_gravity(gravity);
                        return Ok(());
                    }
                }
                */
                // if no gravity found, return TODO: InvalidConnection or ComponentNotFound?
                return Err(MultibodyErrors::InvalidConnection);
            }
        }

        // logic for bodies
        for body in &mut self.bodies {
            // if the body is the 'from' component
            if body.get_name() == from_name {
                // look for valid 'to' components (only joints for now)
                for joint in &mut self.joints {
                    if joint.name == to_name {
                        joint.connect_inner_body(body, transform)?;
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
                for joint in &mut self.joints {
                    if joint.name == from_name {
                        joint.connect_outer_body(body, transform)?;
                        return Ok(());
                    }
                }

                // body specific sensors
                for (_, sensor) in &mut self.sensors {
                    if sensor.name == from_name {
                        sensor.connect_to_body(body, transform)?;
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

    fn check_name_taken(&self, name: &str) -> bool {
        if self.base.name == name {
            return true;
        }

        if self.bodies.iter().any(|body| body.name == name) {
            return true;
        }

        if self.joints.iter().any(|joint| joint.name == name) {
            return true;
        }

        if self.sensors.iter().any(|(_, sensor)| sensor.name == name) {
            return true;
        }

        if self.actuators.iter().any(|(_, sensor)| sensor.name == name) {
            return true;
        }

        false
    }

    /// Reorders the bodies and joints appropraitely at the start of the multibody simulation
    fn permute(&mut self) {
        let mut joint_order = Vec::new();
        let mut body_order = Vec::new();

        // Helper function to recursively traverse joints and bodies
        fn traverse_body(
            sys: &MultibodySystem,
            body_id: &str,
            joint_order: &mut Vec<usize>,
            body_order: &mut Vec<usize>,
        ) {
            // Find the body by its ID
            if let Some((body_idx, body)) = sys
                .bodies
                .iter()
                .enumerate()
                .find(|(_, body)| body.name == body_id)
            {
                body_order.push(body_idx);

                // Process outer joints of this body
                for joint_id in &body.outer_joints {
                    if let Some((joint_idx, joint)) = sys
                        .joints
                        .iter()
                        .enumerate()
                        .find(|(_, joint)| joint.name == *joint_id)
                    {
                        joint_order.push(joint_idx);

                        // Recurse into connected outer bodies
                        if let Some(connection) = &joint.connections.outer_body {
                            traverse_body(sys, &connection.body_id, joint_order, body_order);
                        }
                    }
                }
            }
        }

        // Start the recursion from the base body's outer joints
        for joint_id in &self.base.outer_joints {
            if let Some((joint_idx, joint)) = self
                .joints
                .iter()
                .enumerate()
                .find(|(_, joint)| joint.name == *joint_id)
            {
                joint_order.push(joint_idx);

                // Recurse into connected outer bodies
                if let Some(connection) = &joint.connections.outer_body {
                    traverse_body(self, &connection.body_id, &mut joint_order, &mut body_order);
                }
            }
        }

        // Update the simulation's order of joints and bodies
        let mut ij = 0 as usize;
        for i in joint_order {
            self.joints.swap(ij, i);
            ij += 1;
        }

        let mut ib = 0 as usize;
        for i in body_order {
            self.bodies.swap(ib, i);
            ib += 1;
        }
    }

    pub fn run(
        &mut self,
        dx: &mut JointStates,
        x: &JointStates,
        t: f64,
        spice: &mut Option<Spice>,
    ) {
        self.set_state(x);
        self.update_base(t, spice);
        self.update_joints();
        self.update_forces();

        match self.algorithm {
            MultibodyAlgorithm::ArticulatedBody => {
                let n: usize = self.joints.len();

                // First Pass
                for i in 0..n {
                    let v_ij = if let Some(parent_index) = self.joints[i].inner_joint {
                        self.joints[parent_index].cache.v
                    } else {
                        Velocity::zeros()
                    };
                    self.joints[i].aba_first_pass(v_ij);
                }

                // Second Pass
                for i in (0..n).rev() {
                    let inner_is_base = self.joints[i].inner_joint.is_none();

                    // we split up updating the parent to avoid borrowing issues
                    if let Some((parent_ia, parent_pa)) =
                        self.joints[i].aba_second_pass(inner_is_base)
                    {
                        let parent_joint_index = self.joints[i].inner_joint.unwrap();
                        self.joints[parent_joint_index].cache.aba.inertia_articulated += parent_ia;
                        self.joints[parent_joint_index].cache.aba.p_big_a += parent_pa;
                    }
                }

                // Third Pass
                for i in 0..n {
                    // get acceleration of parent
                    // if parent is the base, (index None), parent acceleration is 0

                    let a_ij = if let Some(parent_index) = self.joints[i].inner_joint {
                        self.joints[parent_index].cache.a
                    } else {
                        Acceleration::zeros()
                    };

                    self.joints[i].aba_third_pass(a_ij);
                }
            }
            MultibodyAlgorithm::CompositeRigidBody => {
                // let n: usize = self.joints.len();

                // // Run the Recursive Newton Euler algorithm to calculate C.
                // let c = &mut self.crb_cache.as_mut().unwrap().c;

                // // first pass
                // for i in 0..n {
                //     let (a_ij, v_ij) = if let Some(parent_index) = self.parent_indeces[i] {
                //         (
                //             *self.joints[parent_index].cache.a,
                //             *self.joints[parent_index].cache.v,
                //         )
                //     } else {
                //         (Acceleration::zeros(), Velocity::zeros())
                //     };

                //     let joint = &mut self.joints[i];
                //     joint.rne_first_pass(a_ij, v_ij, false);
                // }

                // // Second Pass
                // for i in (0..n).rev() {
                //     let joint = &mut self.joints[i];
                //     joint.rne_set_tau();

                //     // set C values
                //     joint.set_c(c);

                //     if let Some(parent_index) = self.parent_indeces[i] {
                //         let ij_jof_from_jof = joint.get_transforms().ij_jof_from_jof;
                //         let parent_force = ij_jof_from_jof * joint.rne_get_force();

                //         let parent = &mut self.joints[parent_index];
                //         parent.rne_add_force(parent_force);
                //     }
                // }

                // // Solve for H with CRB
                // let h = &mut self.crb_cache.as_mut().unwrap().h;
                // h.fill(0.0);

                // // first pass
                // self.joints.iter_mut().for_each(|joint| joint.reset_ic());

                // // second pass
                // for i in 0..n {
                //     let joint = &mut self.joints[i];
                //     joint.set_h(h);

                //     if let Some(parent_index) = self.parent_indeces[i] {
                //         let joint_ic = joint.get_ic();
                //         let ij_jof_from_jof = joint.get_transforms().ij_jof_from_jof;
                //         let joint_ic_in_parent = ij_jof_from_jof * joint_ic;

                //         let parent = &mut self.joints[parent_index];
                //         parent.add_ic(joint_ic_in_parent);

                //         // just commented out until we finish the crb
                //         //let j = i;
                //         //while let Some(parent_index) = self.parent_indeces[j] {}
                //     };
                // }
            }
        }

        self.update_body_states(); //only update body states once joint accels have been calculated (so we can update body accel based on joint accel)
        self.update_sensors(); //only update sensors after body states have been updated
        self.write_derivative(dx);
    }

    pub fn save(&self, path: &Path) {
        let path = path.join("system.ron");

        // Open the file
        let mut file = match File::create(path) {
            Ok(file) => file,
            Err(e) => panic!("{e}"),
        };

        // Convert system to RON string
        let ron_string = match to_string_pretty(self, PrettyConfig::new()) {
            Ok(string) => string,
            Err(e) => panic!("{e}"),
        };

        // Write the RON string to the file
        match file.write_all(ron_string.as_bytes()) {
            Ok(_) => {}
            Err(e) => panic!("{e}"),
        }
    }

    fn set_state(&mut self, states: &JointStates) {
        for (i, joint) in self.joints.iter_mut().enumerate() {
            joint.model.state_vector_read(&states.0[i]);
        }
    }

    pub fn simulate(
        &mut self,
        name: &str,
        tstart: f64,
        tstop: f64,
        dt: f64,
        spice: &mut Option<Spice>,
    ) {
        // validate that the system can be simulated
        print!("validating multibody system...");
        match self.validate() {
            Ok(_) => println!("success"),
            Err(e) => panic!("{e}"),
        };

        // Sim results will be written to the results folder in the current directory
        // Create that folder if it does not exist
        let mut results = std::env::current_dir().unwrap();
        results.push("results");
        let results_dir = match std::fs::read_dir(&results) {
            Ok(results_dir) => results_dir,
            Err(_) => {
                std::fs::create_dir_all(&results).expect("Failed to create directory");
                std::fs::read_dir(&results).unwrap()
            }
        };

        // Read the directory entries
        let mut tokens = Vec::new();
        for entry in results_dir {
            let entry = entry.unwrap();
            let path = entry.path();

            // Check if the entry is a directory
            if path.is_dir() {
                if let Some(file_name) = path.file_name().and_then(|name| name.to_str()) {
                    // Panic if a result already exists with that name
                    if file_name == name {
                        panic!("Result name already exists. Please pick a different one.")
                    }
                    // Check if the folder starts with "sim"
                    if file_name.starts_with("sim") {
                        // Extract the token after "sim"
                        let token = file_name[3..].to_string();
                        tokens.push(token);
                    }
                }
            }
        }
        // if result name is not provided, autogenerate one
        // errors if the sim name is already taken
        let name = if name.is_empty() {
            if !tokens.is_empty() {
                let taken_ids: Vec<i64> = tokens
                    .iter()
                    .map(|string| match string.parse::<i64>() {
                        Ok(i) => i,
                        Err(_) => panic!(
                            "Got a folder name of sim<token> where token is 
                            not a numeric integer. This is not valid nadir syntax. Please 
                            change the name of the folder to be sim<i64> or something else that doesn't start with sim."
                        ),
                    })
                    .collect();
                let new_id = *taken_ids.iter().max().unwrap() + 1;
                format!("sim{new_id}")
            } else {
                "sim0".to_string()
            }
        } else {
            name.to_string()
        };

        // Resort the bodies and joints based on multibody tree philosophy
        self.permute();

        // calculate the joint mass properties
        for i in 0..self.joints.len() {
            // we use split_at_mut here to get an immutable reference to the inner joint and mutable ref to the current joint

            let (left, right) = self.joints.split_at_mut(i);
            let joint = &mut right[0];
            let inner_joint = joint.inner_joint;
            let ij_transforms = match inner_joint {
                None => None,
                //inner joint must be a lower index on the left, current index is on the right
                Some(id) => Some(&left[id].cache.transforms),
            };
            joint.update_transforms(ij_transforms);
            joint.calculate_joint_inertia(&self.bodies[i].mass_properties);
        }

        let start_time = SystemTime::now();
        let instant_start = Instant::now();

        let sim_start_time = Instant::now();

        // solve for the multibody system
        let result = solve_fixed_rk4(self, tstart, tstop, dt, spice);

        let (times, hashmap) = match result {
            Ok(result) => result,
            Err(e) => panic!("{e}"),
        };

        let sim_duration = sim_start_time.elapsed();
        let total_duration = instant_start.elapsed();

        let result = MultibodyResult {
            name: name.clone(),
            sim_time: times,
            result: hashmap,
            time_start: start_time,
            sim_duration: sim_duration,
            total_duration: total_duration,
        };

        let sim_duration = utilities::format_duration(sim_duration);
        let sim_name = name.clone();
        println!("Simulation '{sim_name}' completed in {sim_duration}.");
        result.save(self);
    }

    pub fn update_body_states(&mut self) {
        for (i, body) in self.bodies.iter_mut().enumerate() {
            let inner_joint = &self.joints[i];
            let transforms = &inner_joint.cache.transforms;
            let body_from_joint = transforms.ob_from_jof;

            let base_from_body = transforms.base_from_jof * transforms.jof_from_ob;
            let joint_a = inner_joint.cache.a;

            let body_a = body_from_joint * joint_a;
            // accel in body to accel in base is just a rotation, translation due to rotation should be accounted for in calc of body_a
            let body_a_in_base = base_from_body * body_a;
            //let body_a_in_base_rotation = base_from_body.0.rotation.transform(*body_a.rotation());
            //let body_a_in_base_translation = base_from_body.0.rotation.transform(*body_a.translation());
            //let body_a_in_base = Acceleration(MotionVector(SpatialVector::new(
            //body_a_in_base_rotation,
            //body_a_in_base_translation,
            //)));
            let joint_v = inner_joint.cache.v;
            let body_v = body_from_joint * joint_v;
            // velocity in body to velocity in base is just a rotation, translation due to rotation should be accounted for in calc of body_v
            let body_v_in_base_rotation = base_from_body.0.rotation.transform(*body_v.rotation());
            let body_v_in_base_translation =
                base_from_body.0.rotation.transform(*body_v.translation());
            let body_v_in_base = Velocity(MotionVector(SpatialVector::new(
                body_v_in_base_rotation,
                body_v_in_base_translation,
            )));
            body.state.acceleration_body = *body_a.translation();
            body.state.acceleration_base = *body_a_in_base.translation();
            body.state.angular_accel_body = *body_a.rotation();
            body.state.velocity_base = *body_v_in_base.translation();
            body.state.angular_rate_body = *body_v.rotation();

            let body_from_base = base_from_body.0.inv();
            body.state.position_base = body_from_base.translation.vec();
            body.state.attitude_base = Quaternion::from(&body_from_base.rotation);
        }
    }

    fn update_base(&mut self, t: f64, spice: &mut Option<Spice>) {
        self.base.update(t, spice).unwrap();
    }

    fn update_forces(&mut self) {
        for (i, body) in self.bodies.iter_mut().enumerate() {
            let inner_joint = &mut self.joints[i];

            // get transforms
            let transforms = &inner_joint.cache.transforms;

            // calculate gravity for the outer body
            match &self.base.system {
                BaseSystems::Basic(gravity) => {
                    if let Some(gravity) = gravity {
                        body.calculate_gravity(&transforms.ob_from_base, gravity);
                    }
                }
                BaseSystems::Celestial(celestial) => {
                    body.calculate_gravity_celestial(&transforms.ob_from_base, celestial)
                }
            }

            // calculate total external forces for the outer body
            body.calculate_external_force();

            // transform force to joint
            // cross product terms in spatial calculation will convert force at body cg to torque about joint
            inner_joint.cache.f = transforms.jof_from_ob * body.state.external_spatial_force_body;
        }
    }

    fn update_joints(&mut self) {
        for i in 0..self.joints.len() {
            // we use split_at_mut here to get an immutable reference to the inner joint and mutable ref to the current joint
            let (left, right) = self.joints.split_at_mut(i);
            let joint = &mut right[0];            
            let ij_transforms = match joint.inner_joint {
                // inner body is the base
                None => None,
                //inner joint must be a lower index on the left, current index is on the right
                Some(id) => Some(&left[id].cache.transforms),
            };
            joint.update_transforms(ij_transforms);
            joint.calculate_joint_inertia(&self.bodies[i].mass_properties);

            joint.model.calculate_vj();
            joint.model.calculate_tau();
        }
    }

    fn update_sensors(&mut self) {
        for body in &mut self.bodies {
            body.update_sensors(&mut self.sensors);
        }
    }

    pub fn validate(&self) -> Result<(), MultibodyErrors> {
        // check that the base has an outer joint
        let base_outer_joints = &self.base.outer_joints;
        if base_outer_joints.is_empty() {
            return Err(MultibodyErrors::BaseMissingOuterJoint);
        }

        // check that all base outer joints exist
        for id in base_outer_joints {
            if !self.joints.iter().any(|joint| joint.name == *id) {
                return Err(MultibodyErrors::JointNotFound(id.clone()));
            }
        }

        // check that every body has an inner joint
        for body in &self.bodies {
            if body.inner_joint.is_none() {
                return Err(MultibodyErrors::BodyMissingInnerJoint(body.name.clone()));
            }
        }

        // check that all body inner joints exist
        for body in &self.bodies {
            if let Some(id) = &body.inner_joint {
                if !self.joints.iter().any(|joint| joint.name == *id) {
                    return Err(MultibodyErrors::JointNotFound(id.clone()));
                }
            }
        }

        // check that every joint has an inner and outer body connection
        for joint in &self.joints {
            if joint.connections.inner_body.is_none() {
                return Err(MultibodyErrors::JointMissingInnerBody(joint.name.clone()));
            }
            if joint.connections.outer_body.is_none() {
                return Err(MultibodyErrors::JointMissingOuterBody(joint.name.clone()));
            }
        }

        // check that every joint inner and outer body exists
        for joint in &self.joints {
            let body_id = joint
                .connections
                .inner_body
                .as_ref()
                .unwrap()
                .body_id
                .clone();
            if !self.bodies.iter().any(|body| body.name == body_id) {
                if self.base.name != body_id {
                    return Err(MultibodyErrors::BodyNotFound(body_id));
                }
            }

            let body_id = joint
                .connections
                .outer_body
                .as_ref()
                .unwrap()
                .body_id
                .clone();
            if !self.bodies.iter().any(|body| body.name == body_id) {
                if self.base.name != body_id {
                    return Err(MultibodyErrors::BodyNotFound(body_id));
                }
            }
        }
        Ok(())
    }
}
