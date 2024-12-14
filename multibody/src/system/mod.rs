use crate::{
    base::{BaseRef, BaseSystems},
    body::BodyRef,
    joint::{InnerBody, JointRef, JointStates},
    result::MultibodyResult,
    solver::rk4::solve_fixed_rk4,
};

use super::{
    actuator::Actuator,
    algorithms::MultibodyAlgorithm,
    base::Base,
    body::{Body, BodyTrait},
    joint::Joint,
    result::MultibodyResultTrait,
    sensor::Sensor,
    MultibodyErrors,
};

use aerospace::{celestial_system::CelestialSystem, gravity::Gravity};
use ron::ser::{to_string_pretty, PrettyConfig};
use rotations::{prelude::Quaternion, RotationTrait};
use serde::{Deserialize, Serialize};
use spice::Spice;
use std::{
    cell::RefCell,
    collections::HashMap,
    fs::File,
    io::Write,
    path::Path,
    rc::Rc,
    time::{Instant, SystemTime},
};
use transforms::Transform;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MultibodySystem {
    pub actuators: Vec<Actuator>,
    pub algorithm: MultibodyAlgorithm,
    pub base: BaseRef,
    pub bodies: Vec<BodyRef>,
    pub joints: Vec<JointRef>,
    pub sensors: Vec<Sensor>,
}

impl MultibodySystem {
    pub fn new() -> Self {
        Self {
            actuators: Vec::new(),
            algorithm: MultibodyAlgorithm::ArticulatedBody, // for now, default to this
            base: Rc::new(RefCell::new(Base::default())),
            bodies: Vec::new(),
            joints: Vec::new(),
            sensors: Vec::new(),
        }
    }

    pub fn add_body(&mut self, body: Body) -> Result<(), MultibodyErrors> {
        // Return if a component with this name already exists
        if self.check_name_taken(&body.name) {
            return Err(MultibodyErrors::NameTaken(body.name));
        }

        self.bodies.push(Rc::new(RefCell::new(body)));
        Ok(())
    }

    pub fn add_celestial_system(&mut self, celestial: CelestialSystem) {
        self.base.borrow_mut().add_celestial_system(celestial);
    }

    pub fn add_joint(&mut self, joint: Joint) -> Result<(), MultibodyErrors> {
        // Return if a component with this name already exists
        if self.check_name_taken(&joint.name) {
            return Err(MultibodyErrors::NameTaken(joint.name));
        }

        self.joints.push(Rc::new(RefCell::new(joint)));
        Ok(())
    }

    pub fn add_sensor(&mut self, sensor: Sensor) -> Result<(), MultibodyErrors> {
        // Return if a component with this name already exists
        if self.check_name_taken(&sensor.name) {
            return Err(MultibodyErrors::NameTaken(sensor.name));
        }

        self.sensors.push(sensor);
        Ok(())
    }
    pub fn add_gravity(&mut self, gravity: Gravity) -> Result<(), MultibodyErrors> {
        Ok(self.base.borrow_mut().add_basic_gravity(gravity)?)
    }

    fn write_derivative(&self, dx: &mut JointStates) {
        for (i, joint) in self.joints.iter().enumerate() {
            joint.borrow().state_derivative(&mut dx.0[i]);
        }
    }

    pub fn connect(
        &mut self,
        from_name: &str,
        to_name: &str,
        transform: Transform,
    ) -> Result<(), MultibodyErrors> {
        // if the base is the 'from' component
        let base_name = self.base.borrow().name.clone();
        if base_name == from_name {
            // look for valid 'to' components (only joints for now)
            for jointref in &self.joints {
                let joint_name = jointref.borrow().name.clone();
                if joint_name == to_name {
                    jointref
                        .borrow_mut()
                        .connect_base(self.base.clone(), transform)?;
                    self.base.borrow_mut().connect_outer_joint(jointref)?;
                    return Ok(());
                }
                // if no joint found, return TODO: InvalidConnection or ComponentNotFound?
                return Err(MultibodyErrors::InvalidConnection);
            }
        }

        // logic for bodies
        for bodyref in &self.bodies {
            let body_name = bodyref.borrow().name.clone();
            // if the body is the 'from' component
            if body_name == from_name {
                // look for valid 'to' components (only joints for now)
                for jointref in &self.joints {
                    let joint_name = jointref.borrow().name.clone();
                    if joint_name == to_name {
                        jointref
                            .borrow_mut()
                            .connect_inner_body(bodyref, transform)?;

                        bodyref.borrow_mut().connect_outer_joint(jointref)?;
                        return Ok(());
                    }
                }
                // valid connections found
                return Err(MultibodyErrors::InvalidConnection);
            }

            // if the body is the 'to' component
            if body_name == to_name {
                // look for valid 'to' components (only joints, gravity for now, will need sensor and actuators too)

                // joints
                for jointref in &self.joints {
                    let joint_name = jointref.borrow().name.clone();
                    if joint_name == from_name {
                        jointref
                            .borrow_mut()
                            .connect_outer_body(bodyref, transform)?;
                        bodyref.borrow_mut().connect_inner_joint(jointref)?;
                        return Ok(());
                    }
                }

                // body specific sensors
                for sensor in &mut self.sensors {
                    if sensor.name == from_name {
                        sensor.connect_to_body(bodyref, transform)?;
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
        if self.base.borrow().name == name {
            return true;
        }

        if self.bodies.iter().any(|body| body.borrow().name == name) {
            return true;
        }

        if self.joints.iter().any(|joint| joint.borrow().name == name) {
            return true;
        }

        if self.sensors.iter().any(|sensor| sensor.name == name) {
            return true;
        }

        if self.actuators.iter().any(|sensor| sensor.name == name) {
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
            body_rc: &Rc<RefCell<Body>>,
            joint_order: &mut Vec<usize>,
            body_order: &mut Vec<usize>,
        ) {
            let body = body_rc.borrow();
            body_order.push(
                sys.bodies
                    .iter()
                    .position(|b| Rc::ptr_eq(b, body_rc))
                    .expect("Body not found in system"),
            );

            // Process outer joints of this body
            for outer_joint_weak in &body.outer_joints {
                if let Some(outer_joint_rc) = outer_joint_weak.upgrade() {
                    let outer_joint = outer_joint_rc.borrow();
                    joint_order.push(
                        sys.joints
                            .iter()
                            .position(|j| Rc::ptr_eq(j, &outer_joint_rc))
                            .expect("Joint not found in system"),
                    );

                    // Recurse into connected outer bodies
                    if let Some(connection) = &outer_joint.connections.outer_body {
                        traverse_body(sys, &connection.body, joint_order, body_order);
                    }
                }
            }
        }

        // Start the recursion from the base body's outer joints
        for outer_joint_weak in &self.base.borrow().outer_joints {
            if let Some(outer_joint_rc) = outer_joint_weak.upgrade() {
                let outer_joint = outer_joint_rc.borrow();
                joint_order.push(
                    self.joints
                        .iter()
                        .position(|j| Rc::ptr_eq(j, &outer_joint_rc))
                        .expect("Joint not found in system"),
                );

                // Recurse into connected outer bodies
                if let Some(connection) = &outer_joint.connections.outer_body {
                    traverse_body(self, &connection.body, &mut joint_order, &mut body_order);
                }
            }
        }

        // Update the simulation's order of joints and bodies
        for (i, &joint_idx) in joint_order.iter().enumerate() {
            self.joints.swap(i, joint_idx);
        }
        for (i, &body_idx) in body_order.iter().enumerate() {
            self.bodies.swap(i, body_idx);
        }

        // Set inner joints for the joints
        for joint_ref in &self.joints {
            let mut joint = joint_ref.borrow_mut();

            if let Some(inner_body) = joint.connections.inner_body.as_ref() {
                joint.inner_joint = match inner_body {
                    InnerBody::Base(_) => None,
                    InnerBody::Body(connection) => connection
                        .body
                        .borrow()
                        .inner_joint
                        .as_ref()
                        .and_then(|weak_joint| weak_joint.upgrade()),
                };
            }
        }
    }

    pub fn run(
        &mut self,
        dx: &mut JointStates,
        x: &JointStates,
        t: f64,
        spice: &mut Option<Spice>,
    ) {
        self.set_state(x); // write the integrated states back in to the joints
        self.update_base(t, spice); // update epoch based celestial states based on new time
        self.update_joints(); // update joint state based quantities like transforms
        self.update_body_states(); // need to update the body position for gravity calcs prior to update_forces
                                   // self.update_actuators(); // run the actuators after software has produced commands and before updating forces on the body
        self.update_forces(); // update forces after body states for position based gravity calcs

        match self.algorithm {
            MultibodyAlgorithm::ArticulatedBody => {
                // First Pass
                for joint in &self.joints {
                    joint.borrow_mut().aba_first_pass();
                }

                // Second Pass
                for joint in self.joints.iter().rev() {
                    joint.borrow_mut().aba_second_pass();
                }

                // Third Pass
                for joint in &self.joints {
                    joint.borrow_mut().aba_third_pass();
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
        self.update_body_acceleration(); // update body acceleration after joint accelerations are calculated
                                         //self.update_accelerometers(); // would need to update accelerometers here, or maybe just ZOH from before?
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
        for (i, joint) in self.joints.iter().enumerate() {
            joint.borrow_mut().model.state_vector_read(&states.0[i]);
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
        let total_time = Instant::now();
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

        // initialize the components initial conditions and secondary states
        for jointref in &self.joints {
            let mut joint = jointref.borrow_mut();
            joint.update_transforms();
            joint.calculate_joint_inertia();
        }
        self.update_body_states();
        self.update_sensors();

        // start timers for performance metrics
        let system_time = SystemTime::now();
        let start_time = Instant::now();

        // solve the multibody system
        let result = solve_fixed_rk4(self, tstart, tstop, dt, spice);

        let (times, hashmap, celestial) = match result {
            Ok(result) => result,
            Err(e) => panic!("{e}"),
        };

        let sim_duration = start_time.elapsed();
        let sim_duration_str = utilities::format_duration(sim_duration);
        let sim_name = name.clone();
        println!("Simulation '{sim_name}' completed in {sim_duration_str}.");
        let save_time = Instant::now();
        // collect meshes from the system into the result so we can use for animation
        let mut meshes = HashMap::new();
        for bodyref in &self.bodies {
            let body = bodyref.borrow();
            if let Some(mesh) = &body.mesh {
                meshes.insert(body.name.clone(), mesh.clone());
            }
        }

        let result = MultibodyResult {
            name: name.clone(),
            sim_time: times,
            result: hashmap,
            time_start: system_time,
            sim_duration: sim_duration,
            bodies: meshes,
            celestial: celestial,
        };

        result.save(self);
        let save_duration = save_time.elapsed();
        let save_duration_str = utilities::format_duration(save_duration);
        println!("Results saved in {save_duration_str}.");
        let total_duration = utilities::format_duration(total_time.elapsed());
        println!("Total duration: {total_duration}.");
    }

    fn update_body_acceleration(&mut self) {        
        for bodyref in &self.bodies {
            let mut body = bodyref.borrow_mut();
            body.update_acceleration();
        }
    }

    fn update_body_states(&mut self) {
        for bodyref in &self.bodies {
            let mut body = bodyref.borrow_mut();
            body.update_state();
        }
    }

    fn update_base(&mut self, t: f64, spice: &mut Option<Spice>) {
        self.base.borrow_mut().update(t, spice).unwrap();
    }

    fn update_forces(&mut self) {
        for bodyrc in self.bodies.iter_mut() {
            let mut body = bodyrc.borrow_mut();
            let inner_joint_weak = body.inner_joint.as_mut().unwrap();
            let inner_joint_rc = inner_joint_weak.upgrade().unwrap();
            let mut inner_joint = inner_joint_rc.borrow_mut();

            // get transforms
            let transforms = &inner_joint.cache.transforms;
            // calculate gravity for the outer body
            match &self.base.borrow().system {
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
        for jointref in &self.joints {
            let mut joint = jointref.borrow_mut();
            joint.update_transforms();
            // joint.calculate_joint_inertia(); // currently only need to do this once at initialization
            joint.calculate_vj();
            joint.model.calculate_tau();
        }
    }

    /// Updates the results in the system based on their current states
    /// Used each timestep for reporting the current states
    pub fn update_results(&mut self) {
        // update body results
        for body in &mut self.bodies {
            body.borrow_mut().update_result();
        }

        // update joint results
        for joint in &mut self.joints {
            joint.borrow_mut().model.update_result();
        }

        // update sensor results
        for sensor in &mut self.sensors {
            sensor.model.update_result();
        }

        // update celestial results
        match &mut self.base.borrow_mut().system {
            BaseSystems::Basic(_) => {} //nothing to do
            BaseSystems::Celestial(celestial) => {
                celestial.update_result(); // system.run() will update the values, including epoch
            }
        }
    }

    pub fn update_sensors(&mut self) {
        for sensor in &mut self.sensors {
            sensor.update()
        }
    }

    pub fn validate(&self) -> Result<(), MultibodyErrors> {
        // check that the base has an outer joint
        let base_outer_joints = &self.base.borrow().outer_joints;
        if base_outer_joints.is_empty() {
            return Err(MultibodyErrors::BaseMissingOuterJoint);
        }

        // check that every body has an inner joint
        for body in &self.bodies {
            let body = body.borrow();
            if body.inner_joint.is_none() {
                return Err(MultibodyErrors::BodyMissingInnerJoint(body.name.clone()));
            }
        }

        // check that every joint has an inner and outer body connection
        for joint in &self.joints {
            let joint = joint.borrow();
            if joint.connections.inner_body.is_none() {
                return Err(MultibodyErrors::JointMissingInnerBody(joint.name.clone()));
            }
            if joint.connections.outer_body.is_none() {
                return Err(MultibodyErrors::JointMissingOuterBody(joint.name.clone()));
            }
        }
        Ok(())
    }
}
