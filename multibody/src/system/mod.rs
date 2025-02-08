use crate::{
    actuator::ActuatorSystem,
    base::{BaseRef, BaseSystems},
    body::BodyRef,
    joint::{InnerBody, JointRef},
    sensor::SensorSystem,
    software::SoftwareSystem,
    solver::{rk4::solve_fixed_rk4, SimStates},
};

use super::{algorithms::MultibodyAlgorithm, body::Body, MultibodyErrors};

use nadir_result::{NadirResult, ResultManager};
use ron::ser::{to_string_pretty, PrettyConfig};
use serde::{Deserialize, Serialize};
use spice::Spice;
use std::{
    cell::RefCell,
    collections::HashMap,
    fs::{create_dir_all, File},
    io::Write,
    path::{Path, PathBuf},
    rc::Rc,
    time::Instant,
};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MultibodySystem<A = (), F = (), S = ()>
where
    A: ActuatorSystem,
    F: SoftwareSystem,
    S: SensorSystem,
{
    pub actuators: A,
    pub algorithm: MultibodyAlgorithm,
    pub base: BaseRef,
    pub bodies: Vec<BodyRef>,
    pub joints: Vec<JointRef>,
    pub sensors: S,
    pub software: F,
    pub spice: Option<Spice>,
    pub sim_time_id: Option<u32>,
}

impl<A, F, S> MultibodySystem<A, F, S>
where
    A: ActuatorSystem,
    F: SoftwareSystem<Actuators = A, Sensors = S>,
    S: SensorSystem,
{
    pub fn new<const B: usize, const J: usize>(
        base: BaseRef,
        bodies: [BodyRef; B],
        joints: [JointRef; J],
        sensors: S,
        software: F,
        actuators: A,
    ) -> Self {
        Self {
            actuators,
            algorithm: MultibodyAlgorithm::ArticulatedBody, // for now, default to this
            base,
            bodies: bodies.into(),
            joints: joints.into(),
            sensors,
            software,
            spice: None,
            sim_time_id: None,
        }
    }

    pub fn with_actuators(mut self, actuators: A) -> Self {
        self.actuators = actuators;
        self
    }

    pub fn with_sensors(mut self, sensors: S) -> Self {
        self.sensors = sensors;
        self
    }

    pub fn with_software(mut self, software: F) -> Self {
        self.software = software;
        self
    }

    fn write_derivative(&self, dx: &mut SimStates) {
        for (i, joint) in self.joints.iter().enumerate() {
            joint.borrow().state_derivative(&mut dx.0[i]);
        }
        self.actuators.state_derivative(dx);
    }

    /// Reorders the bodies and joints appropraitely at the start of the multibody simulation
    fn permute(&mut self) {
        let mut joint_order = Vec::new();
        let mut body_order = Vec::new();

        // Helper function to recursively traverse joints and bodies
        fn traverse_body<A, F, S>(
            sys: &MultibodySystem<A, F, S>,
            body_rc: &Rc<RefCell<Body>>,
            joint_order: &mut Vec<usize>,
            body_order: &mut Vec<usize>,
        ) where
            A: ActuatorSystem,
            F: SoftwareSystem,
            S: SensorSystem,
        {
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

    pub fn run(&mut self, dx: &mut SimStates, x: &SimStates, t: f64) {
        self.set_state(x); // write the integrated states back in to the joints
        self.update_base(t); // update epoch based celestial states based on new time
        self.update_joints(); // update joint state based quantities like transforms
        self.update_body_states(); // need to update the body position for gravity calcs prior to update_forces
        self.actuators.update(); // run the actuators before updating forces on the body
        self.update_environments();
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

    pub fn run_software(&mut self) {
        self.software.run(&self.sensors, &mut self.actuators);
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

    fn set_state(&mut self, states: &SimStates) {
        for (i, joint) in self.joints.iter().enumerate() {
            joint.borrow_mut().model.state_vector_read(&states.0[i]);
        }
        self.actuators.state_vector_read(states);
    }

    pub fn simulate(&mut self, sim_name: &str, tstart: f64, tstop: f64, dt: f64) {
        let start_time = Instant::now();
        match self.validate() {
            Ok(_) => {}
            Err(e) => panic!("{e}"),
        };
        let (result_path, sim_name) = self.get_result_path_and_sim_name(sim_name);
        // initialize the result manager
        create_dir_all(&result_path).expect("result folder already exists");
        let mut results = self.initialize_writers(result_path);

        // sort the bodies and joints based on multibody tree philosophy
        self.permute();

        // initialize the components initial conditions and secondary states
        for jointref in &self.joints {
            let mut joint = jointref.borrow_mut();
            joint.update_transforms();
            joint.calculate_joint_inertia();
            joint.calculate_vj();
            joint.aba_first_pass(); //to calculate cache.v, which bodies use to update initial body velocity
        }

        self.update_body_states();
        self.update_sensors();

        // solve the multibody system
        match solve_fixed_rk4(self, tstart, tstop, dt, &mut results) {
            Ok(_) => {}
            Err(e) => eprintln!("{e}"),
        }

        // save the body meshes to the result
        let mut meshes = HashMap::new();
        for bodyref in &self.bodies {
            let body = bodyref.borrow();
            if let Some(mesh) = &body.mesh {
                meshes.insert(body.name.clone(), mesh.clone());
            }
        }

        let sim_duration = start_time.elapsed();
        let sim_duration_str = utilities::format_duration(sim_duration);
        println!("Simulation '{sim_name}' completed in {sim_duration_str}.");
        // collect meshes from the system into the result so we can use for animation
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

    fn update_base(&mut self, t: f64) {
        self.base.borrow_mut().update(t).unwrap();
    }

    fn update_environments(&mut self) {
        let mut base = self.base.borrow_mut();
        match &mut base.system {
            BaseSystems::Celestial(celestial) => {
                for bodyref in &mut self.bodies {
                    let mut body = bodyref.borrow_mut();
                    body.calculate_magnetic_field(celestial);
                }
            }
            _ => {} //continue
        }
    }

    fn update_forces(&mut self) {
        // forces wee already reset and updated from actuators in self.actuators.update()
        // TODO: i dont like this, too inconsistnet, do the updating here
        for bodyrc in self.bodies.iter_mut() {
            let mut body = bodyrc.borrow_mut();
            let inner_joint_weak = body.inner_joint.as_mut().unwrap();
            let inner_joint_rc = inner_joint_weak.upgrade().unwrap();
            let mut inner_joint = inner_joint_rc.borrow_mut();

            // get transforms
            let transforms = &inner_joint.cache.transforms;
            // calculate gravity for the outer body
            match &mut self.base.borrow_mut().system {
                BaseSystems::Basic(gravity) => {
                    if let Some(gravity) = gravity {
                        body.calculate_gravity(&transforms.ob_from_base, gravity);
                    }
                }
                BaseSystems::Celestial(celestial) => body.calculate_gravity_celestial(celestial),
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

    pub fn update_sensors(&mut self) {
        self.sensors.update();
    }

    fn get_result_path_and_sim_name(&self, sim_name: &str) -> (PathBuf, String) {
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
                    if file_name == sim_name {
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
        let sim_name = if sim_name.is_empty() {
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
            sim_name.to_string()
        };

        let results_path = PathBuf::new().join("results").join(&sim_name);
        (results_path, sim_name)
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

    fn initialize_writers(&mut self, results_path: PathBuf) -> ResultManager {
        let mut results = ResultManager::new(results_path.clone());
        let id = results.new_writer("sim_time", &results_path, &["sim_time(sec)"]);
        self.sim_time_id = Some(id);

        for body in &self.bodies {
            let mut body = body.borrow_mut();
            body.new_result(&mut results);
        }

        for joint in &self.joints {
            let mut joint = joint.borrow_mut();
            joint.new_result(&mut results);
        }

        self.sensors.initialize_results(&mut results);
        self.actuators.initialize_results(&mut results);
        self.software.initialize_results(&mut results);

        match &mut self.base.borrow_mut().system {
            BaseSystems::Basic(_) => {}
            BaseSystems::Celestial(celestial) => celestial.initialize_writers(&mut results),
        }
        results
    }

    pub fn write_result_files(&self, t: f64, results: &mut ResultManager) {
        if let Some(id) = self.sim_time_id {
            results.write_record(id, &[t.to_string()]);
        }

        for body in &self.bodies {
            let body = body.borrow();
            body.write_result(results);
        }

        for joint in &self.joints {
            let joint = joint.borrow();
            joint.write_result(results);
        }

        self.sensors.write_results(results);
        self.actuators.write_results(results);
        self.software.write_results(results);

        match &self.base.borrow().system {
            BaseSystems::Basic(_) => {}
            BaseSystems::Celestial(celestial) => celestial.write_results(results),
        }
    }
}
