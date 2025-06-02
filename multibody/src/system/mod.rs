use crate::{
    MultibodyErrors,
    actuator::{Actuator, ActuatorBuilder},
    algorithms::MultibodyAlgorithm,
    base::{Base, BaseBuilder, BaseRef, BaseSystems, BaseSystemsBuilder},
    body::{BodyBuilder, BodyConnection, BodyRef},
    joint::{JointBuilder, JointConnection, JointModel, JointModelBuilders, JointRef},
    sensor::{Sensor, SensorBuilder},
    software::{Software, SoftwareSim},
};

use core::fmt;
use gravity::{Gravity, constant::ConstantGravity, newtonian::NewtonianGravity};
use indicatif::MultiProgress;
use nadir_diffeq::{OdeModel, OdeProblem, state::state_vector::StateVector};
use nadir_result::{NadirResult, ResultManager};

use rand::{Rng, SeedableRng, rngs::SmallRng};
use rayon::prelude::*;
use ron::{
    from_str,
    ser::{PrettyConfig, to_string_pretty},
};
use serde::{Deserialize, Serialize};
use spatial_algebra::SpatialTransform;
use std::{
    cell::RefCell,
    collections::HashMap,
    error::Error,
    fmt::{Display, Formatter},
    fs::File,
    io::{Read, Write},
    path::{Path, PathBuf},
    rc::Rc,
    sync::Arc,
    time::Instant,
};
use thiserror::Error;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MultibodySystemBuilder {
    pub actuators: Vec<ActuatorBuilder>,
    pub algorithm: MultibodyAlgorithm,
    pub base: BaseBuilder,
    pub bodies: HashMap<Id, BodyBuilder>,
    pub identifier: Identifier,
    pub joints: HashMap<Id, JointBuilder>,
    seed: u64,
    pub sensors: Vec<SensorBuilder>,
    pub software: Vec<Software>,
}

impl MultibodySystemBuilder {
    pub fn add_actuator(&mut self, actuator: ActuatorBuilder) {
        self.actuators.push(actuator);
    }

    pub fn add_body(&mut self, body: BodyBuilder) {
        self.bodies.insert(body.id, body);
    }

    pub fn add_joint(&mut self, joint: JointBuilder) {
        self.joints.insert(joint.id, joint);
    }

    pub fn add_sensor(&mut self, sensor: SensorBuilder) {
        self.sensors.push(sensor);
    }

    pub fn add_software(&mut self, software: Software) {
        self.software.push(software);
    }

    // pub fn connect(
    //     &mut self,
    //     from_name: &str,
    //     to_name: &str,
    //     transform: Transform,
    // ) -> Result<(), MultibodyErrors> {
    //     let from = find_by_name(self, from_name)?;
    //     let to = find_by_name(self, to_name)?;

    //     match (from, to) {
    //         (Component::Actuator(from_id), Component::Body(to_id)) => {
    //             let actuator = self.actuators.get_mut(&from_id).unwrap();
    //             actuator.connect_body(to_id, transform);
    //         }
    //         (Component::Sensor(from_id), Component::Body(to_id)) => {
    //             let sensor = self.sensors.get_mut(&from_id).unwrap();
    //             sensor.connect_body(to_id, transform)?;
    //         }
    //         (Component::Joint(from_id), Component::Body(to_id)) => {
    //             let joint = self.joints.get_mut(&from_id).unwrap();
    //             let body = self.bodies.get_mut(&to_id).unwrap();
    //             joint.connect_inner_body(to_id, transform)?;
    //             body.connect_outer_joint(from_id)?;
    //         }
    //         (Component::Body(from_id), Component::Joint(to_id)) => {
    //             let body = self.bodies.get_mut(&from_id).unwrap();
    //             let joint = self.joints.get_mut(&to_id).unwrap();
    //             joint.connect_outer_body(from_id, transform)?;
    //             body.connect_inner_joint(to_id)?;
    //         }
    //         (Component::Joint(from_id), Component::Base(to_id)) => {
    //             let joint = self.joints.get_mut(&from_id).unwrap();
    //             let base = &mut self.base;
    //             joint.connect_inner_body(to_id, transform)?;
    //             base.connect_outer_joint(from_id)?;
    //         }
    //         _ => return Err(MultibodyErrors::InvalidConnection),
    //     }
    //     Ok(())
    // }

    pub fn load(path: &Path) -> Self {
        let path = path.join("system.ron");

        // Open the file
        let mut file = match File::open(&path) {
            Ok(file) => file,
            Err(e) => panic!("Failed to open file {path:?}: {e}"),
        };

        // Read the file into a string
        let mut contents = String::new();
        match file.read_to_string(&mut contents) {
            Ok(_) => {}
            Err(e) => panic!("Failed to read file {path:?}: {e}"),
        };

        // Deserialize from RON
        match from_str(&contents) {
            Ok(system) => system,
            Err(e) => panic!("Failed to parse RON file {path:?}: {e}"),
        }
    }

    pub fn new() -> Self {
        let mut thread_rng = rand::rng(); // Use a fast non-deterministic RNG
        let seed = thread_rng.random::<u64>(); // Generate a random seed
        let mut id = Identifier::new();
        Self {
            actuators: Vec::new(),
            algorithm: MultibodyAlgorithm::ArticulatedBody, // for now, default to this
            base: BaseBuilder::new(id.next()),
            bodies: HashMap::new(),
            identifier: id,
            joints: HashMap::new(),
            seed,
            sensors: Vec::new(),
            software: Vec::new(),
        }
    }

    pub fn new_body(&mut self, name: &str) -> Result<BodyBuilder, MultibodyErrors> {
        let id = self.identifier.next();
        Ok(BodyBuilder::new(name, id)?)
    }

    pub fn new_joint(
        &mut self,
        name: &str,
        model: JointModelBuilders,
    ) -> Result<JointBuilder, MultibodyErrors> {
        let id = self.identifier.next();
        Ok(JointBuilder::new(id, name, model)?)
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

    pub fn set_gravity_constant(
        &mut self,
        gx: f64,
        gy: f64,
        gz: f64,
    ) -> Result<(), MultibodyErrors> {
        self.base.system =
            BaseSystemsBuilder::Basic(Some(Gravity::Constant(ConstantGravity::new(gx, gy, gz))));
        Ok(())
    }

    pub fn set_gravity_newtonian(&mut self, mu: f64) -> Result<(), MultibodyErrors> {
        self.base.system =
            BaseSystemsBuilder::Basic(Some(Gravity::Newtonian(NewtonianGravity::new(mu))));
        Ok(())
    }

    // pub fn simulate(
    //     &mut self,
    //     sim_name: &str,
    //     tstart: f64,
    //     tstop: f64,
    //     dt: f64,
    //     nruns: Option<usize>,
    // ) -> Result<(), MultibodyErrors> {
    //     let (result_path, sim_name) = self.get_result_path_meta(sim_name);

    //     let options = SimOptions {
    //         tstart,
    //         tstop,
    //         dt,
    //         sim_name,
    //         nruns,
    //     };

    //     let mut rng = SmallRng::seed_from_u64(self.seed);

    //     // run the monte carlo if present
    //     if let Some(nruns) = nruns {
    //         // create progress bars
    //         let progress_bars = Arc::new(MultiProgress::new());
    //         // run the nominal
    //         let nominal_path = result_path.join("nominal");
    //         let mut sys = self.sample(None, true, &mut rng)?;
    //         sys.simulate(&options, &nominal_path, None)?;
    //         // parallel Monte Carlo runs
    //         // create a vec of seeds for the local thread rngs
    //         let seeds: Vec<u64> = (0..nruns).map(|_| rng.random::<u64>()).collect();

    //         (1..=nruns)
    //             .into_par_iter()
    //             .try_for_each(|run| -> Result<(), MultibodyErrors> {
    //                 let progress_bars = Arc::clone(&progress_bars);
    //                 let mut local_rng = SmallRng::seed_from_u64(seeds[run - 1]);
    //                 let mut sys = self.sample(Some(run), false, &mut local_rng)?;
    //                 let run_path = result_path.join(format!("run{run}"));
    //                 sys.simulate(&options, &run_path, Some(progress_bars))?;
    //                 Ok(())
    //             })?;
    //     } else {
    //         // just run the nominal
    //         // this is separated logic so we can send the nominal folder as a result  path for monte carlo
    //         // where as this is just stored at the sim level
    //         let mut sys = self.sample(None, true, &mut rng)?;
    //         sys.simulate(&options, &result_path, None)?;
    //     }
    //     Ok(())
    // }

    fn get_result_path_meta(&self, sim_name: &str) -> (PathBuf, String) {
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

    fn sample(
        &self,
        run_id: Option<usize>,
        nominal: bool,
        rng: &mut SmallRng,
    ) -> Result<MultibodySystem, MultibodyErrors> {
        // ensure builder can produce a valid MultibodySystem
        self.validate()?;

        // order matters for bodies and joints
        let mut bodies = Vec::new();
        let mut joints = Vec::new();
        let mut actuators = Vec::new();
        let mut sensors = Vec::new();
        let mut software = Vec::new();

        let seed = rng.random::<u64>();
        let mut sys_rng = SmallRng::seed_from_u64(seed);

        let baseref = Rc::new(RefCell::new(Base::from(&self.base)));

        // Start the recursion from the base's outer joints
        for i in 0..self.base.outer_joints.len() {
            let outer_joint_id = self.base.outer_joints[i];
            let outer_joint_builder = self
                .joints
                .get(&outer_joint_id)
                .expect("validation should catch this");

            // Create the outer joints inner connections
            // Outer connections are created during their turn in the recursion
            // TODO: is there a way to set the outer connections directly without using temporary Options?
            let inner_body = baseref.clone();
            let inner_body_transform = outer_joint_builder
                .connections
                .inner_body
                .as_ref()
                .expect("validation should catch this")
                .transform;
            let inner_body_connection = BodyConnection {
                body: inner_body.into(),
                transform: inner_body_transform,
            };
            let connections = JointConnection {
                inner_body: inner_body_connection,
                outer_body: None,
            };

            // Create the outer joint from the joint builder, sampling for monte carlo if applicable
            let outer_joint = outer_joint_builder.sample(connections, nominal, &mut sys_rng)?;
            joints.push(Rc::new(RefCell::new(outer_joint)));

            // Recurse into connected outer bodies
            let connection = outer_joint_builder
                .connections
                .outer_body
                .as_ref()
                .expect("validation should catch this");
            let next_body_id = connection.body_id;
            traverse_body(
                self,
                next_body_id,
                &mut joints,
                &mut bodies,
                &mut actuators,
                &mut sensors,
                nominal,
                &mut sys_rng,
            )?;
        }

        // Set the joint transforms for joint, these are fixed for the duration of the sim
        // Set joint's mass properties from the outer body and the transform
        // This is done at the end once we've set jof_from_ob in the joint transforms
        //TODO: This could (should?) tehcnically move into traverse_body,
        for jointref in &joints {
            // borrow immutably first to get connection info
            let (outer_transform, outer_body_mass_properties) = {
                let joint = jointref.borrow();
                let outer_connection = joint
                    .connections
                    .outer_body
                    .as_ref()
                    .expect("should be an outer body by now for all joints");

                let outer_transform = outer_connection.transform;
                let outer_body = outer_connection.body.borrow();
                (outer_transform, outer_body.mass_properties)
            };

            //now borrow mutably
            let mut joint = jointref.borrow_mut();
            let inner_transform = joint.connections.inner_body.transform;
            joint.cache.transforms.jif_from_ib = SpatialTransform(inner_transform);
            joint.cache.transforms.ib_from_jif = SpatialTransform(inner_transform.inv());

            joint.cache.transforms.jof_from_ob = SpatialTransform(outer_transform);
            joint.cache.transforms.ob_from_jof = SpatialTransform(outer_transform.inv());
            joint.set_inertia(&outer_body_mass_properties);

            // also set the inner joint
            let inner_body = &joint.connections.inner_body.body;
            let inner_joint = if inner_body.is_body() {
                Some(
                    inner_body
                        .borrow()
                        .inner_joint
                        .upgrade()
                        .expect("all bodies should have inner joint")
                        .clone(),
                )
            } else {
                None
            };
            joint.inner_joint = inner_joint;
        }

        // create software
        for sw in &self.software {
            software.push(SoftwareSim::try_from(sw)?);
        }

        let sys = MultibodySystem {
            actuators,
            algorithm: self.algorithm,
            base: Rc::new(RefCell::new(Base::from(&self.base))),
            bodies,
            joints,
            run_id,
            sensors,
            software,
            sim_time_id: None,
        };

        Ok(sys)
    }

    pub fn validate(&self) -> Result<(), MultibodyErrors> {
        // check that the base has an outer joint
        let base_outer_joints = &self.base.outer_joints;
        if base_outer_joints.is_empty() {
            return Err(MultibodyErrors::BaseMissingOuterJoint);
        }

        // check that all outer joints exist in the hashmap
        for outer_joint_id in base_outer_joints {
            if !self.joints.contains_key(outer_joint_id) {
                return Err(MultibodyErrors::JointNotFound(outer_joint_id.to_string()));
            }
        }

        // check that every body has an inner joint
        for (_, body) in &self.bodies {
            if let Some(inner_joint_id) = &body.inner_joint {
                // check that the inner joint exists in the hashmap
                if !self.joints.contains_key(inner_joint_id) {
                    return Err(MultibodyErrors::JointNotFound(inner_joint_id.to_string()));
                }
            } else {
                return Err(MultibodyErrors::BodyMissingInnerJoint(body.name.clone()));
            }

            // check that if the body has outer joints, that they all exist in the hashmap
            for outer_joint_id in &body.outer_joints {
                if !self.joints.contains_key(outer_joint_id) {
                    return Err(MultibodyErrors::JointNotFound(outer_joint_id.to_string()));
                }
            }
        }

        // check that every joint has an inner and outer body connection
        for (_, joint) in &self.joints {
            if let Some(inner_body_connection) = &joint.connections.inner_body {
                // check that the inner body exists in the hashmap
                if !self.bodies.contains_key(&inner_body_connection.body_id)
                    && &inner_body_connection.body_id != &self.base.id
                {
                    return Err(MultibodyErrors::BodyNotFound(
                        inner_body_connection.body_id.to_string(),
                    ));
                }
            } else {
                return Err(MultibodyErrors::JointMissingInnerBody(joint.name.clone()));
            };
            if let Some(outer_body_connection) = &joint.connections.outer_body {
                // check that the outer body exists in the hashmap
                if !self.bodies.contains_key(&outer_body_connection.body_id) {
                    return Err(MultibodyErrors::BodyNotFound(
                        outer_body_connection.body_id.to_string(),
                    ));
                }
            } else {
                return Err(MultibodyErrors::JointMissingOuterBody(joint.name.clone()));
            }
        }

        // check that every actuator has a body connection
        for actuator in &self.actuators {
            if let Some(connection) = &actuator.connection {
                //check that the body exists in the hashmap
                if !self.bodies.contains_key(&connection.body_id) {
                    return Err(MultibodyErrors::BodyNotFound(
                        connection.body_id.to_string(),
                    ));
                }
            } else {
                return Err(MultibodyErrors::ActuatorMissingBody(actuator.name.clone()));
            }
        }

        // check that every actuator has a body connection
        for sensor in &self.sensors {
            if let Some(connection) = &sensor.connection {
                //check that the body exists in the hashmap
                if !self.bodies.contains_key(&connection.body_id) {
                    return Err(MultibodyErrors::BodyNotFound(
                        connection.body_id.to_string(),
                    ));
                }
            } else {
                return Err(MultibodyErrors::SensorMissingBody(sensor.name.clone()));
            }
        }
        Ok(())
    }
}

#[derive(Debug)]
pub struct MultibodySystem {
    pub actuators: Vec<Actuator>,
    pub algorithm: MultibodyAlgorithm,
    pub base: BaseRef,
    pub bodies: Vec<BodyRef>,
    pub joints: Vec<JointRef>,
    pub run_id: Option<usize>, // Monte Carlo run number for progress bar
    pub sensors: Vec<Sensor>,
    pub software: Vec<SoftwareSim>,
    pub sim_time_id: Option<u32>,
}

impl MultibodySystem {
    fn initialize_writers(&mut self, results_path: &PathBuf) -> ResultManager {
        let mut results = ResultManager::new(results_path.clone());
        let id = results.new_writer("sim_time", &results_path, &["sim_time(sec)"]);
        self.sim_time_id = Some(id);

        for body in &self.bodies {
            body.borrow_mut().new_result(&mut results);
        }

        for joint in &self.joints {
            joint.borrow_mut().new_result(&mut results);
        }

        for sensor in &mut self.sensors {
            sensor.new_result(&mut results);
        }

        for actuator in &mut self.actuators {
            actuator.new_result(&mut results);
        }

        for software in &mut self.software {
            if let Err(e) = software.initialize_results(&mut results) {
                eprintln!(
                    "Failed to initialize results for software component: {:?}",
                    e
                );
            }
        }

        match &mut self.base.borrow_mut().system {
            BaseSystems::Basic(_) => {}
            BaseSystems::Celestial(celestial) => celestial.initialize_writers(&mut results),
        }
        results
    }

    pub fn update_software(&mut self) -> Result<(), MultibodyErrors> {
        for software in &mut self.software {
            software.step(&self.sensors, &mut self.actuators)?;
        }
        Ok(())
    }

    fn update_state(&mut self, states: &StateVector) {
        for joint in &self.joints {
            joint.borrow_mut().state_vector_read(states);
        }
        for actuator in &mut self.actuators {
            actuator.state_vector_read(states);
        }
    }

    // pub fn simulate(
    //     &mut self,
    //     options: &SimOptions,
    //     result_path: &PathBuf,
    //     progress_bars: Option<Arc<MultiProgress>>,
    // ) -> Result<(), MultibodyErrors> {
    //     let start_time = Instant::now();
    //     let mut results = self.initialize_writers(result_path);

    //     // initialize the components initial conditions and secondary states
    //     for jointref in &self.joints {
    //         let mut joint = jointref.borrow_mut();
    //         joint.update_transforms();
    //         joint.calculate_joint_inertia();
    //         joint.calculate_vj();
    //         joint.aba_first_pass(); //to calculate cache.v, which bodies use to update initial body velocity
    //     }

    //     self.update_body_states();
    //     self.update_sensors();

    //     let prob = OdeProblem::new(

    //     );
    //     // solve the multibody system
    //     solve_fixed_rk4(
    //         self,
    //         options.tstart,
    //         options.tstop,
    //         options.dt,
    //         &mut results,
    //         progress_bars,
    //     )?;

    //     // save the body meshes to the result
    //     let mut meshes = HashMap::new();
    //     for body in &self.bodies {
    //         let body = body.borrow();
    //         if let Some(mesh) = &body.mesh {
    //             meshes.insert(body.name.clone(), mesh.clone());
    //         }
    //     }

    //     let sim_duration = start_time.elapsed();
    //     let sim_duration_str = utilities::format_duration(sim_duration);

    //     // only print sim  time info if not a monte carlo, otherwise it gets ugly
    //     if self.run_id.is_none() {
    //         println!(
    //             "Simulation '{}' completed in {sim_duration_str}.",
    //             options.sim_name
    //         );
    //     }

    //     Ok(())
    // }

    fn update_actuators(&mut self) -> Result<(), MultibodyErrors> {
        self.actuators
            .iter_mut()
            .try_for_each(|actuator| actuator.update())?;
        Ok(())
    }

    fn update_body_acceleration(&mut self) {
        for body in &self.bodies {
            body.borrow_mut().update_acceleration();
        }
    }

    fn update_body_states(&mut self) {
        for body in &self.bodies {
            body.borrow_mut().update_state();
        }
    }

    fn update_base(&mut self, t: f64) {
        self.base.borrow_mut().update(t).unwrap();
    }

    fn update_environments(&mut self) {
        match &mut self.base.borrow_mut().system {
            BaseSystems::Celestial(celestial) => {
                for body in &self.bodies {
                    body.borrow_mut().calculate_magnetic_field(celestial);
                }
            }
            _ => {} //continue
        }
    }

    fn update_forces(&mut self) {
        // forces were already reset and updated from actuators in self.actuators.update()
        // TODO: i dont like this, too inconsistent, do the updating here
        for body in &self.bodies {
            let mut body = body.borrow_mut();
            match &mut self.base.borrow_mut().system {
                BaseSystems::Basic(gravity) => {
                    if let Some(gravity) = gravity {
                        body.calculate_gravity(gravity);
                    }
                }
                BaseSystems::Celestial(celestial) => body.calculate_gravity_celestial(celestial),
            }

            // calculate total external forces for the outer body
            body.calculate_external_force();

            // transform force to joint
            // cross product terms in spatial calculation will convert force at body cg to torque about joint
            let inner_joint = body
                .inner_joint
                .upgrade()
                .expect("validation should catch this");
            let mut inner_joint = inner_joint.borrow_mut();
            inner_joint.cache.f =
                inner_joint.cache.transforms.jof_from_ob * body.state.external_spatial_force_body;
        }
    }

    fn update_joints(&mut self) {
        for jointref in &self.joints {
            let mut joint = jointref.borrow_mut();
            joint.update_transforms();
            joint.calculate_vj();
            joint.model.calculate_tau();
        }
    }

    pub fn update_sensors(&mut self) -> Result<(), MultibodyErrors> {
        for sensor in &mut self.sensors {
            sensor.update()?;
        }
        Ok(())
    }

    pub fn write_result_files(&self, t: f64, results: &mut ResultManager) {
        if let Some(id) = self.sim_time_id {
            results.write_record(id, &[t.to_string()]);
        }

        for body in &self.bodies {
            body.borrow().write_result(results);
        }

        for joint in &self.joints {
            joint.borrow().write_result(results);
        }

        for actuator in &self.actuators {
            actuator.write_result(results);
        }

        for sensor in &self.sensors {
            sensor.write_result(results);
        }

        for software in &self.software {
            if let Err(e) = software.write_results(results) {
                eprintln!("Failed to write results for software component: {:?}", e);
            }
        }

        match &self.base.borrow().system {
            BaseSystems::Basic(_) => {}
            BaseSystems::Celestial(celestial) => celestial.write_results(results),
        }
    }

    fn write_derivative(&self, dx: &mut StateVector) {
        for joint in &self.joints {
            joint.borrow().state_derivative(dx);
        }

        for actuator in &self.actuators {
            actuator.state_derivative(dx);
        }
    }
}

impl OdeModel for MultibodySystem {
    type State = StateVector;
    fn f(&mut self, t: f64, x: &StateVector, dx: &mut StateVector) -> Result<(), Box<dyn Error>> {
        self.update_state(x); // write the integrated states back in to the joints
        self.update_base(t); // update epoch based celestial states based on new time
        self.update_joints(); // update joint state based quantities like transforms
        self.update_body_states(); // need to update the body position for gravity calcs prior to update_forces
        self.update_actuators()?; // update the actuators before updating forces on the bodies
        self.update_environments(); //update the environmental forces before updating forcces on the bodies
        self.update_forces(); // update body forces

        match self.algorithm {
            MultibodyAlgorithm::ArticulatedBody => {
                // First Pass
                for jointref in &mut self.joints {
                    let mut joint = jointref.borrow_mut();
                    joint.aba_first_pass();
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

        Ok(())
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Identifier {
    current_id: usize,
}

impl Identifier {
    fn new() -> Self {
        Self { current_id: 0 }
    }

    fn next(&mut self) -> Id {
        let id = Id(self.current_id);
        self.current_id += 1;
        id
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Id(usize);

impl Display for Id {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum Component {
    Actuator(Id),
    Base(Id),
    Body(Id),
    Joint(Id),
    Sensor(Id),
}

#[derive(Debug, Error)]
pub enum SimOptionsErrors {
    #[error("dt cannot be less than or equal to 0")]
    DtCantBeZero,
    #[error("tstart cannot be greater than tstop")]
    StartGreaterThanStop,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimOptions {
    pub sim_name: String,
    pub tstart: f64,
    pub tstop: f64,
    pub dt: f64,
    pub nruns: Option<usize>,
}

impl SimOptions {
    pub fn new(sim_name: &str, tstart: f64, tstop: f64, dt: f64) -> Result<Self, SimOptionsErrors> {
        if tstart > tstop {
            return Err(SimOptionsErrors::StartGreaterThanStop);
        }
        if dt < std::f64::EPSILON {
            return Err(SimOptionsErrors::DtCantBeZero);
        }

        Ok(Self {
            sim_name: sim_name.to_string(),
            tstart,
            tstop,
            dt,
            nruns: None,
        })
    }

    pub fn with_nruns(mut self, nruns: usize) -> Self {
        self.nruns = Some(nruns);
        self
    }
}

// Helper function to recursively traverse joints and bodies
fn traverse_body(
    builder: &MultibodySystemBuilder,
    body_id: Id,
    joints: &mut Vec<JointRef>,
    bodies: &mut Vec<BodyRef>,
    actuators: &mut Vec<Actuator>,
    sensors: &mut Vec<Sensor>,
    nominal: bool,
    rng: &mut SmallRng,
) -> Result<(), MultibodyErrors> {
    let next_outer_joints = {
        let body_builder = builder
            .bodies
            .get(&body_id)
            .expect("validation should catch this");
        // Create the body from the body builder, sampling for monte carlo if applicable
        let inner_joint = joints.last().expect("should be at least 1 element");
        let body = body_builder.sample(inner_joint.clone(), nominal, rng)?;
        bodies.push(body.into());

        // add the bodyref to any connected actuators or sensors
        for actuator_builder in &builder.actuators {
            if actuator_builder
                .connection
                .as_ref()
                .expect("validation should catch this")
                .body_id
                == body_builder.id
            {
                let connection = BodyConnection {
                    body: bodies.last().expect("should be at least 1 element").clone(),
                    transform: actuator_builder
                        .connection
                        .as_ref()
                        .expect("validation should catch this")
                        .transform,
                };
                let actuator = actuator_builder.sample(nominal, rng, connection)?;
                actuators.push(actuator);
            }
        }
        for sensor_builder in &builder.sensors {
            if sensor_builder
                .connection
                .as_ref()
                .expect("validation should catch this")
                .body_id
                == body_builder.id
            {
                let connection = BodyConnection {
                    body: bodies.last().expect("should be at least 1 element").clone(),
                    transform: sensor_builder
                        .connection
                        .as_ref()
                        .expect("validation should catch this")
                        .transform,
                };
                let sensor = sensor_builder.sample(nominal, rng, connection)?;
                sensors.push(sensor);
            }
        }

        // Add this bodyref as the outer body of the inner joint
        // Need to annoyingly get the transform
        let inner_joint_id = body_builder
            .inner_joint
            .expect("validation should catch this");
        let inner_joint_outer_body_transform = builder
            .joints
            .get(&inner_joint_id)
            .expect("validation should catch this")
            .connections
            .outer_body
            .as_ref()
            .expect("validation should catch this")
            .transform;
        inner_joint.borrow_mut().connections.outer_body = Some(BodyConnection {
            body: bodies.last().expect("should be at least 1 element").clone(),
            transform: inner_joint_outer_body_transform,
        });
        body_builder.outer_joints.clone()
    };

    // Start the recursion from the base's outer joints
    for outer_joint_id in next_outer_joints {
        let next_body_id = {
            let outer_joint_builder = builder
                .joints
                .get(&outer_joint_id)
                .expect("validation should catch this");

            // Create the outer joints inner connections
            // Outer connections are created during their turn in the recursion
            // TODO: is there a way to set the outer connections directly without using temporary Options?
            let inner_bodyref = bodies.last().expect("should be at least 1 element").clone();
            let inner_body_transform = outer_joint_builder
                .connections
                .inner_body
                .as_ref()
                .expect("validation should catch this")
                .transform;
            let inner_body_connection = BodyConnection {
                body: inner_bodyref,
                transform: inner_body_transform,
            };
            let connections = JointConnection {
                inner_body: inner_body_connection,
                outer_body: None,
            };

            let outer_joint = outer_joint_builder.sample(connections, nominal, rng)?;
            joints.push(Rc::new(RefCell::new(outer_joint)));

            // Add the outer joint to the inner body
            // TODO: Does the inner body even need the outer joint?
            // let mut inner_body = bodies.last().expect("should be at least 1 element").borrow_mut();
            // inner_body.outer_joints.push(joints.last().expect("should be at least 1 element").clone());

            // Recurse into connected outer bodies
            let connection = outer_joint_builder
                .connections
                .outer_body
                .as_ref()
                .expect("validation should catch this");
            connection.body_id
        };
        traverse_body(
            builder,
            next_body_id,
            joints,
            bodies,
            actuators,
            sensors,
            nominal,
            rng,
        )?;
    }
    Ok(())
}
