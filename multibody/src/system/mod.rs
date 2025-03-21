use crate::{
    actuator::Actuator,
    algorithms::MultibodyAlgorithm,
    base::{Base, BaseSystems},
    body::{Body, BodyBuilder},
    joint::{Joint, JointBuilder},
    sensor::Sensor,
    //software::SoftwareSystem,
    solver::{rk4::solve_fixed_rk4, SimStates},
    MultibodyErrors,
};

use core::fmt;
use indexmap::IndexMap;
use nadir_result::{NadirResult, ResultManager};
use rand::{rngs::StdRng, Rng, SeedableRng};
use ron::ser::{to_string_pretty, PrettyConfig};
use serde::{Deserialize, Serialize};
use spatial_algebra::SpatialTransform;
use std::{
    collections::HashMap,
    fmt::{Display, Formatter},
    fs::{create_dir_all, File},
    io::Write,
    path::{Path, PathBuf},
    time::Instant,
};
use transforms::Transform;
use uncertainty::Uncertainty;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MultibodySystemBuilder {
    pub actuators: HashMap<Id, Actuator>,
    pub algorithm: MultibodyAlgorithm,
    pub base: Base,
    pub bodies: HashMap<Id, BodyBuilder>,
    pub identifier: Identifier,
    pub joints: HashMap<Id, JointBuilder>,
    #[serde(skip)]
    rng: StdRng,
    seed: u64,
    pub sensors: HashMap<Id, Sensor>,
    //pub software: Option<Box<dyn SoftwareSystem>>,
}

impl MultibodySystemBuilder {
    pub fn connect(
        &mut self,
        from_name: &str,
        to_name: &str,
        transform: Transform,
    ) -> Result<(), MultibodyErrors> {
        let from = find_by_name(self, from_name)?;
        let to = find_by_name(self, to_name)?;

        match (from, to) {
            (Component::Actuator(from_id), Component::Body(to_id)) => {
                let actuator = self.actuators.get_mut(&from_id).unwrap();
                actuator.connect_to_body(to_id, transform)?;
            }
            (Component::Sensor(from_id), Component::Body(to_id)) => {
                let sensor = self.sensors.get_mut(&from_id).unwrap();
                sensor.connect_to_body(to_id, transform)?;
            }
            (Component::Joint(from_id), Component::Body(to_id)) => {
                let joint = self.joints.get_mut(&from_id).unwrap();
                let body = self.bodies.get_mut(&to_id).unwrap();
                joint.connect_inner_body(to_id, transform)?;
                body.connect_outer_joint(from_id)?;
            }
            (Component::Body(from_id), Component::Joint(to_id)) => {
                let body = self.bodies.get_mut(&from_id).unwrap();
                let joint = self.joints.get_mut(&to_id).unwrap();
                joint.connect_outer_body(from_id, transform)?;
                body.connect_inner_joint(to_id)?;
            }
            (Component::Joint(from_id), Component::Base(to_id)) => {
                let joint = self.joints.get_mut(&from_id).unwrap();
                let base = &mut self.base;
                joint.connect_inner_body(to_id, transform)?;
                base.connect_outer_joint(from_id)?;
            }
            _ => return Err(MultibodyErrors::InvalidConnection),
        }
        Ok(())
    }

    pub fn new() -> Self {
        let mut thread_rng = rand::thread_rng(); // Use a fast non-deterministic RNG
        let seed = thread_rng.gen::<u64>(); // Generate a random seed
        let rng = StdRng::seed_from_u64(seed);
        Self {
            actuators: HashMap::new(),
            algorithm: MultibodyAlgorithm::ArticulatedBody, // for now, default to this
            base: Base::new(),
            bodies: HashMap::new(),
            identifier: Identifier::new(),
            joints: HashMap::new(),
            rng,
            seed,
            sensors: HashMap::new(),
            //software: None,
        }
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

    pub fn simulate(
        &mut self,
        sim_name: &str,
        tstart: f64,
        tstop: f64,
        dt: f64,
    ) -> Result<(), MultibodyErrors> {
        let (result_path, sim_name) = self.get_result_path_and_sim_name(sim_name);

        let sys = MultibodySystem::try_from(self)?;

        let options = SimOptions {
            tstart,
            tstop,
            dt,
            sim_name,
        };

        sys.simulate(&options, result_path);

        Ok(())
    }

    fn get_result_path_sim_name_run_number(&self, sim_name: &str) -> (PathBuf, String) {
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
}

impl TryFrom<&mut MultibodySystemBuilder> for MultibodySystem {
    type Error = MultibodyErrors;
    fn try_from(builder: &mut MultibodySystemBuilder) -> Result<Self, MultibodyErrors> {
        // ensure builder can produce a valid MultibodySystem
        builder.validate()?;

        // order matters for bodies and joints
        let mut bodies = IndexMap::new();
        let mut joints = IndexMap::new();

        // Order does not matter for actuators and sensors
        let mut actuators = HashMap::new();
        let mut sensors = HashMap::new();

        // Helper function to recursively traverse joints and bodies
        fn traverse_body(
            builder: &mut MultibodySystemBuilder,
            body_id: Id,
            joints: &mut Vec<Id>,
            bodies: &mut Vec<Id>,
        ) -> Result<(), MultibodyErrors> {
            if let Some(body_builder) = builder.bodies.get(&body_id) {
                // Create the body from the body builder, sampling for monte carlo if applicable
                let body = body_builder.sample(&mut builder.rng)?;
                let inner_joint_id = body.inner_joint;
                bodies.insert(body_builder.id, body);

                // Process outer joints of this body
                for outer_joint_id in &body_builder.outer_joints {
                    if let Some(outer_joint_builder) = builder.joints.get(outer_joint_id) {
                        // Create the outer joint from the joint builder, sampling for monte carlo if applicable
                        let mut outer_joint = outer_joint_builder.sample(&mut builder.rng)?;

                        // Set the joints inner_joint to support multibody algorithms
                        outer_joint.inner_joint = Some(inner_joint_id);

                        // Set the joint transforms for this joint
                        if let Some(inner_body) = &outer_joint_builder.connections.inner_body {
                            let transform = inner_body.transform;
                            outer_joint.cache.transforms.jif_from_ib = SpatialTransform(transform);
                            outer_joint.cache.transforms.ib_from_jif =
                                SpatialTransform(transform.inv());
                        } else {
                            return Err(MultibodyErrors::JointMissingInnerBody(
                                outer_joint_builder.name.to_string(),
                            ));
                        }

                        // Set the joint mass properties for this joint from the mass properties of the outer body
                        if let Some(outer_body) = &outer_joint_builder.connections.outer_body {
                            let transform = outer_body.transform;
                            outer_joint.cache.transforms.jof_from_ob = SpatialTransform(transform);
                            outer_joint.cache.transforms.ob_from_jof =
                                SpatialTransform(transform.inv());

                            if let Some(outer_body) = builder.bodies.get(&outer_body.body) {
                                // calculate inertia about the joint
                                // some joints make model specific assumptions about this
                                outer_joint.cache.inertia =
                                    outer_joint.model.calculate_joint_inertia(
                                        &outer_body.mass_properties,
                                        &outer_joint.cache.transforms,
                                    );
                            } else {
                                return Err(MultibodyErrors::BodyNotFound(
                                    outer_body.body.to_string(),
                                ));
                            }
                        } else {
                            return Err(MultibodyErrors::JointMissingOuterBody(
                                outer_joint_builder.name.to_string(),
                            ));
                        }

                        joints.insert(outer_joint_builder.id, outer_joint);

                        // Recurse into connected outer bodies
                        if let Some(connection) = &outer_joint_builder.connections.outer_body {
                            traverse_body(builder, connection.body, joints, bodies)?;
                        } else {
                            return Err(MultibodyErrors::JointMissingOuterBody(
                                outer_joint_builder.name.to_string(),
                            ));
                        }
                    }
                    return Err(MultibodyErrors::JointNotFound(outer_joint_id));
                }
            } else {
                return Err(MultibodyErrors::BodyNotFound(body_id));
            }
            Ok(())
        }

        // Start the recursion from the base's outer joints
        for outer_joint_id in &builder.base.outer_joints {
            if let Some(outer_joint_builder) = builder.joints.get(outer_joint_id) {
                // Create the outer joint from the joint builder, sampling for monte carlo if applicable
                let mut outer_joint = outer_joint_builder.sample(&mut builder.rng)?;
                // Ensure base outer joints have no inner joints
                outer_joint.inner_joint = None;
                joints.insert(outer_joint_builder.id, outer_joint);

                // Recurse into connected outer bodies
                if let Some(connection) = &outer_joint_builder.connections.outer_body {
                    traverse_body(builder, connection.body, joints, bodies)?;
                }
            } else {
                return Err(MultibodyErrors::JointNotFound(outer_joint_id));
            }
        }

        let sys = MultibodySystem {
            actuators,
            algorithm: builder.algorithm,
            base: builder.base.clone(),
            bodies,
            joints,
            sensors,
            sim_time_id: None,
        };

        Ok(sys)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MultibodySystem {
    pub actuators: HashMap<Id, Actuator>,
    pub algorithm: MultibodyAlgorithm,
    pub base: Base,
    pub bodies: IndexMap<Id, Body>,
    pub joints: IndexMap<Id, Joint>,
    pub sensors: HashMap<Id, Sensor>,
    //pub software: Option<Box<dyn SoftwareSystem>>,
    pub sim_time_id: Option<u32>,
}

impl MultibodySystem {
    fn initialize_writers(&mut self, results_path: PathBuf) -> ResultManager {
        let mut results = ResultManager::new(results_path.clone());
        let id = results.new_writer("sim_time", &results_path, &["sim_time(sec)"]);
        self.sim_time_id = Some(id);

        for (_, body) in &mut self.bodies {
            body.new_result(&mut results);
        }

        for (_, joint) in &mut self.joints {
            joint.new_result(&mut results);
        }

        self.sensors.initialize_results(&mut results);
        self.actuators.initialize_results(&mut results);
        //self.software.initialize_results(&mut results);

        match &mut self.base.borrow_mut().system {
            BaseSystems::Basic(_) => {}
            BaseSystems::Celestial(celestial) => celestial.initialize_writers(&mut results),
        }
        results
    }

    pub fn run(&mut self, dx: &mut SimStates, x: &SimStates, t: f64) {
        self.set_state(x); // write the integrated states back in to the joints
        self.update_base(t); // update epoch based celestial states based on new time
        self.update_joints(); // update joint state based quantities like transforms
        self.update_body_states(); // need to update the body position for gravity calcs prior to update_forces
        self.actuators.update(); // update the actuators before updating forces on the bodies
        self.update_environments(); //update the environmental forces before updating forcces on the bodies
        self.update_forces(); // update body forces

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

    fn set_state(&mut self, states: &SimStates) {
        for (i, joint) in self.joints.iter().enumerate() {
            joint.borrow_mut().model.state_vector_read(&states.0[i]);
        }
        self.actuators.state_vector_read(states);
    }

    pub fn simulate(
        &mut self,
        options: &SimOptions,
        result_path: &PathBuf,
    ) -> Result<(), MultibodyErrors> {
        let start_time = Instant::now();
        let mut results = self.initialize_writers(result_path);

        // initialize the components initial conditions and secondary states
        for (_, joint) in &mut self.joints {
            joint.update_transforms();
            joint.calculate_joint_inertia();
            joint.calculate_vj();
            joint.aba_first_pass(); //to calculate cache.v, which bodies use to update initial body velocity
        }

        self.update_body_states();
        self.update_sensors();

        // solve the multibody system
        solve_fixed_rk4(
            self,
            options.tstart,
            options.tstop,
            options.dt,
            &mut results,
        )?;

        // save the body meshes to the result
        let mut meshes = HashMap::new();
        for (_, body) in &self.bodies {
            if let Some(mesh) = &body.mesh {
                meshes.insert(body.name.clone(), mesh.clone());
            }
        }

        let sim_duration = start_time.elapsed();
        let sim_duration_str = utilities::format_duration(sim_duration);
        println!(
            "Simulation '{}' completed in {sim_duration_str}.",
            options.sim_name
        );
        // collect meshes from the system into the result so we can use for animation

        Ok(())
    }

    fn update_body_acceleration(&mut self) {
        for (_, body) in &mut self.bodies {
            if let Some(inner_joint) = self.joints.get(&body.inner_joint) {
                body.update_acceleration(inner_joint);
            } else {
                unreachable!("validation should have caught this");
            }
        }
    }

    fn update_body_states(&mut self) {
        for (_, body) in &mut self.bodies {
            if let Some(inner_joint) = self.joints.get(&body.inner_joint) {
                body.update_state(inner_joint);
            } else {
                unreachable!("validation should have caught this");
            }
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

    fn write_derivative(&self, dx: &mut SimStates) {
        for (i, joint) in self.joints.iter().enumerate() {
            joint.borrow().state_derivative(&mut dx.0[i]);
        }
        self.actuators.state_derivative(dx);
    }
}
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
struct Identifier {
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

fn find_by_name(sys: &MultibodySystemBuilder, name: &str) -> Result<Component, MultibodyErrors> {
    if name == sys.base.name {
        return Ok(Component::Base(sys.base.id));
    }

    if let Some(body) = sys.bodies.iter().find(|(_, body)| body.name == name) {
        return Ok(Component::Body(body.0));
    }

    if let Some(joint) = sys.joints.iter().find(|(_, joint)| joint.name == name) {
        return Ok(Component::Joint(joint.0));
    }

    if let Some(actuator) = sys
        .actuators
        .iter()
        .find(|(_, actuator)| actuator.name == name)
    {
        return Ok(Component::Actuator(actuator.0));
    }

    if let Some(sensor) = sys.sensors.iter().find(|(_, sensor)| sensor.name == name) {
        return Ok(Component::Sensor(sensor.0));
    }

    Err(MultibodyErrors::ComponentNotFound(name.to_string()))
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum Component {
    Actuator(Id),
    Base(Id),
    Body(Id),
    Joint(Id),
    Sensor(Id),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimOptions {
    pub sim_name: String,
    pub tstart: f64,
    pub tstop: f64,
    pub dt: f64,
}
