use std::collections::HashMap;
use std::ops::{Add, AddAssign, Div, Mul};
use std::time::{Instant, SystemTime};
use utilities::generate_unique_id;
use uuid::Uuid;

use crate::{
    algorithms::{articulated_body_algorithm::ArticulatedBodyAlgorithm, MultibodyAlgorithm},
    body::{Body, BodyResult, BodySim, BodyState, BodyTrait},
    joint::{
        revolute::RevoluteResult, Joint, JointResult, JointSim, JointSimTrait, JointState,
        JointTrait,
    },
    result::{update_body_states, MultibodyResult, ResultEntry},
    system::MultibodySystem,
    MultibodyTrait,
};
use differential_equations::solver::{Solver, SolverMethod};
use spatial_algebra::{Acceleration, Velocity};

#[derive(Debug, Clone)]
pub struct MultibodyParameters;

#[derive(Debug, Clone)]
pub struct MultibodySystemSim {
    algorithm: MultibodyAlgorithm,
    bodies: Vec<BodySim>,
    body_names: Vec<String>,
    joints: Vec<JointSim>,
    joint_names: Vec<String>,
    parent_joint_indeces: Vec<usize>,
}

impl From<MultibodySystem> for MultibodySystemSim {
    fn from(sys: MultibodySystem) -> Self {
        sys.validate();

        let mut bodysims = Vec::new();
        let mut bodynames = Vec::new();
        let mut jointsims = Vec::new();
        let mut jointnames = Vec::new();
        let mut parent_joint_indeces = Vec::new();

        if let Some(base) = sys.base {
            let outer_joint_ids = base.get_outer_joints();
            for id in outer_joint_ids {
                if let Some(joint) = sys.joints.get(&id) {
                    let joint_sim = JointSim::from(joint.clone());
                    let joint_index = jointsims.len();

                    jointsims.push(joint_sim);
                    jointnames.push(joint.get_name().to_string());
                    parent_joint_indeces.push(0); // For the base

                    if let Some(next_body_id) = joint.get_outer_body_id() {
                        if let Some(next_body) = sys.bodies.get(next_body_id) {
                            bodynames.push(next_body.get_name().to_string());
                            bodysims.push(BodySim::from(next_body.clone()));
                            recursive_sys_creation(
                                next_body,
                                joint_index,
                                &sys.bodies,
                                &sys.joints,
                                &mut bodysims,
                                &mut bodynames,
                                &mut jointsims,
                                &mut jointnames,
                                &mut parent_joint_indeces,
                            );
                        }
                    }
                }
            }
        }

        MultibodySystemSim {
            algorithm: sys.algorithm,
            bodies: bodysims,
            body_names: bodynames,
            joints: jointsims,
            joint_names: jointnames,
            parent_joint_indeces: parent_joint_indeces,
        }
    }
}

impl MultibodySystemSim {
    fn collect_state(&self) -> JointStates {
        let new_joints: Vec<JointState> = self
            .joints
            .iter()
            .map(|joint| joint.get_aba_derivative())
            .collect();
        JointStates(new_joints)
    }

    pub fn run(
        &mut self,
        x: &JointStates,
        _p: &Option<MultibodyParameters>,
        _t: f64,
    ) -> JointStates {
        self.set_state(x.clone());
        self.update_joints();
        self.update_body_forces();
        match self.algorithm {
            MultibodyAlgorithm::ArticulatedBody => {
                let n = self.joints.len();

                // First Pass
                for i in 0..n {
                    let v_ij = match i {
                        0 => Velocity::zeros(),
                        _ => *self.joints[self.parent_joint_indeces[i]].get_v(),
                    };
                    let f_ob = self.bodies[i].get_external_force();
                    self.joints[i].first_pass(v_ij, f_ob);
                }

                // Second Pass
                for i in (0..n).rev() {
                    let inner_is_base = match i {
                        0 => true,
                        _ => false,
                    };
                    // we split up the updating the parent to avoid borrowing issues
                    if let Some((parent_ia, parent_pa)) = self.joints[i].second_pass(inner_is_base)
                    {
                        self.joints[self.parent_joint_indeces[i]]
                            .add_inertia_articulated(parent_ia);
                        self.joints[self.parent_joint_indeces[i]].add_p_big_a(parent_pa);
                    }
                }

                // Third Pass
                for i in 0..n {
                    let a_ij = match i {
                        0 => Acceleration::zeros(),
                        _ => *self.joints[self.parent_joint_indeces[i]].get_a(),
                    };
                    self.joints[i].third_pass(a_ij);
                }
                update_body_states(&mut self.bodies, &self.joints);
                self.collect_state()
            }
        }
    }

    fn set_state(&mut self, states: JointStates) {
        for i in 0..states.0.len() {
            self.joints[i].set_state(states.0[i]);
        }
    }

    pub fn simulate(&mut self, name: String, tstart: f64, tstop: f64, dt: f64) -> MultibodyResult {
        let start_time = SystemTime::now();
        let instant_start = Instant::now();

        // Create a vec of JointStates
        let initial_joint_states =
            JointStates(self.joints.iter().map(|joint| joint.get_state()).collect());

        // Use Vec to store body states
        let mut body_states: Vec<Vec<BodyState>> = Vec::new();

        // Initialize the solver
        let mut solver = Solver {
            func: |x, p, t| self.run(x, p, t),
            x0: initial_joint_states,
            parameters: None,
            tstart,
            tstop,
            dt,
            solver: SolverMethod::Rk4Classical,
            callbacks: Vec::new(),
        };

        let sim_start_time = Instant::now();

        // Solve for joint states over time
        let (times, joint_states) = solver.solve();

        let sim_duration = sim_start_time.elapsed();

        // Post-process body states
        for (ti, js) in times.iter().zip(joint_states.iter()) {
            self.run(js, &None, *ti);
            update_body_states(&mut self.bodies, &self.joints);
            body_states.push(self.bodies.iter().map(|body| body.state).collect());
        }

        // Convert to a multibody result
        let mut result_hm = HashMap::<String, ResultEntry>::new();
        result_hm.insert("t".to_string(), ResultEntry::VecF64(times));

        for joint_state in &joint_states {
            for (i, joint) in joint_state.0.iter().enumerate() {
                let joint_name = self.joint_names[i].clone();
                match joint {
                    JointState::Revolute(revolute) => {
                        let entry = result_hm.entry(joint_name.clone()).or_insert_with(|| {
                            ResultEntry::Joint(JointResult::Revolute(RevoluteResult::default()))
                        });

                        if let ResultEntry::Joint(JointResult::Revolute(revolute_result)) = entry {
                            revolute_result.theta.push(revolute.theta);
                            revolute_result.omega.push(revolute.omega);
                        }
                    }
                }
            }
        }

        for body_state in body_states {
            for (i, body) in body_state.iter().enumerate() {
                let body_name = self.body_names[i].clone();
                let entry = result_hm
                    .entry(body_name.clone())
                    .or_insert_with(|| ResultEntry::Body(BodyResult::default()));
                if let ResultEntry::Body(body_result) = entry {
                    body_result.position_base.push(body.position_base);
                    body_result.velocity_base.push(body.velocity_base);
                    body_result.acceleration_base.push(body.acceleration_base);
                    body_result.acceleration_body.push(body.acceleration_body);
                    body_result.angular_accel_body.push(body.angular_accel_body);
                    body_result.angular_rate_body.push(body.angular_rate_body);
                    body_result.attitude_base.push(body.attitude_base);
                    body_result
                        .external_force_body
                        .push(body.external_force_body);
                    body_result
                        .external_torque_body
                        .push(body.external_torque_body);
                }
            }
        }

        let total_duration = instant_start.elapsed();

        let mut name = name.clone();
        if name.is_empty() {
            name = format!("sim_{}", generate_unique_id());
        }

        MultibodyResult {
            name: name,
            result: result_hm,
            time_start: start_time,
            sim_duration: sim_duration,
            total_duration: total_duration,
        }
    }

    fn update_body_forces(&mut self) {
        //pub external_force: Force, //used for calculations
        //pub external_force_body: Vector3, //use for reporting
        //pub external_torque_body: Vector3, //use for reporting
    }

    fn update_joints(&mut self) {
        // update joint transforms
        for i in 0..self.joints.len() {
            // if i is 0, parent body is the base, ij_transforms to base are just transforms to inner body
            match i {
                0 => self.joints[i].update_transforms(None),
                _ => {
                    let ij_transforms = self.joints[self.parent_joint_indeces[i]].get_transforms();
                    let ij_ob_from_ij_jof = ij_transforms.ob_from_jof;
                    let ij_jof_from_base = ij_transforms.jof_from_base;
                    self.joints[i].update_transforms(Some((ij_ob_from_ij_jof, ij_jof_from_base)));
                }
            }
        }

        //calculate tau and vj for each joint
        for joint in &mut self.joints {
            joint.calculate_vj();
            joint.calculate_tau();
        }
    }
}

fn recursive_sys_creation(
    body: &Body,
    parent_joint_index: usize,
    bodies: &HashMap<Uuid, Body>,
    joints: &HashMap<Uuid, Joint>,
    bodysims: &mut Vec<BodySim>,
    bodynames: &mut Vec<String>,
    jointsims: &mut Vec<JointSim>,
    jointnames: &mut Vec<String>,
    parent_joint_indeces: &mut Vec<usize>,
) {
    for id in body.get_outer_joints() {
        if let Some(joint) = joints.get(&id) {
            let joint_sim = JointSim::from(joint.clone());
            let joint_index = jointsims.len();

            jointsims.push(joint_sim);
            jointnames.push(joint.get_name().to_string());
            parent_joint_indeces.push(parent_joint_index);

            if let Some(next_body_id) = joint.get_outer_body_id() {
                if let Some(next_body) = bodies.get(next_body_id) {
                    bodynames.push(next_body.get_name().to_string());
                    bodysims.push(BodySim::from(next_body.clone()));
                    recursive_sys_creation(
                        next_body,
                        joint_index,
                        bodies,
                        joints,
                        bodysims,
                        bodynames,
                        jointsims,
                        jointnames,
                        parent_joint_indeces,
                    );
                }
            }
        }
    }
}

#[derive(Clone, Debug)]
pub struct MultibodyState {
    joints: Vec<JointState>,
}

//thought about making this a type JointStates = Vec<Joints> but it needs to be integrable
#[derive(Clone, Debug)]
pub struct JointStates(Vec<JointState>);

impl Add for JointStates {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        assert_eq!(
            self.0.len(),
            rhs.0.len(),
            "Joint vectors must have the same length"
        );

        let joints = self.0.into_iter().zip(rhs.0).map(|(a, b)| a + b).collect();
        JointStates(joints)
    }
}

impl AddAssign for JointStates {
    fn add_assign(&mut self, rhs: Self) {
        assert_eq!(
            self.0.len(),
            rhs.0.len(),
            "Joint vectors must have the same length"
        );
        for (a, b) in self.0.iter_mut().zip(rhs.0.into_iter()) {
            *a += b;
        }
    }
}

impl Mul<f64> for JointStates {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self {
        JointStates(self.0.into_iter().map(|joint| joint * rhs).collect())
    }
}

impl Div<f64> for JointStates {
    type Output = Self;

    fn div(self, rhs: f64) -> Self {
        JointStates(self.0.into_iter().map(|joint| joint / rhs).collect())
    }
}

impl Add for MultibodyState {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        assert_eq!(
            self.joints.len(),
            rhs.joints.len(),
            "Joint vectors must have the same length"
        );
        //assert_eq!(self.bodies.len(), rhs.bodies.len(), "Body vectors must have the same length");

        let joints = self
            .joints
            .into_iter()
            .zip(rhs.joints)
            .map(|(a, b)| a + b)
            .collect();
        MultibodyState { joints } //, bodies }
    }
}

impl AddAssign for MultibodyState {
    fn add_assign(&mut self, rhs: Self) {
        assert_eq!(
            self.joints.len(),
            rhs.joints.len(),
            "Joint vectors must have the same length"
        );
        for (a, b) in self.joints.iter_mut().zip(rhs.joints.into_iter()) {
            *a += b;
        }
    }
}

impl Mul<f64> for MultibodyState {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self {
        // Create a new vector for joints with each joint multiplied by rhs
        let new_joints = self.joints.into_iter().map(|joint| joint * rhs).collect();

        MultibodyState { joints: new_joints }
    }
}

impl Div<f64> for MultibodyState {
    type Output = Self;

    fn div(self, rhs: f64) -> Self {
        // Create a new vector for joints with each joint multiplied by rhs
        let new_joints = self.joints.into_iter().map(|joint| joint / rhs).collect();

        MultibodyState {
            joints: new_joints,
            // bodies: new_bodies, // Uncomment and adjust if you have bodies
        }
    }
}
