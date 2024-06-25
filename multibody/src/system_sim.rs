use crate::{
    algorithms::{articulated_body_algorithm::ArticulatedBodyAlgorithm, MultibodyAlgorithm},
    body::{BodySim, BodyState, BodyTrait},
    joint::{JointSimTrait, JointState, JointTrait},
};
use differential_equations::{
    solver::{Solver, SolverMethod},
    Integrable,
};
use spatial_algebra::{Acceleration, Velocity};
use std::collections::HashMap;
use std::ops::{Add, AddAssign, Div, Mul};
use uuid::Uuid;

use super::{
    body::Body,
    joint::{Joint, JointSim},
    system::MultibodySystem,
};

struct MultibodyParameters;

#[derive(Debug, Clone)]
pub struct MultibodySystemSim {
    algorithm: MultibodyAlgorithm,
    bodies: Vec<BodySim>,
    joints: Vec<JointSim>,
    parent_joint_indeces: Vec<usize>,
}

impl From<MultibodySystem> for MultibodySystemSim {
    fn from(sys: MultibodySystem) -> Self {
        sys.validate();

        let mut bodysims = Vec::new();
        let mut jointsims = Vec::new();
        let mut parent_joint_indeces = Vec::new();

        let base = sys.base.unwrap();
        let outer_joint_ids = base.get_outer_joints();
        for id in outer_joint_ids {
            let joint = sys.joints.get(id).unwrap();
            let joint_sim = JointSim::from(joint.clone());
            jointsims.push(joint_sim);
            parent_joint_indeces.push(0); // just make it something for the base

            let joint_index = jointsims.len() - 1; //-1 since 0 based indexing
            let next_body_id = joint.get_outer_body_id().unwrap();
            let next_body = sys.bodies.get(next_body_id).unwrap();
            bodysims.push(BodySim::from(next_body.clone()));
            recursive_sys_creation(
                next_body,
                &joint_index,
                &sys.bodies,
                &sys.joints,
                &mut bodysims,
                &mut jointsims,
                &mut parent_joint_indeces,
            );
        }
        MultibodySystemSim {
            algorithm: sys.algorithm,
            bodies: bodysims,
            joints: jointsims,
            parent_joint_indeces: parent_joint_indeces,
        }
    }
}

impl MultibodySystemSim {
    fn collect_state(&self) -> MultibodyState {
        let new_joints: Vec<JointState> =
            self.joints.iter().map(|joint| joint.get_state()).collect();
        MultibodyState { joints: new_joints }
    }

    pub fn run(
        &mut self,
        x: &MultibodyState,
        _p: &Option<MultibodyParameters>,
        t: f64,
    ) -> MultibodyState {
        self.set_state(x.clone());
        self.update_transforms();
        self.update_bodies();
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
            }
            _ => {} //TODO: nothing for now
        }

        self.collect_state()
    }

    fn set_state(&mut self, state: MultibodyState) {
        for i in 0..state.joints.len() {
            self.joints[i].set_state(state.joints[i]);
        }
    }
    pub fn simulate(&mut self, tstart: f64, tstop: f64, dt: f64) -> (Vec<f64>, Vec<MultibodyState>) {
        // create a vec of JointStates
        let mut joint_states = Vec::<JointState>::new();
        for joint in &self.joints {
            joint_states.push(joint.get_state());
        }

        let initial_multibody_state = MultibodyState {
            joints: joint_states,
        };

        let mut solver = Solver {
            func: |x, p, t| self.run(x, p, t),
            x0: initial_multibody_state,
            parameters: None,
            tstart: tstart,
            tstop: tstop,
            dt: dt,
            solver: SolverMethod::Rk4Classical,
        };

        solver.solve()
    }

    pub fn update_bodies(&mut self) {
        //TODO update body states based on joint states
        //calculate external forces
    }

    // The main update_transforms function
    pub fn update_transforms(&mut self) {
        for i in 0..self.joints.len() {
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
    }
}

fn recursive_sys_creation(
    body: &Body,
    parent_joint_index: &usize,
    bodies: &HashMap<Uuid, Body>,
    joints: &HashMap<Uuid, Joint>,
    bodysims: &mut Vec<BodySim>,
    jointsims: &mut Vec<JointSim>,
    parent_joint_indeces: &mut Vec<usize>,
) {
    let outer_joint_ids = body.get_outer_joints();
    for id in outer_joint_ids {
        let joint = joints.get(id).unwrap();
        let joint_sim = JointSim::from(joint.clone());

        jointsims.push(joint_sim);
        parent_joint_indeces.push(*parent_joint_index);

        let joint_index = jointsims.len() - 1; //-1 since zero based indexing
        let next_body_id = joint.get_outer_body_id().unwrap();
        let next_body = bodies.get(next_body_id).unwrap();
        bodysims.push(BodySim::from(next_body.clone()));
        recursive_sys_creation(
            next_body,
            &joint_index,
            bodies,
            joints,
            bodysims,
            jointsims,
            parent_joint_indeces,
        );
    }
}

#[derive(Clone, Debug)]
pub struct MultibodyState {
    joints: Vec<JointState>,
    //bodies: Vec<BodyState>,
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
        //let bodies = self.bodies.into_iter().zip(rhs.bodies).map(|(a, b)| a + b).collect();

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

        MultibodyState {
            joints: new_joints,
            // bodies: new_bodies, // Uncomment and adjust if you have bodies
        }
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
