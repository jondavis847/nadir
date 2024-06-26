use std::collections::HashMap;
use std::fmt;
use std::ops::{Add, AddAssign, Div, Mul};
use uuid::Uuid;

use crate::{
    algorithms::{articulated_body_algorithm::ArticulatedBodyAlgorithm, MultibodyAlgorithm},
    body::{Body, BodySim, BodyTrait},
    joint::{
        revolute::RevoluteResult, Joint, JointResult, JointSim, JointSimTrait, JointState,
        JointTrait,
    },
    system::MultibodySystem,
    MultibodyTrait,
};
use differential_equations::solver::{Solver, SolverMethod};
use spatial_algebra::{Acceleration, Velocity};

pub struct MultibodyParameters;

#[derive(Debug, Clone)]
pub struct MultibodySystemSim {
    algorithm: MultibodyAlgorithm,
    bodies: Vec<BodySim>,
    joints: Vec<JointSim>,
    joint_names: Vec<String>,
    parent_joint_indeces: Vec<usize>,
}

impl From<MultibodySystem> for MultibodySystemSim {
    fn from(sys: MultibodySystem) -> Self {
        sys.validate();

        let mut bodysims = Vec::new();
        let mut jointsims = Vec::new();
        let mut jointnames = Vec::new();
        let mut parent_joint_indeces = Vec::new();

        let base = sys.base.unwrap();
        let outer_joint_ids = base.get_outer_joints();
        for id in outer_joint_ids {
            let joint = sys.joints.get(id).unwrap();
            let joint_sim = JointSim::from(joint.clone());

            jointsims.push(joint_sim);
            jointnames.push(joint.get_name().to_string());

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
                &mut jointnames,
                &mut parent_joint_indeces,
            );
        }
        MultibodySystemSim {
            algorithm: sys.algorithm,
            bodies: bodysims,
            joints: jointsims,
            joint_names: jointnames,
            parent_joint_indeces: parent_joint_indeces,
        }
    }
}

impl MultibodySystemSim {
    fn collect_state(&self) -> MultibodyState {
        let new_joints: Vec<JointState> = self
            .joints
            .iter()
            .map(|joint| joint.get_aba_derivative())
            .collect();
        MultibodyState { joints: new_joints }
    }

    pub fn run(
        &mut self,
        x: &MultibodyState,
        _p: &Option<MultibodyParameters>,
        _t: f64,
    ) -> MultibodyState {
        self.set_state(x.clone());
        self.update_joints();
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

                self.collect_state()
            }
        }
    }

    fn set_state(&mut self, state: MultibodyState) {
        for i in 0..state.joints.len() {
            self.joints[i].set_state(state.joints[i]);
        }
    }
    pub fn simulate(&mut self, tstart: f64, tstop: f64, dt: f64) -> MultibodyResult {
        // TODO: Do the same for bodies

        // Create a vec of JointStates
        let joint_states: Vec<JointState> =
            self.joints.iter().map(|joint| joint.get_state()).collect();

        let initial_multibody_state = MultibodyState {
            joints: joint_states,
        };

        let mut solver = Solver {
            func: |x, p, t| self.run(x, p, t),
            x0: initial_multibody_state,
            parameters: None,
            tstart,
            tstop,
            dt,
            solver: SolverMethod::Rk4Classical,
        };

        let (t, states) = solver.solve();

        // Convert to a multibody result
        let mut result_hm = HashMap::<String, ResultEntry>::new();
        result_hm.insert("t".to_string(), ResultEntry::VecF64(t));

        for state in states {
            for (i, joint) in state.joints.iter().enumerate() {
                //let joint_id = joint_ids[i];
                let joint_name = self.joint_names[i].clone();
                if let JointState::Revolute(revolute) = joint {
                    let entry = result_hm.entry(joint_name).or_insert_with(|| {
                        ResultEntry::Joint(JointResult::Revolute(RevoluteResult::default()))
                    });

                    if let ResultEntry::Joint(JointResult::Revolute(revolute_result)) = entry {
                        revolute_result.theta.push(revolute.theta);
                        revolute_result.omega.push(revolute.omega);
                    }
                }
            }
        }

        MultibodyResult(result_hm)
    }

    fn update_bodies(&mut self) {
        //TODO update body states based on joint states
        //calculate external forces
    }

    fn update_joints(&mut self) {
        self.update_transforms();

        //calculate tau for each joint
        for joint in &mut self.joints {
            joint.calculate_tau();
        }
    }

    // The main update_transforms function
    fn update_transforms(&mut self) {
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
    jointnames: &mut Vec<String>,
    parent_joint_indeces: &mut Vec<usize>,
) {
    let outer_joint_ids = body.get_outer_joints();
    for id in outer_joint_ids {
        let joint = joints.get(id).unwrap();
        let joint_sim = JointSim::from(joint.clone());

        jointsims.push(joint_sim);
        jointnames.push(joint.get_name().to_string());
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
            jointnames,
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

pub struct MultibodyResult(HashMap<String, ResultEntry>);

#[derive(Debug)]
enum ResultEntry {
    VecF64(Vec<f64>),
    Joint(JointResult),
}

impl fmt::Debug for MultibodyResult {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Define the column width
        const COL_WIDTH: usize = 10;

        // Function to format numbers without trailing zeros and fit in column width
        fn format_number(num: f64, width: usize) -> String {
            let mut s = format!("{:.5}", num).trim_end_matches('0').trim_end_matches('.').to_string();
            if s.len() > width {
                s.truncate(width);
            }
            format!("{:<width$}", s, width = width)
        }

        // Collect headers with "t" as the first column
        let mut headers: Vec<&String> = self.0.keys().collect();
        headers.sort();
        if let Some(pos) = headers.iter().position(|&header| header == "t") {
            headers.remove(pos);
        }
        let t_title = "t".to_string();
        headers.insert(0, &t_title);

        // Assuming all Vec<f64> are of the same length
        let row_count = self.0.values().next().map_or(0, |entry| match entry {
            ResultEntry::VecF64(vec) => vec.len(),
            ResultEntry::Joint(JointResult::Revolute(revolute_result)) => revolute_result.theta.len(),
        });

        let rows: Vec<String> = (0..row_count)
            .map(|i| {
                headers.iter().flat_map(|header| match self.0.get(*header).unwrap() {
                    ResultEntry::VecF64(vec) => vec![format_number(vec[i], COL_WIDTH)],
                    ResultEntry::Joint(JointResult::Revolute(revolute_result)) => vec![
                        format_number(revolute_result.theta[i], COL_WIDTH),
                        format_number(revolute_result.omega[i], COL_WIDTH),
                    ],
                }).collect::<Vec<String>>().join(" | ")
            })
            .collect();

        // Write the headers
        writeln!(f)?;
        let col_headers: Vec<String> = headers.iter().flat_map(|header| {
            if let ResultEntry::Joint(_) = self.0.get(*header).unwrap() {
                vec![format!("{:^width$}", header, width = COL_WIDTH * 2 + 3)]
            } else {
                vec![format!("{:^width$}", header, width = COL_WIDTH)]
            }
        }).collect();
        writeln!(f, "{}", col_headers.join(" | "))?;

        // Add horizontal line
        let header_line: String = col_headers.iter().map(|header| "-".repeat(header.len())).collect::<Vec<String>>().join("-+-");
        writeln!(f, "{}", header_line)?;

        let sub_headers: Vec<String> = headers.iter().flat_map(|header| {
            if let ResultEntry::Joint(_) = self.0.get(*header).unwrap() {
                vec![
                    format!("{:<width$}", "theta", width = COL_WIDTH),
                    format!("{:<width$}", "omega", width = COL_WIDTH)
                ]
            } else {
                vec![format!("{:<width$}", "", width = COL_WIDTH)]
            }
        }).collect();
        writeln!(f, "{}", sub_headers.join(" | "))?;

        // Add another horizontal line
        let sub_header_line: String = sub_headers.iter().map(|sub_header| "-".repeat(sub_header.len())).collect::<Vec<String>>().join("-+-");
        writeln!(f, "{}", sub_header_line)?;

        // Write the rows
        for row in rows {
            writeln!(f, "{}", row)?;
        }

        Ok(())
    }
}
