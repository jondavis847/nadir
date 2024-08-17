use crate::{
    aerospace::MultibodyGravity, algorithms::{
        articulated_body_algorithm::ArticulatedBodyAlgorithm,
        composite_rigid_body::{CompositeRigidBody, CrbCache},
        recursive_newton_euler::RecursiveNewtonEuler,
        MultibodyAlgorithm,
    }, body::{Body, BodySim, BodyTrait}, joint::{
        joint_sim::{JointSim, JointSimTrait},
        joint_state::{JointState,JointStates},        
        Joint, JointTrait,
    }, result::{update_body_states, MultibodyResult}, solver::rk4::solve_fixed_rk4, system::MultibodySystem, MultibodyErrors, MultibodyTrait
};

use spatial_algebra::{Acceleration, SpatialInertia, Velocity};
use std::collections::HashMap;
use std::ops::{AddAssign, MulAssign};
use std::time::{Instant, SystemTime};
use utilities::generate_unique_id;
use uuid::Uuid;

#[derive(Debug, Clone)]
pub struct MultibodyParameters;

#[derive(Debug, Clone)]
pub struct MultibodySystemSim {
    algorithm: MultibodyAlgorithm,
    pub bodies: Vec<BodySim>,
    pub body_names: Vec<String>,
    pub joints: Vec<JointSim>,
    pub joint_names: Vec<String>,
    gravity: HashMap<Uuid, MultibodyGravity>,
    parent_indeces: Vec<Option<usize>>,
    crb_cache: Option<CrbCache>,
}

impl TryFrom<MultibodySystem> for MultibodySystemSim {
    type Error = MultibodyErrors;

    fn try_from(sys: MultibodySystem) -> Result<Self, MultibodyErrors> {
        let mut bodysims = Vec::new();
        let mut bodynames = Vec::new();
        let mut jointsims = Vec::new();
        let mut jointnames = Vec::new();
        let mut parent_indeces: Vec<Option<usize>> = Vec::new();

        if let Some(base) = &sys.base {
            let outer_joint_ids = base.get_outer_joints();
            for id in outer_joint_ids {
                if let Some(joint) = sys.joints.get(&id) {
                    let joint_sim = JointSim::from(joint.clone()).with_algorithm(sys.algorithm);
                    let joint_index = jointsims.len();

                    jointsims.push(joint_sim);
                    jointnames.push(joint.get_name().to_string());
                    parent_indeces.push(None); // For the base

                    if let Some(next_body_id) = joint.get_outer_body_id() {
                        if let Some(next_body) = sys.bodies.get(next_body_id) {
                            bodynames.push(next_body.get_name().to_string());
                            bodysims.push(BodySim::from(next_body.clone()));

                            // calculate the joint mass properties
                            let joint = &mut jointsims[0];
                            let body_mass_properties = next_body.get_mass_properties();
                            joint.update_transforms(None);
                            let transforms = joint.get_transforms();
                            let jof_from_ob = transforms.jof_from_ob;
                            let spatial_inertia = SpatialInertia::from(*body_mass_properties);
                            let joint_mass_properties = jof_from_ob * spatial_inertia;
                            joint.set_inertia(joint_mass_properties);

                            recursive_sys_creation(
                                &sys.algorithm,
                                next_body,
                                joint_index,
                                &sys.bodies,
                                &sys.joints,
                                &mut bodysims,
                                &mut bodynames,
                                &mut jointsims,
                                &mut jointnames,
                                &mut parent_indeces,
                            );
                        }
                    }
                }
            }
        }

        // push base gravity to all bodies if there is one
        if let Some(base) = &sys.base {
            for body in &mut bodysims {
                body.gravity.extend(base.gravity.clone())
            }
        }

        let crb_cache = match sys.algorithm {
            MultibodyAlgorithm::CompositeRigidBody => {
                let mut n = 0;
                for joint in &mut jointsims {
                    n += joint.get_ndof();
                    joint.set_crb_index(n);
                }

                Some(CrbCache::new(n))
            }
            _ => None,
        };

        Ok(MultibodySystemSim {
            algorithm: sys.algorithm,
            bodies: bodysims,
            body_names: bodynames,
            joints: jointsims,
            joint_names: jointnames,
            gravity: sys.gravities,
            parent_indeces: parent_indeces,
            crb_cache,
        })
    }
}

impl MultibodySystemSim {
    fn collect_state(&self) -> JointStates {
        let new_joints: Vec<JointState> = self
            .joints
            .iter()
            .map(|joint| joint.get_derivative())
            .collect();
        JointStates(new_joints)
    }

    pub fn run(
        &mut self,
        dx: &mut JointStates,
        x: &JointStates,        
        _t: f64,
    ) {
        self.set_state(x.clone());
        self.update_joints();
        self.update_forces();

        match self.algorithm {
            MultibodyAlgorithm::ArticulatedBody => {
                let n: usize = self.joints.len();

                // First Pass
                for i in 0..n {
                    let v_ij = if let Some(parent_index) = self.parent_indeces[i] {
                        *self.joints[parent_index].get_v()
                    } else {
                        Velocity::zeros()
                    };
                    self.joints[i].aba_first_pass(v_ij);
                }

                // Second Pass
                for i in (0..n).rev() {
                    let inner_is_base = self.parent_indeces[i].is_none();

                    // we split up updating the parent to avoid borrowing issues
                    if let Some((parent_ia, parent_pa)) =
                        self.joints[i].aba_second_pass(inner_is_base)
                    {
                        // I believe this unwrap is safe or we wouldnt get into this logic
                        let parent_joint_index = self.parent_indeces[i].unwrap(); // I believe this unwrap is safe or we wouldnt get into this logic
                        self.joints[parent_joint_index].add_inertia_articulated(parent_ia);
                        self.joints[parent_joint_index].add_p_big_a(parent_pa);
                    }
                }

                // Third Pass
                for i in 0..n {
                    // get acceleration of parent
                    // if parent is the base, (index None), parent acceleration is 0

                    let a_ij = if let Some(parent_index) = self.parent_indeces[i] {
                        *self.joints[parent_index].get_a()
                    } else {
                        Acceleration::zeros()
                    };

                    self.joints[i].aba_third_pass(a_ij);
                }
            }
            MultibodyAlgorithm::CompositeRigidBody => {
                let n: usize = self.joints.len();

                // Run the Recursive Newton Euler algorithm to calculate C.
                let c = &mut self.crb_cache.as_mut().unwrap().c;

                // first pass
                for i in 0..n {
                    let (a_ij, v_ij) = if let Some(parent_index) = self.parent_indeces[i] {
                        (
                            *self.joints[parent_index].get_a(),
                            *self.joints[parent_index].get_v(),
                        )
                    } else {
                        (Acceleration::zeros(), Velocity::zeros())
                    };

                    let joint = &mut self.joints[i];
                    joint.rne_first_pass(a_ij, v_ij, false);
                }

                // Second Pass
                for i in (0..n).rev() {
                    let joint = &mut self.joints[i];
                    joint.rne_set_tau();

                    // set C values
                    joint.set_c(c);

                    if let Some(parent_index) = self.parent_indeces[i] {
                        let ij_jof_from_jof = joint.get_transforms().ij_jof_from_jof;
                        let parent_force = ij_jof_from_jof * joint.rne_get_force();

                        let parent = &mut self.joints[parent_index];
                        parent.rne_add_force(parent_force);
                    }
                }

                // Solve for H with CRB
                let h = &mut self.crb_cache.as_mut().unwrap().h;
                h.fill(0.0);

                // first pass
                self.joints.iter_mut().for_each(|joint| joint.reset_ic());

                // second pass
                for i in 0..n {
                    let joint = &mut self.joints[i];
                    joint.set_h(h);

                    if let Some(parent_index) = self.parent_indeces[i] {
                        let joint_ic = joint.get_ic();
                        let ij_jof_from_jof = joint.get_transforms().ij_jof_from_jof;
                        let joint_ic_in_parent = ij_jof_from_jof * joint_ic;

                        let parent = &mut self.joints[parent_index];
                        parent.add_ic(joint_ic_in_parent);

                        let j = i;
                        while let Some(parent_index) = self.parent_indeces[j] {}
                    };
                }
            }
        }

        update_body_states(&mut self.bodies, &self.joints);
        let new_dx = self.collect_state();
        dx.clone_from(&new_dx);

    }

    fn set_state(&mut self, states: JointStates) {
        for i in 0..states.0.len() {
            self.joints[i].set_state(states.0[i]);
        }
    }

    pub fn simulate(
        &mut self,
        name: String,
        tstart: f64,
        tstop: f64,
        dt: f64,
    ) -> Result<MultibodyResult, MultibodyErrors> {
        let start_time = SystemTime::now();
        let instant_start = Instant::now();       

        let sim_start_time = Instant::now();

        // solve for the multibody system
        let result = solve_fixed_rk4(self, tstart, tstop, dt);

        let (times, hashmap) = match result {
            Ok(result) => result,
            Err(e) => return Err(e)
        };        

        let sim_duration = sim_start_time.elapsed();        
        let total_duration = instant_start.elapsed();

        let mut name = name.clone();
        if name.is_empty() {
            name = format!("sim_{}", generate_unique_id());
        }

        Ok(MultibodyResult {
            name: name,
            system: self.clone(),
            sim_time: times,
            result: hashmap,
            time_start: start_time,
            sim_duration: sim_duration,
            total_duration: total_duration,
        })
    }

    fn update_forces(&mut self) {
        for i in 0..self.joints.len() {
            let body = &mut self.bodies[i];
            let joint = &mut self.joints[i];

            // get transforms
            let transforms = joint.get_transforms();

            // calculate gravity for the outer body
            body.calculate_gravity(&transforms.ob_from_base, &self.gravity);

            // calculate total external forces for the outer body
            body.calculate_external_force();

            // transform force to joint
            // cross product terms in spatial calculation will convert force at body cg to torque about joint

            joint.set_force(transforms.jof_from_ob * *body.get_external_force_body());
        }
    }

    fn update_joints(&mut self) {
        // update joint transforms
        for i in 0..self.joints.len() {
            // if parent index is None, parent body is the base, ij_transforms to base are just transforms to inner body
            if let Some(parent_index) = self.parent_indeces[i] {
                let ij_transforms = self.joints[parent_index].get_transforms();
                let ij_ob_from_ij_jof = ij_transforms.ob_from_jof;
                let ij_jof_from_base = ij_transforms.jof_from_base;
                self.joints[i].update_transforms(Some((ij_ob_from_ij_jof, ij_jof_from_base)));
            } else {
                self.joints[i].update_transforms(None)
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
    algorithm: &MultibodyAlgorithm,
    body: &Body,
    parent_joint_index: usize,
    bodies: &HashMap<Uuid, Body>,
    joints: &HashMap<Uuid, Joint>,
    bodysims: &mut Vec<BodySim>,
    bodynames: &mut Vec<String>,
    jointsims: &mut Vec<JointSim>,
    jointnames: &mut Vec<String>,
    parent_indeces: &mut Vec<Option<usize>>,
) {
    for id in body.get_outer_joints() {
        if let Some(joint) = joints.get(&id) {
            let joint_sim = JointSim::from(joint.clone()).with_algorithm(*algorithm);
            let joint_index = jointsims.len();

            jointsims.push(joint_sim);
            jointnames.push(joint.get_name().to_string());
            parent_indeces.push(Some(parent_joint_index));

            if let Some(next_body_id) = joint.get_outer_body_id() {
                if let Some(next_body) = bodies.get(next_body_id) {
                    bodynames.push(next_body.get_name().to_string());
                    bodysims.push(BodySim::from(next_body.clone()));

                    // calculate the joint mass properties
                    let joint = &mut jointsims[joint_index];
                    let body_mass_properties = next_body.get_mass_properties();
                    joint.update_transforms(None);
                    let transforms = joint.get_transforms();
                    let jof_from_ob = transforms.jof_from_ob;
                    let spatial_inertia = SpatialInertia::from(*body_mass_properties);
                    let joint_mass_properties = jof_from_ob * spatial_inertia;
                    joint.set_inertia(joint_mass_properties);

                    recursive_sys_creation(
                        algorithm,
                        next_body,
                        joint_index,
                        bodies,
                        joints,
                        bodysims,
                        bodynames,
                        jointsims,
                        jointnames,
                        parent_indeces,
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

impl<'a> AddAssign<&'a Self> for MultibodyState {
    fn add_assign(&mut self, rhs: &'a Self) {
        assert_eq!(
            self.joints.len(),
            rhs.joints.len(),
            "Joint vectors must have the same length"
        );
        for (a, b) in self.joints.iter_mut().zip(rhs.joints.iter()) {
            *a += b;
        }
    }
}

impl MulAssign<f64> for MultibodyState {
    fn mul_assign(&mut self, rhs: f64) {
        // Create a new vector for joints with each joint multiplied by rhs
        self.joints.iter_mut().for_each(|joint| *joint *= rhs);
    }
}

