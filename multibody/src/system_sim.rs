use crate::{
    aerospace::MultibodyGravity, algorithms::{
        articulated_body_algorithm::ArticulatedBodyAlgorithm,
        composite_rigid_body::{CompositeRigidBody, CrbCache},
        recursive_newton_euler::RecursiveNewtonEuler,
        MultibodyAlgorithm,
    }, base::Base, body::{Body, BodySim, BodyTrait}, joint::{
        joint_sim::{JointSim, JointSimTrait},
        joint_state::{JointState, JointStates},
        Joint, JointTrait,
    }, result::MultibodyResult, sensor::Sensor, solver::rk4::solve_fixed_rk4, system::MultibodySystem, MultibodyErrors, MultibodyTrait
};
use rotations::{RotationTrait, quaternion::Quaternion};

use serde::{Deserialize, Serialize};
use spatial_algebra::{Acceleration, MotionVector, SpatialInertia, SpatialVector, Velocity};
use std::collections::HashMap;
use std::ops::{AddAssign, MulAssign};
use std::time::{Instant, SystemTime};
use utilities::generate_unique_id;
use uuid::Uuid;

#[derive(Debug, Clone)]
pub struct MultibodyParameters;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MultibodySystemSim {
    algorithm: MultibodyAlgorithm,
    pub base: Base,
    pub bodies: Vec<BodySim>,
    pub body_names: Vec<String>,
    pub joints: Vec<JointSim>,
    pub joint_names: Vec<String>,
    gravity: HashMap<Uuid, MultibodyGravity>,
    parent_indeces: Vec<Option<usize>>,
    crb_cache: Option<CrbCache>,
    pub sensors: HashMap<Uuid, Sensor>,
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
                            let joint_sim = &mut jointsims[0];
                            let body_mass_properties = next_body.get_mass_properties();
                            joint_sim.update_transforms(None);                            
                            joint_sim.set_inertia(body_mass_properties);

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
                        } else {
                            return Err(MultibodyErrors::BodyNotFound);
                        }
                    } else {
                        return Err(MultibodyErrors::JointMissingOuterBody(*id));
                    }
                } else {
                    return Err(MultibodyErrors::JointNotFound);
                }
            }

            // push base gravity to all bodies if there is one
            for body in &mut bodysims {
                body.gravity.extend(base.gravity.clone())
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
                base: base.clone(),
                bodies: bodysims,
                body_names: bodynames,
                joints: jointsims,
                joint_names: jointnames,
                gravity: sys.gravities,
                parent_indeces: parent_indeces,
                crb_cache,
                sensors: sys.sensors,
            })
        } else {
            Err(MultibodyErrors::BaseNotFound)
        }
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

    pub fn run(&mut self, dx: &mut JointStates, x: &JointStates, _t: f64) {
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
                        *self.joints[parent_index].get_a_jof()
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
                            *self.joints[parent_index].get_a_jof(),
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

                        // just commented out until we finish the crb
                        //let j = i;
                        //while let Some(parent_index) = self.parent_indeces[j] {}
                    };
                }
            }
        }

        self.update_body_states(); //only update body states once joint accels have been calculated (so we can update body accel based on joint accel)
        self.update_sensors(); //only update sensors after body states have been updated
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
            Err(e) => return Err(e),
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

    pub fn update_body_states(&mut self) {
        for i in 0..self.bodies.len() {
            let body = &mut self.bodies[i];
            let inner_joint = &self.joints[i];
            let transforms = inner_joint.get_transforms();
            let body_from_joint = transforms.ob_from_jof;
    
            let base_from_body = transforms.base_from_jof * transforms.jof_from_ob;
            let joint_a = inner_joint.get_a_jof();
    
            let body_a = body_from_joint * *joint_a;        
            // accel in body to accel in base is just a rotation, translation due to rotation should be accounted for in calc of body_a
            let body_a_in_base = base_from_body * body_a;
            //let body_a_in_base_rotation = base_from_body.0.rotation.transform(*body_a.rotation());
            //let body_a_in_base_translation = base_from_body.0.rotation.transform(*body_a.translation());
            //let body_a_in_base = Acceleration(MotionVector(SpatialVector::new(
                //body_a_in_base_rotation,
                //body_a_in_base_translation,
            //)));        
            let joint_v = inner_joint.get_v();
            let body_v = body_from_joint * *joint_v;        
            // velocity in body to velocity in base is just a rotation, translation due to rotation should be accounted for in calc of body_v
            let body_v_in_base_rotation = base_from_body.0.rotation.transform(*body_v.rotation());
            let body_v_in_base_translation = base_from_body.0.rotation.transform(*body_v.translation());
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
            body.state.attitude_base = Quaternion::from(body_from_base.rotation);
        }
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

    fn update_sensors(&mut self) {
        for body in &mut self.bodies {
            body.update_sensors(&mut self.sensors);
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
                    joint.set_inertia(body_mass_properties);

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
