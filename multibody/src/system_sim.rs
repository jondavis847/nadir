use crate::{
    algorithms::{articulated_body_algorithm::ArticulatedBodyAlgorithm, MultibodyAlgorithm},
    body::{BodySim, BodyTrait},
    joint::JointTrait,
};
use spatial_algebra::{Acceleration, Velocity};
use std::collections::HashMap;
use uuid::Uuid;

use super::{
    body::Body,
    joint::{JointEnum, JointSim},
    system::MultibodySystem,
};

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
            recursive_sys_creation(
                next_body_id,
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
    fn run(&mut self) {
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
    }

    pub fn update_bodies(&mut self) {
        //TODO update body states based on joint states
        //calculate external forces
    }

    // The main update_transforms function
    pub fn update_transforms(&mut self) {
        if let Some(base) = &self.base {
            let outer_joints = base.get_outer_joints();
            for joint_id in outer_joints {
                let joint = self.joints.get_mut(joint_id).unwrap();
                joint.update_transforms(None);

                let outer_body_id = joint.get_outer_body_id().unwrap();
                update_transforms_recursive(*outer_body_id, &self.bodies, &mut self.joints);
            }
        }
    }
}

fn recursive_sys_creation(
    body_id: &Uuid,
    parent_joint_index: &usize,
    bodies: &HashMap<Uuid, Body>,
    joints: &HashMap<Uuid, JointEnum>,
    bodysims: &mut Vec<BodySim>,
    jointsims: &mut Vec<JointSim>,
    parent_joint_indeces: &mut Vec<usize>,
) {
    let body = bodies.get(body_id).unwrap();
    let outer_joint_ids = body.get_outer_joints();
    for id in outer_joint_ids {
        let joint = joints.get(id).unwrap();
        let joint_sim = JointSim::from(joint.clone());

        jointsims.push(joint_sim);
        bodysims.push(BodySim::from(body.clone()));
        parent_joint_indeces.push(*parent_joint_index);

        let joint_index = jointsims.len() - 1; //-1 since zero based indexing
        let next_body_id = joint.get_outer_body_id().unwrap();
        recursive_sys_creation(
            next_body_id,
            &joint_index,
            bodies,
            joints,
            bodysims,
            jointsims,
            parent_joint_indeces,
        );
    }
}
