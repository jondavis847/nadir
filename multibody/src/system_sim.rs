use crate::{
    body::{BodySim, BodyTrait},
    joint::JointTrait,
};
use std::collections::HashMap;
use uuid::Uuid;

use super::{
    body::Body,
    joint::{JointEnum, JointSim},
    system::MultibodySystem,
};

pub struct MultibodySystemSim {
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

            let joint_index = jointsims.len();
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
            bodies: bodysims,
            joints: jointsims,
            parent_joint_indeces: parent_joint_indeces,
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

        let joint_index = jointsims.len();
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

pub fn run(&mut self) {
    self.update_transforms();
    self.update_bodies();
    match self.algorithm {
        MultibodyAlgorithm::ArticulatedBody => {
            self.joints.iter_mut().for_each(|joint| joint.first_pass());
            self.joints
                .iter_mut()
                .rev()
                .for_each(|joint| joint.second_pass());
            self.joints.iter_mut().for_each(|joint| joint.third_pass());
        }
        _ => {} //TODO: nothing for now
    }
}

pub fn sort(&mut self) -> Result<(), MultibodyErrors> {
    //must be valid in order to sort
    match self.validate() {
        Ok(()) => {}
        Err(error) => return Err(error),
    }

    let mut new_bodies = Vec::new();
    let mut new_joints = Vec::new();

    let base = self.find_base();
    new_bodies.push(base.clone());

    // recursive loop to identify all multibody elements
    find_joints_for_sort(base.clone(), &mut new_bodies, &mut new_joints);

    self.bodies = new_bodies;
    self.joints = new_joints;
    Ok(())
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

// The recursive helper function
fn update_transforms_recursive(
    body_id: Uuid,
    bodies: &HashMap<Uuid, Body>,
    joints: &mut HashMap<Uuid, JointEnum>,
) {
    let body = bodies.get(&body_id).unwrap();

    let inner_joint = joints
        .get(&body.get_inner_joint_id().unwrap())
        .unwrap()
        .clone(); // TODO: Figure out how to get rid of this clone, maybe just clone the transform

    let outer_joints = body.get_outer_joints();

    for joint_id in outer_joints {
        let joint = joints.get_mut(joint_id).unwrap();
        joint.update_transforms(Some(&inner_joint));

        let outer_body_id = joint.get_outer_body_id().unwrap();
        update_transforms_recursive(*outer_body_id, bodies, joints);
    }
}

impl ArticulatedBodyAlgorithm for Revolute {
    fn first_pass(&mut self) {
        //TODO: this looks like its the same for every joint, make it at the joint level?
        let parent = self.common.get_inner_joint(); // TODO: benchmark if it's faster to make parent the borrow rather than the Rc<RefCell<>>

        let transforms = &self.common.transforms;
        let body = self.get_outer_body().unwrap().body;

        let aba = &mut self.aba.common;

        aba.v = transforms.jof_from_ij_jof * parent.get_v() + aba.vj;
        aba.c = aba.v.cross_motion(aba.vj); // + cj

        let joint_mass_properties = self.common.mass_properties.unwrap();

        aba.inertia_articulated = joint_mass_properties;
        aba.p_big_a = aba.v.cross_force(joint_mass_properties * aba.v)
            - transforms.jof_from_ob * body.get_external_force();
    }
    fn second_pass(&mut self) {
        let aba = &mut self.aba;
        let inertia_articulated_matrix = aba.common.inertia_articulated.matrix();
        let parent_ref = self.common.get_inner_joint();
        let mut parent = parent_ref.borrow_mut();

        // use the most efficient method for creating these. Indexing is much faster than 6x6 matrix mul
        aba.big_u = inertia_articulated_matrix.get_column(3).unwrap();
        aba.big_d_inv = 1.0 / aba.big_u.e31;
        aba.lil_u = -(aba.common.p_big_a.get_index(3).unwrap());
        if !self.common.connection.inner_is_base {
            let big_u_times_big_d_inv = aba.big_u * aba.big_d_inv;
            let i_lil_a =
                inertia_articulated_matrix - big_u_times_big_d_inv * aba.big_u.transpose();
            aba.common.p_lil_a = aba.common.p_big_a
                + Force::from(i_lil_a * aba.common.c.vector())
                + Force::from(big_u_times_big_d_inv * aba.lil_u);

            let parent_inertia_articulated_contribution =
                self.common.transforms.ij_jof_from_jof * SpatialInertia::from(i_lil_a);
            parent.add_inertia_articulated(parent_inertia_articulated_contribution);
            parent.add_p_big_a(self.common.transforms.ij_jof_from_jof * aba.common.p_big_a);
        }
    }
    fn third_pass(&mut self) {
        let parent_ref = self.common.get_inner_joint();
        let parent = parent_ref.borrow();
        let aba = &mut self.aba;

        aba.common.a_prime = self.common.transforms.jof_from_ij_jof * parent.get_a() + aba.common.c;
        self.state.q_ddot =
            aba.big_d_inv * (aba.lil_u - aba.big_u.transpose() * aba.common.a_prime.vector());
        aba.common.a = aba.common.a_prime
            + Acceleration::from(Vector6::new(0.0, 0.0, self.state.q_ddot, 0.0, 0.0, 0.0));
    }

    fn get_v(&self) -> Velocity {
        self.aba.common.v
    }

    fn get_p_big_a(&self) -> Force {
        self.aba.common.p_big_a
    }

    fn get_a(&self) -> Acceleration {
        self.aba.common.a
    }

    fn add_inertia_articulated(&mut self, inertia: SpatialInertia) {
        self.aba.common.inertia_articulated = self.aba.common.inertia_articulated + inertia;
    }

    fn add_p_big_a(&mut self, p_big_a: Force) {
        self.aba.common.p_big_a = self.aba.common.p_big_a + p_big_a;
    }
}