use crate::{
    algorithms::articulated_body_algorithm::{AbaCache, ArticulatedBodyAlgorithm},
    body::{body_enum::BodyEnum, BodyTrait},
    joint::{
        Connection, JointCommon, JointEnum, JointErrors, JointParameters, JointRef, JointTrait,
        JointTransforms,
    },
    MultibodyTrait,
};
use linear_algebra::{matrix6x1::Matrix6x1, vector6::Vector6};
use spatial_algebra::{Acceleration, Force, SpatialInertia, Velocity};
use std::cell::RefCell;
use std::rc::Rc;
use transforms::Transform;

pub enum RevoluteErrors {}

#[derive(Debug, Default, Clone, Copy)]
pub struct RevoluteState {
    theta: f64,
    omega: f64,
    q_ddot: f64,    
}

impl RevoluteState {
    pub fn new(theta: f64, omega: f64) -> Self {        
        // assume this is about Z until we add more axes        
        let q_ddot = 0.0;
        Self {
            theta,
            omega,
            q_ddot,            
        }
    }
}

#[derive(Debug, Default, Clone, Copy)]

struct RevoluteRk4Cache([RevoluteState; 4]);

#[derive(Debug, Clone)]
pub struct Revolute {
    common: JointCommon,
    parameters: JointParameters,
    state: RevoluteState,
    aba: RevoluteAbaCache,
    rk4: RevoluteRk4Cache,
}

impl Revolute {
    pub fn new(name: &str, parameters: JointParameters, state: RevoluteState) -> JointRef {
        let common = JointCommon::new(name);
        let aba = RevoluteAbaCache::default();
        let rk4 = RevoluteRk4Cache::default();

        Rc::new(RefCell::new(JointEnum::Revolute(Self {
            common,
            parameters,
            state,
            aba,
            rk4,
        })))
    }
}

impl JointTrait for Revolute {
    fn connect_inner_body(
        &mut self,
        bodyref: BodyRef,
        transform: Transform,
    ) -> Result<(), JointErrors> {
        if self.common.connection.inner_body.is_some() {
            return Err(JointErrors::InnerBodyExists);
        }
        let connection = Connection::new(bodyref.clone(), transform);
        self.common.connection.inner_body = Some(connection);
        let body = bodyref.borrow();
        match &*body {
            BodyEnum::Base(_) => self.common.connection.inner_is_base = true,
            BodyEnum::Body(_) => self.common.connection.inner_is_base = false,
        }
        Ok(())
    }

    fn connect_outer_body(
        &mut self,
        body: BodyRef,
        transform: Transform,
    ) -> Result<(), JointErrors> {
        if self.common.connection.outer_body.is_some() {
            return Err(JointErrors::OuterBodyExists);
        }
        let connection = Connection::new(body.clone(), transform);
        self.common.connection.outer_body = Some(connection);
        self.common.mass_properties = Some(SpatialInertia(
            transform * body.borrow().get_mass_properties(),
        ));
        Ok(())
    }

    fn delete_inner_body(&mut self) {
        if self.common.connection.inner_body.is_some() {
            self.common.connection.inner_body = None;
        }
    }

    fn delete_outer_body(&mut self) {
        if self.common.connection.outer_body.is_some() {
            self.common.connection.outer_body = None;
        }
        self.common.mass_properties = None;
    }

    fn get_inner_body(&self) -> Option<Connection> {
        self.common.connection.inner_body.clone()
    }
    fn get_outer_body(&self) -> Option<Connection> {
        self.common.connection.outer_body.clone()
    }
    fn get_transforms(&self) -> JointTransforms {
        self.common.transforms
    }

    fn update_transforms(&mut self) {        
        self.common.update_transforms();
    }
}

impl MultibodyTrait for Revolute {
    fn get_name(&self) -> &str {
        &self.common.name
    }

    fn set_name(&mut self, name: String) {
        self.common.name = name;
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct RevoluteAbaCache {
    common: AbaCache,
    lil_u: f64,
    big_d_inv: f64,
    big_u: Matrix6x1,
    q: f64,
    q_dot: f64,
    q_ddot: f64,
    inertia_lil_a: f64,
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
