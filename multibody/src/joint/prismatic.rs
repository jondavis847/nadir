use crate::{
    algorithms::{
        articulated_body_algorithm::ArticulatedBodyAlgorithm, recursive_newton_euler::RneCache,
    },
    joint::{joint_transforms::JointTransforms, JointParameters},
    solver::SimStateVector,
};
use coordinate_systems::{cartesian::Cartesian, CoordinateSystem};
use mass_properties::MassProperties;
use nadir_result::ResultManager;
use nalgebra::{Matrix6x1, Vector6};
use rand::rngs::SmallRng;
use rotations::{Rotation, RotationTrait};
use serde::{Deserialize, Serialize};
use spatial_algebra::{Acceleration, Force, SpatialInertia, SpatialTransform, Velocity};
use std::ops::{AddAssign, MulAssign};
use thiserror::Error;
use transforms::Transform;
use uncertainty::{Normal, SimValue, Uncertainty, UncertaintyErrors, Uniform};

use super::{JointCache, JointErrors, JointModel, JointParametersBuilder, JointRef};

#[derive(Debug, Error)]
pub enum PrismaticErrors {
    #[error("{0}")]
    Uncertainty(#[from] UncertaintyErrors),
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct PrismaticStateBuilder {
    pub position: SimValue,
    pub velocity: SimValue,
}

impl Uncertainty for PrismaticStateBuilder {
    type Output = PrismaticState;
    type Error = PrismaticErrors;

    fn sample(&self, nominal: bool, rng: &mut SmallRng) -> Result<Self::Output, Self::Error> {
        Ok(PrismaticState {
            position: self.position.sample(nominal, rng),
            velocity: self.velocity.sample(nominal, rng),
        })
    }
}

#[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
pub struct PrismaticState {
    pub position: f64,
    pub velocity: f64,
}

impl<'a> AddAssign<&'a Self> for PrismaticState {
    fn add_assign(&mut self, rhs: &'a Self) {
        self.position += rhs.position;
        self.velocity += rhs.velocity;
    }
}

impl MulAssign<f64> for PrismaticState {
    fn mul_assign(&mut self, rhs: f64) {
        self.position *= rhs;
        self.velocity *= rhs;
    }
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct PrismaticParametersBuilder(JointParametersBuilder);

impl PrismaticParametersBuilder {}

impl Uncertainty for PrismaticParametersBuilder {
    type Output = PrismaticParameters;
    type Error = JointErrors;

    fn sample(&self, nominal: bool, rng: &mut SmallRng) -> Result<Self::Output, Self::Error> {
        Ok(PrismaticParameters(self.0.sample(nominal, rng)?))
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrismaticParameters(JointParameters);

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrismaticBuilder {
    pub parameters: PrismaticParametersBuilder,
    pub state: PrismaticStateBuilder,
}

impl PrismaticBuilder {
    pub fn new() -> Self {
        Self {
            parameters: PrismaticParametersBuilder::default(),
            state: PrismaticStateBuilder::default(),
        }
    }

    /// Sets the nominal initial position state
    pub fn set_position(&mut self, position: f64) {
        self.state.position = SimValue::new(position);
    }

    /// Sets the nominal initial velocity state
    pub fn set_velocity(&mut self, velocity: f64) {
        self.state.velocity = SimValue::new(velocity);
    }

    /// Sets the joint damping parameter
    pub fn set_damping(&mut self, damping: f64) {
        self.parameters.0.damping.nominal = damping;
    }

    /// Sets the joint equilibrium parameter
    pub fn set_equilibrium(&mut self, equilibrium: f64) {
        self.parameters.0.equilibrium.nominal = equilibrium;
    }

    /// Sets the joint spring_constant parameter
    pub fn set_spring_constant(&mut self, spring_constant: f64) {
        self.parameters.0.spring_constant.nominal = spring_constant;
    }

    /// Sets the joint constant_force parameter
    pub fn set_constant_force(&mut self, constant_force: f64) {
        self.parameters.0.constant_force.nominal = constant_force;
    }

    /// Sets an initial position state uncertainty with a normal distribution
    /// For use with Monte Carlo simulations
    /// This does not change the nominal value, see .set_position()
    pub fn set_uncertain_position_normal(
        &mut self,
        mean: f64,
        std: f64,
    ) -> Result<(), PrismaticErrors> {
        let dist = Normal::new(mean, std)?;
        self.state.position.set_distribution(dist.into())?;
        Ok(())
    }

    /// Sets an initial position state uncertainty with a uniform distribution
    /// For use with Monte Carlo simulations
    /// This does not change the nominal value, see .set_position()
    pub fn set_uncertain_position_uniform(
        &mut self,
        low: f64,
        high: f64,
    ) -> Result<(), PrismaticErrors> {
        let dist = Uniform::new(low, high)?;
        self.state.position.set_distribution(dist.into())?;
        Ok(())
    }

    /// Sets the joint damping parameter uncertainty with a normal distribution
    /// For use with Monte Carlo simulations
    /// This does not change the nominal value, see .set_damping()
    pub fn set_uncertain_damping_normal(
        &mut self,
        mean: f64,
        std: f64,
    ) -> Result<(), PrismaticErrors> {
        let dist = Normal::new(mean, std)?;
        self.parameters.0.damping.set_distribution(dist.into())?;
        Ok(())
    }

    /// Sets the joint damping parameter uncertainty with a uniform distribution
    /// For use with Monte Carlo simulations
    /// This does not change the nominal value, see .set_damping()
    pub fn set_uncertain_damping_uniform(
        &mut self,
        low: f64,
        high: f64,
    ) -> Result<(), PrismaticErrors> {
        let dist = Uniform::new(low, high)?;
        self.parameters.0.damping.set_distribution(dist.into())?;
        Ok(())
    }

    /// Sets the joint equilibrium parameter uncertainty with a normal distribution
    /// For use with Monte Carlo simulations
    /// This does not change the nominal value, see .set_equilibrium()
    pub fn set_uncertain_equilibrium_normal(
        &mut self,
        mean: f64,
        std: f64,
    ) -> Result<(), PrismaticErrors> {
        let dist = Normal::new(mean, std)?;
        self.parameters
            .0
            .equilibrium
            .set_distribution(dist.into())?;
        Ok(())
    }

    /// Sets the joint equilibrium parameter uncertainty with a uniform distribution
    /// For use with Monte Carlo simulations
    /// This does not change the nominal value, see .set_equilibrium()
    pub fn set_uncertain_equilibrium_uniform(
        &mut self,
        low: f64,
        high: f64,
    ) -> Result<(), PrismaticErrors> {
        let dist = Uniform::new(low, high)?;
        self.parameters
            .0
            .equilibrium
            .set_distribution(dist.into())?;
        Ok(())
    }

    /// Sets the joint spring_constant parameter uncertainty with a normal distribution
    /// For use with Monte Carlo simulations
    /// This does not change the nominal value, see .set_spring_constant()
    pub fn set_uncertain_spring_constant_normal(
        &mut self,
        mean: f64,
        std: f64,
    ) -> Result<(), PrismaticErrors> {
        let dist = Normal::new(mean, std)?;
        self.parameters
            .0
            .spring_constant
            .set_distribution(dist.into())?;
        Ok(())
    }

    /// Sets the joint spring_constant parameter uncertainty with a uniform distribution
    /// For use with Monte Carlo simulations
    /// This does not change the nominal value, see .set_spring_constant()
    pub fn set_uncertain_spring_constant_uniform(
        &mut self,
        low: f64,
        high: f64,
    ) -> Result<(), PrismaticErrors> {
        let dist = Uniform::new(low, high)?;
        self.parameters
            .0
            .spring_constant
            .set_distribution(dist.into())?;
        Ok(())
    }

    /// Sets the joint constant_force parameter uncertainty with a normal distribution
    /// For use with Monte Carlo simulations
    /// This does not change the nominal value, see .set_constant_force()
    pub fn set_uncertain_constant_force_normal(
        &mut self,
        mean: f64,
        std: f64,
    ) -> Result<(), PrismaticErrors> {
        let dist = Normal::new(mean, std)?;
        self.parameters
            .0
            .constant_force
            .set_distribution(dist.into())?;
        Ok(())
    }

    /// Sets the joint constant_force parameter uncertainty with a uniform distribution
    /// For use with Monte Carlo simulations
    /// This does not change the nominal value, see .set_constant_force()
    pub fn set_uncertain_constant_force_uniform(
        &mut self,
        low: f64,
        high: f64,
    ) -> Result<(), PrismaticErrors> {
        let dist = Uniform::new(low, high)?;
        self.parameters
            .0
            .constant_force
            .set_distribution(dist.into())?;
        Ok(())
    }

    /// Builder method to set the initial position state
    pub fn with_position(mut self, position: f64) -> Self {
        self.state.position = SimValue::new(position);
        self
    }

    /// Builder method to set the initial velocity state
    pub fn with_velocity(mut self, velocity: f64) -> Self {
        self.state.velocity = SimValue::new(velocity);
        self
    }

    /// Builder method to set the joint damping parameter
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.parameters.0.damping.nominal = damping;
        self
    }

    /// Builder method to set the joint equilibrium parameter
    pub fn with_equilibrium(mut self, equilibrium: f64) -> Self {
        self.parameters.0.equilibrium.nominal = equilibrium;
        self
    }

    /// Builder method to set the joint spring_constant parameter
    pub fn with_spring_constant(mut self, spring_constant: f64) -> Self {
        self.parameters.0.spring_constant.nominal = spring_constant;
        self
    }

    /// Builder method to set the joint constant_force parameter
    pub fn with_constant_force(mut self, constant_force: f64) -> Self {
        self.parameters.0.constant_force.nominal = constant_force;
        self
    }

    /// Builder method for adding a joint constant_force parameter uncertainty with a normal distribution
    /// For use with Monte Carlo simulations
    /// This does not change the nominal value, see .with_constant_force()
    pub fn with_uncertain_constant_force_normal(
        mut self,
        mean: f64,
        std: f64,
    ) -> Result<Self, PrismaticErrors> {
        let dist = Normal::new(mean, std)?;
        self.parameters
            .0
            .constant_force
            .set_distribution(dist.into())?;
        Ok(self)
    }

    /// Builder method for adding a joint constant_force parameter uncertainty with a uniform distribution
    /// For use with Monte Carlo simulations
    /// This does not change the nominal value, see .with_constant_force()
    pub fn with_uncertain_constant_force_uniform(
        mut self,
        low: f64,
        high: f64,
    ) -> Result<Self, PrismaticErrors> {
        let dist = Uniform::new(low, high)?;
        self.parameters
            .0
            .constant_force
            .set_distribution(dist.into())?;
        Ok(self)
    }

    /// Builder method for adding a joint damping parameter uncertainty with a normal distribution
    /// For use with Monte Carlo simulations
    /// This does not change the nominal value, see .with_damping()
    pub fn with_uncertain_damping_normal(
        mut self,
        mean: f64,
        std: f64,
    ) -> Result<Self, PrismaticErrors> {
        let dist = Normal::new(mean, std)?;
        self.parameters.0.damping.set_distribution(dist.into())?;
        Ok(self)
    }

    /// Builder method for adding a joint damping parameter uncertainty with a uniform distribution
    /// For use with Monte Carlo simulations
    /// This does not change the nominal value, see .with_damping()
    pub fn with_uncertain_damping_uniform(
        mut self,
        low: f64,
        high: f64,
    ) -> Result<Self, PrismaticErrors> {
        let dist = Uniform::new(low, high)?;
        self.parameters.0.damping.set_distribution(dist.into())?;
        Ok(self)
    }

    /// Builder method for adding a joint equilibrium parameter uncertainty with a normal distribution
    /// For use with Monte Carlo simulations
    /// This does not change the nominal value, see .with_equilibrium()
    pub fn with_uncertain_equilibrium_normal(
        mut self,
        mean: f64,
        std: f64,
    ) -> Result<Self, PrismaticErrors> {
        let dist = Normal::new(mean, std)?;
        self.parameters
            .0
            .equilibrium
            .set_distribution(dist.into())?;
        Ok(self)
    }

    /// Builder method for adding a joint equilibrium parameter uncertainty with a uniform distribution
    /// For use with Monte Carlo simulations
    /// This does not change the nominal value, see .with_equilibrium()
    pub fn with_uncertain_equilibrium_uniform(
        mut self,
        low: f64,
        high: f64,
    ) -> Result<Self, PrismaticErrors> {
        let dist = Uniform::new(low, high)?;
        self.parameters
            .0
            .equilibrium
            .set_distribution(dist.into())?;
        Ok(self)
    }

    /// Builder method for adding a joint spring_constant parameter uncertainty with a normal distribution
    /// For use with Monte Carlo simulations
    /// This does not change the nominal value, see .with_spring_constant()
    pub fn with_uncertain_spring_constant_normal(
        mut self,
        mean: f64,
        std: f64,
    ) -> Result<Self, PrismaticErrors> {
        let dist = Normal::new(mean, std)?;
        self.parameters
            .0
            .spring_constant
            .set_distribution(dist.into())?;
        Ok(self)
    }

    /// Builder method for adding a joint spring_constant parameter uncertainty with a uniform distribution
    /// For use with Monte Carlo simulations
    /// This does not change the nominal value, see .with_spring_constant()
    pub fn with_uncertain_spring_constant_uniform(
        mut self,
        low: f64,
        high: f64,
    ) -> Result<Self, PrismaticErrors> {
        let dist = Uniform::new(low, high)?;
        self.parameters
            .0
            .spring_constant
            .set_distribution(dist.into())?;
        Ok(self)
    }
}

impl Uncertainty for PrismaticBuilder {
    type Output = Prismatic;
    type Error = JointErrors;

    /// Creates a Prismatic joint from the PrismaticBuilder
    /// Samples the parameters and state uncertainty if present
    fn sample(&self, nominal: bool, rng: &mut SmallRng) -> Result<Self::Output, Self::Error> {
        Ok(Prismatic::new(
            self.parameters.sample(nominal, rng)?,
            self.state.sample(nominal, rng)?,
        ))
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Prismatic {
    pub parameters: PrismaticParameters,
    pub state: PrismaticState,
    #[serde(skip)]
    cache: PrismaticCache,
}

impl Prismatic {
    fn new(parameters: PrismaticParameters, state: PrismaticState) -> Self {
        Self {
            parameters,
            state,
            cache: PrismaticCache::default(),
        }
    }
}

impl JointModel for Prismatic {
    fn calculate_joint_inertia(
        &mut self,
        mass_properties: &MassProperties,
        transforms: &JointTransforms,
    ) -> SpatialInertia {
        transforms.jof_from_ob * SpatialInertia::from(mass_properties)
    }

    fn calculate_tau(&mut self) {
        let JointParameters {
            constant_force,
            damping,
            equilibrium,
            spring_constant,
            ..
        } = self.parameters.0;
        self.cache.tau = constant_force
            - spring_constant * (equilibrium - self.state.position)
            - damping * self.state.velocity;
    }

    fn calculate_vj(&self, _transforms: &JointTransforms) -> Velocity {
        Velocity::from(Vector6::new(0.0, 0.0, 0.0, self.state.velocity, 0.0, 0.0))
    }

    fn ndof(&self) -> u32 {
        1
    }

    fn state_derivative(&self, derivative: &mut SimStateVector, _transforms: &JointTransforms) {
        derivative.0[0] = self.state.velocity;
        derivative.0[1] = self.cache.q_ddot;
    }

    fn state_vector_init(&self) -> SimStateVector {
        SimStateVector(vec![self.state.position, self.state.velocity])
    }

    fn state_vector_read(&mut self, state: &SimStateVector) {
        self.state.position = state.0[0];
        self.state.velocity = state.0[1];
    }

    fn update_transforms(
        &mut self,
        transforms: &mut JointTransforms,
        inner_joint: &Option<JointRef>,
    ) {
        let rotation = Rotation::identity();
        let translation = CoordinateSystem::from(Cartesian::new(self.state.position, 0.0, 0.0));
        let transform = Transform::new(rotation, translation);

        transforms.jof_from_jif = SpatialTransform(transform);
        transforms.jif_from_jof = transforms.jof_from_jif.inv();
        transforms.update(inner_joint)
    }

    fn result_headers(&self) -> &[&str] {
        &["position", "velocity", "acceleration", "tau"]
    }

    fn result_content(&self, id: u32, results: &mut ResultManager) {
        results.write_record(
            id,
            &[
                self.state.position.to_string(),
                self.state.velocity.to_string(),
                self.cache.q_ddot.to_string(),
                self.cache.tau.to_string(),
            ],
        );
    }
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
struct PrismaticAbaCache {
    lil_u: f64,
    big_d_inv: f64,
    big_u: Matrix6x1<f64>,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
struct PrismaticCrbCache {
    cache_index: usize,
    ic: SpatialInertia,
}

#[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
struct PrismaticCache {
    aba: PrismaticAbaCache,
    crb: PrismaticCrbCache,
    q_ddot: f64,
    rne: RneCache,
    tau: f64,
}

impl ArticulatedBodyAlgorithm for Prismatic {
    fn aba_second_pass(&mut self, joint_cache: &mut JointCache, inner_joint: &Option<JointRef>) {
        let aba = &mut self.cache.aba;
        let inertia_articulated_matrix = joint_cache.aba.inertia_articulated.matrix();

        // use the most efficient method for creating these. Indexing is much faster than 6x6 matrix mul
        aba.big_u = inertia_articulated_matrix.column(3).into();
        aba.big_d_inv = 1.0 / aba.big_u[3];
        aba.lil_u = self.cache.tau - (joint_cache.aba.p_big_a.get_index(4).unwrap()); //note force is 1 indexed, so

        if let Some(inner_joint) = inner_joint {
            let mut inner_joint = inner_joint.borrow_mut();
            let big_u_times_big_d_inv = aba.big_u * aba.big_d_inv;
            let i_lil_a = SpatialInertia(
                inertia_articulated_matrix - big_u_times_big_d_inv * aba.big_u.transpose(),
            );

            joint_cache.aba.p_lil_a = joint_cache.aba.p_big_a
                + Force::from(i_lil_a * joint_cache.aba.c)
                + Force::from(big_u_times_big_d_inv * aba.lil_u);

            inner_joint.cache.aba.inertia_articulated +=
                joint_cache.transforms.ij_jof_from_jof * i_lil_a;
            inner_joint.cache.aba.p_big_a +=
                joint_cache.transforms.ij_jof_from_jof * joint_cache.aba.p_lil_a;
        }
    }

    fn aba_third_pass(&mut self, joint_cache: &mut JointCache, inner_joint: &Option<JointRef>) {
        let a_ij = if let Some(inner_joint) = inner_joint {
            inner_joint.borrow().cache.a
        } else {
            Acceleration::zeros()
        };
        let a_prime = joint_cache.transforms.jof_from_ij_jof * a_ij + joint_cache.aba.c;
        self.cache.q_ddot = self.cache.aba.big_d_inv
            * (self.cache.aba.lil_u - (self.cache.aba.big_u.transpose() * a_prime.vector())[0]); // indexing just pull value out of 1x1 vector
        joint_cache.a =
            a_prime + Acceleration::from(Vector6::new(0.0, 0.0, 0.0, self.cache.q_ddot, 0.0, 0.0));
    }
}

// impl RecursiveNewtonEuler for Prismatic {
//     fn rne_first_pass(&mut self, a_ij: Acceleration, v_ij: Velocity, use_qddot: bool) {
//         let a = &mut self.cache.common.a;
//         let f = &mut self.cache.rne.as_mut().unwrap().f;
//         let q_ddot = &mut self.cache.q_ddot;
//         let v = &mut self.cache.common.v;
//         let vj = &self.cache.common.vj;
//         let f_b = &self.cache.common.f;

//         let jof_from_ij_jof = &self.transforms.jof_from_ij_jof;
//         let joint_inertia = &self.mass_properties.unwrap();

//         *v = *jof_from_ij_jof * v_ij + *vj;

//         let a_new = match use_qddot {
//             true => {
//                 *jof_from_ij_jof * a_ij
//                     + Acceleration::from(Vector6::new(0.0, 0.0, 0.0, *q_ddot, 0.0, 0.0))
//                     + v.cross_motion(*vj)
//             }
//             false => *jof_from_ij_jof * a_ij + v.cross_motion(*vj),
//         };

//         *a = a_new;

//         *f = *joint_inertia * *a + v.cross_force(*joint_inertia * *v) - *f_b;
//     }

//     fn rne_second_pass(&mut self) {
//         self.cache.tau = self.cache.rne.unwrap().f.translation()[0];
//     }

//     fn rne_add_force(&mut self, force: Force) {
//         self.cache.rne.as_mut().unwrap().f = self.cache.rne.as_mut().unwrap().f + force;
//     }

//     fn rne_get_force(&self) -> Force {
//         self.cache.rne.unwrap().f
//     }

//     fn rne_set_tau(&mut self) {
//         self.cache.tau = self.cache.rne.unwrap().f.vector()[3];
//     }
// }

// impl CompositeRigidBody for Prismatic {
//     fn add_ic(&mut self, new_ic: SpatialInertia) {
//         let ic = &mut self.cache.crb.as_mut().unwrap().ic;
//         *ic += new_ic;
//     }

//     fn get_crb_index(&self) -> usize {
//         self.cache.crb.unwrap().cache_index
//     }

//     fn get_ic(&self) -> SpatialInertia {
//         self.cache.crb.unwrap().ic
//     }

//     fn reset_ic(&mut self) {
//         let ic = &mut self.cache.crb.as_mut().unwrap().ic;
//         *ic = self.mass_properties.unwrap();
//     }

//     fn set_crb_index(&mut self, n: usize) {
//         if let Some(crb) = &mut self.cache.crb {
//             crb.cache_index = n;
//         }
//     }

//     fn set_c(&self, c: &mut DVector<f64>) {
//         let index = self.cache.crb.unwrap().cache_index;
//         let tau = Vector1::new(self.cache.tau);
//         c.set_row(index, &tau);
//     }

//     fn set_h(&self, h: &mut DMatrix<f64>) {
//         let crb = self.cache.crb.unwrap();
//         let index = crb.cache_index;
//         let ic = &crb.ic;

//         let mut view = h.fixed_view_mut::<1, 1>(index, index);
//         view[0] = ic.matrix()[(3, 3)];
//     }
// }
