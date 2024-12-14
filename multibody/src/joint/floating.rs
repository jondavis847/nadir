use crate::{
    algorithms::{
        articulated_body_algorithm::{AbaCache, ArticulatedBodyAlgorithm},
        recursive_newton_euler::RneCache,
    },
    joint::{joint_transforms::JointTransforms, JointParameters},
    result::{MultibodyResultTrait, ResultEntry},
};
use aerospace::orbit::Orbit;
use coordinate_systems::{cartesian::Cartesian, CoordinateSystem};
use mass_properties::{CenterOfMass, MassProperties};
use nalgebra::{Matrix3, Matrix4x3, Matrix6, Vector3, Vector6};
use rotations::{
    euler_angles::EulerAngles, prelude::RotationMatrix, quaternion::Quaternion, Rotation,
    RotationTrait,
};
use serde::{Deserialize, Serialize};
use spatial_algebra::{Acceleration, Force, SpatialInertia, SpatialTransform, Velocity};
use std::ops::{AddAssign, MulAssign};
use std::{collections::HashMap, mem::take};
use transforms::Transform;

use super::{JointCache, JointModel, JointRef, JointStateVector};

#[derive(Debug, Copy, Clone)]
pub enum FloatingErrors {}

/// IMPORTANT: State values are in the JOF
/// This just makes sense for applying angular quantities
/// translation frame is tied to rotation frame due to the use of spatial vectors (since they both live in the same vector)
/// i.e. r (position) is the JIFs position in the JOF!
#[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
pub struct FloatingState {
    pub q: Quaternion,
    pub w: Vector3<f64>,
    pub r: Vector3<f64>,
    pub v: Vector3<f64>,
}

impl FloatingState {
    pub fn new() -> Self {
        Self {
            q: Quaternion::IDENTITY,
            w: Vector3::zeros(),
            r: Vector3::zeros(),
            v: Vector3::zeros(),
        }
    }

    pub fn with_attitude(mut self, q: Quaternion) -> Self {
        self.q = q;
        self
    }

    pub fn with_rates(mut self, w: Vector3<f64>) -> Self {
        self.w = w;
        self
    }

    pub fn with_position(mut self, r: Vector3<f64>) -> Self {
        self.r = r;
        self
    }

    pub fn with_velocity(mut self, v: Vector3<f64>) -> Self {
        self.v = v;
        self
    }

    pub fn with_orbit(mut self, orbit: Orbit) -> Self {
        let (r, v) = match orbit {
            Orbit::Keplerian(kep) => kep.get_rv(),
        };
        self.r = r;
        self.v = v;
        self
    }
}

impl<'a> AddAssign<&'a Self> for FloatingState {
    fn add_assign(&mut self, rhs: &'a Self) {
        self.q += &rhs.q; //note this should only be used for adding quaternion derivatives in an ODE
        self.w += &rhs.w;
        self.r += &rhs.r;
        self.v += &rhs.v;
    }
}

impl MulAssign<f64> for FloatingState {
    fn mul_assign(&mut self, rhs: f64) {
        self.q *= rhs;
        self.w *= rhs;
        self.r *= rhs;
        self.v *= rhs;
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FloatingParameters {
    xr: JointParameters,
    yr: JointParameters,
    zr: JointParameters,
    xt: JointParameters,
    yt: JointParameters,
    zt: JointParameters,
}

impl FloatingParameters {
    pub fn new() -> Self {
        Self {
            xr: JointParameters::default(),
            yr: JointParameters::default(),
            zr: JointParameters::default(),
            xt: JointParameters::default(),
            yt: JointParameters::default(),
            zt: JointParameters::default(),
        }
    }

    /// Adds JointParameters for rotation in the x, y, and z axis.
    /// Sets the parameters for all axes!
    /// Use with_xr for example if you only want to set x.
    pub fn with_rotation(mut self, p: JointParameters) -> Self {
        self.xr = p;
        self.yr = p;
        self.zr = p;
        self
    }

    /// Adds JointParameters for translation in the x, y, and z axis.
    /// Sets the parameters for all axes!
    /// Use with_xr for example if you only want to set x.
    pub fn with_translation(mut self, p: JointParameters) -> Self {
        self.xt = p;
        self.yt = p;
        self.zt = p;
        self
    }

    /// Adds JointParameters for rotation in the x axis
    pub fn with_xr(mut self, p: JointParameters) -> Self {
        self.xr = p;
        self
    }

    /// Adds JointParameters for rotation in the y axis
    pub fn with_yr(mut self, p: JointParameters) -> Self {
        self.yr = p;
        self
    }

    /// Adds JointParameters for rotation in the z axis
    pub fn with_zr(mut self, p: JointParameters) -> Self {
        self.zr = p;
        self
    }

    /// Adds JointParameters for translation in the x axis
    pub fn with_xt(mut self, p: JointParameters) -> Self {
        self.xt = p;
        self
    }

    /// Adds JointParameters for translation in the y axis
    pub fn with_yt(mut self, p: JointParameters) -> Self {
        self.yt = p;
        self
    }

    /// Adds JointParameters for translation in the z axis
    pub fn with_zt(mut self, p: JointParameters) -> Self {
        self.zt = p;
        self
    }
}

#[derive(Debug, Default, Clone)]
struct FloatingResult {
    qx: Vec<f64>,
    qy: Vec<f64>,
    qz: Vec<f64>,
    qw: Vec<f64>,
    wx: Vec<f64>,
    wy: Vec<f64>,
    aax: Vec<f64>,
    aay: Vec<f64>,
    aaz: Vec<f64>,
    wz: Vec<f64>,
    rx: Vec<f64>,
    ry: Vec<f64>,
    rz: Vec<f64>,
    vx: Vec<f64>,
    vy: Vec<f64>,
    vz: Vec<f64>,
    ax: Vec<f64>,
    ay: Vec<f64>,
    az: Vec<f64>,
    tau_xr: Vec<f64>,
    tau_yr: Vec<f64>,
    tau_zr: Vec<f64>,
    tau_xt: Vec<f64>,
    tau_yt: Vec<f64>,
    tau_zt: Vec<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Floating {
    pub parameters: FloatingParameters,
    pub state: FloatingState,
    #[serde(skip)]
    cache: FloatingCache,
    #[serde(skip)]
    result: FloatingResult,
}

impl Floating {
    pub fn new(parameters: FloatingParameters, state: FloatingState) -> Self {
        Self {
            parameters,
            state,
            cache: FloatingCache::default(),
            result: FloatingResult::default(),
        }
    }
}

#[typetag::serde]
impl JointModel for Floating {
    fn calculate_joint_inertia(
        &mut self,
        inertia: &MassProperties,
        transforms: &JointTransforms,
    ) -> SpatialInertia {
        let jof_from_ob = transforms.jof_from_ob;
        // IMPORTANT: floating joint must assume that jof is at cm
        // otherwise you will have a moment arm and body will torque with linear force at cm
        // jof_from_ob has already made this correction, but so do mass props
        let original_cm = inertia.center_of_mass.vector();
        let mut inertia = inertia.clone();
        inertia.center_of_mass = CenterOfMass::new(0.0, 0.0, 0.0);
        let spatial_inertia = SpatialInertia::from(inertia);

        // only rotate the inertia to the jof, dont translate since cm is @ jof
        let mut jof_from_ob_rotation_only = jof_from_ob.clone();
        jof_from_ob_rotation_only.0.translation = CoordinateSystem::ZERO;
        let joint_mass_properties = jof_from_ob_rotation_only * spatial_inertia;

        // if there is translation in jof_from_ob or the body frame cm is non zero
        // then we need to add them to the joint state position so that the jof frame is at the cm
        // r is in the jif frame, so need to transform jof and cm to jif frame
        let jif_from_jof = transforms.jif_from_jof;
        let cm_in_jof = jof_from_ob.0.rotation.transform(original_cm);
        let ob_from_jof_translation = Cartesian::from(transforms.ob_from_jof.0.translation).vec();
        let total_in_jof = cm_in_jof + ob_from_jof_translation;
        let total_in_jif = jif_from_jof.0.rotation.transform(total_in_jof);
        let r = &mut self.state.r;
        *r += total_in_jif;

        joint_mass_properties
    }

    fn calculate_tau(&mut self) {
        let p = &self.parameters;
        // this assume tait-bryan euler angle sequence (ZYX)
        let angles = EulerAngles::from(&self.state.q);

        self.cache.tau[0] = p.xr.constant_force
            + p.xr.spring_constant * (p.xr.equilibrium - angles.psi)
            - p.xr.damping * self.state.w[0];
        self.cache.tau[1] = p.yr.constant_force
            + p.yr.spring_constant * (p.yr.equilibrium - angles.theta)
            - p.yr.damping * self.state.w[1];
        self.cache.tau[2] = p.zr.constant_force
            + p.zr.spring_constant * (p.zr.equilibrium - angles.phi)
            - p.zr.damping * self.state.w[2];

        // translation quantities are + instead of - since they are expressed in JOF
        // i.e. if the JOF is +1 units in the x direction represented in the JIF frame,
        // then r would be -1 in the JOF frame, and the spring force would be K*r instead of -K*r
        // to get back to a JIF equilibrium
        self.cache.tau[3] = p.xt.constant_force
            + p.xt.spring_constant * (p.xt.equilibrium - self.state.r[0])
            - p.xt.damping * self.state.v[0];
        self.cache.tau[4] = p.yt.constant_force
            + p.yt.spring_constant * (p.yt.equilibrium - self.state.r[1])
            - p.yt.damping * self.state.v[1];
        self.cache.tau[5] = p.zt.constant_force
            + p.zt.spring_constant * (p.zt.equilibrium - self.state.r[2])
            - p.zt.damping * self.state.v[2];
    }

    // !!!!note!!!! this must convert state.v, which is in the jif frome, to vj, which is in the jof frame
    fn calculate_vj(&self, transforms: &JointTransforms) -> Velocity {
        let v_jif = self.state.v;        
        let v_jof = transforms.jof_from_jif.0.rotation.transform(v_jif);        
        Velocity::from(Vector6::new(
            self.state.w[0],
            self.state.w[1],
            self.state.w[2],
            self.state.v[0],
            self.state.v[1],
            self.state.v[2],            
        ))
    }

    fn ndof(&self) -> u32 {
        6
    }

    fn state_derivative(&self, dx: &mut JointStateVector, transforms: &JointTransforms) {
        // Quaternion is from the body to base, or the body's orientation in the base frame
        //due to quaternion kinematic equations
        let q = self.state.q;
        // Markley eq 3.20 & 2.88
        let tmp = Matrix4x3::new(
            q.s, -q.z, q.y, q.z, q.s, -q.x, -q.y, q.x, q.s, -q.x, -q.y, -q.z,
        );
        let dq = Quaternion::from(0.5 * tmp * self.state.w);
        // for floating joint, need to transform v and a to jif frame so it makes sense for position translation integration
        // when calculating vj, we retransform state.v back to jof frame,
        // a doesnt need to be be retransformed because it's reset and recalcualted from scratch every step,
        // but vj affects other child joints and needs to accurately represent the jofs velocity

        // we need to account for the rotating frame when converting from the jof to the jif
        // for example, if the jof is translating with 1m/s in y and rotating about x by 1 rad/s
        // the velocity in the body frame on the next cycle will have some component of velocity
        // in the -z axis due to the rotation, which means there must have been some acceleration in z
        // we will see that acceleration in the jof, but jif accel should still b 0
        // v_jof = R * v_jif where R = jof_from_jif
        // a_jof = dv_jof/dt = R * a_jof + dR/dt * v_jif where dR/dt = R * wx (w skew symetric matrix)
        

        // transform self.state.v (v_jof) to the jif for integrating r via transport theorem
        // this is techinically v_jif = R * v_jof + w x r, but r is 0 since jof frame is coincident with itself for floating joint
        let v_jif = transforms.jif_from_jof.0.rotation.transform(self.state.v);

        dx.0[0] = dq.x;
        dx.0[1] = dq.y;
        dx.0[2] = dq.z;
        dx.0[3] = dq.s;
        dx.0[4] = v_jif[0];
        dx.0[5] = v_jif[1];
        dx.0[6] = v_jif[2];
        dx.0[7] = self.cache.q_ddot[0];
        dx.0[8] = self.cache.q_ddot[1];
        dx.0[9] = self.cache.q_ddot[2];
        dx.0[10] = self.cache.q_ddot[3];
        dx.0[11] = self.cache.q_ddot[4];
        dx.0[12] = self.cache.q_ddot[5];
    }

    fn state_vector_init(&self) -> JointStateVector {
        let state = vec![
            self.state.q.x,
            self.state.q.y,
            self.state.q.z,
            self.state.q.s,
            self.state.r[0],
            self.state.r[1],
            self.state.r[2],
            self.state.w[0],
            self.state.w[1],
            self.state.w[2],
            self.state.v[0],
            self.state.v[1],
            self.state.v[2],
        ];
        JointStateVector(state)
    }

    fn state_vector_read(&mut self, state: &JointStateVector) {
        // need to normalize the integrated quaternion
        let q = Quaternion::new(state.0[0], state.0[1], state.0[2], state.0[3]).normalize();
        self.state.q = q;
        self.state.r[0] = state.0[4];
        self.state.r[1] = state.0[5];
        self.state.r[2] = state.0[6];
        self.state.w[0] = state.0[7];
        self.state.w[1] = state.0[8];
        self.state.w[2] = state.0[9];
        self.state.v[0] = state.0[10];
        self.state.v[1] = state.0[11];
        self.state.v[2] = state.0[12];
    }

    fn update_transforms(
        &mut self,
        transforms: &mut JointTransforms,
        inner_joint: &Option<JointRef>,
    ) {
        let rotation = Rotation::from(&self.state.q);
        let translation = Cartesian::from(self.state.r); // r is already jif to jof
        let transform = Transform::new(rotation, translation.into());

        transforms.jof_from_jif = SpatialTransform(transform);
        transforms.jif_from_jof = transforms.jof_from_jif.inv();
        transforms.update(inner_joint);
    }
}

impl MultibodyResultTrait for Floating {
    fn get_result_entry(&mut self) -> ResultEntry {
        let mut result = HashMap::new();
        result.insert("attitude[x]".to_string(), take(&mut self.result.qx));
        result.insert("attitude[y]".to_string(), take(&mut self.result.qy));
        result.insert("attitude[z]".to_string(), take(&mut self.result.qz));
        result.insert("attitude[w]".to_string(), take(&mut self.result.qw));
        result.insert("angular_rate[x]".to_string(), take(&mut self.result.wx));
        result.insert("angular_rate[y]".to_string(), take(&mut self.result.wy));
        result.insert("angular_rate[z]".to_string(), take(&mut self.result.wz));
        result.insert("angular_accel[x]".to_string(), take(&mut self.result.aax));
        result.insert("angular_accel[y]".to_string(), take(&mut self.result.aay));
        result.insert("angular_accel[z]".to_string(), take(&mut self.result.aaz));
        result.insert("position[x]".to_string(), take(&mut self.result.rx));
        result.insert("position[y]".to_string(), take(&mut self.result.ry));
        result.insert("position[z]".to_string(), take(&mut self.result.rz));
        result.insert("velocity[x]".to_string(), take(&mut self.result.vx));
        result.insert("velocity[y]".to_string(), take(&mut self.result.vy));
        result.insert("velocity[z]".to_string(), take(&mut self.result.vz));
        result.insert("acceleration[x]".to_string(), take(&mut self.result.ax));
        result.insert("acceleration[y]".to_string(), take(&mut self.result.ay));
        result.insert("acceleration[z]".to_string(), take(&mut self.result.az));
        result.insert(
            "tau_translation[x]".to_string(),
            take(&mut self.result.tau_xt),
        );
        result.insert(
            "tau_translation[y]".to_string(),
            take(&mut self.result.tau_yt),
        );
        result.insert(
            "tau_translation[z]".to_string(),
            take(&mut self.result.tau_zt),
        );
        result.insert("tau_rotation[x]".to_string(), take(&mut self.result.tau_xr));
        result.insert("tau_rotation[y]".to_string(), take(&mut self.result.tau_yr));
        result.insert("tau_rotation[z]".to_string(), take(&mut self.result.tau_zr));
        ResultEntry::new(result)
    }

    fn initialize_result(&mut self, capacity: usize) {
        self.result.qx = Vec::with_capacity(capacity);
        self.result.qy = Vec::with_capacity(capacity);
        self.result.qz = Vec::with_capacity(capacity);
        self.result.qw = Vec::with_capacity(capacity);
        self.result.wx = Vec::with_capacity(capacity);
        self.result.wy = Vec::with_capacity(capacity);
        self.result.aax = Vec::with_capacity(capacity);
        self.result.aay = Vec::with_capacity(capacity);
        self.result.aaz = Vec::with_capacity(capacity);
        self.result.wz = Vec::with_capacity(capacity);
        self.result.rx = Vec::with_capacity(capacity);
        self.result.ry = Vec::with_capacity(capacity);
        self.result.rz = Vec::with_capacity(capacity);
        self.result.vx = Vec::with_capacity(capacity);
        self.result.vy = Vec::with_capacity(capacity);
        self.result.vz = Vec::with_capacity(capacity);
        self.result.ax = Vec::with_capacity(capacity);
        self.result.ay = Vec::with_capacity(capacity);
        self.result.az = Vec::with_capacity(capacity);
        self.result.tau_xr = Vec::with_capacity(capacity);
        self.result.tau_yr = Vec::with_capacity(capacity);
        self.result.tau_zr = Vec::with_capacity(capacity);
        self.result.tau_xt = Vec::with_capacity(capacity);
        self.result.tau_yt = Vec::with_capacity(capacity);
        self.result.tau_zt = Vec::with_capacity(capacity);
    }

    fn update_result(&mut self) {
        self.result.qx.push(self.state.q.x);
        self.result.qy.push(self.state.q.y);
        self.result.qz.push(self.state.q.z);
        self.result.qw.push(self.state.q.s);
        self.result.wx.push(self.state.w[0]);
        self.result.wy.push(self.state.w[1]);
        self.result.wz.push(self.state.w[2]);
        self.result.aax.push(self.cache.q_ddot[0]);
        self.result.aay.push(self.cache.q_ddot[1]);
        self.result.aaz.push(self.cache.q_ddot[2]);
        self.result.rx.push(self.state.r[0]);
        self.result.ry.push(self.state.r[1]);
        self.result.rz.push(self.state.r[2]);
        self.result.vx.push(self.state.v[0]);
        self.result.vy.push(self.state.v[1]);
        self.result.vz.push(self.state.v[2]);
        self.result.ax.push(self.cache.q_ddot[3]);
        self.result.ay.push(self.cache.q_ddot[4]);
        self.result.az.push(self.cache.q_ddot[5]);
        self.result.tau_xr.push(self.cache.tau[0]);
        self.result.tau_yr.push(self.cache.tau[1]);
        self.result.tau_zr.push(self.cache.tau[2]);
        self.result.tau_xt.push(self.cache.tau[3]);
        self.result.tau_yt.push(self.cache.tau[4]);
        self.result.tau_zt.push(self.cache.tau[5]);
    }
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
struct FloatingAbaCache {
    common: AbaCache,
    lil_u: Vector6<f64>,
    big_d_inv: Matrix6<f64>,
    big_u: Matrix6<f64>,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
struct FloatingCrbCache {
    cache_index: usize,
    ic: SpatialInertia,
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
struct FloatingCache {
    aba: FloatingAbaCache,
    crb: FloatingCrbCache,
    q_ddot: Vector6<f64>,
    rne: RneCache,
    tau: Vector6<f64>,
}

impl ArticulatedBodyAlgorithm for Floating {
    fn aba_second_pass(&mut self, joint_cache: &mut JointCache, inner_joint: &Option<JointRef>) {
        let aba = &mut self.cache.aba;
        let inertia_articulated_matrix = joint_cache.aba.inertia_articulated.matrix();

        // use the most efficient method for creating these. Indexing is much faster than 6x6 matrix mul
        aba.big_u = inertia_articulated_matrix; // S is just identity for floating joint
        aba.big_d_inv = aba.big_u.try_inverse().unwrap();
        aba.lil_u = self.cache.tau - joint_cache.aba.p_big_a.vector();

        if let Some(inner_joint_ref) = inner_joint {
            let big_u_times_big_d_inv = aba.big_u * aba.big_d_inv;
            let i_lil_a = SpatialInertia(
                inertia_articulated_matrix - big_u_times_big_d_inv * aba.big_u.transpose(),
            );

            joint_cache.aba.p_lil_a = joint_cache.aba.p_big_a
                + Force::from(i_lil_a * joint_cache.aba.c)
                + Force::from(big_u_times_big_d_inv * aba.lil_u);

            let mut inner_joint = inner_joint_ref.borrow_mut();
            inner_joint.cache.aba.inertia_articulated +=
                joint_cache.transforms.ij_jof_from_jof * i_lil_a;
            inner_joint.cache.aba.p_big_a +=
                joint_cache.transforms.ij_jof_from_jof * joint_cache.aba.p_lil_a;
        }
    }

    fn aba_third_pass(&mut self, joint_cache: &mut JointCache, inner_joint: &Option<JointRef>) {
        let a_ij = if let Some(inner_joint_ref) = inner_joint {
            inner_joint_ref.borrow().cache.a
        } else {
            Acceleration::zeros()
        };
        let a_prime = joint_cache.transforms.jof_from_ij_jof * a_ij + joint_cache.aba.c;
        self.cache.q_ddot = self.cache.aba.big_d_inv
            * (self.cache.aba.lil_u - self.cache.aba.big_u.transpose() * a_prime.vector());
        joint_cache.a = a_prime + Acceleration::from(self.cache.q_ddot);
    }
}

// impl RecursiveNewtonEuler for Floating {
//     fn rne_first_pass(&mut self, a_ij: Acceleration, v_ij: Velocity, use_qddot: bool) {
//         let a = &mut self.cache.common.a;
//         let v = &mut self.cache.common.v;
//         let vj = &mut self.cache.common.vj;
//         let q_ddot = &mut self.cache.q_ddot;
//         let f = &mut self.cache.rne.as_mut().unwrap().f;
//         let f_b = &mut self.cache.common.f;

//         let jof_from_ij_jof = &self.transforms.jof_from_ij_jof;
//         let joint_inertia = &self.mass_properties.unwrap();

//         *v = *jof_from_ij_jof * v_ij + *vj;

//         let a_new = match use_qddot {
//             true => *jof_from_ij_jof * a_ij + Acceleration::from(*q_ddot) + v.cross_motion(*vj),
//             false => *jof_from_ij_jof * a_ij + v.cross_motion(*vj),
//         };

//         *a = a_new;

//         *f = *joint_inertia * *a + v.cross_force(*joint_inertia * *v) - *f_b;
//     }

//     fn rne_second_pass(&mut self) {
//         self.cache.tau = self.cache.rne.unwrap().f.vector();
//     }

//     fn rne_add_force(&mut self, force: Force) {
//         self.cache.rne.as_mut().unwrap().f = self.cache.rne.as_mut().unwrap().f + force;
//     }

//     fn rne_get_force(&self) -> Force {
//         self.cache.rne.unwrap().f
//     }

//     fn rne_set_tau(&mut self) {
//         self.cache.tau = self.cache.rne.unwrap().f.vector(); //TODO: duplicate of second pass?
//     }
// }

// impl CompositeRigidBody for Floating {
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
//         let mut my_space = c.fixed_rows_mut::<6>(index);
//         my_space.copy_from(&self.cache.tau);
//     }

//     fn set_h(&self, h: &mut DMatrix<f64>) {
//         let crb = &self.cache.crb.unwrap();
//         let index = crb.cache_index;
//         let ic = &crb.ic;

//         let mut view = h.fixed_view_mut::<1, 1>(index, index);
//         view[0] = ic.matrix()[0];
//     }
// }
