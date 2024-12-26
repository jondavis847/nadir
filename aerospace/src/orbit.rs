use nalgebra::Vector3;
use rotations::{
    prelude::{EulerAngles, EulerSequence},
    Rotation, RotationTrait,
};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;
use thiserror::Error;
use time::{Time, TimeSystem};

use crate::celestial_system::CelestialBodies;

/// Tolerance for determining if orbit is elliptical/circular/parabolic/hyperbolic
const ORBIT_EPSILON: f64 = 1e-8;

#[derive(Error, Debug)]
pub enum OrbitErrors {
    #[error("reached max iterations while solving kepler's equation")]
    KeplerMaxIters,
}

#[derive(Clone,Debug,Serialize,Deserialize)]
pub enum OrbitType {
    Circular,
    Elliptical,
    Parabolic,
    Hyperbolic,
}

pub enum Orbit {
    //Defauney(DelauneyElements), TODO?
    //Equinoctal(EquinoctalElements), TODO
    Keplerian(KeplerianElements),
    //Poincare(PoincareElements), TODO?
}

impl From<KeplerianElements> for Orbit {
    fn from(value: KeplerianElements) -> Self {
        Orbit::Keplerian(value)
    }
}
/// Represents the orbital elements of a Keplerian orbit.
///
/// This struct contains the orbital parameters necessary to describe an object's orbit
/// around a central body, based on the Keplerian model. It includes elements such as the
/// semi-major axis, eccentricity, inclination, RAAN, argument of periapsis, and anomalies.
#[derive(Clone,Debug,Serialize,Deserialize)]
pub struct KeplerianElements {
    pub epoch: Time,
    /// Gravitational parameter of the central body (in m^3/s^2).
    pub mu: f64,
    /// Radius of the central body (in m).
    pub radius: f64,
    /// Semi-major axis of the orbit (in m).
    pub semimajor_axis: f64,
    /// Semi-parameter (semi-latus rectum) of the orbit (in m).
    pub semiparameter: f64,
    /// Argument of periapsis (in radians).
    pub argument_of_periapsis: f64,
    /// Eccentricity of the orbit (dimensionless).
    pub eccentricity: f64,
    /// Inclination of the orbit (in radians).
    pub inclination: f64,
    /// Right Ascension of the Ascending Node (RAAN) (in radians).
    pub raan: f64,
    /// True anomaly (in radians).
    pub true_anomaly: f64,
    /// Type of orbit
    pub orbit_type: OrbitType,
}

impl KeplerianElements {
    /// Creates a `KeplerianOrbit` instance from position and velocity vectors.
    ///
    /// # Arguments    
    /// * `r` - Position vector in Cartesian coordinates (in m).
    /// * `v` - Velocity vector in Cartesian coordinates (in m/s).
    /// * `epoch` - Epoch of the orbit (see the Time crate)
    ///
    /// # Returns
    /// A new `KeplerianOrbit` instance representing the orbit based on the provided position and velocity vectors.
    pub fn from_rv(
        r: Vector3<f64>,
        v: Vector3<f64>,
        epoch: Time,
        central_body: CelestialBodies,
    ) -> Self {
        // calculate canonical units
        let mu = central_body.get_mu();
        let radius = central_body.get_radius();

        let du = radius;
        let tu = (du.powi(3) / mu).sqrt();
        // convert to canonical units
        let r = r / du;
        let v = v * tu / du;
        let rm = r.norm();
        let vm = v.norm();
        let rdotv = r.dot(&v);

        let h = r.cross(&v);
        let hm = h.norm();

        let k = Vector3::new(0.0, 0.0, 1.0);
        let n = k.cross(&h);
        let nm = n.norm();

        let e_vector = (vm.powi(2) - 1.0 / rm) * r - rdotv * v;
        let e = e_vector.norm();

        let zeta = vm.powi(2) / 2.0 - 1.0 / rm;

        // semimajor axis and semilatus rectum (semiparameter)
        let (a, p) = if (e - 1.0).abs() > ORBIT_EPSILON {
            let a = -du / (2.0 * zeta);
            let p = a * (1.0 - e.powi(2));
            (a, p)
        } else {
            (f64::INFINITY, du * hm.powi(2))
        };

        // inclination
        let i = (h[2] / hm).acos();

        // handles special cases
        // raan is not defined for equatorial
        // argp is not defined for circular
        let is_elliptical_equatorial = e > ORBIT_EPSILON && i < ORBIT_EPSILON;
        let is_circular_inclined = e < ORBIT_EPSILON && i > ORBIT_EPSILON;
        let is_circular_equatorial = e < ORBIT_EPSILON && i < ORBIT_EPSILON;

        let i_hat = Vector3::new(1.0, 0.0, 0.0);
        let (raan, argp, f) = if is_circular_equatorial {
            // lambda true is the true longitude
            let lambda_true = if r[1] < 0.0 {
                2.0 * PI - (i_hat.dot(&r) / rm).acos()
            } else {
                (i_hat.dot(&r) / rm).acos()
            };
            (0.0, 0.0, lambda_true)
        } else if is_circular_inclined {
            // u is the argument of latitude
            let u = if r[2] < 0.0 {
                2.0 * PI - (n.dot(&r) / (nm * rm)).acos()
            } else {
                (n.dot(&r) / (nm * rm)).acos()
            };
            let raan = if n[1] < 0.0 {
                2.0 * PI - (n[0] / nm).acos()
            } else {
                (n[0] / nm).acos()
            };

            (raan, 0.0, u)
        } else if is_elliptical_equatorial {
            //w_true is the true longitude of periapsis
            let w_true = if e_vector[1] < 0.0 {
                2.0 * PI - (i_hat.dot(&e_vector) / e).acos()
            } else {
                (i_hat.dot(&e_vector) / e).acos()
            };

            let f = if rdotv < 0.0 {
                2.0 * PI - (e_vector.dot(&r) / (e * rm)).acos()
            } else {
                (e_vector.dot(&r) / (e * rm)).acos()
            };

            (0.0, w_true, f)
        } else {
            let raan = if n[1] < 0.0 {
                2.0 * PI - (n[0] / nm).acos()
            } else {
                (n[0] / nm).acos()
            };

            let argp = if e_vector[2] < 0.0 {
                2.0 * PI - (n.dot(&e_vector) / (nm * e)).acos()
            } else {
                (n.dot(&e_vector) / (nm * e)).acos()
            };

            let f = if rdotv < 0.0 {
                2.0 * PI - (e_vector.dot(&r) / (e * rm)).acos()
            } else {
                (e_vector.dot(&r) / (e * rm)).acos()
            };
            (raan, argp, f)
        };

        let orbit_type = if e < ORBIT_EPSILON {
            OrbitType::Circular
        } else if e <= 1.0 - ORBIT_EPSILON {
            OrbitType::Elliptical
        } else if e < 1.0 + ORBIT_EPSILON {
            OrbitType::Parabolic
        } else {
            OrbitType::Hyperbolic
        };

        KeplerianElements {
            mu,
            radius,
            epoch,
            semimajor_axis: a,
            semiparameter: p,
            argument_of_periapsis: argp,
            eccentricity: e,
            inclination: i,
            raan,
            true_anomaly: f,
            orbit_type,
        }
    }

    /// Calculates the altitude of the apoapsis (the farthest point from the central body) of the orbit.
    ///
    /// # Returns
    /// - Returns `f64::INFINITY` for parabolic and hyperbolic orbits, as these orbit types have no bounded apoapsis.
    /// - For other orbit types (e.g., elliptical), returns the altitude of the apoapsis in m, calculated as:
    ///   `semimajor_axis * (1.0 + eccentricity) - radius`.
    ///    
    pub fn get_apoapsis_altitude(&self) -> f64 {
        match self.orbit_type {
            OrbitType::Parabolic | OrbitType::Hyperbolic => f64::INFINITY,
            _ => self.semimajor_axis * (1.0 + self.eccentricity) - self.radius,
        }
    }

    /// Calculates the altitude of the periapsis (the closest point to the central body) of the orbit.
    ///
    /// # Returns
    /// - For parabolic and hyperbolic orbits, returns the periapsis altitude as `semiparameter - radius`,
    ///   because the semi-major axis is considered infinite in these cases, but the periapsis distance is finite.
    /// - For other orbit types (e.g., elliptical), returns the altitude of the periapsis in m, calculated as:
    ///   `semimajor_axis * (1.0 - eccentricity) - radius`.    
    pub fn get_periapsis_altitude(&self) -> f64 {
        match self.orbit_type {
            OrbitType::Parabolic | OrbitType::Hyperbolic => self.semiparameter - self.radius, //a is inifinite but p is the periapsis_distance
            _ => self.semimajor_axis * (1.0 - self.eccentricity) - self.radius,
        }
    }

    /// Converts the Keplerian orbital elements into position and velocity vectors.
    ///
    /// # Returns
    /// A tuple containing the position vector (in m) and velocity vector (in m/s) in Cartesian coordinates.
    pub fn get_rv(&self) -> (Vector3<f64>, Vector3<f64>) {
        let p = self.semiparameter;
        let f = self.true_anomaly;
        let e = self.eccentricity;
        let cf = f.cos();
        let sf = f.sin();

        let rp = p * cf / (1.0 + e * cf);
        let rq = p * sf / (1.0 + e * cf);

        let r_pqw = Vector3::new(rp, rq, 0.0);

        let root_mu_over_p = (self.mu / p).sqrt();
        let vp = -root_mu_over_p * sf;
        let vq = root_mu_over_p * (e + cf);
        let v_pqw = Vector3::new(vp, vq, 0.0);

        let euler_angles = EulerAngles::new(
            -self.argument_of_periapsis,
            -self.inclination,
            -self.raan,
            EulerSequence::ZXZ,
        );
        let rotation = Rotation::from(&euler_angles);

        // Rotate position and velocity from orbital plane to inertial frame
        let position = rotation.transform(r_pqw);
        let velocity = rotation.transform(v_pqw);

        (position, velocity)
    }

    pub fn iss() -> Self {
        // from the following NORAD TLE data
        // ISS (ZARYA)
        // 1 25544U 98067A   24315.45505231  .00030775  00000+0  51963-3 0  9998
        // 2 25544  51.6411 310.7613 0008478 159.7942 297.9503 15.51320041481230

        let earth = CelestialBodies::Earth;
        let mu = earth.get_mu();

        let n = 15.51320041481230;
        let a = (mu / (2.0 * PI * n / 86400.0).powi(2)).powf(1.0 / 3.0);

        let e = 0.0008478;
        let i = 51.6411 * PI / 180.0;
        let raan = 310.7613 * PI / 180.0;
        let argp = 159.7942 * PI / 180.0;
        let f = 297.9503 * PI / 180.0; // actually mean anomaly but circular so close enough

        let epoch = Time::from_doy(2024, 315.45505231, TimeSystem::UTC).unwrap();

        KeplerianElements::new(a, e, i, raan, argp, f, epoch, earth)
    }

    pub fn keplers_problem(&self, new_t: Time) -> Result<Self, OrbitErrors> {
        let delta_t = new_t - self.epoch;

        let old_anomaly = match self.orbit_type {
            OrbitType::Circular => {
                let (r, _) = self.get_rv();
                let rm = r.norm();
                let i_hat = Vector3::new(1.0, 0.0, 0.0);

                let lambda_true = if r[1] < 0.0 {
                    2.0 * PI - (i_hat.dot(&r) / rm).acos()
                } else {
                    (i_hat.dot(&r) / rm).acos()
                };
                lambda_true
            }
            _ => self.nu_to_anomaly(),
        };

        let new_anomaly = match self.orbit_type {
            OrbitType::Circular | OrbitType::Elliptical => {
                let n = (self.mu / (self.semimajor_axis).powi(3)).sqrt(); // mean motion
                let m0 = old_anomaly - self.eccentricity * old_anomaly.sin(); // initial mean anomaly
                let mf = m0 + n * delta_t; // final mean anomaly
                self.keplers_equation_elliptical(mf)
            }
            OrbitType::Parabolic => Ok(self.keplers_equation_parabolic(delta_t)),
            OrbitType::Hyperbolic => {
                let n = (self.mu / (self.semimajor_axis).powi(3)).sqrt(); // mean motion
                let m0 = self.eccentricity * old_anomaly.sinh() - old_anomaly; // initial mean anomaly
                let mf = m0 + n * delta_t; // final mean anomaly
                self.keplers_equation_hyperbolic(mf)
            }
        }?;

        let new_true_anomaly = match self.orbit_type {
            OrbitType::Circular => new_anomaly,
            _ => self.anomaly_to_nu(new_anomaly),
        };

        let mut new_kep = self.clone();
        new_kep.true_anomaly = new_true_anomaly;

        Ok(new_kep)
    }

    // Takes mean anomaly (m) as argument and return eccentric anomaly (ea) as output
    fn keplers_equation_elliptical(&self, ma: f64) -> Result<f64, OrbitErrors> {
        // calculate the new eccentric anomaly
        const TOL: f64 = 1e-8;
        const MAX_ITER: usize = 20;
        let e = self.eccentricity;
        let mut ea = if (ma > -PI && ma < 0.0) || ma > PI {
            ma - e
        } else {
            ma + e
        };
        let mut residual = 1.0;
        let mut prev = ea;
        let mut iter = 0;
        while residual > TOL {
            ea = ea + (ma - ea + e * ea.sin()) / (1.0 - e * ea.cos());
            residual = (ea - prev).abs();
            prev = ea;
            iter += 1;
            if iter > MAX_ITER {
                return Err(OrbitErrors::KeplerMaxIters);
            }
        }
        Ok(ea)
    }

    fn keplers_equation_parabolic(&self, dt: f64) -> f64 {
        // Vallado Algorithm 3
        // Barker's equation
        let np = 2.0 * (self.mu / self.semiparameter.powi(3)).sqrt();
        let s = 0.5 / (1.5 * np * dt).atan();
        let w = s.tan().cbrt().atan();
        let b = 2.0 * 1.0 / (2.0 * w).tan();
        b
    }

    fn keplers_equation_hyperbolic(&self, ma: f64) -> Result<f64, OrbitErrors> {
        const TOL: f64 = 1e-8;
        const MAX_ITER: usize = 20;

        // Vallado Algorithm 4
        let e = self.eccentricity;

        let mut h = if e < 1.6 {
            if (-PI < ma && ma < 0.0) || ma > PI {
                ma - e
            } else {
                ma + e
            }
        } else if e < 3.6 && ma.abs() > PI {
            ma - ma.signum() * e
        } else {
            ma / (e - 1.0)
        };

        let mut residual = 1.0;
        let mut prev = h;
        let mut iter = 0;
        while residual > TOL {
            h = h + (ma - h + e * h.sinh()) / (e * h.cosh() - 1.0);
            residual = (h - prev).abs();
            prev = h;
            iter += 1;
            if iter > MAX_ITER {
                return Err(OrbitErrors::KeplerMaxIters);
            }
        }
        Ok(h)
    }

    fn anomaly_to_nu(&self, a: f64) -> f64 {
        let e = self.eccentricity;

        match self.orbit_type {
            OrbitType::Circular | OrbitType::Elliptical => {
                let c = a.cos();
                ((c - e) / (1.0 - e * c)).acos()
            }
            OrbitType::Parabolic => {
                // i make the choice to calculate rv here instead of storing the values in
                // the struct since it's probably very rare that real orbits are parabolic, maybe?
                // just trying to avoid the overhead of always calculating rv
                let (r, _) = self.get_rv();
                let rm = r.norm();
                (self.semiparameter - rm) / rm
            }
            OrbitType::Hyperbolic => {
                let ch = a.cosh();
                ((ch - e) / (1.0 - e * ch)).acos()
            }
        }
    }

    fn nu_to_anomaly(&self) -> f64 {
        // Vallado Algorithm 5

        // converts true anomaly (nu in vallado, f here) and returns:
        // eccentric anomaly E for elliptical orbits
        // parabolic anomaly B for parabolic orbits
        // hyperbolic anomaly H for hyperbolic orbits

        let e = self.eccentricity;
        let f = self.true_anomaly;
        let cf = f.cos();

        match self.orbit_type {
            OrbitType::Circular | OrbitType::Elliptical => ((e + cf) / (1.0 + e * cf)).acos(),
            OrbitType::Parabolic => (f / 2.0).tan(),
            OrbitType::Hyperbolic => ((e + cf) / (1.0 + e * cf)).acosh(),
        }
    }

    pub fn new(
        a: f64,
        e: f64,
        i: f64,
        raan: f64,
        argp: f64,
        f: f64,
        epoch: Time,
        central_body: CelestialBodies,
    ) -> Self {
        let mu = central_body.get_mu();
        let radius = central_body.get_radius();
        let (orbit_type, p) = if e < ORBIT_EPSILON {
            (OrbitType::Circular, a)
        } else if e <= 1.0 - ORBIT_EPSILON {
            (OrbitType::Elliptical, a * (1.0 - e.powi(2)))
        } else if e < 1.0 + ORBIT_EPSILON {
            (OrbitType::Parabolic, a * (1.0 - e.powi(2)))
        } else {
            (OrbitType::Hyperbolic, a * (1.0 - e.powi(2)))
        };
        Self {
            semimajor_axis: a,
            semiparameter: p,
            eccentricity: e,
            inclination: i,
            argument_of_periapsis: argp,
            raan,
            epoch,
            true_anomaly: f,
            mu,
            radius,
            orbit_type,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use time::TimeSystem;

    #[test]
    fn test_from_pv() {
        let position = Vector3::new(6524834.0, 6862875.0, 6448296.0);
        let velocity = Vector3::new(4901.327, 5533.756, -1976.341);

        let epoch = Time::from_ymdhms(2000, 1, 1, 12, 0, 0.0, TimeSystem::UTC).unwrap();

        let orbit = KeplerianElements::from_rv(position, velocity, epoch, CelestialBodies::Earth);

        // note that Vallado doesn't include enough decimal places for e-6 accuracy, so we did the float division he provides to get the f64 value
        let expected_semi_major_axis = 36127343.0;
        let expected_semiparameter = 11067790.0;
        let expected_eccentricity = 0.832853;
        let expected_inclination = 87.86912591666639 * PI / 180.0;
        let expected_raan = 227.8982527706177 * PI / 180.0;
        let expected_argument_of_periapsis = 53.384930670193846 * PI / 180.0; //float value grabbed from julia satellite toolbox for precision
        let expected_true_anomaly = 92.3351567104033 * PI / 180.0; //float value grabbed from julia satellite toolbox for precision

        assert_abs_diff_eq!(
            orbit.semimajor_axis,
            expected_semi_major_axis,
            epsilon = 10.0 // 10m, since vallado used a slightly different value for mu and r
        );
        assert_abs_diff_eq!(orbit.semiparameter, expected_semiparameter, epsilon = 10.0);
        assert_abs_diff_eq!(orbit.eccentricity, expected_eccentricity, epsilon = 1e-6);
        assert_abs_diff_eq!(orbit.inclination, expected_inclination, epsilon = 1e-6);
        assert_abs_diff_eq!(orbit.raan, expected_raan, epsilon = 1e-6);
        assert_abs_diff_eq!(
            orbit.argument_of_periapsis,
            expected_argument_of_periapsis,
            epsilon = 1e-6
        );
        assert_abs_diff_eq!(orbit.true_anomaly, expected_true_anomaly, epsilon = 1e-6);
    }

    #[test]
    fn test_round_trip() {
        let position = Vector3::new(6524834.0, 6862875.0, 6448296.0);
        let velocity = Vector3::new(4901.327, 5533.756, -1976.341);

        let epoch = Time::from_ymdhms(2000, 1, 1, 12, 0, 0.0, TimeSystem::UTC).unwrap();

        // Create a Keplerian orbit using the from_pv method
        let orbit = KeplerianElements::from_rv(position, velocity, epoch, CelestialBodies::Earth);

        // Convert back to position and velocity using get_pv
        let (computed_position, computed_velocity) = orbit.get_rv();

        // Check that the original and computed values match (within tolerance for floating-point calculations)
        assert_abs_diff_eq!(position, computed_position, epsilon = 1.0);
        assert_abs_diff_eq!(velocity, computed_velocity, epsilon = 1.0);
    }

    #[test]
    fn test_from_pv_circular() {
        let position = Vector3::new(7000000.0, 0.0, 0.0);
        let velocity = Vector3::new(0.0, 0.0, 7546.0533);

        let epoch = Time::from_ymdhms(2000, 1, 1, 12, 0, 0.0, TimeSystem::UTC).unwrap();

        let orbit = KeplerianElements::from_rv(position, velocity, epoch, CelestialBodies::Earth);

        // note that Vallado doesn't include enough decimal places for e-6 accuracy, so we did the float division he provides to get the f64 value
        let expected_semi_major_axis = 7000000.0;
        let expected_eccentricity = 0.0;
        let expected_inclination = PI / 2.0;
        let expected_raan = 0.0;
        let expected_argument_of_periapsis = 0.0; //float value grabbed from julia satellite toolbox for precision
        let expected_true_anomaly = 0.0; //float value grabbed from julia satellite toolbox for precision

        assert_abs_diff_eq!(
            orbit.semimajor_axis,
            expected_semi_major_axis,
            epsilon = 1.0
        );
        assert_abs_diff_eq!(orbit.eccentricity, expected_eccentricity, epsilon = 1e-6);
        assert_abs_diff_eq!(orbit.inclination, expected_inclination, epsilon = 1e-6);
        assert_abs_diff_eq!(orbit.raan, expected_raan, epsilon = 1e-6);
        assert_abs_diff_eq!(
            orbit.argument_of_periapsis,
            expected_argument_of_periapsis,
            epsilon = 1e-6
        );
        assert_abs_diff_eq!(orbit.true_anomaly, expected_true_anomaly, epsilon = 1e-6);
    }

    // fn test_keplers_equation_elliptical() {
    //     let orb = KeplerianElements::new()
    // }
}
