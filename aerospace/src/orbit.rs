use nalgebra::Vector3;
use rotations::{
    prelude::{EulerAngles, EulerSequence},
    Rotation, RotationTrait,
};
use std::f64::{consts::PI, INFINITY};

/// Represents the orbital elements of a Keplerian orbit.
///
/// This struct contains the orbital parameters necessary to describe an object's orbit
/// around a central body, based on the Keplerian model. It includes elements such as the
/// semi-major axis, eccentricity, inclination, RAAN, argument of periapsis, and anomalies.
pub struct KeplerianOrbit {
    /// Gravitational parameter of the central body (in km^3/s^2).
    pub mu: f64,
    /// Radius of the central body (in km).
    pub radius: f64,
    /// Semi-major axis of the orbit (in km).
    pub semimajor_axis: f64,
    /// Semi-parameter (semi-latus rectum) of the orbit (in km).
    pub semiparameter: f64,
    /// Argument of periapsis (in radians).
    pub argument_of_periapsis: f64,
    /// Eccentric anomaly (in radians).
    pub eccentric_anomaly: f64,
    /// Eccentricity of the orbit (dimensionless).
    pub eccentricity: f64,
    /// Inclination of the orbit (in radians).
    pub inclination: f64,
    /// Mean anomaly (in radians).
    pub mean_anomaly: f64,
    /// Right Ascension of the Ascending Node (RAAN) (in radians).
    pub raan: f64,
    /// True anomaly (in radians).
    pub true_anomaly: f64,
}

impl KeplerianOrbit {
    /// Tolernance for determining if orbit is elliptical/circular/parabolic/hyperbolic
    const ORBIT_EPSILON: f64 = 1e-8;

    /// Creates a new `KeplerianOrbit` instance for an elliptic orbit.
    ///
    /// # Arguments
    /// * `mu` - Gravitational parameter of the central body (in km^3/s^2).
    /// * `radius` - Radius of the central body (in km).
    /// * `semimajor_axis` - Semi-major axis of the orbit (in km).
    /// * `eccentricity` - Eccentricity of the orbit (dimensionless).
    /// * `inclination` - Inclination of the orbit (in radians).
    /// * `raan` - Right Ascension of the Ascending Node (in radians).
    /// * `argument_of_periapsis` - Argument of periapsis (in radians).
    /// * `true_anomaly` - True anomaly (in radians).
    ///
    /// # Returns
    /// A new `KeplerianOrbit` instance representing the specified elliptic orbit.
    pub fn new(
        mu: f64,
        radius: f64,
        semimajor_axis: f64,
        eccentricity: f64,
        inclination: f64,
        raan: f64,
        argument_of_periapsis: f64,
        true_anomaly: f64,
    ) -> Self {
        // Calculate the mean anomaly (M) from the true anomaly (v) and eccentricity (e)
        let eccentric_anomaly = 2.0
            * ((true_anomaly / 2.0).tan())
                .atan2((1.0 + eccentricity).sqrt() * (1.0 - eccentricity).sqrt());
        let mean_anomaly = eccentric_anomaly - eccentricity * eccentric_anomaly.sin();
        let semiparameter = semimajor_axis * (1.0 - eccentricity.powi(2));

        KeplerianOrbit {
            mu,
            radius,
            semimajor_axis,
            semiparameter,
            argument_of_periapsis,
            eccentric_anomaly,
            eccentricity,
            inclination,
            mean_anomaly,
            raan,
            true_anomaly,
        }
    }

    /// Creates a `KeplerianOrbit` instance from position and velocity vectors.
    ///
    /// # Arguments
    /// * `mu` - Gravitational parameter of the central body (in km^3/s^2).
    /// * `radius` - Radius of the central body (in km).
    /// * `r` - Position vector in Cartesian coordinates (in km).
    /// * `v` - Velocity vector in Cartesian coordinates (in km/s).
    ///
    /// # Returns
    /// A new `KeplerianOrbit` instance representing the orbit based on the provided position and velocity vectors.
    pub fn from_rv(mu: f64, radius: f64, r: Vector3<f64>, v: Vector3<f64>) -> Self {
        // calculate canonical units
        let du = radius;
        let tu = (du.powi(3) / mu).sqrt();
        // convert to units of body radii
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

        let e = (vm.powi(2) - 1.0 / rm) * r - rdotv * v;
        let em = e.norm();

        let zeta = vm.powi(2) / 2.0 - 1.0 / rm;

        let (a, p) = if (em - 1.0).abs() > Self::ORBIT_EPSILON {
            let a = -1.0 / (2.0 * zeta);
            let p = a * (1.0 - em.powi(2));
            (a, p)
        } else {
            (INFINITY, hm.powi(2))
        };

        let i = (h[2] / hm).acos();
        let raan = if n[1] < 0.0 {
            2.0 * PI - (n[0] / nm).acos()
        } else {
            (n[0] / nm).acos()
        };

        let argp = if e[2] < 0.0 {
            2.0 * PI - (n.dot(&e) / (nm * em)).acos()
        } else {
            (n.dot(&e) / (nm * em)).acos()
        };

        let true_anomaly = if rdotv < 0.0 {
            2.0 * PI - (e.dot(&r) / (em * rm)).acos()
        } else {
            (e.dot(&r) / (em * rm)).acos()
        };
        
        KeplerianOrbit::new(mu,radius,a * du, em,i,raan,argp,true_anomaly)
        
    }

    /// Converts the Keplerian orbital elements into position and velocity vectors.
    ///
    /// # Returns
    /// A tuple containing the position vector (in km) and velocity vector (in km/s) in Cartesian coordinates.
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
        let rotation = Rotation::from(euler_angles);

        // Rotate position and velocity from orbital plane to inertial frame
        let position = rotation.transform(r_pqw);
        let velocity = rotation.transform(v_pqw);

        (position, velocity)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn test_from_pv() {
        // values from Vallado
        let earth_mu = 398600.4415;
        let earth_radius = 6378.1363;

        let position = Vector3::new(6524.834, 6862.875, 6448.296);
        let velocity = Vector3::new(4.901327, 5.533756, -1.976341);

        let orbit = KeplerianOrbit::from_rv(earth_mu, earth_radius, position, velocity);

        // note that Vallado doesn't include enough decimal places for e-6 accuracy, so we did the float division he provides to get the f64 value
        let expected_semi_major_axis = 36127.343;
        let expected_semiparameter = 11067.790;
        let expected_eccentricity = 0.832853;
        let expected_inclination = 87.86912591666639 * PI / 180.0;
        let expected_raan = 227.8982527706177 * PI / 180.0;
        let expected_argument_of_periapsis = 53.384930670193846 * PI / 180.0; //float value grabbed from julia satellite toolbox for precision
        let expected_true_anomaly = 92.3351567104033 * PI / 180.0; //float value grabbed from julia satellite toolbox for precision

        assert_abs_diff_eq!(
            orbit.semimajor_axis,
            expected_semi_major_axis,
            epsilon = 1e-2
        );
        assert_abs_diff_eq!(orbit.semiparameter, expected_semiparameter, epsilon = 1e-2);
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
        // values from Vallado
        let earth_mu = 398600.4415;
        let earth_radius = 6378.1363;

        let position = Vector3::new(6524.834, 6862.875, 6448.296);
        let velocity = Vector3::new(4.901327, 5.533756, -1.976341);

        // Create a Keplerian orbit using the from_pv method
        let orbit = KeplerianOrbit::from_rv(earth_mu, earth_radius, position, velocity);

        // Convert back to position and velocity using get_pv
        let (computed_position, computed_velocity) = orbit.get_rv();

        // Check that the original and computed values match (within tolerance for floating-point calculations)
        assert_abs_diff_eq!(position, computed_position, epsilon = 1e-2);
        assert_abs_diff_eq!(velocity, computed_velocity, epsilon = 1e-2);
    }
}
