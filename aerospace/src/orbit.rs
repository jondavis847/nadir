use nalgebra::Vector3;
pub struct Orbit {
    pub semimajor_axis: f64,
    pub apoapsis_altitude: f64,
    pub argument_of_perigee: f64,
    pub eccentric_anomaly: f64, 
    pub eccentricity: f64,    
    pub inclination: f64,
    pub mean_anomaly: f64,
    pub mltan: f64,
    pub periapsis_altitude: f64,
    pub raan: f64,
    pub true_anomaly: f64,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
}

impl Orbit {
    pub fn from_keplerian(
        semimajor_axis: f64,
        eccentricity: f64,
        inclination: f64,
        raan: f64,
        argument_of_perigee: f64,
        true_anomaly: f64,
    ) -> Self {
        // Calculate periapsis and apoapsis altitudes
        let periapsis_altitude = semimajor_axis * (1.0 - eccentricity);
        let apoapsis_altitude = semimajor_axis * (1.0 + eccentricity);

        // Calculate the mean anomaly (M) from the true anomaly (v) and eccentricity (e)
        let eccentric_anomaly = 2.0 * ((true_anomaly / 2.0).tan()).atan2((1.0 + eccentricity).sqrt() * (1.0 - eccentricity).sqrt());
        let mean_anomaly = eccentric_anomaly - eccentricity * eccentric_anomaly.sin();

        Orbit {
            semimajor_axis,
            apoapsis_altitude,
            argument_of_perigee,
            eccentric_anomaly,
            eccentricity,
            inclination,
            mean_anomaly,
            periapsis_altitude,
            raan,
            true_anomaly,
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
        }
    }

    pub fn from_pvt(position: Vector3<f64>, velocity: Vector3<f64>, mu: f64) -> Self {
        let r = position.norm();
        let v = velocity.norm();
        let vr = position.dot(&velocity) / r;

        let specific_angular_momentum = position.cross(&velocity);
        let h = specific_angular_momentum.norm();

        let inclination = (specific_angular_momentum[2] / h).acos();
        let raan = specific_angular_momentum[1].atan2(-specific_angular_momentum[0]);

        let eccentricity_vector = (v.powi(2) - mu / r) * position - r * vr * velocity;
        let eccentricity = eccentricity_vector.norm() / mu;

        let semimajor_axis = 1.0 / ((2.0 / r) - (v.powi(2) / mu));
        let periapsis_altitude = semimajor_axis * (1.0 - eccentricity);
        let apoapsis_altitude = semimajor_axis * (1.0 + eccentricity);

        let argument_of_perigee = eccentricity_vector[2].atan2(eccentricity_vector[0]);
        let true_anomaly = position[2].atan2(position[0]);

        let eccentric_anomaly = 2.0 * ((true_anomaly / 2.0).tan()).atan2((1.0 + eccentricity).sqrt() * (1.0 - eccentricity).sqrt());
        let mean_anomaly = eccentric_anomaly - eccentricity * eccentric_anomaly.sin();

        Orbit {
            semimajor_axis,
            apoapsis_altitude,
            argument_of_perigee,
            eccentric_anomaly,
            eccentricity,
            inclination,
            mean_anomaly,
            periapsis_altitude,
            raan,
            true_anomaly,
            position,
            velocity,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    #[test]
    fn test_from_keplerian() {
        let semimajor_axis = 7000.0;
        let eccentricity = 0.1;
        let inclination = 0.5;
        let raan = 1.0;
        let argument_of_perigee = 0.3;
        let true_anomaly = 0.7;

        let orbit = Orbit::from_keplerian(semimajor_axis, eccentricity, inclination, raan, argument_of_perigee, true_anomaly);

        assert_eq!(orbit.semimajor_axis, semimajor_axis);
        assert_eq!(orbit.eccentricity, eccentricity);
        assert_eq!(orbit.inclination, inclination);
        assert_eq!(orbit.raan, raan);
        assert_eq!(orbit.argument_of_perigee, argument_of_perigee);
        assert_eq!(orbit.true_anomaly, true_anomaly);
    }

    #[test]
    fn test_from_pvt() {
        let position = Vector3::new(7000.0, 0.0, 0.0);
        let velocity = Vector3::new(0.0, 7.5, 0.0);
        let mu = 398600.4418;

        let orbit = Orbit::from_pvt(position, velocity, mu);

        assert!(orbit.semimajor_axis > 0.0);
        assert!(orbit.eccentricity >= 0.0);
        assert!(orbit.inclination >= 0.0 && orbit.inclination <= std::f64::consts::PI);
        assert!(orbit.raan >= -std::f64::consts::PI && orbit.raan <= std::f64::consts::PI);
        assert!(orbit.argument_of_perigee >= -std::f64::consts::PI && orbit.argument_of_perigee <= std::f64::consts::PI);
        assert!(orbit.true_anomaly >= -std::f64::consts::PI && orbit.true_anomaly <= std::f64::consts::PI);
    }
}