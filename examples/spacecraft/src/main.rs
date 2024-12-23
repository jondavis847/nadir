use aerospace::{
    celestial_system::{CelestialBodies, CelestialSystem},
    orbit::KeplerianElements,
};
use color::Color;
use mass_properties::{CenterOfMass, Inertia, MassProperties};
use multibody::{
    body::Body,
    joint::{
        floating::{Floating, FloatingParameters, FloatingState},
        Joint,
    },
    sensor::{
        noise::{gaussian::GaussianNoise, NoiseModels}, rate_gyro::RateGyro, star_tracker::StarTracker, Sensor, SensorModel
    },
    system::MultibodySystem,
};

use nadir_3d::{
    geometry::{cuboid::Cuboid, Geometry, GeometryState},
    material::Material,
    mesh::Mesh,
};
use time::Time;
use transforms::Transform;

fn main() {
    // Create the MultibodySystem
    // sys is will be created with nothing but a default base
    let mut sys = MultibodySystem::new();

    // Create the CelestialSystem which contains the planetary models
    // In NADIR the base is GCRF (J2000) when a CelestialSystem is present
    let epoch = Time::now().unwrap();
    let mut celestial = CelestialSystem::new(epoch).unwrap();
    celestial
        .add_body(CelestialBodies::Earth, true, false)
        .unwrap();
    celestial
        .add_body(CelestialBodies::Moon, false, false)
        .unwrap();
    sys.add_celestial_system(celestial);

    // Create the Floating joint that represents the kinematics between the base and the spacecraft
    // A with_orbit() method is provided for Floating joints
    let orbit =
        KeplerianElements::new(8e6, 0.0, 0.0, 0.0, 0.0, 3.14, epoch, CelestialBodies::Earth);
    let state = FloatingState::new()
        .with_rates([0.0, 0.0, 0.1].into())
        .with_orbit(orbit.into());
    let parameters = FloatingParameters::new();
    let f_model = Floating::new(parameters, state);
    let f = Joint::new("f", f_model).unwrap();
    sys.add_joint(f).unwrap();

    // Create the main body of the spacecraft
    let cm = CenterOfMass::new(0.0, 0.0, 0.0);
    let inertia = Inertia::new(1000.0, 1000.0, 1000.0, 0.0, 0.0, 0.0).unwrap();
    let mp = MassProperties::new(1000.0, cm, inertia).unwrap();
    let geometry = Geometry::Cuboid(Cuboid::new(1.0, 1.0, 1.0));
    let material = Material::Phong {
        color: Color::GREEN,
        specular_power: 32.0,
    };
    let mesh = Mesh {
        name: "b".to_string(),
        geometry,
        material,
        state: GeometryState::default(),
        texture: None,
    };

    let b = Body::new("b", mp).unwrap().with_mesh(mesh);
    sys.add_body(b).unwrap();

    // Add a star tracker
    let st_model = StarTracker::new()
        .with_noise(NoiseModels::Gaussian(GaussianNoise::new(
        0.0,
        50.0 / 3600.0 * 180.0 / std::f64::consts::PI,
    )));
    let st = Sensor::new(
        "st",
        SensorModel::StarTracker(st_model));
    sys.add_sensor(st).unwrap();

    // Add a rate gyro
    let imu_model = RateGyro::new()
        .with_noise(NoiseModels::Gaussian(GaussianNoise::new(
            0.0,
            1.0 * std::f64::consts::PI / 180.0,
        )));

    let imu = Sensor::new(
        "imu",
        SensorModel::RateGyro(imu_model)
    );
    sys.add_sensor(imu).unwrap();

    // Connect the components together.
    // Connections are made by the components names.
    // The direction of the connection matters - (from,to)
    sys.connect("base", "f", Transform::IDENTITY).unwrap();
    sys.connect("f", "b", Transform::IDENTITY).unwrap();
    sys.connect("st", "b", Transform::IDENTITY).unwrap();
    sys.connect("imu", "b", Transform::IDENTITY).unwrap();

    // Run the simulation
    sys.simulate("", 0.0, 7000.0, 1.0);
}
