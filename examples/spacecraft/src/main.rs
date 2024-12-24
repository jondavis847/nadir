use std::{cell::RefCell, rc::Rc};

use aerospace::{
    celestial_system::{CelestialBodies, CelestialSystem},
    orbit::KeplerianElements,
};
use color::Color;
use hardware::SpacecraftSensors;
use mass_properties::{CenterOfMass, Inertia, MassProperties};
use multibody::{
    base::{Base, BaseSystems}, body::{Body, BodyTrait}, joint::{
        floating::{Floating, FloatingParameters, FloatingState},
        Joint,
    }, sensor::{
        noise::{gaussian::GaussianNoise, NoiseModels}, rate_gyro::RateGyro, star_tracker::StarTracker, Sensor,
    }, system::MultibodySystem
};

use nadir_3d::{
    geometry::{cuboid::Cuboid, Geometry, GeometryState},
    material::Material,
    mesh::Mesh,
};
use software::SpacecraftSoftware;
use time::Time;
use transforms::Transform;

mod hardware;
mod software;

fn main() {
    
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
    
    let base = Base::new("base", BaseSystems::Celestial(celestial));

    // Create the Floating joint that represents the kinematics between the base and the spacecraft
    // A with_orbit() method is provided for Floating joints
    let orbit =
        KeplerianElements::new(8e6, 0.0, 0.0, 0.0, 0.0, 3.14, epoch, CelestialBodies::Earth);
    let state = FloatingState::new()
        .with_rates([0.0, 0.0, 0.1].into())
        .with_orbit(orbit.into());
    let parameters = FloatingParameters::new();
    let f_model = Floating::new(parameters, state);
    let f = Rc::new(RefCell::new(Joint::new("f", f_model).unwrap()));

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

    let b = Rc::new(RefCell::new(Body::new("b", mp).unwrap().with_mesh(mesh)));

    // Add a star tracker    
    let mut st = Sensor::new(
        "st",
        StarTracker::new()
        .with_noise(NoiseModels::Gaussian(GaussianNoise::new(
        0.0,
        50.0 / 3600.0 * 180.0 / std::f64::consts::PI,
    ))));
    

    // Add a rate gyro
    let mut imu = Sensor::new(
        "imu",
        RateGyro::new()
        .with_noise(NoiseModels::Gaussian(GaussianNoise::new(
            0.0,
            1.0 * std::f64::consts::PI / 180.0,
        )))
    );
    

    // Connect the components together.
    // Connections are made by the components names.
    // The direction of the connection matters - (from,to)
    base.borrow_mut().connect_outer_joint(&f);
    f.borrow_mut().connect_base(&base, Transform::IDENTITY);
    b.borrow_mut().connect_inner_joint(&f);
    f.borrow_mut().connect_outer_body(&b, Transform::IDENTITY);

    st.connect_to_body(&b, Transform::IDENTITY);
    imu.connect_to_body(&b, Transform::IDENTITY);

    let sensor_system = SpacecraftSensors {
        st, imu,
    };

    let software = SpacecraftSoftware::default();

    // Create the system
    let mut sys = MultibodySystem::new(base,[b],[f]).with_sensors(sensor_system).with_software(software);
    // Run the simulation
    sys.simulate("", 0.0, 7000.0, 1.0);
}
