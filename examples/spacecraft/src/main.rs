use std::{cell::RefCell, rc::Rc};

use aerospace::orbit::KeplerianElements;
use celestial::{CelestialBodies, CelestialSystem};
use color::Color;
use hardware::{
    actuators::{reaction_wheel::ReactionWheel, SpacecraftActuators},
    sensors::{gps::Gps, rate_gyro::RateGyro, star_tracker::StarTracker, SpacecraftSensors},
};
use mass_properties::{CenterOfMass, Inertia, MassProperties};
use multibody::{
    actuator::Actuator,
    base::{Base, BaseSystems},
    body::{Body, BodyTrait},
    joint::{
        floating::{Floating, FloatingParameters, FloatingState},
        Joint,
    },
    sensor::{
        noise::{gaussian::GaussianNoise, NoiseModels},
        Sensor,
    },
    system::MultibodySystem,
};

use nadir_3d::{
    geometry::{cuboid::Cuboid, Geometry, GeometryState},
    material::Material,
    mesh::Mesh,
};
use rotations::{
    prelude::{AlignedAxes, Axis, AxisPair, Quaternion},
    Rotation,
};
use software::SpacecraftFsw;
use time::Time;
use transforms::{prelude::CoordinateSystem, Transform};

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
    let orbit = KeplerianElements::new(
        6.4e6,
        0.0,
        0.0,
        0.0,
        0.0,
        3.14,
        epoch,
        CelestialBodies::Earth,
    );
    let state = FloatingState::new()
        .with_rates([0.0, 0.0, 0.0].into())
        .with_attitude(Quaternion::new(-0.4, 0.5, -0.5, 0.5))
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

    let mut gps = Sensor::new(
        "gps",
        Gps::new()
            .with_noise_position(NoiseModels::Gaussian(GaussianNoise::new(0.0, 50.0)))
            .with_noise_velocity(NoiseModels::Gaussian(GaussianNoise::new(0.0, 1.0))),
    );
    // Add a star tracker
    let mut st = Sensor::new(
        "st",
        StarTracker::new().with_noise(NoiseModels::Gaussian(GaussianNoise::new(
            0.0,
            50.0 / 3600.0 * std::f64::consts::PI / 180.0,
        ))),
    );

    // Add a rate gyro
    let mut imu = Sensor::new(
        "imu",
        RateGyro::new().with_noise(NoiseModels::Gaussian(GaussianNoise::new(
            0.0,
            1.0e-3 * std::f64::consts::PI / 180.0,
        ))),
    );

    // Create the reaction wheels
    let mut rw1 = Actuator::new("rw1", ReactionWheel::new(0.25, 0.5, 0.0).unwrap());
    let mut rw2 = Actuator::new("rw2", ReactionWheel::new(0.25, 0.5, 0.0).unwrap());
    let mut rw3 = Actuator::new("rw3", ReactionWheel::new(0.25, 0.5, 0.0).unwrap());
    let mut rw4 = Actuator::new("rw4", ReactionWheel::new(0.25, 0.5, 0.0).unwrap());

    let rw1_transform = Transform::new(
        Rotation::from(
            &AlignedAxes::new(
                AxisPair::new(Axis::Zp, Axis::Xp),
                AxisPair::new(Axis::Yp, Axis::Yp),
            )
            .unwrap(),
        ),
        CoordinateSystem::ZERO,
    );
    let rw2_transform = Transform::new(
        Rotation::from(
            &AlignedAxes::new(
                AxisPair::new(Axis::Zp, Axis::Yp),
                AxisPair::new(Axis::Xp, Axis::Xp),
            )
            .unwrap(),
        ),
        CoordinateSystem::ZERO,
    );

    // wheel axis default is z axis, so no rotation needed
    let rw3_transform = Transform::new(Rotation::IDENTITY, CoordinateSystem::ZERO);

    // not using rw4 yet. zero'd out in calcs
    let rw4_transform = Transform::new(Rotation::IDENTITY, CoordinateSystem::ZERO);

    rw1.connect_to_body(&b, rw1_transform).unwrap();
    rw2.connect_to_body(&b, rw2_transform).unwrap();
    rw3.connect_to_body(&b, rw3_transform).unwrap();
    rw4.connect_to_body(&b, rw4_transform).unwrap();

    let actuator_system = SpacecraftActuators {
        rw: [rw1, rw2, rw3, rw4],
    };

    // Connect the components together.
    // Connections are made by the components names.
    // The direction of the connection matters - (from,to)
    base.borrow_mut().connect_outer_joint(&f).unwrap();
    f.borrow_mut()
        .connect_base(&base, Transform::IDENTITY)
        .unwrap();
    b.borrow_mut().connect_inner_joint(&f).unwrap();
    f.borrow_mut()
        .connect_outer_body(&b, Transform::IDENTITY)
        .unwrap();

    gps.connect_to_body(&b, Transform::IDENTITY).unwrap();
    st.connect_to_body(&b, Transform::IDENTITY).unwrap();
    imu.connect_to_body(&b, Transform::IDENTITY).unwrap();

    let sensor_system = SpacecraftSensors { gps, st, imu };

    let software = SpacecraftFsw::default();

    // Create the system
    let mut sys = MultibodySystem::new(base, [b], [f], sensor_system, software, actuator_system);
    // Run the simulation
    sys.simulate("", 0.0, 15000.0, 1.0);
}
