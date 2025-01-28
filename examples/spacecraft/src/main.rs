use std::{cell::RefCell, f64::consts::PI, rc::Rc};

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
        revolute::{Revolute, RevoluteParameters, RevoluteState},
        Joint, JointParameters,
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
use transforms::{
    prelude::{Cartesian, CoordinateSystem},
    Transform,
};

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

    // Create the base reference frame of the system and add the CelestialSystem
    let base = Base::new("base", BaseSystems::Celestial(celestial));

    // Create the Floating joint that represents the kinematics between the base and the spacecraft
    // A with_orbit() method is provided for Floating joints
    let orbit = KeplerianElements::new(8e6, 0.0, 0.5, 0.0, 0.0, 0.0, epoch, CelestialBodies::Earth);
    let state = FloatingState::new()
        .with_rates([0.0, 0.0, 0.0].into())
        .with_attitude(Quaternion::new(-0.4, -0.5, 0.5, 0.5))
        .with_orbit(orbit.into());
    let parameters = FloatingParameters::new();
    let f_model = Floating::new(parameters, state);

    // TODO: This isnt the ideal interface and will be improved in the future
    let f = Rc::new(RefCell::new(Joint::new("f", f_model).unwrap()));

    // Create the main body of the spacecraft
    let cm = CenterOfMass::new(0.0, 0.0, 0.0);
    let inertia = Inertia::new(1000.0, 1000.0, 1000.0, 0.0, 0.0, 0.0).unwrap();
    let mp = MassProperties::new(1000.0, cm, inertia).unwrap();

    // Add a mesh for 3d animation
    let geometry = Geometry::Cuboid(Cuboid::new(3.0, 2.0, 2.0));
    let material = Material::Phong {
        color: Color::new(0.5, 0.5, 0.5, 1.0),
        specular_power: 32.0,
    };
    let mesh = Mesh {
        name: "bus".to_string(),
        geometry,
        material,
        state: GeometryState::default(),
        texture: None,
    };

    let bus = Rc::new(RefCell::new(Body::new("bus", mp).unwrap().with_mesh(mesh)));

    // Create the solar array panels of the spacecraft
    //solar array 1
    let inertia = Inertia::new(100.0, 100.0, 100.0, 0.0, 0.0, 0.0).unwrap();
    let mp = MassProperties::new(100.0, cm, inertia).unwrap();
    let geometry = Geometry::Cuboid(Cuboid::new(2.0, 2.0, 0.1));
    let material = Material::Phong {
        color: Color::new(0.05, 0.05, 0.05, 1.0),
        specular_power: 32.0,
    };
    let mesh = Mesh {
        name: "sa1".to_string(),
        geometry,
        material,
        state: GeometryState::default(),
        texture: None,
    };

    let sa1 = Rc::new(RefCell::new(Body::new("sa1", mp).unwrap().with_mesh(mesh)));

    //solar array 2
    let inertia = Inertia::new(100.0, 100.0, 100.0, 0.0, 0.0, 0.0).unwrap();
    let mp = MassProperties::new(100.0, cm, inertia).unwrap();
    let geometry = Geometry::Cuboid(Cuboid::new(2.0, 2.0, 0.1));
    let material = Material::Phong {
        color: Color::new(0.05, 0.05, 0.05, 1.05),
        specular_power: 32.0,
    };
    let mesh = Mesh {
        name: "sa2".to_string(),
        geometry,
        material,
        state: GeometryState::default(),
        texture: None,
    };

    let sa2 = Rc::new(RefCell::new(Body::new("sa2", mp).unwrap().with_mesh(mesh)));

    //solar array 3
    let inertia = Inertia::new(100.0, 100.0, 100.0, 0.0, 0.0, 0.0).unwrap();
    let mp = MassProperties::new(100.0, cm, inertia).unwrap();
    let geometry = Geometry::Cuboid(Cuboid::new(2.0, 2.0, 0.1));
    let material = Material::Phong {
        color: Color::new(0.05, 0.05, 0.05, 1.0),
        specular_power: 32.0,
    };
    let mesh = Mesh {
        name: "sa3".to_string(),
        geometry,
        material,
        state: GeometryState::default(),
        texture: None,
    };

    let sa3 = Rc::new(RefCell::new(Body::new("sa3", mp).unwrap().with_mesh(mesh)));

    // create solar array hinges
    // hinge 1
    let p = RevoluteParameters(JointParameters {
        constant_force: 0.0,
        damping: 100.0,
        equilibrium: 0.0,
        spring_constant: 10.0,
    });
    let x = RevoluteState::new(PI / 2.0, 0.0);
    let hinge1 = Rc::new(RefCell::new(
        Joint::new("hinge1", Revolute::new(p, x)).unwrap(),
    ));

    // hinge 2
    let p = RevoluteParameters(JointParameters {
        constant_force: 0.0,
        damping: 100.0,
        equilibrium: 0.0,
        spring_constant: 10.0,
    });
    let x = RevoluteState::new(-3.14, 0.0);
    let hinge2 = Rc::new(RefCell::new(
        Joint::new("hinge2", Revolute::new(p, x)).unwrap(),
    ));

    // hinge 3
    let p = RevoluteParameters(JointParameters {
        constant_force: 0.0,
        damping: 100.0,
        equilibrium: 0.0,
        spring_constant: 10.0,
    });
    let x = RevoluteState::new(3.14, 0.0);
    let hinge3 = Rc::new(RefCell::new(
        Joint::new("hinge3", Revolute::new(p, x)).unwrap(),
    ));

    // Add a GPS model
    let mut gps = Sensor::new(
        "gps",
        Gps::new()
            .with_noise_position(NoiseModels::Gaussian(GaussianNoise::new(0.0, 50.0)))
            .with_noise_velocity(NoiseModels::Gaussian(GaussianNoise::new(0.0, 1.0))),
    );
    // Add a star tracker model
    let mut st = Sensor::new(
        "st",
        StarTracker::new().with_noise(NoiseModels::Gaussian(GaussianNoise::new(
            0.0,
            50.0 / 3600.0 * std::f64::consts::PI / 180.0,
        ))),
    );

    // Add a rate gyro model
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

    // Connect the components together.
    // Connections are made by the components names.
    // The direction of the connection matters - (from,to)

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

    rw1.connect_to_body(&bus, rw1_transform).unwrap();
    rw2.connect_to_body(&bus, rw2_transform).unwrap();
    rw3.connect_to_body(&bus, rw3_transform).unwrap();
    rw4.connect_to_body(&bus, rw4_transform).unwrap();

    let actuator_system = SpacecraftActuators {
        rw: [rw1, rw2, rw3, rw4],
    };

    // TODO: These connections are not the ideal interface and will be improved in the future
    base.borrow_mut().connect_outer_joint(&f).unwrap();
    f.borrow_mut()
        .connect_base(&base, Transform::IDENTITY)
        .unwrap();
    bus.borrow_mut().connect_inner_joint(&f).unwrap();
    f.borrow_mut()
        .connect_outer_body(&bus, Transform::IDENTITY)
        .unwrap();

    let bus_h1 = Transform::new(
        Rotation::IDENTITY,
        CoordinateSystem::Cartesian(Cartesian::new(0.0, 1.0, -1.0)),
    );
    bus.borrow_mut().connect_outer_joint(&hinge1).unwrap();
    hinge1
        .borrow_mut()
        .connect_inner_body(&bus, bus_h1)
        .unwrap();

    let h1_sa1 = Transform::new(
        Rotation::IDENTITY,
        CoordinateSystem::Cartesian(Cartesian::new(0.0, -1.0, 0.05)),
    );
    sa1.borrow_mut().connect_inner_joint(&hinge1).unwrap();
    hinge1
        .borrow_mut()
        .connect_outer_body(&sa1, h1_sa1)
        .unwrap();

    let sa1_h2 = Transform::new(
        Rotation::IDENTITY,
        CoordinateSystem::Cartesian(Cartesian::new(0.0, 1.0, -0.05)),
    );
    sa1.borrow_mut().connect_outer_joint(&hinge2).unwrap();
    hinge2
        .borrow_mut()
        .connect_inner_body(&sa1, sa1_h2)
        .unwrap();

    let h2_sa2 = Transform::new(
        Rotation::IDENTITY,
        CoordinateSystem::Cartesian(Cartesian::new(0.0, -1.0, -0.05)),
    );
    sa2.borrow_mut().connect_inner_joint(&hinge2).unwrap();
    hinge2
        .borrow_mut()
        .connect_outer_body(&sa2, h2_sa2)
        .unwrap();

    let sa2_h3 = Transform::new(
        Rotation::IDENTITY,
        CoordinateSystem::Cartesian(Cartesian::new(0.0, 1.0, 0.05)),
    );
    sa2.borrow_mut().connect_outer_joint(&hinge3).unwrap();
    hinge3
        .borrow_mut()
        .connect_inner_body(&sa2, sa2_h3)
        .unwrap();

    let h3_sa3 = Transform::new(
        Rotation::IDENTITY,
        CoordinateSystem::Cartesian(Cartesian::new(0.0, -1.0, 0.05)),
    );
    sa3.borrow_mut().connect_inner_joint(&hinge3).unwrap();
    hinge3
        .borrow_mut()
        .connect_outer_body(&sa3, h3_sa3)
        .unwrap();

    gps.connect_to_body(&bus, Transform::IDENTITY).unwrap();
    st.connect_to_body(&bus, Transform::IDENTITY).unwrap();
    imu.connect_to_body(&bus, Transform::IDENTITY).unwrap();

    let sensor_system = SpacecraftSensors { gps, st, imu };

    let software = SpacecraftFsw::default();

    // Create the system
    let mut sys = MultibodySystem::new(
        base,
        [bus, sa1, sa2, sa3],
        [f, hinge1, hinge2, hinge3],
        sensor_system,
        software,
        actuator_system,
    );
    // Run the simulation
    sys.simulate("", 0.0, 10000.0, 1.0);
}
