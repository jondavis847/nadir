use std::{cell::RefCell, error::Error, f64::consts::PI, rc::Rc};

use aerospace::orbit::KeplerianElements;
use celestial::{CelestialBodies, CelestialBody, CelestialSystem};
use color::Color;
use gravity::{
    egm::{EgmGravity, EgmModel},
    Gravity,
};
use hardware::{
    actuators::{reaction_wheel::ReactionWheel, SpacecraftActuators},
    sensors::{
        gps::Gps, magnetometer::Magnetometer, rate_gyro::RateGyro, star_tracker::StarTracker,
        SpacecraftSensors,
    },
};
use magnetics::{igrf::Igrf, MagneticField};
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

fn main() -> Result<(), Box<dyn Error>> {
    // Create the CelestialSystem which contains the planetary models
    // In NADIR the base is GCRF (J2000) when a CelestialSystem is present
    let epoch = Time::now()?;
    let earth = CelestialBody::new(CelestialBodies::Earth)
        .with_gravity(Gravity::Egm(EgmGravity::new(EgmModel::Egm2008, 7, 7)?))
        .with_magnetic_field(MagneticField::Igrf(Igrf::new(13, 13, &epoch)?));
    let celestial = CelestialSystem::new(epoch)?
        .with_body(earth)?
        .with_body(CelestialBody::new(CelestialBodies::Moon))?;

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
    let f = Rc::new(RefCell::new(Joint::new("f", f_model)?));

    // Create the main body of the spacecraft
    let cm = CenterOfMass::new(0.0, 0.0, 0.0);
    let inertia = Inertia::new(1000.0, 1000.0, 1000.0, 0.0, 0.0, 0.0)?;
    let mp = MassProperties::new(1000.0, cm, inertia)?;

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

    let bus = Rc::new(RefCell::new(Body::new("bus", mp)?.with_mesh(mesh)));

    // Create the solar array panels of the spacecraft
    //solar array 1
    let inertia = Inertia::new(100.0, 100.0, 100.0, 0.0, 0.0, 0.0)?;
    let mp = MassProperties::new(100.0, cm, inertia)?;
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

    let sa1 = Rc::new(RefCell::new(Body::new("sa1", mp)?.with_mesh(mesh)));

    //solar array 2
    let inertia = Inertia::new(100.0, 100.0, 100.0, 0.0, 0.0, 0.0)?;
    let mp = MassProperties::new(100.0, cm, inertia)?;
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

    let sa2 = Rc::new(RefCell::new(Body::new("sa2", mp)?.with_mesh(mesh)));

    //solar array 3
    let inertia = Inertia::new(100.0, 100.0, 100.0, 0.0, 0.0, 0.0)?;
    let mp = MassProperties::new(100.0, cm, inertia)?;
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

    let sa3 = Rc::new(RefCell::new(Body::new("sa3", mp)?.with_mesh(mesh)));

    // create solar array hinges
    // hinge 1
    let p = RevoluteParameters(JointParameters {
        constant_force: 0.0,
        damping: 100.0,
        equilibrium: 0.0,
        spring_constant: 10.0,
    });
    let x = RevoluteState::new(PI / 2.0, 0.0);
    let hinge1 = Rc::new(RefCell::new(Joint::new("hinge1", Revolute::new(p, x))?));

    // hinge 2
    let p = RevoluteParameters(JointParameters {
        constant_force: 0.0,
        damping: 100.0,
        equilibrium: 0.0,
        spring_constant: 10.0,
    });
    let x = RevoluteState::new(-3.14, 0.0);
    let hinge2 = Rc::new(RefCell::new(Joint::new("hinge2", Revolute::new(p, x))?));

    // hinge 3
    let p = RevoluteParameters(JointParameters {
        constant_force: 0.0,
        damping: 100.0,
        equilibrium: 0.0,
        spring_constant: 10.0,
    });
    let x = RevoluteState::new(3.14, 0.0);
    let hinge3 = Rc::new(RefCell::new(Joint::new("hinge3", Revolute::new(p, x))?));

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

    // Add the magnetometer
    let mut mag = Sensor::new(
        "mag",
        Magnetometer::new().with_noise(NoiseModels::Gaussian(GaussianNoise::new(0.0, 100.0))),
    );

    // Create the reaction wheels
    let mut rw1 = Actuator::new("rw1", ReactionWheel::new(0.25, 0.5, 0.0)?);
    let mut rw2 = Actuator::new("rw2", ReactionWheel::new(0.25, 0.5, 0.0)?);
    let mut rw3 = Actuator::new("rw3", ReactionWheel::new(0.25, 0.5, 0.0)?);
    let mut rw4 = Actuator::new("rw4", ReactionWheel::new(0.25, 0.5, 0.0)?);

    // Connect the components together.
    // Connections are made by the components names.
    // The direction of the connection matters - (from,to)

    let rw1_transform = Transform::new(
        Rotation::from(&AlignedAxes::new(
            AxisPair::new(Axis::Zp, Axis::Xp),
            AxisPair::new(Axis::Yp, Axis::Yp),
        )?),
        CoordinateSystem::ZERO,
    );
    let rw2_transform = Transform::new(
        Rotation::from(&AlignedAxes::new(
            AxisPair::new(Axis::Zp, Axis::Yp),
            AxisPair::new(Axis::Xp, Axis::Xp),
        )?),
        CoordinateSystem::ZERO,
    );

    // wheel axis default is z axis, so no rotation needed
    let rw3_transform = Transform::new(Rotation::IDENTITY, CoordinateSystem::ZERO);

    // not using rw4 yet. zero'd out in calcs
    let rw4_transform = Transform::new(Rotation::IDENTITY, CoordinateSystem::ZERO);

    rw1.connect_to_body(&bus, rw1_transform)?;
    rw2.connect_to_body(&bus, rw2_transform)?;
    rw3.connect_to_body(&bus, rw3_transform)?;
    rw4.connect_to_body(&bus, rw4_transform)?;

    let actuator_system = SpacecraftActuators {
        rw: [rw1, rw2, rw3, rw4],
    };

    // TODO: These connections are not the ideal interface and will be improved in the future
    base.borrow_mut().connect_outer_joint(&f)?;
    f.borrow_mut().connect_base(&base, Transform::IDENTITY)?;
    bus.borrow_mut().connect_inner_joint(&f)?;
    f.borrow_mut()
        .connect_outer_body(&bus, Transform::IDENTITY)?;

    let bus_h1 = Transform::new(
        Rotation::IDENTITY,
        CoordinateSystem::Cartesian(Cartesian::new(0.0, 1.0, -1.0)),
    );
    bus.borrow_mut().connect_outer_joint(&hinge1)?;
    hinge1.borrow_mut().connect_inner_body(&bus, bus_h1)?;

    let h1_sa1 = Transform::new(
        Rotation::IDENTITY,
        CoordinateSystem::Cartesian(Cartesian::new(0.0, -1.0, 0.05)),
    );
    sa1.borrow_mut().connect_inner_joint(&hinge1)?;
    hinge1.borrow_mut().connect_outer_body(&sa1, h1_sa1)?;

    let sa1_h2 = Transform::new(
        Rotation::IDENTITY,
        CoordinateSystem::Cartesian(Cartesian::new(0.0, 1.0, -0.05)),
    );
    sa1.borrow_mut().connect_outer_joint(&hinge2)?;
    hinge2.borrow_mut().connect_inner_body(&sa1, sa1_h2)?;

    let h2_sa2 = Transform::new(
        Rotation::IDENTITY,
        CoordinateSystem::Cartesian(Cartesian::new(0.0, -1.0, -0.05)),
    );
    sa2.borrow_mut().connect_inner_joint(&hinge2)?;
    hinge2.borrow_mut().connect_outer_body(&sa2, h2_sa2)?;

    let sa2_h3 = Transform::new(
        Rotation::IDENTITY,
        CoordinateSystem::Cartesian(Cartesian::new(0.0, 1.0, 0.05)),
    );
    sa2.borrow_mut().connect_outer_joint(&hinge3)?;
    hinge3.borrow_mut().connect_inner_body(&sa2, sa2_h3)?;

    let h3_sa3 = Transform::new(
        Rotation::IDENTITY,
        CoordinateSystem::Cartesian(Cartesian::new(0.0, -1.0, 0.05)),
    );
    sa3.borrow_mut().connect_inner_joint(&hinge3)?;
    hinge3.borrow_mut().connect_outer_body(&sa3, h3_sa3)?;

    gps.connect_to_body(&bus, Transform::IDENTITY)?;
    st.connect_to_body(&bus, Transform::IDENTITY)?;
    imu.connect_to_body(&bus, Transform::IDENTITY)?;
    mag.connect_to_body(&bus, Transform::IDENTITY)?;

    let sensor_system = SpacecraftSensors { gps, st, imu, mag };

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
    sys.simulate("", 0.0, 1000.0, 0.1);

    Ok(())
}
