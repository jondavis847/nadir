//mod software;

use std::{error::Error, f64::consts::PI};

use aerospace::orbit::KeplerianElements;
use celestial::{CelestialBodies, CelestialBody, CelestialSystem};
use color::Color;
use gravity::{
    egm::{EgmGravity, EgmModel},
    Gravity,
};
use magnetics::{igrf::Igrf, MagneticField};
use mass_properties::MassPropertiesBuilder;
use multibody::{
    joint::{floating::FloatingBuilder, revolute::RevoluteBuilder, JointBuilder},
    system::MultibodySystemBuilder,
};
use rotations::prelude::{Quaternion, UnitQuaternion};
use time::Time;

fn main() -> Result<(), Box<dyn Error>> {
    let mut sys = MultibodySystemBuilder::new();

    // Create the CelestialSystem which contains the planetary models
    // In NADIR the base is GCRF (J2000) when a CelestialSystem is present
    let epoch = Time::now()?;
    let earth = CelestialBody::new(CelestialBodies::Earth)
        .with_gravity(Gravity::Egm(
            EgmGravity::new(EgmModel::Egm2008, 7, 7)?.with_newtonian(),
        ))
        .with_magnetic_field(MagneticField::Igrf(Igrf::new(13, 13, &epoch)?));
    let celestial = CelestialSystem::new(epoch)?
        .with_body(earth)?
        .with_body(CelestialBody::new(CelestialBodies::Moon))?;
    sys.base.set_celestial(celestial);

    // Create the Floating joint that represents the kinematics between the base and the spacecraft
    // A with_orbit() method is provided for Floating joints
    let orbit = KeplerianElements::new(8e6, 0.0, 0.5, 0.0, 0.0, 0.0, epoch, CelestialBodies::Earth);
    let f = FloatingBuilder::new()
        .with_attitude(UnitQuaternion::new(-0.4, -0.5, 0.5, 0.5))
        .with_orbit(orbit.into());
    let j = sys.new_joint("f", f.into())?;

    // Create the main body of the spacecraft
    let mut bus = sys.new_body("bus")?;
    bus.set_mass_properties(
        MassPropertiesBuilder::new()
            .with_mass(1000.0)?
            .with_ixx(1000.0)?
            .with_iyy(1000.0)?
            .with_izz(1000.0)?,
    );
    bus.set_geometry_cuboid(3.0, 2.0, 2.0)?;
    bus.set_material_phong(Color::new(0.5, 0.5, 0.5, 1.0), 32.0);

    // Create the solar array panels of the spacecraft
    // solar array 1
    let mut sa1 = sys.new_body("sa1")?;
    sa1.set_mass_properties(
        MassPropertiesBuilder::new()
            .with_mass(100.0)?
            .with_ixx(100.0)?
            .with_iyy(100.0)?
            .with_izz(100.0)?,
    );
    sa1.set_geometry_cuboid(2.0, 2.0, 0.1)?;
    sa1.set_material_phong(Color::new(0.05, 0.05, 0.05, 1.0), 32.0);

    // solar array 2
    let mut sa2 = sys.new_body("sa2")?;
    sa2.set_mass_properties(
        MassPropertiesBuilder::new()
            .with_mass(100.0)?
            .with_ixx(100.0)?
            .with_iyy(100.0)?
            .with_izz(100.0)?,
    );
    sa2.set_geometry_cuboid(2.0, 2.0, 0.1)?;
    sa2.set_material_phong(Color::new(0.05, 0.05, 0.05, 1.0), 32.0);

    // solar array 3
    let mut sa3 = sys.new_body("sa3")?;
    sa3.set_mass_properties(
        MassPropertiesBuilder::new()
            .with_mass(100.0)?
            .with_ixx(100.0)?
            .with_iyy(100.0)?
            .with_izz(100.0)?,
    );
    sa3.set_geometry_cuboid(2.0, 2.0, 0.1)?;
    sa3.set_material_phong(Color::new(0.05, 0.05, 0.05, 1.0), 32.0);

    // create solar array hinges
    // hinge 1
    let r1 = RevoluteBuilder::new()
        .with_angle(PI / 2.0)
        .with_damping(100.0)
        .with_spring_constant(10.0);
    let mut h1 = sys.new_joint("hinge1", r1.into())?;

    // hinge 1
    let r2 = RevoluteBuilder::new()
        .with_angle(PI / 2.0)
        .with_damping(100.0)
        .with_spring_constant(10.0);
    let mut h2 = sys.new_joint("hinge2", r2.into())?;

    // hinge 1
    let r3 = RevoluteBuilder::new()
        .with_angle(PI / 2.0)
        .with_damping(100.0)
        .with_spring_constant(10.0);
    let mut h3 = sys.new_joint("hinge3", r3.into())?;

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
