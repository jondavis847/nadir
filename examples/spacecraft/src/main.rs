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
    actuator::{reaction_wheel::ReactionWheelBuilder, ActuatorBuilder},
    joint::{floating::FloatingBuilder, revolute::RevoluteBuilder},
    sensor::{
        gps::GpsBuilder, magnetometer::MagnetometerBuilder, rate_gyro::RateGyroBuilder,
        star_tracker::StarTrackerBuilder, SensorBuilder,
    },
    system::MultibodySystemBuilder,
};
use rotations::{
    prelude::{AlignedAxes, Axis, AxisPair, UnitQuaternion},
    Rotation,
};
use time::Time;
use transforms::{
    prelude::{Cartesian, CoordinateSystem},
    Transform,
};

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
    let mut j = sys.new_joint("f", f.into())?;

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

    // hinge 2
    let r2 = RevoluteBuilder::new()
        .with_angle(-PI)
        .with_damping(100.0)
        .with_spring_constant(10.0);
    let mut h2 = sys.new_joint("hinge2", r2.into())?;

    // hinge 3
    let r3 = RevoluteBuilder::new()
        .with_angle(PI)
        .with_damping(100.0)
        .with_spring_constant(10.0);
    let mut h3 = sys.new_joint("hinge3", r3.into())?;

    // Add a GPS model
    let mut gps = SensorBuilder::new(
        "gps",
        GpsBuilder::new()
            .with_noise_position_normal(0.0, 50.0 / 3.0)
            .with_noise_velocity_normal(0.0, 1.0 / 3.0)
            .into(),
    );
    // Add a star tracker model
    let mut st = SensorBuilder::new("st", StarTrackerBuilder::new().into());

    // Add a rate gyro model
    let mut imu = SensorBuilder::new(
        "imu",
        RateGyroBuilder::new()
            .with_noise_normal(0.0, 1.0e-3 * PI / 180.0)
            .into(),
    );

    // Add the magnetometer
    let mut mag = SensorBuilder::new(
        "mag",
        MagnetometerBuilder::new()
            .with_noise_normal(0.0, 100.0)
            .into(),
    );

    // Create the reaction wheels
    let mut rw1 = ActuatorBuilder::new("rw1", ReactionWheelBuilder::new(0.25, 0.5)?.into());
    let mut rw2 = ActuatorBuilder::new("rw2", ReactionWheelBuilder::new(0.25, 0.5)?.into());
    let mut rw3 = ActuatorBuilder::new("rw3", ReactionWheelBuilder::new(0.25, 0.5)?.into());
    let mut rw4 = ActuatorBuilder::new("rw4", ReactionWheelBuilder::new(0.25, 0.5)?.into());

    // Connect the components together.
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

    rw1.connect_body(bus.id, rw1_transform);
    rw2.connect_body(bus.id, rw2_transform);
    rw3.connect_body(bus.id, rw3_transform);
    rw4.connect_body(bus.id, rw4_transform);

    // TODO: These connections are not the ideal interface and will be improved in the future
    sys.base.connect_outer_joint(&mut j, Transform::IDENTITY)?;
    bus.connect_inner_joint(&mut j, Transform::IDENTITY)?;
    bus.connect_outer_joint(
        &mut h1,
        Transform::new(Rotation::IDENTITY, Cartesian::new(0.0, 1.0, -1.0).into()),
    )?;
    sa1.connect_inner_joint(
        &mut h1,
        Transform::new(Rotation::IDENTITY, Cartesian::new(0.0, -1.0, 0.05).into()),
    )?;
    sa1.connect_outer_joint(
        &mut h2,
        Transform::new(Rotation::IDENTITY, Cartesian::new(0.0, 1.0, -0.05).into()),
    )?;

    sa2.connect_inner_joint(
        &mut h2,
        Transform::new(Rotation::IDENTITY, Cartesian::new(0.0, -1.0, -0.05).into()),
    )?;

    sa2.connect_outer_joint(
        &mut h3,
        Transform::new(Rotation::IDENTITY, Cartesian::new(0.0, 1.0, 0.05).into()),
    )?;

    sa3.connect_inner_joint(
        &mut h3,
        Transform::new(Rotation::IDENTITY, Cartesian::new(0.0, -1.0, 0.05).into()),
    )?;

    gps.connect_body(bus.id, Transform::IDENTITY)?;
    st.connect_body(bus.id, Transform::IDENTITY)?;
    imu.connect_body(bus.id, Transform::IDENTITY)?;
    mag.connect_body(bus.id, Transform::IDENTITY)?;

    sys.add_body(bus);
    sys.add_body(sa1);
    sys.add_body(sa2);
    sys.add_body(sa3);
    sys.add_joint(j);
    sys.add_joint(h1);
    sys.add_joint(h2);
    sys.add_joint(h3);
    sys.add_sensor(gps);
    sys.add_sensor(st);
    sys.add_sensor(imu);
    sys.add_sensor(mag);
    sys.add_actuator(rw1);
    sys.add_actuator(rw2);
    sys.add_actuator(rw3);
    sys.add_actuator(rw4);

    // Run the simulation
    sys.simulate("", 0.0, 1000.0, 1.0, Some(10))?;

    Ok(())
}
