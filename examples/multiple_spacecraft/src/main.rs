use aerospace::orbit::KeplerianElements;
use celestial::{CelestialBodies, CelestialBodyBuilder, CelestialSystemBuilder};
use color::Color;
use gravity::{
    Gravity,
    egm::{EgmGravity, EgmModel},
};
use magnetics::{MagneticField, igrf::Igrf};
use mass_properties::MassPropertiesBuilder;
use multibody::{
    actuator::{ActuatorBuilder, reaction_wheel::ReactionWheelBuilder},
    joint::floating::FloatingBuilder,
    sensor::{
        SensorBuilder, gps::GpsBuilder, magnetometer::MagnetometerBuilder,
        rate_gyro::RateGyroBuilder, star_tracker::StarTrackerBuilder,
    },
    software::Software,
    system::{MultibodySystem, MultibodySystemBuilder},
};
use nadir_diffeq::{
    OdeProblem,
    events::{PeriodicEvent, PostSimEvent, SaveEvent},
    solvers::OdeSolver,
    state::state_vector::StateVector,
    stepping::AdaptiveStepControl,
};
use rotations::{
    Rotation,
    prelude::{AlignedAxes, Axis, AxisPair},
};
use std::{env::current_dir, error::Error, path::Path};
use time::Time;
use transforms::{Transform, prelude::CoordinateSystem};

fn main() -> Result<(), Box<dyn Error>> {
    let mut sys = MultibodySystemBuilder::new();

    // Create the CelestialSystem which contains the planetary models
    // In NADIR the base is GCRF (J2000) when a CelestialSystem is present
    let epoch = Time::now()?;

    let celestial = CelestialSystemBuilder::new(epoch)?
        .with_body(
            CelestialBodyBuilder::new(CelestialBodies::Earth)
                .with_gravity(Gravity::Egm(
                    EgmGravity::new(EgmModel::Egm2008, 7, 7)?.with_newtonian(),
                ))
                .with_magnetic_field(MagneticField::Igrf(
                    Igrf::new(13, 13, &epoch)?,
                )),
        )?
        .with_body(CelestialBodyBuilder::new(
            CelestialBodies::Moon,
        ))?;
    sys.base
        .set_celestial(celestial);

    // Spacecraft 1
    let orbit1 = KeplerianElements::new(
        8e6,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        epoch,
        CelestialBodies::Earth,
    );
    let f1 = FloatingBuilder::new().with_orbit(orbit1.into());

    let mut j1 = sys.new_joint("f1", f1.into())?;

    let mut sc1 = sys.new_body("sc1")?;
    sc1.set_mass_properties(
        MassPropertiesBuilder::new()
            .with_mass(10.0)?
            .with_ixx(10.0)?
            .with_iyy(10.0)?
            .with_izz(10.0)?,
    );
    sc1.set_geometry_cuboid(1.0, 0.5, 0.5)?;
    sc1.set_material_phong(
        Color::new(0.5, 0.5, 0.5, 1.0),
        32.0,
    );
    let mut gps1 = SensorBuilder::new(
        "gps1",
        GpsBuilder::new().into(),
    );
    let mut st1 = SensorBuilder::new(
        "st1",
        StarTrackerBuilder::new().into(),
    );
    let mut imu1 = SensorBuilder::new(
        "imu1",
        RateGyroBuilder::new().into(),
    );
    let mut mag1 = SensorBuilder::new(
        "mag1",
        MagnetometerBuilder::new().into(),
    );
    let mut rw11 = ActuatorBuilder::new(
        "rw11",
        ReactionWheelBuilder::new(0.25, 0.5)?.into(),
    );
    let mut rw12 = ActuatorBuilder::new(
        "rw12",
        ReactionWheelBuilder::new(0.25, 0.5)?.into(),
    );
    let mut rw13 = ActuatorBuilder::new(
        "rw13",
        ReactionWheelBuilder::new(0.25, 0.5)?.into(),
    );
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
    let rw3_transform = Transform::new(
        Rotation::IDENTITY,
        CoordinateSystem::ZERO,
    );
    rw11.connect_body(sc1.id, rw1_transform);
    rw12.connect_body(sc1.id, rw2_transform);
    rw13.connect_body(sc1.id, rw3_transform);
    sys.base
        .connect_outer_joint(&mut j1, Transform::IDENTITY)?;
    sc1.connect_inner_joint(&mut j1, Transform::IDENTITY)?;

    gps1.connect_body(sc1.id, Transform::IDENTITY)?;
    st1.connect_body(sc1.id, Transform::IDENTITY)?;
    imu1.connect_body(sc1.id, Transform::IDENTITY)?;
    mag1.connect_body(sc1.id, Transform::IDENTITY)?;

    // Spacecraft 2
    let orbit2 = KeplerianElements::new(
        8e6,
        0.0,
        0.0,
        0.0,
        0.0,
        1e-6,
        epoch,
        CelestialBodies::Earth,
    );
    let f2 = FloatingBuilder::new().with_orbit(orbit2.into());
    let mut j2 = sys.new_joint("f2", f2.into())?;

    let mut sc2 = sys.new_body("sc2")?;
    sc2.set_mass_properties(
        MassPropertiesBuilder::new()
            .with_mass(10.0)?
            .with_ixx(10.0)?
            .with_iyy(10.0)?
            .with_izz(10.0)?,
    );
    sc2.set_geometry_cuboid(1.0, 0.5, 0.5)?;
    sc2.set_material_phong(
        Color::new(0.5, 0.5, 0.5, 1.0),
        32.0,
    );
    let mut gps2 = SensorBuilder::new(
        "gps2",
        GpsBuilder::new().into(),
    );
    let mut st2 = SensorBuilder::new(
        "st2",
        StarTrackerBuilder::new().into(),
    );
    let mut imu2 = SensorBuilder::new(
        "imu2",
        RateGyroBuilder::new().into(),
    );
    let mut mag2 = SensorBuilder::new(
        "mag2",
        MagnetometerBuilder::new().into(),
    );
    let mut rw21 = ActuatorBuilder::new(
        "rw21",
        ReactionWheelBuilder::new(0.25, 0.5)?.into(),
    );
    let mut rw22 = ActuatorBuilder::new(
        "rw22",
        ReactionWheelBuilder::new(0.25, 0.5)?.into(),
    );
    let mut rw23 = ActuatorBuilder::new(
        "rw23",
        ReactionWheelBuilder::new(0.25, 0.5)?.into(),
    );
    let rw21_transform = Transform::new(
        Rotation::from(&AlignedAxes::new(
            AxisPair::new(Axis::Zp, Axis::Xp),
            AxisPair::new(Axis::Yp, Axis::Yp),
        )?),
        CoordinateSystem::ZERO,
    );
    let rw22_transform = Transform::new(
        Rotation::from(&AlignedAxes::new(
            AxisPair::new(Axis::Zp, Axis::Yp),
            AxisPair::new(Axis::Xp, Axis::Xp),
        )?),
        CoordinateSystem::ZERO,
    );
    let rw23_transform = Transform::new(
        Rotation::IDENTITY,
        CoordinateSystem::ZERO,
    );
    rw21.connect_body(sc2.id, rw21_transform);
    rw22.connect_body(sc2.id, rw22_transform);
    rw23.connect_body(sc2.id, rw23_transform);
    sys.base
        .connect_outer_joint(&mut j2, Transform::IDENTITY)?;
    sc2.connect_inner_joint(&mut j2, Transform::IDENTITY)?;

    gps2.connect_body(sc2.id, Transform::IDENTITY)?;
    st2.connect_body(sc2.id, Transform::IDENTITY)?;
    imu2.connect_body(sc2.id, Transform::IDENTITY)?;
    mag2.connect_body(sc2.id, Transform::IDENTITY)?;

    sys.add_body(sc1);
    sys.add_joint(j1);
    sys.add_sensor(gps1);
    sys.add_sensor(st1);
    sys.add_sensor(imu1);
    sys.add_sensor(mag1);
    sys.add_actuator(rw11);
    sys.add_actuator(rw12);
    sys.add_actuator(rw13);

    sys.add_body(sc2);
    sys.add_joint(j2);
    sys.add_sensor(gps2);
    sys.add_sensor(st2);
    sys.add_sensor(imu2);
    sys.add_sensor(mag2);
    sys.add_actuator(rw21);
    sys.add_actuator(rw22);
    sys.add_actuator(rw23);

    // Add the software
    let lib_name = match std::env::consts::OS {
        "windows" => "multi_software.dll",
        "macos" => "libmulti_software.dylib",
        _ => "libmulti_software.so", //Linux
    };

    let lib_path = format!(
        "../../target/release/{}",
        lib_name
    );
    let lib_path_ref = Path::new(&lib_path);

    if !lib_path_ref.exists() {
        eprintln!(
            "Error: Dynamic library not found at {}\n\
             Please compile it first with: `cargo build --release --lib` in the src\\software crate.",
            lib_path
        );
        std::process::exit(1);
    }

    let software1 = Software::new("fsw1", lib_path_ref)
        .with_actuator_indices(vec![0, 1, 2])
        .with_sensor_indices(vec![0, 1, 2, 3]);

    let software2 = Software::new("fsw2", lib_path_ref)
        .with_actuator_indices(vec![3, 4, 5])
        .with_sensor_indices(vec![4, 5, 6, 7]);

    sys.add_software(software1);
    sys.add_software(software2);

    let problem = OdeProblem::new(sys.nominal()?)
        .with_periodic_event(PeriodicEvent::new(
            1.0,
            0.0,
            |sys: &mut MultibodySystem, _x: &mut StateVector, _t| {
                sys.software[0]
                    .step(
                        &sys.sensors,
                        &mut sys.actuators,
                    )
                    .unwrap();
            },
        ))
        .with_periodic_event(PeriodicEvent::new(
            1.0,
            0.0,
            |sys: &mut MultibodySystem, _x: &mut StateVector, _t| {
                sys.software[1]
                    .step(
                        &sys.sensors,
                        &mut sys.actuators,
                    )
                    .unwrap();
            },
        ))
        .with_saving(current_dir()?.join("results"))
        .with_save_event(SaveEvent::new(
            MultibodySystem::init_fn,
            MultibodySystem::save_fn,
        ))
        .with_postsim_event(PostSimEvent::new(
            MultibodySystem::post_sim_fn,
        ));
    let solver = OdeSolver::default();
    solver.solve_model_adaptive_mut(
        problem,
        (0.0, 1000.0),
        AdaptiveStepControl::default()
            .with_abs_tol(1e-10)
            .with_rel_tol(1e-10),
    )?;

    Ok(())
}
