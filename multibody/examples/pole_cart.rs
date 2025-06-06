use std::env::current_dir;

use color::Color;
use mass_properties::MassPropertiesBuilder;
use multibody::{
    actuator::{ActuatorBuilder, thruster::ThrusterBuilder},
    joint::{prismatic::PrismaticBuilder, revolute::RevoluteBuilder},
    system::{MultibodySystem, MultibodySystemBuilder},
};
use nadir_diffeq::{
    OdeProblem,
    events::{PostSimEvent, SaveEvent},
    saving::SaveMethod,
    solvers::Solver,
};
use rotations::{
    Rotation,
    prelude::{AlignedAxes, Axis, AxisPair},
};
use transforms::{Transform, prelude::Cartesian};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create a new system
    let mut sys = MultibodySystemBuilder::new();
    // Add some constant gravity to the frame
    sys.set_gravity_constant(0.0, 0.0, -9.8)?;

    // Create the bodies
    let mut cart = sys.new_body("cart")?;
    cart.set_mass_properties(MassPropertiesBuilder::new());
    cart.set_geometry_cuboid(1.0, 2.0, 1.0)?;
    cart.set_material_phong(Color::RED, 32.0);

    let mut pole = sys.new_body("pole")?;
    pole.set_mass_properties(MassPropertiesBuilder::new());
    pole.set_geometry_cuboid(0.1, 0.1, 2.0)?;
    pole.set_material_phong(Color::BLUE, 32.0);

    // Create the joints
    let mut hinge = sys.new_joint(
        "hinge",
        RevoluteBuilder::new().with_angle(0.05).into(),
    )?;
    let mut prismatic = sys.new_joint("prismatic", PrismaticBuilder::new().into())?;

    // Create the actuators
    let mut left_thruster =
        ActuatorBuilder::new("left_thruster", ThrusterBuilder::new(1.0)?.into());
    let mut right_thruster = ActuatorBuilder::new(
        "right_thruster",
        ThrusterBuilder::new(1.0)?.into(),
    );

    // connect the system
    sys.base
        .connect_outer_joint(&mut prismatic, Transform::IDENTITY)?;
    cart.connect_inner_joint(
        &mut prismatic,
        Transform::new(
            Rotation::IDENTITY,
            Cartesian::new(0.0, 0.0, -0.5).into(),
        ),
    )?;
    cart.connect_outer_joint(
        &mut hinge,
        Transform::new(
            Rotation::IDENTITY,
            Cartesian::new(0.0, 0.0, 0.5).into(),
        ),
    )?;
    pole.connect_inner_joint(
        &mut hinge,
        Transform::new(
            Rotation::IDENTITY,
            Cartesian::new(0.0, 0.0, -1.0).into(),
        ),
    )?;

    left_thruster.connect_body(
        cart.id,
        Transform::new(
            Rotation::from(&AlignedAxes::new(
                AxisPair::new(Axis::Xp, Axis::Yp),
                AxisPair::new(Axis::Zp, Axis::Zp),
            )?),
            Cartesian::new(0.0, -1.0, 0.0).into(),
        ),
    );

    right_thruster.connect_body(
        cart.id,
        Transform::new(
            Rotation::from(&AlignedAxes::new(
                AxisPair::new(Axis::Xp, Axis::Yn),
                AxisPair::new(Axis::Zp, Axis::Zp),
            )?),
            Cartesian::new(0.0, 1.0, 0.0).into(),
        ),
    );

    sys.add_body(cart);
    sys.add_body(pole);
    sys.add_joint(hinge);
    sys.add_joint(prismatic);
    sys.add_actuator(left_thruster);
    sys.add_actuator(right_thruster);

    // run the simulation
    let mut sys = sys.nominal()?;
    let x0 = sys.initial_state();

    // run the simulation
    let mut problem = OdeProblem::new(sys)
        .with_saving(current_dir()?.join("results"))
        .with_save_event(SaveEvent::new(
            MultibodySystem::init_fn,
            MultibodySystem::save_fn,
        ))
        .with_postsim_event(PostSimEvent::new(MultibodySystem::post_sim_fn));

    problem.solve_fixed(
        &x0,
        (0.0, 10.0),
        0.1,
        Solver::Tsit5,
        SaveMethod::None,
    )?;

    Ok(())
}
