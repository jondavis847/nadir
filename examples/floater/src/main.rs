use color::Color;
use mass_properties::MassPropertiesBuilder;
use multibody::{
    joint::floating::FloatingBuilder,
    system::{MultibodySystem, MultibodySystemBuilder},
};
use nadir_diffeq::{
    OdeProblem, events::SaveEvent, saving::SaveMethod, solvers::Solver,
    stepping::AdaptiveStepControl,
};
use std::{env::current_dir, error::Error};
use transforms::Transform;

fn main() -> Result<(), Box<dyn Error>> {
    let mut sys = MultibodySystemBuilder::new();

    // create a floating joint
    let f = FloatingBuilder::new()
        .with_angular_rate(1.0, 1.0, 1.0)
        .with_velocity(0.0, 1.0, 0.0);
    let mut j = sys.new_joint("j", f.into())?;

    let mut b = sys.new_body("b")?;
    b.set_mass_properties(MassPropertiesBuilder::new());
    b.set_geometry_cuboid(1.0, 1.0, 1.0)?;
    b.set_material_phong(Color::RED, 32.0);

    sys.base.connect_outer_joint(&mut j, Transform::IDENTITY)?;
    b.connect_inner_joint(&mut j, Transform::IDENTITY)?;

    sys.add_body(b);
    sys.add_joint(j);

    let mut sys = sys.nominal()?;

    let mut problem = OdeProblem::new(sys)
        .with_saving(current_dir()?.join("results"))
        .with_save_event(SaveEvent::new(
            MultibodySystem::init_fn,
            MultibodySystem::save_fn,
        ));

    problem.solve_adaptive(
        &sys.initial_state(),
        (0.0, 10.0),
        AdaptiveStepControl::default(),
        Solver::Tsit5,
        SaveMethod::None,
    )?;

    // Run the simulation
    Ok(())
}
