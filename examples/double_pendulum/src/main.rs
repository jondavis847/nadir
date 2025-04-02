use color::Color;
use mass_properties::MassPropertiesBuilder;
use multibody::{joint::revolute::RevoluteBuilder, system::MultibodySystemBuilder};
use rotations::Rotation;
use transforms::{prelude::Cartesian, Transform};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create a new system
    let mut sys = MultibodySystemBuilder::new();
    // Add some constant gravity to the frame
    sys.set_gravity_constant(0.0, 0.0, -9.8)?;

    // Create the bodies
    let mut b1 = sys.new_body("b1")?;
    b1.set_mass_properties(MassPropertiesBuilder::new());
    b1.set_geometry_cuboid(0.1, 0.1, 1.0)?;
    b1.set_material_phong(Color::RED, 32.0);

    let mut b2 = sys.new_body("b2")?;
    b2.set_mass_properties(MassPropertiesBuilder::new());
    b2.set_geometry_cuboid(0.1, 0.1, 1.0)?;
    b2.set_material_phong(Color::BLUE, 32.0);

    // Create the joints

    let r1 = RevoluteBuilder::new().with_angle(1.0);
    let mut j1 = sys.new_joint("r1", r1.into())?;

    let r2 = RevoluteBuilder::new().with_angular_rate(0.1);
    let mut j2 = sys.new_joint("r2", r2.into())?;

    // connect the system
    // we need transforms to place the joint frames (hinges) at the right place in the bodies
    let top = Transform::new(Rotation::IDENTITY, Cartesian::new(0.0, 0.0, 0.5).into());
    let bottom = Transform::new(Rotation::IDENTITY, Cartesian::new(0.0, 0.0, -0.5).into());

    sys.base.connect_outer_joint(&mut j1, Transform::IDENTITY)?;
    b1.connect_inner_joint(&mut j1, top)?;
    b1.connect_outer_joint(&mut j2, bottom)?;
    b2.connect_inner_joint(&mut j2, top)?;

    sys.add_body(b1);
    sys.add_body(b2);
    sys.add_joint(j1);
    sys.add_joint(j2);

    // run the simulation
    sys.simulate("", 0.0, 20.0, 0.1, None)?;

    Ok(())
}
