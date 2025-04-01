use color::Color;
use mass_properties::MassPropertiesBuilder;
use multibody::system::MultibodySystemBuilder;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create a new system
    let mut sys = MultibodySystemBuilder::new();
    // Add some constant gravity to the frame
    sys.set_gravity_constant(0.0, 0.0, -9.8)?;

    // Create the bodies
    let mut b1 = sys.new_body("b1")?;
    b1.set_mass_properties(MassPropertiesBuilder::new());
    b1.set_geometry_cuboid(0.1,0.1,1.0);
    b1.set_material_phong(Color::RED, 32.0);

    let mut b2 = sys.new_body("b2")?;
    b2.set_mass_properties(MassPropertiesBuilder::new());
    b2.set_geometry_cuboid(0.1,0.1,1.0);
    b2.set_material_phong(Color::BLUE, 32.0);

    // Create the joints



    let r1 = sys.new_revolute("r1")?;
    r1.model.
    // create the revolute joints
    let r1_state = RevoluteState::new(1.0, 0.0);
    let r1_parameters = RevoluteParameters::default();
    let r1_model = Revolute::new(r1_parameters, r1_state);
    let r1 = Rc::new(RefCell::new(Joint::new("r1", r1_model).unwrap()));

    let r2_state = RevoluteState::new(0.0, 0.1);
    let r2_parameters = RevoluteParameters::default();
    let r2_model = Revolute::new(r2_parameters, r2_state);
    let r2 = Rc::new(RefCell::new(Joint::new("r2", r2_model).unwrap()));

    // connect the system
    // we need transforms to place the joint frames (hinges) at the right place in the bodies
    let top = Transform::new(Rotation::IDENTITY, Cartesian::new(0.0, 0.0, 0.5).into());
    let bottom = Transform::new(Rotation::IDENTITY, Cartesian::new(0.0, 0.0, -0.5).into());

    base.borrow_mut().connect_outer_joint(&r1).unwrap();
    r1.borrow_mut()
        .connect_base(&base, Transform::IDENTITY)
        .unwrap();
    r1.borrow_mut()
        .connect_outer_body(&b1, top.clone())
        .unwrap();
    b1.borrow_mut().connect_inner_joint(&r1).unwrap();
    b1.borrow_mut().connect_outer_joint(&r2).unwrap();
    r2.borrow_mut()
        .connect_inner_body(&b1, bottom.clone())
        .unwrap();
    r2.borrow_mut()
        .connect_outer_body(&b2, top.clone())
        .unwrap();
    b2.borrow_mut().connect_inner_joint(&r2).unwrap();

    let mut sys = MultibodySystem::new(base, [b1, b2], [r1, r2], (), (), ());
    // run the simulation
    sys.simulate("", 0.0, 20.0, 0.1);
}
