use std::{cell::RefCell, rc::Rc};

use color::Color;
use mass_properties::{CenterOfMass, Inertia, MassProperties};
use multibody::{
    base::Base,
    body::{Body, BodyTrait},
    joint::{
        floating::{Floating, FloatingParameters, FloatingState},
        Joint,
    },
    system::MultibodySystem,
};

use nadir_3d::{
    geometry::{cuboid::Cuboid, Geometry, GeometryState},
    material::Material,
    mesh::Mesh,
};
use transforms::Transform;

fn main() {
    let base = Base::new("base", multibody::base::BaseSystems::Basic(None));

    // create a floating joint
    let state = FloatingState::new()
        .with_rates([1.0, 0.0, 1.0].into())
        .with_velocity([0.0, 1.0, 0.0].into());

    let parameters = FloatingParameters::new();
    let f_model = Floating::new(parameters, state);
    let f = Rc::new(RefCell::new(Joint::new("f", f_model).unwrap()));

    // Create the a body
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

    // Connect the components together.
    // Connections are made by the components names.
    // The direction of the connection matters - (from,to)
    // base.borrow_mut().connect_outer_joint(&f);
    // f.borrow_mut().connect_base(&base,Transform::IDENTITY);
    // f.borrow_mut().connect_outer_body(&b,Transform::IDENTITY);
    // b.borrow_mut().connect_inner_joint(&f);
    base.borrow_mut().connect_outer_joint(&f).unwrap();
    f.borrow_mut()
        .connect_base(&base, Transform::IDENTITY)
        .unwrap();
    b.borrow_mut().connect_inner_joint(&f).unwrap();
    f.borrow_mut()
        .connect_outer_body(&b, Transform::IDENTITY)
        .unwrap();

    let mut sys = MultibodySystem::new(base, [b], [f], (), (), ());
    // Run the simulation
    sys.simulate("", 0.0, 10.0, 0.1);
}
