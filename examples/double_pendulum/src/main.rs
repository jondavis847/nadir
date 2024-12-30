use std::{cell::RefCell, rc::Rc};

use aerospace::gravity::{ConstantGravity, Gravity};
use color::Color;
use mass_properties::{CenterOfMass, Inertia, MassProperties};
use multibody::{
    base::Base,
    body::{Body, BodyTrait},
    joint::{
        revolute::{Revolute, RevoluteParameters, RevoluteState},
        Joint,
    },
    system::MultibodySystem,
};
use nadir_3d::{
    geometry::{cuboid::Cuboid, Geometry, GeometryState},
    material::Material,
    mesh::Mesh,
};

use rotations::Rotation;
use transforms::{prelude::Cartesian, Transform};
fn main() {
    let base = Base::new(
        "base",
        multibody::base::BaseSystems::Basic(Some(Gravity::Constant(ConstantGravity::new(
            0.0, 0.0, -9.8,
        )))),
    );

    // create the revolute bodies/links
    let cm = CenterOfMass::new(0.0, 0.0, 0.0);
    let inertia = Inertia::new(1.0, 1.0, 1.0, 0.0, 0.0, 0.0).unwrap();
    let mp = MassProperties::new(1.0, cm, inertia).unwrap();
    let geometry = Geometry::Cuboid(Cuboid::new(0.1, 0.1, 1.0));
    let material = Material::Phong {
        color: Color::RED,
        specular_power: 32.0,
    };
    let b1_mesh = Mesh {
        name: "b1".to_string(),
        geometry,
        material,
        state: GeometryState::default(),
        texture: None,
    };
    let b1 = Rc::new(RefCell::new(
        Body::new("b1", mp).unwrap().with_mesh(b1_mesh),
    ));

    let cm = CenterOfMass::new(0.0, 0.0, 0.0);
    let inertia = Inertia::new(1.0, 1.0, 1.0, 0.0, 0.0, 0.0).unwrap();
    let mp = MassProperties::new(1.0, cm, inertia).unwrap();
    let geometry = Geometry::Cuboid(Cuboid::new(0.1, 0.1, 1.0));
    let material = Material::Phong {
        color: Color::BLUE,
        specular_power: 32.0,
    };
    let b2_mesh = Mesh {
        name: "b2".to_string(),
        geometry,
        material,
        state: GeometryState::default(),
        texture: None,
    };
    let b2 = Rc::new(RefCell::new(
        Body::new("b2", mp).unwrap().with_mesh(b2_mesh),
    ));

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
