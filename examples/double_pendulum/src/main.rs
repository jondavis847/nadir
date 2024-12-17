use aerospace::gravity::ConstantGravity;
use color::Color;
use mass_properties::{CenterOfMass, Inertia, MassProperties};
use multibody::{
    body::Body,
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
    // create the system
    let mut sys = MultibodySystem::new();

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
    let b1 = Body::new("b1", mp).unwrap().with_mesh(b1_mesh);

    // we can just clone b2 mutably and edit it
    let mut b2 = b1.clone();
    b2.name = "b2".to_string();
    if let Some(mesh) = &mut b2.mesh {
        mesh.name = "b2".to_string();
        match &mut mesh.material {
            Material::Phong {
                color,
                specular_power: _,
            } => *color = Color::BLUE,
            _ => unreachable!("we made it phong"),
        }
    }
    sys.add_body(b1).unwrap();
    sys.add_body(b2).unwrap();

    // create the revolute joints
    let r1_state = RevoluteState::new(1.0, 0.0);
    let r1_parameters = RevoluteParameters::default();
    let r1_model = Revolute::new(r1_parameters, r1_state);
    let r1 = Joint::new("r1", r1_model).unwrap();
    sys.add_joint(r1).unwrap();

    let r2_state = RevoluteState::new(0.0, 0.1);
    let r2_parameters = RevoluteParameters::default();
    let r2_model = Revolute::new(r2_parameters, r2_state);
    let r2 = Joint::new("r2", r2_model).unwrap();
    sys.add_joint(r2).unwrap();

    // connect the system
    // we need transforms to place the joint frames (hinges) at the right place in the bodies
    let top = Transform::new(Rotation::IDENTITY, Cartesian::new(0.0, 0.0, 0.5).into());
    let bottom = Transform::new(Rotation::IDENTITY, Cartesian::new(0.0, 0.0, -0.5).into());
    sys.connect("base", "r1", Transform::IDENTITY).unwrap();
    sys.connect("r1", "b1", top.clone()).unwrap();
    sys.connect("b1", "r2", bottom.clone()).unwrap();
    sys.connect("r2", "b2", top.clone()).unwrap();

    // need gravity for the pendulum to move!
    sys.add_gravity(ConstantGravity::new(0.0, 0.0, -9.8).into())
        .unwrap();

    // run the simulation
    sys.simulate("", 0.0, 20.0, 0.1);
}
