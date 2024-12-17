use color::Color;
use mass_properties::{CenterOfMass, Inertia, MassProperties};
use multibody::{
    body::Body,
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
    // Create the MultibodySystem
    // sys is will be created with nothing but a default base
    let mut sys = MultibodySystem::new();
    let state = FloatingState::new()
        .with_rates([1.0, 0.0, 1.0].into())
        .with_velocity([0.0, 1.0, 0.0].into());

    let parameters = FloatingParameters::new();
    let f_model = Floating::new(parameters, state);
    let f = Joint::new("f", f_model).unwrap();
    sys.add_joint(f).unwrap();

    // Create the main body of the spacecraft
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

    let b = Body::new("b", mp).unwrap().with_mesh(mesh);
    sys.add_body(b).unwrap();

    // Connect the components together.
    // Connections are made by the components names.
    // The direction of the connection matters - (from,to)
    sys.connect("base", "f", Transform::IDENTITY).unwrap();
    sys.connect("f", "b", Transform::IDENTITY).unwrap();
    // Run the simulation
    sys.simulate("", 0.0, 10.0, 0.1);
}
