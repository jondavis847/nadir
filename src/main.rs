use multibody::{
    base::Base,
    body::{Body, BodyTrait},
    joint::{
        revolute::{Revolute, RevoluteState},
        JointParameters, JointTrait,
    },    
    MultibodySystem,
};
use mass_properties::{CenterOfMass,Inertia,MassProperties};

use transforms::Transform;

fn main() {
    let mut sys = MultibodySystem::new();

    let mut base = Base::new("base").unwrap();
    sys.add_body(base.clone());

    let mut joint1 = Revolute::new(
        "joint1",
        JointParameters::default(),
        RevoluteState::default(),
    );
    sys.add_joint(joint1.clone());

    let mut joint2 = Revolute::new(
        "joint2",
        JointParameters::default(),
        RevoluteState::default(),
    );
    sys.add_joint(joint2.clone());

    let mp = MassProperties::new(1.0, CenterOfMass::new(0.0, 0.0, 0.0), Inertia::new(1.0, 1.0, 1.0, 0.0, 0.0, 0.0).unwrap());
    let mut body1 = Body::new("body1", mp.unwrap()).unwrap();
    sys.add_body(body1.clone());

    let mp = MassProperties::new(1.0, CenterOfMass::new(0.0, 0.0, 0.0), Inertia::new(1.0, 1.0, 1.0, 0.0, 0.0, 0.0).unwrap());
    let mut body2 = Body::new("body2", mp.unwrap()).unwrap();
    sys.add_body(body2.clone());

    dbg!(&sys);

    base.connect_outer_joint(joint2.clone());
    joint2.connect_inner_body(base.clone(), Transform::default());
    body2.connect_inner_joint(joint2.clone());
    joint2.connect_outer_body(body2.clone(), Transform::default());
    body2.connect_outer_joint(joint1.clone());
    joint1.connect_inner_body(body2.clone(), Transform::default());
    body1.connect_inner_joint(joint1.clone());
    joint1.connect_outer_body(body1.clone(), Transform::default());

    sys.sort();
    dbg!(&sys);
}
