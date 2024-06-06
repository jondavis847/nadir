use multibody::{
    base::Base,
    body::{Body, BodyTrait},
    joint::{
        revolute::{Revolute, RevoluteState},
        JointParameters,
        JointTrait,
    },
    mass_properties::MassProperties,
    MultibodySystem,
};

use transforms::Transform;

fn main() {
    let mut sys = MultibodySystem::new();

    let mut base = Base::new("base").unwrap();
    sys.add_body(base.clone());

    let mut joint = Revolute::new(
        "joint",
        JointParameters::default(),
        RevoluteState::default(),
    );
    sys.add_joint(joint.clone());

    let mp = MassProperties::new(1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0);
    let mut body = Body::new("body", mp.unwrap()).unwrap();
    sys.add_body(body.clone());

    base.connect_outer_joint(joint.clone(),Transform::default());
    joint.connect_inner_body(base.clone());
    body.connect_inner_joint(joint.clone(), Transform::default());
    joint.connect_outer_body(body.clone());
    
    dbg!(&sys);
    sys.validate();
}
