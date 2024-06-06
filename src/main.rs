use multibody::{
    base::Base,
    body::{Body, BodyTrait},
    joint::{
        revolute::{Revolute, RevoluteState},
        JointParameters,
    },
    mass_properties::MassProperties,
    MultibodySystem,
};

use transforms::Transform;

fn main() {
    let mut sys = MultibodySystem::new();

    let base = Base::new("base").unwrap();
    sys.add_body(base);

    let joint = Revolute::new(
        "joint",
        JointParameters::default(),
        RevoluteState::default(),
    );
    sys.add_joint(joint.clone());

    let mp = MassProperties::new(1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0);
    let mut body = Body::new("body", mp.unwrap()).unwrap();
    sys.add_body(body.clone());

    body.connect_inner_joint(joint, Transform::default());

    dbg!(sys);
}
