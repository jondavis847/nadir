use multibody::{
    base::Base,
    body::Body,
    joint::{
        revolute::{Revolute, RevoluteState},
        Joint, JointParameters,
    },
    mass_properties::MassProperties,
    MultibodySystem, MultibodyTrait,
};

use transforms::Transform;

fn main() {
    let mut sys = MultibodySystem::new();

    let base = Base::new("base");
    sys.add_base(base);

    let joint = Joint::Revolute(Revolute::new(
        "joint",
        JointParameters::default(),
        RevoluteState::<f64>::default(),
    ));
    sys.add_joint(joint);

    let mp = MassProperties::new(1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0);
    let body = Body::new("body", mp.unwrap());
    sys.add_body(body);

    sys.connect("base","joint",Transform::default());
    sys.connect("joint","body",Transform::default());
    dbg!(sys);
}
