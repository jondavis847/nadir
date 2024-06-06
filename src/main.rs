use multibody::{
    base::Base,
    body::Body,
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

    let base = Base::new("base");
    sys.add_body(base);

    let joint = Revolute::new(
        "joint",
        JointParameters::default(),
        RevoluteState::<f64>::default(),
    );
    sys.add_joint(joint);

    let mp = MassProperties::new(1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0);
    let body = Body::new("body", mp.unwrap());
    sys.add_body(body);

    sys.connect(
        "joint",
        "base",
        Transform::default(),
        "body",
        Transform::default(),
    );

    dbg!(sys);
}
