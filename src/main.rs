use mass_properties::{CenterOfMass, Inertia, MassProperties};
use multibody::{
    base::Base,
    body::Body,
    joint::{
        revolute::{Revolute, RevoluteState},
        JointParameters, JointTrait,
    },
    system::MultibodySystem,
    system_sim::MultibodySystemSim,
};

use transforms::Transform;

fn main() {
    let mut sys = MultibodySystem::new();

    let mut base = Base::new("base");
    let mut joint1 = Revolute::new(
        "joint1",
        JointParameters::default(),
        RevoluteState::default(),
    );

    let mut joint2 = Revolute::new(
        "joint2",
        JointParameters::default(),
        RevoluteState::default(),
    );

    let mp = MassProperties::new(
        1.0,
        CenterOfMass::new(0.0, 0.0, 0.0),
        Inertia::new(1.0, 1.0, 1.0, 0.0, 0.0, 0.0).unwrap(),
    );
    let mut body1 = Body::new("body1", mp.unwrap()).unwrap();

    let mp = MassProperties::new(
        1.0,
        CenterOfMass::new(0.0, 0.0, 0.0),
        Inertia::new(1.0, 1.0, 1.0, 0.0, 0.0, 0.0).unwrap(),
    );
    let mut body2 = Body::new("body2", mp.unwrap()).unwrap();

    joint2
        .connect_inner_body(&mut base, Transform::default())
        .unwrap();
    joint2
        .connect_outer_body(&mut body2, Transform::default())
        .unwrap();
    joint1
        .connect_inner_body(&mut body2, Transform::default())
        .unwrap();
    joint1
        .connect_outer_body(&mut body1, Transform::default())
        .unwrap();

    sys.add_base(base).unwrap();
    sys.add_joint(joint1.into()).unwrap();
    sys.add_joint(joint2.into()).unwrap();
    sys.add_body(body1).unwrap();
    sys.add_body(body2).unwrap();

    let mut sim = MultibodySystemSim::from(sys);

    let result = sim.simulate(0.0, 10.0, 1.0);

    dbg!(result);
}
