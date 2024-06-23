use mass_properties::{CenterOfMass, Inertia, MassProperties};
use multibody::{
    base::Base, body::{Body, BodyTrait}, joint::{
        revolute::{Revolute, RevoluteState},
        Joint, JointParameters, JointTrait,
    }, system::MultibodySystem, system_sim::MultibodySystemSim, MultibodyTrait
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

    
    joint2.connect_inner_body(&mut base, Transform::default());    
    joint2.connect_outer_body(&mut body2, Transform::default());    
    joint1.connect_inner_body(&mut body2, Transform::default());    
    joint1.connect_outer_body(&mut body1, Transform::default());

    sys.add_base(base);
    sys.add_joint(joint1.into());
    sys.add_joint(joint2.into());
    sys.add_body(body1);
    sys.add_body(body2);

    let mut sim = MultibodySystemSim::from(sys);
    
    sim.run();

}
