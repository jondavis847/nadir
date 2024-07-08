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
use plotting::{ChartTheme,plot_column_against_time};
use transforms::Transform;

fn main() {
    let mut sys = MultibodySystem::new();

    let mut base = Base::new("base");
    let jp1 = JointParameters {
        constant_force: 0.0,
        spring_constant: 1.0,
        damping: 0.0,
        mass_properties: None,
    };

    let js1 = RevoluteState::new(0.0, 1.0);

    let mut joint1 = Revolute::new("joint1", jp1, js1);

    let mp = MassProperties::new(
        1.0,
        CenterOfMass::new(0.0, 0.0, 0.0),
        Inertia::new(1.0, 1.0, 1.0, 0.0, 0.0, 0.0).unwrap(),
    );
    let mut body1 = Body::new("body1", mp.unwrap()).unwrap();

    joint1
        .connect_inner_body(&mut base, Transform::default())
        .unwrap();
    joint1
        .connect_outer_body(&mut body1, Transform::default())
        .unwrap();

    sys.add_base(base).unwrap();
    sys.add_joint(joint1.into()).unwrap();
    sys.add_body(body1).unwrap();

    let mut sim = MultibodySystemSim::from(sys);

    let result = sim.simulate("sim_test".to_string(), 0.0, 10.0, 0.1);

    let joint1 = result.get_component("joint1");
    dbg!(joint1);
    let body1_rate = result.get_component_state(
        "body1",
        vec![
            "angular_rate_body_x",
            "angular_rate_body_y",
            "angular_rate_body_z",
        ],
    );

    let body1_quat = result.get_component_state(
        "body1",
        vec![
            "attitude_base_s",
            "attitude_base_x",
            "attitude_base_y",
            "attitude_base_z"
        ],
    );

    dbg!(body1_rate);    
    dbg!(body1_quat);    

plot_column_against_time(&result.get_component("body1"), "angular_rate_body_z", "angular_rate_body_z.svg", ChartTheme::default()).unwrap();
}
