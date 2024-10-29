pub struct MultibodySpacecraft {
    name: String,
    base_body: Body,
    base_joint: Floating,
    bodies: Vec<Body>,
    joints: Vec<Joint>,
    orbit: Orbit,
}