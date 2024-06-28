// see the dependency multibody for actual multibody dynamics
// this is just the ui part of multibody

#[derive(Debug,Clone,Copy)]
pub enum MultibodyComponent {
    Base,
    Body,
    Joint,
}

pub enum BodyField {
    Name,
    Mass,
    Cmx,
    Cmy,
    Cmz,
    Ixx,
    Iyy,
    Izz,
    Ixy,
    Ixz,
    Iyz,
}

pub enum RevoluteField {
    Name,
    ConstantForce,
    damping,
    SpringConstant,
    Omega,
    Theta,
}
