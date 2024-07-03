// see the dependency multibody for actual multibody dynamics
// this is just the ui part of multibody

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
    Damping,
    SpringConstant,
    Omega,
    Theta,
}
