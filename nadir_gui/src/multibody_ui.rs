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

pub enum PrismaticField {
    Name,
    ConstantForce,
    Damping,
    SpringConstant,
    Velocity,
    Position,
}

pub enum CuboidField {
    Length,
    Width,
    Height,
}

pub enum QuaternionField {
    W,
    X,
    Y,
    Z,
}

pub enum EulerAnglesField {
    Phi,
    Theta,
    Psi,
}

pub enum RotationMatrixField {
    E11,
    E12,
    E13,
    E21,
    E22,
    E23,
    E31,
    E32,
    E33,
}

pub enum CartesianField {
    X,
    Y,
    Z,
}

pub enum CylindricalField {
    Azimuth,
    Height,
    Radius,
}
pub enum SphericalField {
    Azimuth,
    Inclination,
    Radius,
}
