use std::ops::Neg;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Axis {
    Xp,
    Xn,
    Yp,
    Yn,
    Zp,
    Zn,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisPair {
    pub old: Axis,
    pub new: Axis,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AlignedAxes {
    pub primary: AxisPair,
    pub secondary: AxisPair,
}

impl AlignedAxes {
    pub fn new(mut primary: AxisPair, mut secondary: AxisPair) -> Self {
        // convert old negative axes to positive axes to make converting to rotations easier
        match primary.old {
            Axis::Xn | Axis::Yn | Axis::Zn => {
                primary.old = -primary.old;
                primary.new = -primary.new;
            }
            _ => {} // all good
        }
        match secondary.old {
            Axis::Xn | Axis::Yn | Axis::Zn => {
                secondary.old = -secondary.old;
                secondary.new = -secondary.new;
            }
            _ => {} // all good
        }

        Self { primary, secondary }
    }
}

impl Neg for Axis {
    type Output = Self;
    fn neg(self) -> Self {
        match self {
            Axis::Xp => Axis::Xn,
            Axis::Xn => Axis::Xp,
            Axis::Yp => Axis::Yn,
            Axis::Yn => Axis::Yp,
            Axis::Zp => Axis::Zn,
            Axis::Zn => Axis::Zp,
        }
    }
}
