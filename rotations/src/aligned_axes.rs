use std::ops::Neg;

#[derive(Debug)]
pub enum AlignedAxesErrors {
    InvalidCombo,
}

impl std::fmt::Display for AlignedAxesErrors {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AlignedAxesErrors::InvalidCombo => write!(
                f,
                "Invalid combination of aligned axes. Primary and Secondary axes cannot be the same."
            ),
        }
    }
}

impl std::error::Error for AlignedAxesErrors {}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Axis {
    Xp,
    Xn,
    Yp,
    Yn,
    Zp,
    Zn,
}
// needed for iced pick_list
//TODO just put this in gui as EulerSequencePickList or something
impl Axis {
    pub const ALL: [Axis; 6] = [Axis::Xp, Axis::Yp, Axis::Zp, Axis::Xn, Axis::Yn, Axis::Zn];
}

impl std::fmt::Display for Axis {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}",
            match self {
                Axis::Xp => "+X",
                Axis::Yp => "+Y",
                Axis::Zp => "+Z",
                Axis::Xn => "-X",
                Axis::Yn => "-Y",
                Axis::Zn => "-Z",
            }
        )
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisPair {
    pub old: Axis,
    pub new: Axis,
}
impl AxisPair {
    pub fn new(old: Axis, new: Axis) -> Self {
        Self { old, new }
    }
}

/// Please note the convertion for the defintion
/// You should be saying 'my new <x> direction is in my old <y> direction'
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AlignedAxes {
    pub primary: AxisPair,
    pub secondary: AxisPair,
}

impl AlignedAxes {
    pub fn new(mut primary: AxisPair, mut secondary: AxisPair) -> Result<Self, AlignedAxesErrors> {
        // check that primary and secondary axes are not the same
        if primary.new == secondary.new || primary.old == secondary.new {
            return Err(AlignedAxesErrors::InvalidCombo);
        }

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

        Ok(Self { primary, secondary })
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
