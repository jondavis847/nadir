use std::{
    error::Error,
    fmt::{self, Display},
};

use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};
use uncertainty::UncertainValue;

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct CenterOfMass {
    x: UncertainValue,
    y: f64,
    z: f64,
}

impl CenterOfMass {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn vector(&self) -> Vector3<f64> {
        Vector3::new(self.x, self.y, self.z)
    }
}

impl From<Vector3<f64>> for CenterOfMass {
    fn from(v: Vector3<f64>) -> CenterOfMass {
        CenterOfMass {
            x: v[0],
            y: v[1],
            z: v[2],
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Inertia {
    pub ixx: f64,
    pub ixy: f64,
    pub ixz: f64,
    pub iyy: f64,
    pub iyz: f64,
    pub izz: f64,
}

#[derive(Debug)]
pub enum InertiaErrors {
    IxxLessThanOrEqualToZero,
    IyyLessThanOrEqualToZero,
    IzzLessThanOrEqualToZero,
}

impl std::fmt::Display for InertiaErrors {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::IxxLessThanOrEqualToZero => writeln!(f, "Ixx cannot be less than zero."),
            Self::IyyLessThanOrEqualToZero => writeln!(f, "Ixx cannot be less than zero."),
            Self::IzzLessThanOrEqualToZero => writeln!(f, "Ixx cannot be less than zero."),
        }
    }
}

impl std::error::Error for InertiaErrors {}

impl Inertia {
    pub fn new(
        ixx: f64,
        iyy: f64,
        izz: f64,
        ixy: f64,
        ixz: f64,
        iyz: f64,
    ) -> Result<Self, InertiaErrors> {
        if ixx <= f64::EPSILON {
            return Err(InertiaErrors::IxxLessThanOrEqualToZero);
        }
        if iyy <= f64::EPSILON {
            return Err(InertiaErrors::IyyLessThanOrEqualToZero);
        }
        if izz <= f64::EPSILON {
            return Err(InertiaErrors::IzzLessThanOrEqualToZero);
        }
        Ok(Self {
            ixx,
            iyy,
            izz,
            ixy,
            ixz,
            iyz,
        })
    }

    pub fn matrix(&self) -> Matrix3<f64> {
        Matrix3::new(
            self.ixx, self.ixy, self.ixz, self.ixy, self.iyy, self.iyz, self.ixz, self.iyz,
            self.izz,
        )
    }
}

impl From<Matrix3<f64>> for Inertia {
    fn from(m: Matrix3<f64>) -> Inertia {
        //TODO add checks on the matrix
        Inertia::new(
            m[(0, 0)],
            m[(1, 1)],
            m[(2, 2)],
            m[(0, 1)],
            m[(0, 2)],
            m[(2, 1)],
        )
        .unwrap()
    }
}

/// Enum representing possible errors when creating or modifying `MassProperties`.
#[derive(Debug)]
pub enum MassPropertiesErrors {
    InertiaErrors(InertiaErrors),
    MassLessThanOrEqualToZero,
}

impl Display for MassPropertiesErrors {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InertiaErrors(e) => writeln!(f, "Inertia Error: {e}"),
            Self::MassLessThanOrEqualToZero => {
                writeln!(f, "Mass cannot be less than or equal to zero.")
            }
        }
    }
}

impl Error for MassPropertiesErrors {}

/// Represents the mass properties of an object
/// Mass, Center of Mass, Inertia
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct MassProperties {
    pub center_of_mass: CenterOfMass,
    pub mass: f64,
    pub inertia: Inertia,
}

impl Default for MassProperties {
    fn default() -> Self {
        let mass = 1.0;
        let center_of_mass = CenterOfMass::new(0.0, 0.0, 0.0);
        let inertia = Inertia::new(1.0, 1.0, 1.0, 0.0, 0.0, 0.0).unwrap();
        Self {
            center_of_mass,
            mass,
            inertia,
        }
    }
}

impl MassProperties {
    /// Creates a new `MassProperties` instance.
    ///
    /// # Arguments
    ///
    /// * `mass` - The mass of the object.
    /// * `cmx` - The x-coordinate of the center of mass.
    /// * `cmy` - The y-coordinate of the center of mass.
    /// * `cmz` - The z-coordinate of the center of mass.
    /// * `ixx` - The moment of inertia around the x-axis.
    /// * `iyy` - The moment of inertia around the y-axis.
    /// * `izz` - The moment of inertia around the z-axis.
    /// * `ixy` - The product of inertia for the xy-plane.
    /// * `ixz` - The product of inertia for the xz-plane.
    /// * `iyz` - The product of inertia for the yz-plane.
    ///
    /// # Errors
    ///
    /// Returns a `MassPropertiesError` if any of the mass or principal moments of inertia
    /// are less than or equal to zero.
    pub fn new(
        mass: f64,
        center_of_mass: CenterOfMass,
        inertia: Inertia,
    ) -> Result<Self, MassPropertiesErrors> {
        if mass <= f64::EPSILON {
            return Err(MassPropertiesErrors::MassLessThanOrEqualToZero);
        }
        Ok(MassProperties {
            mass,
            center_of_mass,
            inertia,
        })
    }
}
