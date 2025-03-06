use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};
use thiserror::Error;
use uncertainty::{Distributions, SimValue, Uncertainty};

#[derive(Debug, Error)]
pub enum MassPropertiesErrors {
    #[error("Ixx cant be less than or equal to  zero")]
    IxxLessThanOrEqualToZero,
    #[error("Iyy cant be less than or equal to zero")]
    IyyLessThanOrEqualToZero,
    #[error("Izz cant be less than or equal to zero")]
    IzzLessThanOrEqualToZero,
    #[error("mass cannot be less than or equal to zero")]
    MassLessThanOrEqualToZero,
    #[error("{0}")]
    UncertaintyErrors(#[from] uncertainty::Error),
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct CenterOfMass {
    x: SimValue,
    y: SimValue,
    z: SimValue,
}

impl CenterOfMass {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            x: SimValue::new(x),
            y: SimValue::new(y),
            z: SimValue::new(z),
        }
    }

    pub fn with_uncertainty_x(
        mut self,
        distribution: Distributions,
    ) -> Result<Self, MassPropertiesErrors> {
        self.x = self.x.with_distribution(distribution)?;
        Ok(self)
    }

    pub fn with_uncertainty_y(
        mut self,
        distribution: Distributions,
    ) -> Result<Self, MassPropertiesErrors> {
        self.y = self.y.with_distribution(distribution)?;
        Ok(self)
    }

    pub fn with_uncertainty_z(
        mut self,
        distribution: Distributions,
    ) -> Result<Self, MassPropertiesErrors> {
        self.z = self.z.with_distribution(distribution)?;
        Ok(self)
    }

    pub fn vector(&self) -> Vector3<f64> {
        Vector3::new(self.x.value, self.y.value, self.z.value)
    }
}

impl Uncertainty for CenterOfMass {
    fn sample(&mut self) {
        self.x.sample();
        self.y.sample();
        self.z.sample();
    }
}

impl From<Vector3<f64>> for CenterOfMass {
    fn from(v: Vector3<f64>) -> CenterOfMass {
        CenterOfMass::new(v[0], v[1], v[2])
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Inertia {
    pub ixx: SimValue,
    pub ixy: SimValue,
    pub ixz: SimValue,
    pub iyy: SimValue,
    pub iyz: SimValue,
    pub izz: SimValue,
}

impl Inertia {
    pub fn new(
        ixx: f64,
        iyy: f64,
        izz: f64,
        ixy: f64,
        ixz: f64,
        iyz: f64,
    ) -> Result<Self, MassPropertiesErrors> {
        if ixx <= f64::EPSILON {
            return Err(MassPropertiesErrors::IxxLessThanOrEqualToZero);
        }
        if iyy <= f64::EPSILON {
            return Err(MassPropertiesErrors::IyyLessThanOrEqualToZero);
        }
        if izz <= f64::EPSILON {
            return Err(MassPropertiesErrors::IzzLessThanOrEqualToZero);
        }
        Ok(Self {
            ixx: SimValue::new(ixx),
            iyy: SimValue::new(iyy),
            izz: SimValue::new(izz),
            ixy: SimValue::new(ixy),
            ixz: SimValue::new(ixz),
            iyz: SimValue::new(iyz),
        })
    }

    pub fn with_uncertainty_ixx(
        mut self,
        distribution: Distributions,
    ) -> Result<Self, MassPropertiesErrors> {
        self.ixx = self.ixx.with_distribution(distribution)?;
        Ok(self)
    }

    pub fn with_uncertainty_iyy(
        mut self,
        distribution: Distributions,
    ) -> Result<Self, MassPropertiesErrors> {
        self.iyy = self.iyy.with_distribution(distribution)?;
        Ok(self)
    }

    pub fn with_uncertainty_izz(
        mut self,
        distribution: Distributions,
    ) -> Result<Self, MassPropertiesErrors> {
        self.izz = self.izz.with_distribution(distribution)?;
        Ok(self)
    }

    pub fn with_uncertainty_ixy(
        mut self,
        distribution: Distributions,
    ) -> Result<Self, MassPropertiesErrors> {
        self.ixy = self.ixy.with_distribution(distribution)?;
        Ok(self)
    }

    pub fn with_uncertainty_ixz(
        mut self,
        distribution: Distributions,
    ) -> Result<Self, MassPropertiesErrors> {
        self.ixz = self.ixz.with_distribution(distribution)?;
        Ok(self)
    }

    pub fn with_uncertainty_iyz(
        mut self,
        distribution: Distributions,
    ) -> Result<Self, MassPropertiesErrors> {
        self.iyz = self.iyz.with_distribution(distribution)?;
        Ok(self)
    }

    pub fn matrix(&self) -> Matrix3<f64> {
        Matrix3::new(
            self.ixx.value,
            self.ixy.value,
            self.ixz.value,
            self.ixy.value,
            self.iyy.value,
            self.iyz.value,
            self.ixz.value,
            self.iyz.value,
            self.izz.value,
        )
    }
}

impl Uncertainty for Inertia {
    fn sample(&mut self) {
        self.ixx.sample();
        self.iyy.sample();
        self.izz.sample();
        self.ixy.sample();
        self.ixz.sample();
        self.iyz.sample();
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

/// Represents the mass properties of an object
/// Mass, Center of Mass, Inertia
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MassProperties {
    pub center_of_mass: CenterOfMass,
    pub mass: SimValue,
    pub inertia: Inertia,
}

impl Default for MassProperties {
    fn default() -> Self {
        let mass = SimValue::new(1.0);
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
    pub fn new(
        mass: f64,
        center_of_mass: CenterOfMass,
        inertia: Inertia,
    ) -> Result<Self, MassPropertiesErrors> {
        if mass <= f64::EPSILON {
            return Err(MassPropertiesErrors::MassLessThanOrEqualToZero);
        }
        Ok(MassProperties {
            mass: SimValue::new(mass),
            center_of_mass,
            inertia,
        })
    }

    pub fn with_uncertainty_mass(
        mut self,
        distribution: Distributions,
    ) -> Result<Self, MassPropertiesErrors> {
        self.mass = self.mass.with_distribution(distribution)?;
        Ok(self)
    }
}

impl Uncertainty for MassProperties {
    fn sample(&mut self) {
        self.mass.sample();
        self.center_of_mass.sample();
        self.inertia.sample()
    }
}
