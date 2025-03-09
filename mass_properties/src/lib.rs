use nalgebra::{Matrix3, Vector3};
use rand_distr::{Normal, NormalError, Uniform};
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
    NormalErrors(#[from] NormalError),
    #[error("{0}")]
    UncertaintyErrors(#[from] uncertainty::Error),
    #[error("for uniform dist, low must be less than high")]
    UniformLowMustBeGreaterThanHigh,
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

pub struct MassPropertiesBuilder {
    mass: SimValue,
    cmx: SimValue,
    cmy: SimValue,
    cmz: SimValue,
    ixx: SimValue,
    iyy: SimValue,
    izz: SimValue,
    ixy: SimValue,
    ixz: SimValue,
    iyz: SimValue,
}

impl MassPropertiesBuilder {
    pub fn new() -> Self {
        Self {
            mass: SimValue::new(1.0),
            cmx: SimValue::new(0.0),
            cmy: SimValue::new(0.0),
            cmz: SimValue::new(0.0),
            ixx: SimValue::new(1.0),
            iyy: SimValue::new(1.0),
            izz: SimValue::new(1.0),
            ixy: SimValue::new(0.0),
            ixz: SimValue::new(0.0),
            iyz: SimValue::new(0.0),
        }
    }

    pub fn with_mass(mut self, mass: f64) -> Result<Self, MassPropertiesErrors> {
        if mass < std::f64::EPSILON {
            Err(MassPropertiesErrors::MassLessThanOrEqualToZero)
        } else {
            self.mass.value = mass;
            Ok(self)
        }
    }

    pub fn with_cmx(mut self, cmx: f64) -> Self {
        self.cmx.value = cmx;
        self
    }

    pub fn with_cmy(mut self, cmy: f64) -> Self {
        self.cmy.value = cmy;
        self
    }

    pub fn with_cmz(mut self, cmz: f64) -> Self {
        self.cmz.value = cmz;
        self
    }

    pub fn with_ixx(mut self, ixx: f64) -> Result<Self, MassPropertiesErrors> {
        if ixx < std::f64::EPSILON {
            Err(MassPropertiesErrors::IxxLessThanOrEqualToZero)
        } else {
            self.ixx.value = ixx;
            Ok(self)
        }
    }

    pub fn with_iyy(mut self, iyy: f64) -> Result<Self, MassPropertiesErrors> {
        if iyy < std::f64::EPSILON {
            Err(MassPropertiesErrors::IyyLessThanOrEqualToZero)
        } else {
            self.iyy.value = iyy;
            Ok(self)
        }
    }

    pub fn with_izz(mut self, izz: f64) -> Result<Self, MassPropertiesErrors> {
        if izz < std::f64::EPSILON {
            Err(MassPropertiesErrors::IzzLessThanOrEqualToZero)
        } else {
            self.izz.value = izz;
            Ok(self)
        }
    }

    pub fn with_ixy(mut self, ixy: f64) -> Self {
        self.ixy.value = ixy;
        self
    }

    pub fn with_ixz(mut self, ixz: f64) -> Self {
        self.ixz.value = ixz;
        self
    }

    pub fn with_iyz(mut self, iyz: f64) -> Self {
        self.iyz.value = iyz;
        self
    }

    pub fn with_mass_normal(mut self, mean: f64, std: f64) -> Result<Self, MassPropertiesErrors> {
        // if mass can be less than 0.0, return an error
        // sample should be redrawing if mass is ever sampled below 0.0 on outliers
        if mean < std::f64::EPSILON || mean - 3.0 * std < std::f64::EPSILON {
            return Err(MassPropertiesErrors::MassLessThanOrEqualToZero);
        }

        let dist = Normal::new(mean, std)?;
        self.mass = self.mass.with_distribution(dist.into())?;
        Ok(self)
    }

    pub fn with_mass_uniform(mut self, low: f64, high: f64) -> Result<Self, MassPropertiesErrors> {
        // if mass can be less than 0.0, return an error
        if low < std::f64::EPSILON {
            return Err(MassPropertiesErrors::MassLessThanOrEqualToZero);
        }

        // if low is not less than high, return an error
        if low > high {
            return Err(MassPropertiesErrors::UniformLowMustBeGreaterThanHigh);
        }

        let dist = Uniform::new(low, high);
        self.mass = self.mass.with_distribution(dist.into())?;
        Ok(self)
    }

    pub fn with_cmx_normal(mut self, mean: f64, std: f64) -> Result<Self, MassPropertiesErrors> {
        let dist = Normal::new(mean, std)?;
        self.cmx = self.cmx.with_distribution(dist.into())?;
        Ok(self)
    }

    pub fn with_cmx_uniform(mut self, low: f64, high: f64) -> Result<Self, MassPropertiesErrors> {
        // if low is not less than high, return an error
        if low > high {
            return Err(MassPropertiesErrors::UniformLowMustBeGreaterThanHigh);
        }
        let dist = Uniform::new(low, high);
        self.cmx = self.cmx.with_distribution(dist.into())?;
        Ok(self)
    }

    pub fn with_cmy_normal(mut self, mean: f64, std: f64) -> Result<Self, MassPropertiesErrors> {
        let dist = Normal::new(mean, std)?;
        self.cmy = self.cmy.with_distribution(dist.into())?;
        Ok(self)
    }

    pub fn with_cmy_uniform(mut self, low: f64, high: f64) -> Result<Self, MassPropertiesErrors> {
        // if low is not less than high, return an error
        if low > high {
            return Err(MassPropertiesErrors::UniformLowMustBeGreaterThanHigh);
        }
        let dist = Uniform::new(low, high);
        self.cmy = self.cmy.with_distribution(dist.into())?;
        Ok(self)
    }

    pub fn with_cmz_normal(mut self, mean: f64, std: f64) -> Result<Self, MassPropertiesErrors> {
        let dist = Normal::new(mean, std)?;
        self.cmz = self.cmz.with_distribution(dist.into())?;
        Ok(self)
    }

    pub fn with_cmz_uniform(mut self, low: f64, high: f64) -> Result<Self, MassPropertiesErrors> {
        // if low is not less than high, return an error
        if low > high {
            return Err(MassPropertiesErrors::UniformLowMustBeGreaterThanHigh);
        }
        let dist = Uniform::new(low, high);
        self.cmz = self.cmz.with_distribution(dist.into())?;
        Ok(self)
    }

    pub fn with_ixx_normal(mut self, mean: f64, std: f64) -> Result<Self, MassPropertiesErrors> {
        // if moi can be less than 0.0, return an error
        // sample should be redrawing if moi is ever sampled below 0.0 on outliers
        if mean < std::f64::EPSILON || mean - 3.0 * std < std::f64::EPSILON {
            return Err(MassPropertiesErrors::IxxLessThanOrEqualToZero);
        }

        let dist = Normal::new(mean, std)?;
        self.ixx = self.ixx.with_distribution(dist.into())?;
        Ok(self)
    }

    pub fn with_ixx_uniform(mut self, low: f64, high: f64) -> Result<Self, MassPropertiesErrors> {
        // if moi can be less than 0.0, return an error
        if low < std::f64::EPSILON {
            return Err(MassPropertiesErrors::IxxLessThanOrEqualToZero);
        }

        // if low is not less than high, return an error
        if low > high {
            return Err(MassPropertiesErrors::UniformLowMustBeGreaterThanHigh);
        }

        let dist = Uniform::new(low, high);
        self.ixx = self.ixx.with_distribution(dist.into())?;
        Ok(self)
    }

    pub fn with_iyy_normal(mut self, mean: f64, std: f64) -> Result<Self, MassPropertiesErrors> {
        // if moi can be less than 0.0, return an error
        // sample should be redrawing if moi is ever sampled below 0.0 on outliers
        if mean < std::f64::EPSILON || mean - 3.0 * std < std::f64::EPSILON {
            return Err(MassPropertiesErrors::IyyLessThanOrEqualToZero);
        }

        let dist = Normal::new(mean, std)?;
        self.iyy = self.iyy.with_distribution(dist.into())?;
        Ok(self)
    }

    pub fn with_iyy_uniform(mut self, low: f64, high: f64) -> Result<Self, MassPropertiesErrors> {
        // if moi can be less than 0.0, return an error
        if low < std::f64::EPSILON {
            return Err(MassPropertiesErrors::IyyLessThanOrEqualToZero);
        }

        // if low is not less than high, return an error
        if low > high {
            return Err(MassPropertiesErrors::UniformLowMustBeGreaterThanHigh);
        }

        let dist = Uniform::new(low, high);
        self.iyy = self.iyy.with_distribution(dist.into())?;
        Ok(self)
    }

    pub fn with_izz_normal(mut self, mean: f64, std: f64) -> Result<Self, MassPropertiesErrors> {
        // if moi can be less than 0.0, return an error
        // sample should be redrawing if moi is ever sampled below 0.0 on outliers
        if mean < std::f64::EPSILON || mean - 3.0 * std < std::f64::EPSILON {
            return Err(MassPropertiesErrors::IzzLessThanOrEqualToZero);
        }

        let dist = Normal::new(mean, std)?;
        self.izz = self.izz.with_distribution(dist.into())?;
        Ok(self)
    }

    pub fn with_izz_uniform(mut self, low: f64, high: f64) -> Result<Self, MassPropertiesErrors> {
        // if moi can be less than 0.0, return an error
        if low < std::f64::EPSILON {
            return Err(MassPropertiesErrors::IzzLessThanOrEqualToZero);
        }

        // if low is not less than high, return an error
        if low > high {
            return Err(MassPropertiesErrors::UniformLowMustBeGreaterThanHigh);
        }

        let dist = Uniform::new(low, high);
        self.izz = self.izz.with_distribution(dist.into())?;
        Ok(self)
    }

    pub fn with_ixy_normal(mut self, mean: f64, std: f64) -> Result<Self, MassPropertiesErrors> {
        let dist = Normal::new(mean, std)?;
        self.ixy = self.ixy.with_distribution(dist.into())?;
        Ok(self)
    }

    pub fn with_ixy_uniform(mut self, low: f64, high: f64) -> Result<Self, MassPropertiesErrors> {
        // if low is not less than high, return an error
        if low > high {
            return Err(MassPropertiesErrors::UniformLowMustBeGreaterThanHigh);
        }

        let dist = Uniform::new(low, high);
        self.ixy = self.ixy.with_distribution(dist.into())?;
        Ok(self)
    }

    pub fn with_ixz_normal(mut self, mean: f64, std: f64) -> Result<Self, MassPropertiesErrors> {
        let dist = Normal::new(mean, std)?;
        self.ixz = self.ixz.with_distribution(dist.into())?;
        Ok(self)
    }

    pub fn with_ixz_uniform(mut self, low: f64, high: f64) -> Result<Self, MassPropertiesErrors> {
        // if low is not less than high, return an error
        if low > high {
            return Err(MassPropertiesErrors::UniformLowMustBeGreaterThanHigh);
        }

        let dist = Uniform::new(low, high);
        self.ixz = self.ixz.with_distribution(dist.into())?;
        Ok(self)
    }

    pub fn with_iyz_normal(mut self, mean: f64, std: f64) -> Result<Self, MassPropertiesErrors> {
        let dist = Normal::new(mean, std)?;
        self.iyz = self.iyz.with_distribution(dist.into())?;
        Ok(self)
    }

    pub fn with_iyz_uniform(mut self, low: f64, high: f64) -> Result<Self, MassPropertiesErrors> {
        // if low is not less than high, return an error
        if low > high {
            return Err(MassPropertiesErrors::UniformLowMustBeGreaterThanHigh);
        }

        let dist = Uniform::new(low, high);
        self.iyz = self.iyz.with_distribution(dist.into())?;
        Ok(self)
    }
}
