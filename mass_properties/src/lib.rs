use nalgebra::{Matrix3, Vector3};
use rand_distr::{Normal, NormalError, Uniform};
use serde::{Deserialize, Serialize};
use thiserror::Error;
use uncertainty::{SimValue, Uncertainty};

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

impl Uncertainty for MassPropertiesBuilder {
    type Output = MassProperties;
    type Error = MassPropertiesErrors;

    fn sample(&mut self) -> Result<Self::Output, MassPropertiesErrors> {
        Ok(MassProperties::from(self))
    }
}

#[derive(Clone, Debug, Copy)]
pub struct MassProperties {
    pub mass: f64,
    pub cmx: f64,
    pub cmy: f64,
    pub cmz: f64,
    pub ixx: f64,
    pub iyy: f64,
    pub izz: f64,
    pub ixy: f64,
    pub ixz: f64,
    pub iyz: f64,
}

impl From<&mut MassPropertiesBuilder> for MassProperties {
    fn from(builder: &mut MassPropertiesBuilder) -> Self {
        Self {
            mass: builder.mass.sample(),
            cmx: builder.cmx.sample(),
            cmy: builder.cmy.sample(),
            cmz: builder.cmz.sample(),
            ixx: builder.ixx.sample(),
            iyy: builder.iyy.sample(),
            izz: builder.izz.sample(),
            ixy: builder.ixy.sample(),
            ixz: builder.ixz.sample(),
            iyz: builder.iyz.sample(),
        }
    }
}

impl MassProperties {
    pub fn new(mass: f64, cm: Vector3<f64>, inertia: Matrix3<f64>) -> Self {
        Self {
            mass,
            cmx: cm.x,
            cmy: cm.y,
            cmz: cm.z,
            ixx: inertia[(0, 0)],
            iyy: inertia[(1, 1)],
            izz: inertia[(2, 2)],
            ixy: inertia[(0, 1)],
            ixz: inertia[(0, 2)],
            iyz: inertia[(1, 2)],
        }
    }

    pub fn mass(&self) -> f64 {
        self.mass
    }

    pub fn cm(&self) -> Vector3<f64> {
        Vector3::new(self.cmx, self.cmy, self.cmz)
    }

    pub fn inertia(&self) -> Matrix3<f64> {
        Matrix3::new(
            self.ixx, self.ixy, self.ixz, self.ixy, self.iyy, self.iyz, self.ixz, self.iyz,
            self.izz,
        )
    }
}
