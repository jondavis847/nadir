#[derive(Clone, Copy, Debug, Default)]
struct CenterOfMass {
    cmx: f64,
    cmy: f64,
    cmz: f64,
}

impl CenterOfMass {
    fn new(cmx: f64, cmy: f64, cmz: f64) -> Self {
        Self { cmx, cmy, cmz }
    }

    /// Returns the x-coordinate of the center of mass.
    pub fn get_cmx(&self) -> f64 {
        self.cmx
    }

    /// Returns the y-coordinate of the center of mass.
    pub fn get_cmy(&self) -> f64 {
        self.cmy
    }

    /// Returns the y-coordinate of the center of mass.
    pub fn get_cmz(&self) -> f64 {
        self.cmz
    }

    /// Sets the x-coordinate of the center of mass.
    pub fn set_cmx(&mut self, cmx: f64) {
        self.cmx = cmx;
    }

    /// Sets the y-coordinate of the center of mass.
    pub fn set_cmy(&mut self, cmy: f64) {
        self.cmy = cmy;
    }

    /// Sets the z-coordinate of the center of mass.
    pub fn set_cmz(&mut self, cmz: f64) {
        self.cmz = cmz;
    }
}

#[derive(Debug, Clone, Copy)]
struct Inertia {
    ixx: f64,
    ixy: f64,
    ixz: f64,
    iyy: f64,
    iyz: f64,
    izz: f64,
}

#[derive(Debug, Clone, Copy)]
pub enum InertiaErrors {
    IxxLessThanOrEqualToZero,
    IyyLessThanOrEqualToZero,
    IzzLessThanOrEqualToZero,
}

impl Inertia {
    fn new(
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

    fn get_ixx(&self) -> f64 {
        self.ixx
    }
    fn get_iyy(&self) -> f64 {
        self.iyy
    }
    fn get_izz(&self) -> f64 {
        self.izz
    }
    fn get_ixy(&self) -> f64 {
        self.ixy
    }
    fn get_ixz(&self) -> f64 {
        self.ixz
    }
    fn get_iyz(&self) -> f64 {
        self.iyz
    }

    pub fn set_ixx(&mut self, ixx: f64) -> Result<(), InertiaErrors> {
        if ixx < f64::EPSILON {
            return Err(InertiaErrors::IxxLessThanOrEqualToZero);
        }
        self.ixx = ixx;
        Ok(())
    }

    /// Sets the product of inertia for the xy-plane.
    pub fn set_ixy(&mut self, ixy: f64) {
        self.ixy = ixy;
    }

    /// Sets the product of inertia for the xz-plane.
    pub fn set_ixz(&mut self, ixz: f64) {
        self.ixz = ixz;
    }

    /// Sets the moment of inertia around the y-axis.
    ///
    /// # Errors
    ///
    /// Returns a `MassPropertiesError::IyyLessThanOrEqualToZero` if `iyy` is less than or equal to zero.
    pub fn set_iyy(&mut self, iyy: f64) -> Result<(), InertiaErrors> {
        if iyy < f64::EPSILON {
            return Err(InertiaErrors::IyyLessThanOrEqualToZero);
        }
        self.iyy = iyy;
        Ok(())
    }

    /// Sets the product of inertia for the yz-plane.
    pub fn set_iyz(&mut self, iyz: f64) {
        self.iyz = iyz;
    }

    /// Sets the moment of inertia around the z-axis.
    ///
    /// # Errors
    ///
    /// Returns a `MassPropertiesError::IzzLessThanOrEqualToZero` if `izz` is less than or equal to zero.
    pub fn set_izz(&mut self, izz: f64) -> Result<(), InertiaErrors> {
        if izz < f64::EPSILON {
            return Err(InertiaErrors::IzzLessThanOrEqualToZero);
        }
        self.izz = izz;
        Ok(())
    }
}

/// Represents the mass properties of an object
/// Mass, Center of Mass, Inertia
#[derive(Debug, Clone, Copy)]
pub struct MassProperties {
    center_of_mass: CenterOfMass,
    mass: f64,
    inertia: Inertia,
}

/// Enum representing possible errors when creating or modifying `MassProperties`.
#[derive(Debug, Clone, Copy)]
pub enum MassPropertiesErrors {
    Inertia(InertiaErrors),
    MassLessThanOrEqualToZero,
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
        cmx: f64,
        cmy: f64,
        cmz: f64,
        ixx: f64,
        iyy: f64,
        izz: f64,
        ixy: f64,
        ixz: f64,
        iyz: f64,
    ) -> Result<Self, MassPropertiesErrors> {
        if mass <= f64::EPSILON {
            return Err(MassPropertiesErrors::MassLessThanOrEqualToZero);
        }

        let center_of_mass = CenterOfMass::new(cmx, cmy, cmz);

        let inertia = match Inertia::new(ixx, iyy, izz, ixy, ixz, iyz) {
            Ok(inertia) => inertia,
            Err(error) => return Err(MassPropertiesErrors::Inertia(error)),
        };

        Ok(Self {
            center_of_mass,
            inertia,
            mass,
        })
    }

    /// Returns the x-coordinate of the center of mass.
    pub fn get_cmx(&self) -> f64 {
        self.center_of_mass.get_cmx()
    }

    /// Returns the y-coordinate of the center of mass.
    pub fn get_cmy(&self) -> f64 {
        self.center_of_mass.get_cmy()
    }

    /// Returns the y-coordinate of the center of mass.
    pub fn get_cmz(&self) -> f64 {
        self.center_of_mass.get_cmz()
    }

    /// Returns the moment of inertia around the x-axis.
    pub fn get_ixx(&self) -> f64 {
        self.inertia.get_ixx()
    }

    /// Returns the product of inertia for the xy-plane.
    pub fn get_ixy(&self) -> f64 {
        self.inertia.get_ixy()
    }

    /// Returns the product of inertia for the xz-plane.
    pub fn get_ixz(&self) -> f64 {
        self.inertia.get_ixz()
    }

    /// Returns the moment of inertia around the y-axis.
    pub fn get_iyy(&self) -> f64 {
        self.inertia.get_iyy()
    }

    /// Returns the product of inertia for the yz-plane.
    pub fn get_iyz(&self) -> f64 {
        self.inertia.get_iyz()
    }

    /// Returns the moment of inertia around the z-axis.
    pub fn get_izz(&self) -> f64 {
        self.inertia.get_izz()
    }

    /// Returns the mass of the object.
    pub fn get_mass(&self) -> f64 {
        self.mass
    }

    /// Sets the x-coordinate of the center of mass.
    pub fn set_cmx(&mut self, cmx: f64) {
        self.center_of_mass.set_cmx(cmx);
    }

    /// Sets the y-coordinate of the center of mass.
    pub fn set_cmy(&mut self, cmy: f64) {
        self.center_of_mass.set_cmy(cmy);
    }

    /// Sets the z-coordinate of the center of mass.
    pub fn set_cmz(&mut self, cmz: f64) {
        self.center_of_mass.set_cmz(cmz);
    }

    /// Sets the moment of inertia around the x-axis.
    ///
    /// # Errors
    ///
    /// Returns a `MassPropertiesError::IxxLessThanOrEqualToZero` if `ixx` is less than or equal to zero.
    pub fn set_ixx(&mut self, ixx: f64) -> Result<(), MassPropertiesErrors> {
        match self.inertia.set_ixx(ixx) {
            Ok(_) => Ok(()),
            Err(error) => Err(MassPropertiesErrors::Inertia(error)),
        }
    }

    /// Sets the product of inertia for the xy-plane.
    pub fn set_ixy(&mut self, ixy: f64) {
        self.inertia.set_ixy(ixy);
    }

    /// Sets the product of inertia for the xz-plane.
    pub fn set_ixz(&mut self, ixz: f64) {
        self.inertia.set_ixz(ixz);
    }
    /// Sets the moment of inertia around the y-axis.
    ///
    /// # Errors
    ///
    /// Returns a `MassPropertiesError::IyyLessThanOrEqualToZero` if `iyy` is less than or equal to zero.
    pub fn set_iyy(&mut self, iyy: f64) -> Result<(), MassPropertiesErrors> {
        match self.inertia.set_iyy(iyy) {
            Ok(_) => Ok(()),
            Err(error) => Err(MassPropertiesErrors::Inertia(error)),
        }
    }

    /// Sets the product of inertia for the yz-plane.
    pub fn set_iyz(&mut self, iyz: f64) -> Result<(), MassPropertiesErrors> {
        self.inertia.set_iyz(iyz);
        Ok(())
    }

    /// Sets the moment of inertia around the z-axis.
    ///
    /// # Errors
    ///
    /// Returns a `MassPropertiesError::IzzLessThanOrEqualToZero` if `izz` is less than or equal to zero.
    pub fn set_izz(&mut self, izz: f64) -> Result<(), MassPropertiesErrors> {
        match self.inertia.set_izz(izz) {
            Ok(_) => Ok(()),
            Err(error) => Err(MassPropertiesErrors::Inertia(error)),
        }
    }

    /// Sets the mass of the object.
    ///
    /// # Errors
    ///
    /// Returns a `MassPropertiesError::MassLessThanOrEqualToZero` if `mass` is less than or equal to zero.
    pub fn set_mass(&mut self, mass: f64) -> Result<(), MassPropertiesErrors> {
        if mass < f64::EPSILON {
            return Err(MassPropertiesErrors::MassLessThanOrEqualToZero);
        }
        self.mass = mass;
        Ok(())
    }
}
