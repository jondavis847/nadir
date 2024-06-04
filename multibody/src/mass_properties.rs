use sim_value::SimValue;
#[derive(Clone, Copy, Debug, Default)]
struct CenterOfMass<T>
where
    T: SimValue,
{
    cmx: T,
    cmy: T,
    cmz: T,
}

impl<T> CenterOfMass<T>
where
    T: SimValue,
{
    fn new(cmx: T, cmy: T, cmz: T) -> Self {
        Self { cmx, cmy, cmz }
    }

    /// Returns the x-coordinate of the center of mass.
    pub fn get_cmx(&self) -> T {
        self.cmx
    }

    /// Returns the y-coordinate of the center of mass.
    pub fn get_cmy(&self) -> T {
        self.cmy
    }

    /// Returns the y-coordinate of the center of mass.
    pub fn get_cmz(&self) -> T {
        self.cmz
    }

    /// Sets the x-coordinate of the center of mass.
    pub fn set_cmx(&mut self, cmx: T) {
        self.cmx = cmx;
    }

    /// Sets the y-coordinate of the center of mass.
    pub fn set_cmy(&mut self, cmy: T) {
        self.cmy = cmy;
    }

    /// Sets the z-coordinate of the center of mass.
    pub fn set_cmz(&mut self, cmz: T) {
        self.cmz = cmz;
    }
}

#[derive(Debug, Clone, Copy)]
struct Inertia<T>
where
    T: SimValue,
{
    ixx: T,
    ixy: T,
    ixz: T,
    iyy: T,
    iyz: T,
    izz: T,
}

#[derive(Debug, Clone, Copy)]
pub enum InertiaErrors {
    IxxLessThanOrEqualToZero,
    IyyLessThanOrEqualToZero,
    IzzLessThanOrEqualToZero,
}

impl<T> Inertia<T>
where
    T: SimValue,
{
    fn new(ixx: T, iyy: T, izz: T, ixy: T, ixz: T, iyz: T) -> Result<Self, InertiaErrors> {
        if ixx <= T::zero() {
            return Err(InertiaErrors::IxxLessThanOrEqualToZero);
        }
        if iyy <= T::zero() {
            return Err(InertiaErrors::IyyLessThanOrEqualToZero);
        }
        if izz <= T::zero() {
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

    fn get_ixx(&self) -> T {
        self.ixx
    }
    fn get_iyy(&self) -> T {
        self.iyy
    }
    fn get_izz(&self) -> T {
        self.izz
    }
    fn get_ixy(&self) -> T {
        self.ixy
    }
    fn get_ixz(&self) -> T {
        self.ixz
    }
    fn get_iyz(&self) -> T {
        self.iyz
    }

    pub fn set_ixx(&mut self, ixx: T) -> Result<(), InertiaErrors> {
        if ixx <= T::zero() {
            return Err(InertiaErrors::IxxLessThanOrEqualToZero);
        }
        self.ixx = ixx;
        Ok(())
    }

    /// Sets the product of inertia for the xy-plane.
    pub fn set_ixy(&mut self, ixy: T) {
        self.ixy = ixy;
    }

    /// Sets the product of inertia for the xz-plane.
    pub fn set_ixz(&mut self, ixz: T) {
        self.ixz = ixz;
    }

    /// Sets the moment of inertia around the y-axis.
    ///
    /// # Errors
    ///
    /// Returns a `MassPropertiesError::IyyLessThanOrEqualToZero` if `iyy` is less than or equal to zero.
    pub fn set_iyy(&mut self, iyy: T) -> Result<(), InertiaErrors> {
        if iyy <= T::zero() {
            return Err(InertiaErrors::IyyLessThanOrEqualToZero);
        }
        self.iyy = iyy;
        Ok(())
    }

    /// Sets the product of inertia for the yz-plane.
    pub fn set_iyz(&mut self, iyz: T) {
        self.iyz = iyz;
    }

    /// Sets the moment of inertia around the z-axis.
    ///
    /// # Errors
    ///
    /// Returns a `MassPropertiesError::IzzLessThanOrEqualToZero` if `izz` is less than or equal to zero.
    pub fn set_izz(&mut self, izz: T) -> Result<(), InertiaErrors> {
        if izz <= T::zero() {
            return Err(InertiaErrors::IzzLessThanOrEqualToZero);
        }
        self.izz = izz;
        Ok(())
    }
}

/// Represents the mass properties of an object
/// Mass, Center of Mass, Inertia
#[derive(Debug, Clone, Copy)]
pub struct MassProperties<T>
where
    T: SimValue,
{
    center_of_mass: CenterOfMass<T>,
    mass: T,
    inertia: Inertia<T>,
}

/// Enum representing possible errors when creating or modifying `MassProperties`.
#[derive(Debug, Clone, Copy)]
pub enum MassPropertiesErrors {
    Inertia(InertiaErrors),
    MassLessThanOrEqualToZero,
}

impl<T> MassProperties<T>
where
    T: SimValue,
{
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
        mass: T,
        cmx: T,
        cmy: T,
        cmz: T,
        ixx: T,
        iyy: T,
        izz: T,
        ixy: T,
        ixz: T,
        iyz: T,
    ) -> Result<Self, MassPropertiesErrors> {
        if mass <= T::zero() {
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
    pub fn get_cmx(&self) -> T {
        self.center_of_mass.get_cmx()
    }

    /// Returns the y-coordinate of the center of mass.
    pub fn get_cmy(&self) -> T {
        self.center_of_mass.get_cmy()
    }

    /// Returns the y-coordinate of the center of mass.
    pub fn get_cmz(&self) -> T {
        self.center_of_mass.get_cmz()
    }

    /// Returns the moment of inertia around the x-axis.
    pub fn get_ixx(&self) -> T {
        self.inertia.get_ixx()
    }

    /// Returns the product of inertia for the xy-plane.
    pub fn get_ixy(&self) -> T {
        self.inertia.get_ixy()
    }

    /// Returns the product of inertia for the xz-plane.
    pub fn get_ixz(&self) -> T {
        self.inertia.get_ixz()
    }

    /// Returns the moment of inertia around the y-axis.
    pub fn get_iyy(&self) -> T {
        self.inertia.get_iyy()
    }

    /// Returns the product of inertia for the yz-plane.
    pub fn get_iyz(&self) -> T {
        self.inertia.get_iyz()
    }

    /// Returns the moment of inertia around the z-axis.
    pub fn get_izz(&self) -> T {
        self.inertia.get_izz()
    }

    /// Returns the mass of the object.
    pub fn get_mass(&self) -> T {
        self.mass
    }

    /// Sets the x-coordinate of the center of mass.
    pub fn set_cmx(&mut self, cmx: T) {
        self.center_of_mass.set_cmx(cmx);
    }

    /// Sets the y-coordinate of the center of mass.
    pub fn set_cmy(&mut self, cmy: T) {
        self.center_of_mass.set_cmy(cmy);
    }

    /// Sets the z-coordinate of the center of mass.
    pub fn set_cmz(&mut self, cmz: T) {
        self.center_of_mass.set_cmz(cmz);
    }

    /// Sets the moment of inertia around the x-axis.
    ///
    /// # Errors
    ///
    /// Returns a `MassPropertiesError::IxxLessThanOrEqualToZero` if `ixx` is less than or equal to zero.
    pub fn set_ixx(&mut self, ixx: T) -> Result<(), MassPropertiesErrors> {
        match self.inertia.set_ixx(ixx) {
            Ok(_) => Ok(()),
            Err(error) => Err(MassPropertiesErrors::Inertia(error)),
        }
    }

    /// Sets the product of inertia for the xy-plane.
    pub fn set_ixy(&mut self, ixy: T) {
        self.inertia.set_ixy(ixy);
    }

    /// Sets the product of inertia for the xz-plane.
    pub fn set_ixz(&mut self, ixz: T) {
        self.inertia.set_ixz(ixz);
    }
    /// Sets the moment of inertia around the y-axis.
    ///
    /// # Errors
    ///
    /// Returns a `MassPropertiesError::IyyLessThanOrEqualToZero` if `iyy` is less than or equal to zero.
    pub fn set_iyy(&mut self, iyy: T) -> Result<(), MassPropertiesErrors> {
        match self.inertia.set_iyy(iyy) {
            Ok(_) => Ok(()),
            Err(error) => Err(MassPropertiesErrors::Inertia(error)),
        }
    }

    /// Sets the product of inertia for the yz-plane.
    pub fn set_iyz(&mut self, iyz: T) -> Result<(), MassPropertiesErrors> {
        self.inertia.set_iyz(iyz);
        Ok(())
    }

    /// Sets the moment of inertia around the z-axis.
    ///
    /// # Errors
    ///
    /// Returns a `MassPropertiesError::IzzLessThanOrEqualToZero` if `izz` is less than or equal to zero.
    pub fn set_izz(&mut self, izz: T) -> Result<(), MassPropertiesErrors> {
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
    pub fn set_mass(&mut self, mass: T) -> Result<(), MassPropertiesErrors> {
        if mass <= T::zero() {
            return Err(MassPropertiesErrors::MassLessThanOrEqualToZero);
        }
        self.mass = mass;
        Ok(())
    }
}
