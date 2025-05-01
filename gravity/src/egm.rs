use legendre::{LegendreErrors, LegendreNormalization, factorial};
use nalgebra::Vector3;
use serde::de::{self, MapAccess, Visitor};
use serde::{Deserialize, Deserializer, Serialize};
use spherical_harmonics::{SphericalHarmonics, SphericalHarmonicsErrors};
use thiserror::Error;

use crate::{GravityErrors, GravityModel};

#[derive(Debug, Error)]
pub enum EgmErrors {
    #[error("SphericalError: {0}")]
    SphericalHarmonicsError(#[from] SphericalHarmonicsErrors),
    #[error("LegendreError: {0}")]
    LegendreError(#[from] LegendreErrors),
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum EgmModel {
    Egm96,
    Egm2008,
}

pub struct EgmGravityBuilder {}

#[derive(Clone, Debug, Serialize)]
pub struct EgmGravity {
    pub model: EgmModel,
    degree: usize,
    order: usize,
    add_centrifugal: bool,
    add_newtonian: bool,
    #[serde(skip)]
    spherical_harmonics: SphericalHarmonics,
    #[serde(skip)]
    c: Vec<Vec<f64>>,
    #[serde(skip)]
    s: Vec<Vec<f64>>,
}

impl EgmGravity {
    const RE: f64 = 6.3781363e6; // radius of WGS84 earth
    const MU: f64 = 3.986004415e14; // gravitational parameter of WGS84 earth
    const WE: [f64; 3] = [0.0, 0.0, 7.292115e-5]; // rotation rate of WGS84 earth
    const EGM96_DATA: &str = include_str!("../resources/EGM96_50.txt");
    const EGM2008_DATA: &str = include_str!("../resources/EGM2008_50.txt");

    pub fn new(model: EgmModel, degree: usize, order: usize) -> Result<Self, EgmErrors> {
        let (c, s) = Self::parse_file(&model, degree, order);
        let spherical_harmonics =
            SphericalHarmonics::new(degree, order)?.with_normalization(LegendreNormalization::Full);

        Ok(Self {
            model,
            degree,
            order,
            spherical_harmonics,
            c,
            s,
            add_newtonian: false,
            add_centrifugal: false,
        })
    }

    /// By default this gravity model just provides the acceleration perturbation
    /// This includes the newtonian gravity in the calculation
    pub fn with_newtonian(mut self) -> Self {
        self.add_newtonian = true;
        self
    }

    /// By default this gravity model does not include centrifugal acceleration
    /// This includes the centrifugal acceleration in the calculation
    pub fn with_centrifugal(mut self) -> Self {
        self.add_centrifugal = true;
        self
    }

    fn parse_file(model: &EgmModel, degree: usize, order: usize) -> (Vec<Vec<f64>>, Vec<Vec<f64>>) {
        let mut c = vec![vec![0.0; order + 1]; degree + 1];
        let mut s = vec![vec![0.0; order + 1]; degree + 1];

        let mut lines = match model {
            EgmModel::Egm96 => Self::EGM96_DATA.lines(),
            EgmModel::Egm2008 => Self::EGM2008_DATA.lines(),
        };

        for line in &mut lines {
            let columns: Vec<&str> = line.split_whitespace().collect();
            if columns.len() == 6 {
                match (
                    columns[0].parse::<usize>(),
                    columns[1].parse::<usize>(),
                    columns[2].parse::<f64>(),
                    columns[3].parse::<f64>(),
                ) {
                    (Ok(l), Ok(m), Ok(c_file), Ok(s_file)) => {
                        // TODO: never mind on normalizing, assume theyre normalized already
                        // implement a normalize method if coefficients need to be normalized
                        if l <= degree && m <= order {
                            let lf = l as f64;
                            let mf = m as f64;
                            let k = if m == 0 { 1.0 } else { 2.0 };

                            // normalize the coeffs
                            let n = (factorial(lf + mf)
                                / (factorial(lf - mf) * k * (2.0 * lf + 1.0)))
                                .sqrt();

                            c[l][m] = c_file / n;
                            s[l][m] = s_file / n;
                        }
                    }
                    _ => eprintln!("Failed to parse one or more columns in line: {}", line),
                }
            } else {
                eprintln!("Unexpected number of columns: {}", columns.len());
            }
        }
        (c, s)
    }
}

impl GravityModel for EgmGravity {
    fn calculate(&mut self, r_ecef: &Vector3<f64>) -> Result<Vector3<f64>, GravityErrors> {
        let mut a = self
            .spherical_harmonics
            .calculate_from_cartesian(r_ecef, Self::MU / Self::RE, Self::RE, &self.c, &self.s)
            .map_err(|e| GravityErrors::EgmErrors(e.into()))?;

        let r = r_ecef.norm();
        if self.add_newtonian {
            a += -r_ecef * Self::MU / r.powi(3);
        };
        if self.add_centrifugal {
            let we = Vector3::from(Self::WE);
            a += we.cross(&we.cross(&r_ecef));
        };

        Ok(a)
    }
}

impl<'de> Deserialize<'de> for EgmGravity {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        // Define a visitor to handle deserialization
        struct EgmVisitor;

        impl<'de> Visitor<'de> for EgmVisitor {
            type Value = EgmGravity;

            fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
                formatter.write_str("struct EgmGravity")
            }

            fn visit_map<V>(self, mut map: V) -> Result<EgmGravity, V::Error>
            where
                V: MapAccess<'de>,
            {
                // Temporary variables to hold deserialized values
                let mut model = None;
                let mut degree = None;
                let mut order = None;
                let mut add_centrifugal = None;
                let mut add_newtonian = None;

                // Extract values from the map
                while let Some(key) = map.next_key()? {
                    match key {
                        "model" => {
                            if model.is_some() {
                                return Err(de::Error::duplicate_field("model"));
                            }
                            model = Some(map.next_value()?);
                        }
                        "degree" => {
                            if degree.is_some() {
                                return Err(de::Error::duplicate_field("degree"));
                            }
                            degree = Some(map.next_value()?);
                        }
                        "order" => {
                            if order.is_some() {
                                return Err(de::Error::duplicate_field("order"));
                            }
                            order = Some(map.next_value()?);
                        }
                        "add_centrifugal" => {
                            if add_centrifugal.is_some() {
                                return Err(de::Error::duplicate_field("add_centrifugal"));
                            }
                            add_centrifugal = Some(map.next_value()?);
                        }
                        "add_newtonian" => {
                            if add_newtonian.is_some() {
                                return Err(de::Error::duplicate_field("add_newtonian"));
                            }
                            add_newtonian = Some(map.next_value()?);
                        }
                        _ => {
                            return Err(de::Error::unknown_field(key, FIELDS));
                        }
                    }
                }

                // Ensure all required fields are present
                let model = model.ok_or_else(|| de::Error::missing_field("model"))?;
                let degree = degree.ok_or_else(|| de::Error::missing_field("degree"))?;
                let order = order.ok_or_else(|| de::Error::missing_field("order"))?;
                let add_centrifugal = add_centrifugal.unwrap_or(false);
                let add_newtonian = add_newtonian.unwrap_or(false);

                // Reinitialize fields after deserialization
                let (c, s) = EgmGravity::parse_file(&model, degree, order);
                let spherical_harmonics = SphericalHarmonics::new(degree, order)
                    .map_err(de::Error::custom)?
                    .with_normalization(LegendreNormalization::Full);

                Ok(EgmGravity {
                    model,
                    degree,
                    order,
                    add_centrifugal,
                    add_newtonian,
                    spherical_harmonics,
                    c,
                    s,
                })
            }
        }

        // Define the fields expected in the input
        const FIELDS: &'static [&'static str] = &[
            "model",
            "degree",
            "order",
            "add_centrifugal",
            "add_newtonian",
        ];

        // Deserialize the struct using the visitor
        deserializer.deserialize_struct("EgmGravity", FIELDS, EgmVisitor)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use utilities::assert_equal;

    #[test]
    fn test_egm96_1() {
        let re = [7e6, 0.0, 0.0];

        let mut g = EgmGravity::new(EgmModel::Egm96, 10, 10)
            .unwrap()
            .with_newtonian()
            .with_centrifugal();
        let a = g.calculate(&re.into()).unwrap();
        dbg!(a);
        assert_equal(a[0], -8.145745669956069);
        assert_equal(a[1], -2.191201471327777e-5);
        assert_equal(a[2], 3.013126351887712e-5);
    }
}
