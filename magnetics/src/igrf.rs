use chrono::Datelike;
use legendre::{LegendreErrors, LegendreNormalization};
use nalgebra::Vector3;
use serde::de::{self, MapAccess, Visitor};
use serde::{Deserialize, Deserializer, Serialize};
use spherical_harmonics::{SphericalHarmonics, SphericalHarmonicsErrors};
use thiserror::Error;
use time::Time;

#[derive(Debug, Error)]
pub enum IgrfErrors {
    #[error("LegendreError: {0}")]
    LegendreError(#[from] LegendreErrors),
    #[error("SphericalHarmonicsError: {0}")]
    SphericalHarmonicsError(#[from] SphericalHarmonicsErrors),
    #[error("could not parse igrf file")]
    ParseError,
    #[error("out of bounds index of epoch array")]
    InvalidEpochIndex,
    #[error(
        "epoch is before fisrt index of igrf coeffs, provide an initial year that covers the epoch at initialization"
    )]
    EpochTooEarly,
}

#[derive(Clone, Debug)]
struct IgrfCoeffs(Vec<Vec<Vec<f64>>>);

/// coefs from https://www.ngdc.noaa.gov/IAGA/vmod/coeffs/igrf14coeffs.txt
#[derive(Clone, Debug, Serialize)]
pub struct Igrf {
    initial_year: i32, // just needed for deserialization
    degree: usize,
    order: usize,
    #[serde(skip)]
    pub spherical_harmonics: SphericalHarmonics,
    #[serde(skip)]
    g: IgrfCoeffs,
    #[serde(skip)]
    h: IgrfCoeffs,
    #[serde(skip)]
    svg: Vec<Vec<f64>>,
    #[serde(skip)]
    svh: Vec<Vec<f64>>,
    #[serde(skip)]
    epochs: Vec<f64>,
    #[serde(skip)]
    g_cache: Vec<Vec<f64>>, // preallocation for interp/extrap igrf coefs
    #[serde(skip)]
    h_cache: Vec<Vec<f64>>, // preallocation for interp/extrap igrf coefs
}

impl Igrf {
    const RE: f64 = 6.3712e6; // radius of WGS84 earth
    const IGRF_DATA: &str = include_str!("../resources/igrf14coeffs.txt");

    pub fn new(degree: usize, order: usize, initial_epoch: &Time) -> Result<Self, IgrfErrors> {
        // Skip the first 3 header lines
        let mut lines = Self::IGRF_DATA
            .lines()
            .skip(3);

        // Parse the fourth line to extract epochs
        let epochs_line = lines
            .next()
            .expect("Missing epochs header line")
            .split_whitespace()
            .collect::<Vec<&str>>();

        // Extract epochs, ignoring the first three columns and the last SV column
        let epochs_str = &epochs_line[3..epochs_line.len() - 1];

        let initial_year = initial_epoch
            .get_datetime()
            .year();
        let mut epochs = Vec::new();

        for &epoch_str in epochs_str {
            match epoch_str.parse::<f64>() {
                Ok(year) => {
                    epochs.push(year);
                }
                Err(_) => {
                    return Err(IgrfErrors::ParseError);
                }
            }
        }

        let mut g_coeffs = vec![vec![vec![0.0; order + 1]; degree + 1]; epochs.len()];
        let mut h_coeffs = vec![vec![vec![0.0; order + 1]; degree + 1]; epochs.len()];
        let mut svg = vec![vec![0.0; order + 1]; degree + 1];
        let mut svh = vec![vec![0.0; order + 1]; degree + 1];

        // Process each line of coefficients
        while let Some(line) = lines.next() {
            let mut columns: Vec<&str> = line
                .split_whitespace()
                .collect();
            // get the secular variation coefficient
            let sv = columns
                .pop()
                .unwrap()
                .parse::<f64>()
                .expect("Invalid SV value");

            let coef_type = columns[0];
            let n = columns[1]
                .parse::<usize>()
                .expect("Invalid degree value");
            let m = columns[2]
                .parse::<usize>()
                .expect("Invalid order value");

            if n <= degree && m <= order {
                let coeffs: Vec<f64> = columns[3..]
                    .iter()
                    .map(|s| {
                        s.parse::<f64>()
                            .expect("Invalid coefficient value")
                    })
                    .collect();

                if coeffs.len() != epochs.len() {
                    return Err(IgrfErrors::ParseError);
                }

                match coef_type {
                    "g" => {
                        for y in 0..epochs.len() {
                            g_coeffs[y][n][m] = coeffs[y];
                        }
                        svg[n][m] = sv;
                    }
                    "h" => {
                        for y in 0..epochs.len() {
                            h_coeffs[y][n][m] = coeffs[y];
                        }
                        svh[n][m] = sv;
                    }
                    _ => eprintln!(
                        "Unexpected coefficient type: {}",
                        coef_type
                    ),
                }
            }
        }

        Ok(Self {
            initial_year,
            degree,
            order,
            spherical_harmonics: SphericalHarmonics::new(degree, order)?
                .with_normalization(LegendreNormalization::SchmidtQuasi),
            g: IgrfCoeffs(g_coeffs),
            h: IgrfCoeffs(h_coeffs),
            svg,
            svh,
            epochs,
            g_cache: vec![vec![0.0; order + 1]; degree + 1],
            h_cache: vec![vec![0.0; order + 1]; degree + 1],
        })
    }

    fn search_epoch(&mut self, decimal_year: f64) -> Result<(usize, InterpExtrap), IgrfErrors> {
        if decimal_year < self.epochs[0] {
            return Err(IgrfErrors::EpochTooEarly);
        }

        let mut index = 0;
        while index
            < self
                .epochs
                .len()
                - 1
            && decimal_year > self.epochs[index]
        {
            index += 1;
        }
        index -= 1; // take one away since it was found on previous iteration

        let method = if index
            == self
                .epochs
                .len()
                - 1
        {
            if decimal_year > self.epochs[index] + 5.0 {
                println!(
                    "Warning: IGRF coeffs are extrapolated more than 5 years, may not be accurate"
                )
            }
            InterpExtrap::Extrapolate
        } else {
            InterpExtrap::Interpolate
        };
        Ok((index, method))
    }

    fn calculate_gh(&mut self, decimal_year: f64) -> Result<(), IgrfErrors> {
        // determine the lower index of the igrf year/coeffs, and determine if we're interping or extraping
        let (index, method) = self.search_epoch(decimal_year)?;
        match method {
            InterpExtrap::Extrapolate => {
                dbg!("extrapolate");
                // Time difference from the latest epoch
                let time_diff = decimal_year - self.epochs[index];

                // Extrapolate 'g' coefficients using SV
                let g_latest = &self
                    .g
                    .0[index];
                for l in 0..g_latest.len() {
                    for m in 0..g_latest[l].len() {
                        self.g_cache[l][m] = g_latest[l][m] + self.svg[l][m] * time_diff;
                    }
                }

                // Extrapolate 'h' coefficients using SV
                let h_latest = &self
                    .h
                    .0[index];
                for l in 0..h_latest.len() {
                    for m in 0..h_latest[l].len() {
                        self.h_cache[l][m] = h_latest[l][m] + self.svh[l][m] * time_diff;
                    }
                }
            }
            InterpExtrap::Interpolate => {
                let interp_factor = (decimal_year - self.epochs[index])
                    / (self.epochs[index + 1] - self.epochs[index]);

                let g0 = &self
                    .g
                    .0[index];
                let g1 = &self
                    .g
                    .0[index + 1];
                for l in 0..g0.len() {
                    for m in 0..g0[l].len() {
                        self.g_cache[l][m] = (g1[l][m] - g0[l][m]) * interp_factor + g0[l][m];
                    }
                }

                let h0 = &self
                    .h
                    .0[index];
                let h1 = &self
                    .h
                    .0[index + 1];
                for l in 0..h0.len() {
                    for m in 0..h0[l].len() {
                        self.h_cache[l][m] = (h1[l][m] - h0[l][m]) * interp_factor + h0[l][m];
                    }
                }
            }
        }
        Ok(())
    }

    pub fn calculate_ecef(
        &mut self,
        r_ecef: &Vector3<f64>,
        epoch: &Time,
    ) -> Result<Vector3<f64>, IgrfErrors> {
        // linearly interpolate between igrf epochs (5 years) based on current epoch
        let datetime = epoch.get_datetime();
        let epoch_year = datetime.year() as f64;
        let epoch_day = datetime.ordinal0() as f64;
        let decimal_year = if datetime
            .date()
            .leap_year()
        {
            epoch_year + epoch_day / 366.0
        } else {
            epoch_year + epoch_day / 365.0
        };

        // calculate g and h based on interp or extrap
        self.calculate_gh(decimal_year)?;

        let b = self
            .spherical_harmonics
            .calculate_from_cartesian(
                r_ecef,
                Self::RE,
                Self::RE,
                &self.g_cache,
                &self.h_cache,
            )?;

        Ok(b)
    }

    pub fn calculate_spherical(
        &mut self,
        r: f64,
        colat: f64,
        lon: f64,
        epoch: &Time,
    ) -> Result<Vector3<f64>, IgrfErrors> {
        // linearly interpolate between igrf epochs (5 years) based on current epoch
        let datetime = epoch.get_datetime();
        let epoch_year = datetime.year() as f64;
        let epoch_day = datetime.ordinal0() as f64;
        let decimal_year = if datetime
            .date()
            .leap_year()
        {
            epoch_year + epoch_day / 366.0
        } else {
            epoch_year + epoch_day / 365.0
        };

        // calculate g and h based on interp or extrap
        self.calculate_gh(decimal_year)?;

        let b = self
            .spherical_harmonics
            .calculate_from_colatitude(
                r,
                colat,
                lon,
                Self::RE,
                Self::RE,
                &self.g_cache,
                &self.h_cache,
            )?;

        Ok(b)
    }
}

enum InterpExtrap {
    Interpolate,
    Extrapolate,
}

impl<'de> Deserialize<'de> for Igrf {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        // Define a visitor to handle deserialization
        struct IgrfVisitor;

        impl<'de> Visitor<'de> for IgrfVisitor {
            type Value = Igrf;

            fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
                formatter.write_str("struct IgrfGravity")
            }

            fn visit_map<V>(self, mut map: V) -> Result<Igrf, V::Error>
            where
                V: MapAccess<'de>,
            {
                // Temporary variables to hold deserialized values
                let initial_year = None;
                let mut degree = None;
                let mut order = None;

                // Extract values from the map
                while let Some(key) = map.next_key()? {
                    match key {
                        "initial_year" => {
                            if initial_year.is_some() {
                                return Err(de::Error::duplicate_field(
                                    "initial_year",
                                ));
                            }
                            degree = Some(map.next_value()?);
                        }
                        "degree" => {
                            if degree.is_some() {
                                return Err(de::Error::duplicate_field(
                                    "degree",
                                ));
                            }
                            degree = Some(map.next_value()?);
                        }
                        "order" => {
                            if order.is_some() {
                                return Err(de::Error::duplicate_field(
                                    "order",
                                ));
                            }
                            order = Some(map.next_value()?);
                        }
                        _ => {
                            return Err(de::Error::unknown_field(
                                key, FIELDS,
                            ));
                        }
                    }
                }

                // Ensure all required fields are present
                let initial_year =
                    initial_year.ok_or_else(|| de::Error::missing_field("initial_year"))?;
                let degree = degree.ok_or_else(|| de::Error::missing_field("degree"))?;
                let order = order.ok_or_else(|| de::Error::missing_field("order"))?;

                // Reinitialize fields after deserialization
                // time doesnt really matter, we just need to give it the year to intialize igrf
                let time = Time::from_ymdhms(
                    initial_year,
                    1,
                    1,
                    0,
                    0,
                    0.0,
                    time::TimeSystem::UTC,
                )
                .unwrap();
                Ok(Igrf::new(degree, order, &time).unwrap())
            }
        }

        // Define the fields expected in the input
        const FIELDS: &'static [&'static str] = &["degree", "order"];

        // Deserialize the struct using the visitor
        deserializer.deserialize_struct(
            "IgrfGravity",
            FIELDS,
            IgrfVisitor,
        )
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::*;
    use utilities::{assert_equal, assert_equal_reltol};

    #[test]
    fn test_igrf_matches_dipole() {
        let t = Time::from_ymdhms(
            2005,
            1,
            1,
            0,
            0,
            0.0,
            time::TimeSystem::UTC,
        )
        .unwrap();
        let r = Vector3::new(7e6, 0.0, 0.0);

        let mut igrf = Igrf::new(1, 1, &t).unwrap();
        let b = igrf
            .calculate_ecef(&r, &t)
            .unwrap();

        assert_equal(b[0], -2516.9172529558114);
        assert_equal(b[1], -3828.7890240966663);
        assert_equal(b[2], 22284.101180829042);
    }

    #[test]
    fn test_igrf_13_matches_ngdc_1() {
        // we dont match exactly unfortunately.
        // it's possible this is due to the time system we use, or maybe the igrf system ( we use 14, other might have older models that extrapolate?)
        // julia satellitetoolbox doesnt match exactly either, but we match them exactly through igrf13
        // just check that we're very close (<1%?)

        let t = Time::from_ymdhms(
            2024,
            6,
            7,
            0,
            0,
            0.0,
            time::TimeSystem::UTC,
        )
        .unwrap();
        let r = 6.7e6;
        let theta = 60.0 * PI / 180.0; // colatitude
        let phi = 30.0 * PI / 180.0;

        let mut igrf = Igrf::new(13, 13, &t).unwrap();
        let b = igrf
            .calculate_spherical(r, theta, phi, &t)
            .unwrap();
        assert_equal_reltol(b[0], -25784.8, 0.01);
        assert_equal_reltol(b[1], -26473.1, 0.01);
        assert_equal_reltol(b[2], 1902.2, 0.01);
    }

    #[test]
    fn test_igrf_13_matches_ngdc_2() {
        // we dont match exactly unfortunately.
        // it's possible this is due to the time system we use, or maybe the igrf system ( we use 14, other might have older models that extrapolate?)
        // julia satellitetoolbox doesnt match exactly either, but we match them exactly through igrf13
        // just check that we're very close (<1%?)
        let t = Time::from_ymdhms(
            2028,
            11,
            30,
            0,
            0,
            0.0,
            time::TimeSystem::UTC,
        )
        .unwrap();
        let r = 7.7e6;
        let theta = 120.0 * PI / 180.0;
        let phi = -60.0 * PI / 180.0;

        let mut igrf = Igrf::new(13, 13, &t).unwrap();
        let b = igrf
            .calculate_spherical(r, theta, phi, &t)
            .unwrap();
        assert_equal_reltol(b[0], 8400.9, 0.01);
        assert_equal_reltol(b[1], -11168.5, 0.01);
        assert_equal_reltol(b[2], -1620.8, 0.01);
    }
}
