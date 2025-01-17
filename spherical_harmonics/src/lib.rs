use std::f64::consts::PI;

use legendre::{Legendre, LegendreErrors, LegendreNormalization};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum SphericalHarmonicsErrors {
    #[error("LegendreError: {0}")]
    LegendreErrors(#[from] LegendreErrors),
}

#[derive(Clone, Debug, Default)]
pub struct SphericalHarmonics {
    order: usize,
    legendre: Legendre,
}

impl SphericalHarmonics {
    pub fn new(degree: usize, order: usize) -> Result<Self, SphericalHarmonicsErrors> {
        // +1 since using Vallados method, which require P[l][m+1]
        Ok(Self {
            order,
            legendre: Legendre::new(degree + 1, order + 1)?.with_derivatives(),
        })
    }

    pub fn with_normalization(mut self, normalization: LegendreNormalization) -> Self {
        self.legendre = self.legendre.with_normalization(normalization);
        self
    }

    /// returns the gradient of V
    pub fn calculate(
        &mut self,
        r_ecef: [f64; 3],
        c: &Vec<Vec<f64>>,
        s: &Vec<Vec<f64>>,
        re: f64,
        mu: f64,
    ) -> Result<[f64; 3], SphericalHarmonicsErrors> {
        let x = r_ecef[0];
        let y = r_ecef[1];
        let z = r_ecef[2];
        let r = (x * x + y * y + z * z).sqrt();
        let xy = (x * x + y * y).sqrt();
        let rer = re / r;

        let latgc = (z / r).asin();
        let lon = {
            let lon = if xy < 1e-9 {
                y.signum() * PI * 0.5
            } else {
                y.atan2(x)
            };
            lon % (2.0 * PI)
        };

        self.legendre.calculate(z / r)?; // z/r = latgc.sin()

        let mut partial_r = 0.0;
        let mut partial_lat = 0.0;
        let mut partial_lon = 0.0;

        let p = &self.legendre.p;

        for l in 2..=self.order {
            let rerl = rer.powi(l as i32);
            for m in 0..=l {
                let mf = m as f64;

                let clm = (mf * lon).cos();
                let slm = (mf * lon).sin();

                partial_r += rerl * (l as f64 + 1.0) * p[l][m] * (c[l][m] * clm + s[l][m] * slm);
                partial_lat += rerl
                    * (p[l][m + 1] - mf * latgc.tan() * p[l][m])
                    * (c[l][m] * clm + s[l][m] * slm);
                partial_lon += rerl * mf * p[l][m] * (s[l][m] * clm - c[l][m] * slm);
            }
        }

        partial_r *= -mu / r.powi(2);
        partial_lat *= mu / r;
        partial_lon *= mu / r;

        let tmp1 = partial_r / r - z * partial_lat / (r.powi(2) * xy);
        let tmp2 = partial_lon / (xy * xy);

        let ax = tmp1 * x - tmp2 * y;
        let ay = tmp1 * y + tmp2 * x;
        let az = partial_r * z / r + xy * partial_lat / r.powi(2);

        Ok([ax, ay, az])
    }
}

#[cfg(test)]
mod tests {
    // use crate::egm96::EGM96;

    // use super::*;
    // use approx::assert_abs_diff_eq;
    // const TOL: f64 = 1e-10;

    // #[test]
    // fn test_spherical_harmonics() {
    //     let g = EGM96::new(3, 3);
    //     let mut spherical_harmonics = SphericalHarmonics::new(3, 3);
    //     let gradient = spherical_harmonics.calculate(&g.c, &g.s, 1e7, 0.0, 0.0, 1.0, 1.0);

    //     assert_abs_diff_eq!(gradient.x, 0.0, epsilon = TOL);
    //     assert_abs_diff_eq!(gradient.y, 0.0, epsilon = TOL);
    //     assert_abs_diff_eq!(gradient.z, 0.0, epsilon = TOL);
    // }
}
