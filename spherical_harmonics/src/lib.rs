use legendre::{Legendre, LegendreErrors, LegendreNormalization};
use nalgebra::Vector3;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum SphericalHarmonicsErrors {
    #[error("LegendreError: {0}")]
    LegendreErrors(#[from] LegendreErrors),
}

#[derive(Clone, Debug, Default)]
pub struct SphericalHarmonics {
    degree: usize,
    order: usize,
    legendre: Legendre,
    cml: Vec<f64>, // preallocation for cos(m lon)
    sml: Vec<f64>, // preallocation for sin(m lon)
}

impl SphericalHarmonics {
    pub fn new(degree: usize, order: usize) -> Result<Self, SphericalHarmonicsErrors> {
        Ok(Self {
            degree,
            order,
            legendre: Legendre::new(degree, order)?.with_derivatives(),
            cml: vec![0.0; order + 1],
            sml: vec![0.0; order + 1],
        })
    }

    pub fn with_normalization(mut self, normalization: LegendreNormalization) -> Self {
        self.legendre = self.legendre.with_normalization(normalization);
        self
    }

    /// returns the gradient of V
    pub fn calculate_from_colatitude(
        &mut self,
        r: f64,
        colat: f64,
        lon: f64,
        k: f64,
        a: f64,
        g: &Vec<Vec<f64>>,
        h: &Vec<Vec<f64>>,
    ) -> Result<Vector3<f64>, SphericalHarmonicsErrors> {
        // we calculate trig functions in a recursion since it's faster than using cos and sin on each element
        let cml = &mut self.cml;
        let sml = &mut self.sml;

        cml[0] = 1.0;
        sml[0] = 0.0;
        if self.order > 0 {
            cml[1] = lon.cos();
            sml[1] = lon.sin();
            if self.order > 1 {
                for m in 2..=self.order {
                    cml[m] = cml[m - 1] * cml[1] - sml[m - 1] * sml[1];
                    sml[m] = sml[m - 1] * cml[1] + cml[m - 1] * sml[1];
                }
            }
        }
        let x = colat.cos();
        let sin_colat = (1.0 - x * x).sqrt();
        self.legendre.calculate(x)?;

        let mut partial_r = 0.0;
        let mut partial_colat = 0.0;
        let mut partial_lon = 0.0;

        let p = &self.legendre.p;
        let dp = match &self.legendre.dp {
            Some(dp) => dp,
            None => unreachable!("derivatives always initiliazed in new"),
        };

        let ar = a / r;
        let mut arl = ar;
        for l in 1..=self.degree {
            let lf = l as f64;
            arl *= ar;
            for m in 0..=l {
                if m <= self.order {
                    let mf = m as f64;
                    let gchs = g[l][m] * cml[m] + h[l][m] * sml[m];
                    let hcgs = h[l][m] * cml[m] - g[l][m] * sml[m];

                    partial_r += arl * (lf + 1.0) * p[l][m] * gchs;
                    partial_colat += arl * dp[l][m] * gchs;
                    partial_lon += arl * mf * p[l][m] * hcgs;
                }
            }
        }

        partial_r *= -k / r;
        partial_colat *= -sin_colat * k;
        partial_lon *= k;

        // negative since -grad(V)
        let ar = -partial_r;
        let acolat = -partial_colat / r;
        let alon = -partial_lon / (r * sin_colat);
        Ok(Vector3::new(ar, acolat, alon))
    }

    pub fn calculate_from_cartesian(
        &mut self,
        r_ecef: &Vector3<f64>,
        k: f64,
        a: f64,
        c: &Vec<Vec<f64>>,
        s: &Vec<Vec<f64>>,
    ) -> Result<Vector3<f64>, SphericalHarmonicsErrors> {
        let x = r_ecef[0];
        let y = r_ecef[1];
        let z = r_ecef[2];
        let r = (x * x + y * y + z * z).sqrt();

        let colat = (z / r).acos();
        let lon = y.atan2(x);

        let a_spherical = self.calculate_from_colatitude(r, colat, lon, k, a, c, s)?;
        dbg!(a_spherical);
        let ar = a_spherical[0];
        let acolat = a_spherical[1];
        let alon = a_spherical[2];

        let ax = (ar * colat.sin() + acolat * colat.cos()) * lon.cos() - alon * lon.sin();
        let ay = (ar * colat.sin() + acolat * colat.cos()) * lon.sin() + alon * lon.cos();
        let az = ar * colat.cos() - acolat * colat.sin();
        Ok(Vector3::new(ax, ay, az))
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
