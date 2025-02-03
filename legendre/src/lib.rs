use thiserror::Error;

#[derive(Debug, Default, Clone, Copy)]
pub enum LegendreNormalization {
    FourPi,
    #[default]
    Full,
    SchmidtQuasi,
    Unnormalized,
}

#[derive(Error, Debug)]
pub enum LegendreErrors {
    #[error("order must be less than or equal to degree")]
    OrderGreaterThanDegree,
    #[error("value must be between -1 and 1 inclusive")]
    ValueOutOfRange,
}

/// Calculates legendre functions
/// // just need default so we can deserialize, will get reinitialized after that
#[derive(Debug, Clone, Default)]
pub struct Legendre {
    pub p: Vec<Vec<f64>>,
    pub dp: Option<Vec<Vec<f64>>>,
    degree: usize,
    order: usize,
    condon_shortley: bool,
    normalization: LegendreNormalization,
    norm: Vec<Vec<f64>>,
}

impl Legendre {
    pub fn new(degree: usize, order: usize) -> Result<Self, LegendreErrors> {
        if order > degree {
            return Err(LegendreErrors::OrderGreaterThanDegree);
        }
        // need to go to + 2 for derivative recursion
        let mut p = vec![vec![0.0; order + 2]; degree + 2];
        p[0][0] = 1.0;

        // populate norm with all 1's to start. updated in with_normalization
        let mut norm = vec![vec![0.0; order + 2]; degree + 2];
        for l in 0..=degree + 1 {
            for m in 0..=l {
                if m <= order {
                    norm[l][m] = 1.0
                }
            }
        }

        Ok(Self {
            p,
            dp: None,
            degree,
            order,
            condon_shortley: false,
            normalization: LegendreNormalization::Unnormalized,
            norm,
        })
    }

    /// just preallocates at construction
    pub fn with_derivatives(mut self) -> Self {
        let mut dp = self.p.clone();
        dp[0][0] = 0.0;
        // the rest get overwritten in calculate

        self.dp = Some(dp);
        self
    }

    pub fn with_condon_shortley(mut self) -> Self {
        self.condon_shortley = true;
        // lump condon-shortley factor into the norm
        for l in 0..=self.degree + 1 {
            for m in 0..=l {
                self.norm[l][m] *= (-1.0_f64).powi(m as i32);
            }
        }
        self
    }

    pub fn with_normalization(mut self, norm: LegendreNormalization) -> Self {
        // precompute norm factors for l and m
        for l in 0..=self.degree + 1 {
            let lf = l as f64;
            for m in 0..=l {
                if m <= self.order + 1 {
                    let mf = m as f64;
                    self.norm[l][m] = match norm {
                        LegendreNormalization::Unnormalized => 1.0,
                        LegendreNormalization::FourPi => {
                            let delta = if m == 0 { 1.0 } else { 0.0 };
                            ((2.0 - delta) * (2.0 * lf + 1.0) * factorial(lf - mf)
                                / factorial(lf + mf))
                            .sqrt()
                        }
                        LegendreNormalization::Full => {
                            //let k = if m == 0 { 1.0 } else { 2.0 };
                            //(k * (2.0 * lf + 1.0) * factorial(lf - mf) / (2.0 * factorial(lf + mf)))
                            ((2.0 * lf + 1.0) * factorial(lf - mf) / (2.0 * factorial(lf + mf)))
                                .sqrt()
                        }
                        LegendreNormalization::SchmidtQuasi => {
                            let delta = if m == 0 { 1.0 } else { 0.0 };
                            ((2.0 - delta) * factorial(lf - mf) / factorial(lf + mf)).sqrt()
                        }
                    };
                }

                // lump condon-shortley factor into the norm
                if self.condon_shortley {
                    self.norm[l][m] *= (-1.0_f64).powi(m as i32);
                }
            }
        }
        self.normalization = norm;
        self
    }

    pub fn calculate(&mut self, x: f64) -> Result<(), LegendreErrors> {
        if !(x >= -1.0 && x <= 1.0) {
            return Err(LegendreErrors::ValueOutOfRange);
        }

        let one_minus_x_2 = 1.0 - x * x;
        // first couple values, p[0][0] is always 1.0 from initialization

        self.p[1][0] = x;
        if self.order > 0 {
            self.p[1][1] = one_minus_x_2.sqrt();
        }

        for l in 2..=self.degree + 1 {
            let lf = l as f64;

            for m in 0..=l {
                if m <= self.order + 1 {
                    let p = &mut self.p;

                    if m == 0 {
                        p[l][0] = ((2.0 * lf - 1.0) * p[1][0] * p[l - 1][0]
                            - (lf - 1.0) * p[l - 2][0])
                            / lf; //zonal
                    } else {
                        p[l][m] = if m == l {
                            (2.0 * lf - 1.0) * p[1][1] * p[l - 1][m - 1] // sectoral
                        } else {
                            p[l - 2][m] + (2.0 * lf - 1.0) * p[1][1] * p[l - 1][m - 1]
                            // tesseral
                        };
                    }
                }
            }
        }
        // apply normalization and condon-shortley after recursion
        for l in 0..=self.degree + 1 {
            for m in 0..=l {
                if m <= self.order + 1 {
                    self.p[l][m] *= self.norm[l][m];
                }
            }
        }

        // calculate derivatives
        if let Some(dp) = &mut self.dp {
            for l in 1..=self.degree {
                let lf = l as f64;
                for m in 0..=l {
                    if m <= self.order {
                        let mf = m as f64;
                        dp[l][m] =
                            ((lf + mf) * self.p[l - 1][m] - lf * x * self.p[l][m]) / one_minus_x_2;
                    }
                }
            }
        }

        Ok(())
    }
}
// factorial needs to bef64 for legendre or we over flow at like order,degree 10,10.
pub fn factorial(n: f64) -> f64 {
    let mut result = 1.0;
    let mut i = 1.0;
    while i <= n + 0.1 {
        //0.1 for some margin since f64! and not int64!
        result = result * i;
        i += 1.0;
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use utilities::assert_equal;

    #[test]
    fn test_legendre_1() {
        let mut legendre = Legendre::new(18, 18).unwrap();
        legendre.calculate(0.5).unwrap();

        assert_equal(legendre.p[0][0], 1.0);
        assert_equal(legendre.p[1][0], 0.5);
        assert_equal(legendre.p[1][1], 0.8660254037844386);
        assert_equal(legendre.p[2][0], -0.125);
        assert_equal(legendre.p[2][1], 1.299038105676658);
        assert_equal(legendre.p[2][2], 2.25);
        assert_equal(legendre.p[6][0], 0.3232421875);
        assert_equal(legendre.p[6][3], -12.787406352754603);
        assert_equal(legendre.p[6][6], 4385.390624999996);
        assert_equal(legendre.p[10][0], -0.18822860717773438);
        assert_equal(legendre.p[10][3], 259.1875968441698);
        assert_equal(legendre.p[10][6], -82397.37854003915);
        assert_equal(legendre.p[10][10], 1.55370278540039e8);
        assert_equal(legendre.p[18][18], 16642002289840226304.0);
    }

    #[test]
    fn test_legendre_2() {
        let mut legendre = Legendre::new(50, 50).unwrap();
        legendre.calculate(-0.99).unwrap();

        assert_equal(legendre.p[0][0], 1.0);
        assert_equal(legendre.p[1][0], -0.99);
        assert_equal(legendre.p[1][1], 0.14106735979665894);
        assert_equal(legendre.p[2][0], 0.9701499999999998);
        assert_equal(legendre.p[2][1], -0.41897005859607717);
        assert_equal(legendre.p[2][2], 0.059700000000000066);
        assert_equal(legendre.p[12][0], 0.3581855121242232);
        assert_equal(legendre.p[12][3], -175.30908571682124);
        assert_equal(legendre.p[12][12], 19.639354822743886);
        assert_equal(legendre.p[47][0], -0.28768056966031036);
        assert_equal(legendre.p[47][3], -10385.344699425346);
        assert_equal(legendre.p[47][6], -3.605392577986418e9);
        assert_equal(legendre.p[47][47], 3.150255369803779e32);
    }

    #[test]
    fn test_legendre_condon_shortley() {
        let mut legendre = Legendre::new(10, 10).unwrap().with_condon_shortley();
        legendre.calculate(0.5).unwrap();

        assert_equal(legendre.p[0][0], 1.0);
        assert_equal(legendre.p[1][0], 0.5);
        assert_equal(legendre.p[1][1], -0.8660254037844386);
        assert_equal(legendre.p[2][0], -0.125);
        assert_equal(legendre.p[2][1], -1.299038105676658);
        assert_equal(legendre.p[2][2], 2.25);
        assert_equal(legendre.p[6][0], 0.3232421875);
        assert_equal(legendre.p[6][3], 12.787406352754603);
        assert_equal(legendre.p[6][6], 4385.390624999996);
        assert_equal(legendre.p[10][0], -0.18822860717773438);
        assert_equal(legendre.p[10][3], -259.1875968441698);
        assert_equal(legendre.p[10][6], -82397.37854003915);
        assert_equal(legendre.p[10][10], 1.55370278540039e8);
    }

    #[test]
    fn test_legendre_normalization_full() {
        let mut legendre = Legendre::new(10, 10)
            .unwrap()
            .with_normalization(LegendreNormalization::Full)
            .with_condon_shortley();
        legendre.calculate(0.5).unwrap();

        assert_equal(legendre.p[0][0], 0.7071067811865476);
        assert_equal(legendre.p[1][0], 0.6123724356957945);
        assert_equal(legendre.p[1][1], -0.75);
        assert_equal(legendre.p[2][2], 0.7261843774138904);
        assert_equal(legendre.p[6][6], 0.5108536257714203);
        assert_equal(legendre.p[10][0], -0.6099303975706846);
        assert_equal(legendre.p[10][10], 0.3227752940307167);
    }

    #[test]
    fn test_legendre_derivative() {
        // independent values from pyshtools
        let mut legendre = Legendre::new(10, 10).unwrap().with_derivatives();
        legendre.calculate(0.5).unwrap();
        let dp = legendre.dp.unwrap();
        assert_equal(dp[0][0], 0.0);
        assert_equal(dp[1][0], 1.0);
        assert_equal(dp[1][1], -0.5773502691896257);
        assert_equal(dp[2][0], 1.5);
        assert_equal(dp[2][1], 1.7320508075688774);
        assert_equal(dp[2][2], -3.0);
    }

    #[test]
    fn test_legendre_normalization_schmidt() {
        let mut legendre = Legendre::new(10, 10)
            .unwrap()
            .with_normalization(LegendreNormalization::SchmidtQuasi);
        legendre.calculate(0.5).unwrap();

        assert_equal(legendre.p[0][0], 1.0);
        assert_equal(legendre.p[1][0], 0.5);
        assert_equal(legendre.p[1][1], 0.8660254037844386);
        assert_equal(legendre.p[2][2], 0.6495190528383288);
        assert_equal(legendre.p[6][6], 0.2833706064577766);
        assert_equal(legendre.p[10][0], -0.18822860717773438);
        assert_equal(legendre.p[10][10], 0.14087068736737018);
    }
}
