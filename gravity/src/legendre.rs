use utilities::factorial;

#[derive(Debug, Clone, Copy)]
pub enum LegendreNormalization {
    FourPi,
    Full,
    None,    
    Schmidt,
    SchmidtQuasi,
    Standard,    
}
/// Calculates legendre functions 
/// Automatically implements the Condon-Shortley convention
#[derive(Debug, Clone)]
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
    pub fn new(degree: usize, order: usize) -> Self {        
        if degree < order {
            panic!("Legendre polynomial degree must be >= order")
        }
        let mut p = vec![vec![0.0; order + 1];degree + 1];
        // p00 is always 1.0 and this way we never have to set it again
        p[0][0] = 1.0;

        // populate norm with all 1's to start. updated in with_normalization
        let mut norm = vec![vec![0.0; order + 1];degree + 1];
        for l in 0..=degree {
            for m in 0..=l {
                norm[l][m] = 1.0
            }
        }

        Self { p , dp: None, degree, order, condon_shortley: false, normalization: LegendreNormalization::None, norm}
    }
    
    /// just preallocates at construction
    pub fn with_derivatives(mut self) -> Self {
        let mut dp = self.p.clone();
        dp[0][0] = 0.0;
        dp[1][0] = 1.0;

        self.dp = Some(dp);
        self
    }

    pub fn with_condon_shortley(mut self) -> Self {
        self.condon_shortley = true;
        // lump condon-shortley factor into the norm
        for l in 0..= self.degree {
            for m in 0..=l {
                self.norm[l][m] *= (-1.0_f64).powi(m as i32);
            }
        }
        self
    }

    pub fn with_normalization(mut self, norm: LegendreNormalization) -> Self {
        // precomute norm factors for l and m
        for l in 0..= self.degree {
            for m in 0..=l {
                self.norm[l][m] = match norm{
                    LegendreNormalization::FourPi => {
                        todo!()
                    }
                    LegendreNormalization::None => 1.0,
                    LegendreNormalization::Full => {                        
                        let num = (2 * l + 1) * factorial(l - m);
                        let denom = 2 * factorial(l + m);
                        (num as f64 / denom as f64).sqrt()
                    },
                    LegendreNormalization::Schmidt => {
                        todo!()
                    },
                    LegendreNormalization::SchmidtQuasi => {
                        todo!()
                    },
                    LegendreNormalization::Standard => {
                        todo!()
                    },
                };

                // lump condon-shortley factor into the norm        
                if self.condon_shortley {
                    self.norm[l][m] *= (-1.0_f64).powi(m as i32);                            
                }
                
            }
        }  
        self.normalization = norm;
        self
    }

    pub fn calculate(&mut self, x: f64) {                
        // first couple values, p[0][0] is always 1.0 from initialization        
        self.p[1][0] = x;        
        self.p[1][1] = (1.0 - x*x).sqrt();        

        if let Some(dp) = &mut self.dp {            
            dp[1][1] = x/(1.0 - x*x).sqrt();            
        }        

        for l in 2..=self.degree {
            let lf = l as f64;

            for m in 0..=l {                          
                if m <= self.order {                          
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
                        } ;
                    }                                        
                }
            }
        }
        // apply normalization and condon-shortley after recursion
        // p00 is still 1.0
        for l in 1..=self.degree {
            for m in 0..=l {
                if m <= self.order {
                    self.p[l][m] *= self.norm[l][m];
                }
            }
        }

        // calculate derivatives         
        if let Some(dp) = &mut self.dp {
            for l in 1..=self.degree {
                let lf = l as f64;
                for m in 0..=l {
                    let mf = m as f64;
                    dp[l][m] = ((lf + mf) * self.p[l-1][m] - lf * x * self.p[l][m])/ (1.0 - x * x);
                }
            }
        }
    }
}



#[cfg(test)]
mod tests {
    use super::*;
    use utilities::assert_equal;

    #[test]
    fn test_legendre_1() {
        let mut legendre = Legendre::new(10,10);
        legendre.calculate(0.5);

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
    }

    #[test]
    fn test_legendre_2() {
        let mut legendre = Legendre::new(50,50);
        legendre.calculate(-0.99);

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
        let mut legendre = Legendre::new(10,10).with_condon_shortley();
        legendre.calculate(0.5);

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
        let mut legendre = Legendre::new(10,10).with_normalization(LegendreNormalization::Full);
        legendre.calculate(0.5);

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
}
