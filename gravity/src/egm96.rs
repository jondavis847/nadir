use nalgebra::{DMatrix, Vector3};

use crate::spherical_harmonics::SphericalHarmonics;

pub struct EGM96 {
    spherical_harmonics: SphericalHarmonics,
    pub c: Vec<Vec<f64>>,
    pub s: Vec<Vec<f64>>,
}

impl EGM96 {
    const RE: f64 = 6.3781363e6;
    const MU: f64 = 3.986004415e14;
    const EGM96_DATA: &str = include_str!("../resources/EGM96_50.txt");

    pub fn new(degree: usize, order: usize) -> Self {
        let (c, s) = Self::parse_file(degree, order);
        let spherical_harmonics = SphericalHarmonics::new(degree, order);
        Self {
            spherical_harmonics,
            c,
            s,
        }
    }

    pub fn calculate(&mut self, r_ecef: Vector3<f64>) -> Vector3<f64> {
        self.spherical_harmonics
            .vallado(r_ecef, &self.c, &self.s, Self::RE, Self::MU)
    }

    fn parse_file(degree: usize, order: usize) -> (Vec<Vec<f64>>, Vec<Vec<f64>>) {
        let mut c = vec![vec![0.0;order+1];degree+1];
        let mut s = vec![vec![0.0;order+1];degree+1];

        for line in Self::EGM96_DATA.lines() {
            let columns: Vec<&str> = line.split_whitespace().collect();
            if columns.len() == 6 {
                match (
                    columns[0].parse::<usize>(),
                    columns[1].parse::<usize>(),
                    columns[2].parse::<f64>(),
                    columns[3].parse::<f64>(),
                ) {
                    (Ok(col1), Ok(col2), Ok(col3), Ok(col4)) => {
                        if col1 <= degree && col2 <= order {

                            let l = col1 as u64;
                            let m = col2 as u64;
                            let k = if m == 0 {
                                1 
                            } else {
                                2
                            };

                            // normalize the coeffs Vallado 8-22
                            let n = (fact(l+m) as f64/((fact(l-m) * k * (2 * l + 1)) as f64)).sqrt();

                            c[col1][col2] = n * col3;
                            s[col1][col2] = n * col4/n;
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

fn fact(n: u64) -> u64 {
    (1..=n).product()
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use nalgebra::Matrix3;
    const TOL: f64 = 1e-10;

    #[test]
    fn test_egm96_1() {
        let re = Vector3::new(6.4e6, 0.0, 0.0);

        let mut g = EGM96::new(1, 1);
        let a = g.calculate(re);
        assert_abs_diff_eq!(a[0], -9.747237502896724, epsilon = 1.0);
        //assert_abs_diff_eq!(a[1], -1.3778135992666715e-5, epsilon = 1.0);
        assert_abs_diff_eq!(a[2], -9.808708996195295e-6, epsilon = 1.0);
    }
}
