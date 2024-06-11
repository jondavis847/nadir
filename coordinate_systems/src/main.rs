pub mod cartesian;
pub mod coordinate_system;
pub mod cylindrical;
pub mod spherical;

use cartesian::Cartesian;
use cylindrical::Cylindrical;
use spherical::Spherical;
use std::f64::consts::PI;
fn main() {
    let cyl1 = Cylindrical::new(1.0, PI / 2.0, 1.0);
    let cyl2 = Cylindrical::new(1.0, PI / 2.0, 1.0);
    dbg!(cyl1);
    dbg!(cyl2);
    let ca1 = Cartesian::from(cyl1);
    let ca2 = Cartesian::from(cyl2);
dbg!(ca1);
dbg!(ca2);
    dbg!(ca1 + ca2);
    dbg!(cyl1 + cyl2);
}
