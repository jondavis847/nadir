use gravity::egm96::EGM96;
use nalgebra::Vector3;

fn main() {
    let mut g = EGM96::new(10, 10);
    let r_ecef = Vector3::new(6.4e6, 0.0, 0.0);
    let a = g.calculate(r_ecef);
    dbg!(a);    
}
