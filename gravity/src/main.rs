use gravity::egm::{EgmGravity, EgmModel};
use gravity::Gravity;
use nalgebra::Vector3;

fn main() {
    let mut g = Gravity::Egm(EgmGravity::new(EgmModel::Egm96, 50, 50).unwrap());
    let r_ecef = Vector3::new(6.4e6, 0.0, 0.0);
    let a = g.calculate(r_ecef).unwrap();
    dbg!(a);
}
