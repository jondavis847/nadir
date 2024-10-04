use clap::{Arg, Command};
use rotations::prelude::{EulerAngles, Quaternion};
use spice::{Bodies, Spice};

fn main() {
    // this creates the spice data file
    let mut spice = match Spice::from_naif() {
        Ok(spice) => spice,
        Err(e) => {
            dbg!(e);
            panic!()
        }
    };

    spice.save_spice_data();

    // Define the CLI structure using clap
    let matches = Command::new("spice")
        .version("1.0")
        .author("Your Name")
        .about("Calculates celestial positions")
        .arg(
            Arg::new("body")
                .short('b')
                .long("body")
                .value_name("BODY")
                .help("The celestial body to calculate")
                .num_args(1)
                .required(true),
        )
        .arg(
            Arg::new("et")
                .short('e')
                .long("et")
                .value_name("ET")
                .help("The epoch time in seconds past J2000")
                .num_args(1)
                .required(true),
        )
        .get_matches();

    let body_str = matches.get_one::<String>("body").unwrap();
    let et_str = matches.get_one::<String>("et").unwrap();

    let body = Bodies::from_string(body_str);
    let et: f64 = et_str.parse().expect("Invalid ET");

    if let Some(body) = body {
        let position = spice.calculate_position(et, body);
        match position {
            Ok(position) => {
                println!("J2000 Position:");
                println!("  x: {}", position[0]);
                println!("  y: {}", position[1]);
                println!("  z: {}", position[2]);
            }
            Err(e) => {
                dbg!(e);
            }
        };
        match body {
            Bodies::Earth => {
                let orientation = spice.calculate_orientation(et, body);
                match orientation {
                    Ok(orientation) => {
                        let (rotation,ra,dec,gha) = orientation;
                        let q = Quaternion::from(rotation);
                        println!("J2000/ITRF Quaternion:");
                        println!("  x: {}", q.x);
                        println!("  y: {}", q.y);
                        println!("  z: {}", q.z);
                        println!("  w: {}", q.s);

                        println!("Right Ascension: {}", ra);
                        println!("Declination: {}", dec);
                        println!("GHA: {}", gha);

                    }
                    Err(e) => {
                        dbg!(e);
                    }
                }
            }
            _ => {}//nothing
        }
    }
}
