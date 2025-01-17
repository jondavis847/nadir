use clap::{Arg, Command};
use spice::{Spice, SpiceBodies};
use time::{Time, TimeSystem};

fn main() {
    // this creates the spice data file
    let mut spice = match Spice::from_local() {
        Ok(spice) => spice,
        Err(e) => {
            dbg!(e);
            panic!()
        }
    };

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
                .help("The TAI epoch in seconds past J2000")
                .num_args(1)
                .required(true),
        )
        .get_matches();

    let body_str = matches.get_one::<String>("body").unwrap();
    let et_str = matches.get_one::<String>("et").unwrap();

    let body = SpiceBodies::from_string(body_str);
    let et: f64 = et_str.parse().expect("Invalid ET");
    let epoch = Time::from_sec_j2k(et, TimeSystem::TAI);

    if let Some(body) = body {
        let position = spice.calculate_position(epoch, body);
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
        if body == SpiceBodies::Earth {
            let orientation = spice.calculate_orientation(epoch, body);
            match orientation {
                Ok(q) => {
                    println!("J2000/ITRF Quaternion:");
                    println!("  x: {}", q.x);
                    println!("  y: {}", q.y);
                    println!("  z: {}", q.z);
                    println!("  w: {}", q.w);
                }
                Err(e) => {
                    dbg!(e);
                }
            }
        }
    }
}
