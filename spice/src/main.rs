use clap::{Arg, Command};
use spice::{Bodies, Spice};

fn main() {
    /*
    // this creates the spice data file
    let file = "resources/de440s.bsp";
    let mut spice = Spice::new(file).unwrap();
    spice.save_spice_data();
    */

    // this uses the spice data file to get states
    let spice_bytes: &[u8] = include_bytes!("../resources/spice.dat");
    let mut spice: Spice = bincode::deserialize(spice_bytes).unwrap();
    // Define the CLI structure using clap
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
        let position = spice.get_position(et, body);
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
    }
}
