use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out_dir = env::var("OUT_DIR").unwrap();
    let dest_path = PathBuf::from(out_dir).join("systems.ron");
    // Copy the RON file to the OUT_DIR
    fs::copy("multibody/resources/systems.ron", &dest_path).expect("Failed to copy RON file");

    println!("cargo:rerun-if-changed=resources/systems.ron");
}