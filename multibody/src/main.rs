use clap::{Parser, Subcommand, ValueEnum};
use colored::*;
use mass_properties::{CenterOfMass, Inertia, MassProperties};
use multibody::{
    base::Base,
    body::Body,
    component::MultibodyComponent,
    joint::{
        revolute::{Revolute, RevoluteState},
        Joint, JointParameters,
    },
    system::MultibodySystem,
    MultibodyErrors,
};
use rotations::{Rotation, quaternion::Quaternion, rotation_matrix::RotationMatrix, euler_angles::{EulerAngles,EulerSequence},axes::AlignedAxes};
use coordinate_systems::{CoordinateSystem, cartesian::Cartesian};
use transforms::Transform;

use rustyline::{
    completion::Completer, error::ReadlineError, highlight::Highlighter, hint::Hinter,
    DefaultEditor,
};
use smol_str::SmolStr;
use std::collections::HashMap;

#[derive(Debug, Parser)]
#[command(version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Option<Commands>,
}

#[derive(Debug, Subcommand)]
enum Commands {
    /// Make a connection from one component to another
    Connect {
        sys_name: String,
        from_name: String,
        to_name: String,
    },
    /// Create a new MultibodySystem
    Create { name: String },
    /// Exit the program
    Exit,
    /// View the MultibodySystem or a Component by its name
    View { name: String },
    /// Add a new Component to a MultibodySystem
    Add {
        system_name: String,
        #[arg(value_enum)]
        component: Components,
    },
}

#[derive(ValueEnum, Clone, Debug)]
enum Components {
    Base,
    Body,
    Revolute,
}

fn main() -> Result<(), ReadlineError> {
    let mut systems = HashMap::<String, MultibodySystem>::new();

    // Create an editor for reading lines from the user
    let mut rl = DefaultEditor::new()?;

    loop {
        let prompt = "Multibody>>".to_string();
        let readline = rl.readline(&prompt);
        match readline {
            Ok(input) => {
                rl.add_history_entry(input.as_str())?;

                // Trim the input to remove newline and whitespace
                let input = input.trim();

                // Skip empty input
                if input.is_empty() {
                    continue;
                }

                // Parse the input into CLI arguments
                let args = match Cli::try_parse_from(
                    std::iter::once("Multibody").chain(input.split_whitespace()),
                ) {
                    Ok(args) => args,
                    Err(err) => {
                        eprintln!("{}", err);
                        continue;
                    }
                };

                // Execute the command
                if let Some(command) = args.command {
                    match command {
                        Commands::Create { name } => {
                            systems.insert((&name).into(), MultibodySystem::new());
                            println!("{} added to systems", name)
                        }
                        Commands::Connect {
                            sys_name,
                            from_name,
                            to_name,
                        } => {
                            if let Some(sys) = systems.get_mut(&sys_name) {
                                let from_component_data = match sys.get_from_name(&from_name) {
                                    Some(component_data) => component_data,
                                    None => {
                                        eprintln!("Error: 'From' name not found");
                                        continue;
                                    }
                                };
                                let to_component_data = match sys.get_from_name(&to_name) {
                                    Some(component_data) => component_data,
                                    None => {
                                        eprintln!("Error: 'To' name not found");
                                        continue;
                                    }
                                };

                                let transform = match (from_component_data.0, to_component_data.0) {
                                    (
                                        MultibodyComponent::Base | MultibodyComponent::Body,
                                        MultibodyComponent::Joint,
                                    ) => Some(prompt_transform()),
                                    (
                                        MultibodyComponent::Joint,
                                        MultibodyComponent::Base | MultibodyComponent::Body,
                                    ) => Some(prompt_transform()),
                                    (
                                        MultibodyComponent::Gravity,
                                        MultibodyComponent::Base | MultibodyComponent::Body,
                                    ) => None,
                                    _ => {
                                        eprintln!(
                                            "Error: invalid connection ({:?} to {:?})",
                                            from_component_data.0, to_component_data.0
                                        );
                                        continue;
                                    }
                                };

                                sys.connect(&from_name, &to_name, transform);
                            } else {
                                let e = "System not found".red();
                                println!("{}", e)
                            }
                        }
                        Commands::Exit => {
                            break;
                        }
                        Commands::View { name } => {
                            if let Some(sys) = systems.get(&name) {
                                println!("{:#?}", sys);
                            }
                        }
                        Commands::Add {
                            system_name,
                            component,
                        } => {
                            match component {
                                Components::Base => {
                                    if let Some(system) = systems.get_mut(&system_name) {
                                        let name = Prompts::Name.prompt();
                                        let base = Base::new(&name);
                                        system.add_base(base);
                                        println!("{} added to {}", &name, system_name);
                                    } else {
                                        println!("System '{}' not found.", system_name);
                                    }
                                }
                                Components::Body => {
                                    if let Some(system) = systems.get_mut(&system_name) {
                                        let name = Prompts::Name.prompt();
                                        let mass =
                                            Prompts::Mass.prompt().parse::<f64>().unwrap_or(1.0);
                                        let cmx =
                                            Prompts::Cmx.prompt().parse::<f64>().unwrap_or(0.0);
                                        let cmy =
                                            Prompts::Cmy.prompt().parse::<f64>().unwrap_or(0.0);
                                        let cmz =
                                            Prompts::Cmz.prompt().parse::<f64>().unwrap_or(0.0);
                                        let ixx =
                                            Prompts::Ixx.prompt().parse::<f64>().unwrap_or(1.0);
                                        let iyy =
                                            Prompts::Iyy.prompt().parse::<f64>().unwrap_or(1.0);
                                        let izz =
                                            Prompts::Izz.prompt().parse::<f64>().unwrap_or(1.0);
                                        let ixy =
                                            Prompts::Ixy.prompt().parse::<f64>().unwrap_or(0.0);
                                        let ixz =
                                            Prompts::Ixz.prompt().parse::<f64>().unwrap_or(0.0);
                                        let iyz =
                                            Prompts::Iyz.prompt().parse::<f64>().unwrap_or(0.0);

                                        let com = CenterOfMass::new(cmx, cmy, cmz);
                                        let inertia =
                                            Inertia::new(ixx, iyy, izz, ixy, ixz, iyz).unwrap();
                                        let mass_properties =
                                            MassProperties::new(mass, com, inertia).unwrap();
                                        let body = Body::new(&name, mass_properties).unwrap();

                                        let body_name = body.name.clone();
                                        system.add_body(body); // Assuming you have a method to add a body to the system
                                        println!("{} added to {}", body_name, system_name);
                                    } else {
                                        println!("System '{}' not found.", system_name);
                                    }
                                }
                                Components::Revolute => {
                                    if let Some(system) = systems.get_mut(&system_name) {
                                        let name = Prompts::Name.prompt();
                                        let angle =
                                            Prompts::Angle.prompt().parse::<f64>().unwrap_or(0.0);
                                        let angular_rate = Prompts::AngularRate
                                            .prompt()
                                            .parse::<f64>()
                                            .unwrap_or(0.0);

                                        let state = RevoluteState::new(angle, angular_rate);
                                        let force = Prompts::JointForce
                                            .prompt()
                                            .parse::<f64>()
                                            .unwrap_or(0.0);
                                        let spring = Prompts::JointSpring
                                            .prompt()
                                            .parse::<f64>()
                                            .unwrap_or(0.0);
                                        let damping = Prompts::JointDamping
                                            .prompt()
                                            .parse::<f64>()
                                            .unwrap_or(0.0);
                                        let parameters =
                                            JointParameters::new(force, damping, spring);

                                        let revolute = Revolute::new(&name, parameters, state);
                                        let joint = Joint::Revolute(revolute);
                                        system.add_joint(joint);
                                        println!("{} added to {}", name, system_name);
                                    } else {
                                        println!("System '{}' not found.", system_name);
                                    }
                                }
                                _ => {
                                    println!("Invalid component for adding.");
                                }
                            }
                        }
                    }
                } else {
                    println!("Unrecognized command");
                }
            }
            Err(ReadlineError::Interrupted) => {
                println!("CTRL-C");
                break;
            }
            Err(ReadlineError::Eof) => {
                println!("CTRL-D");
                break;
            }
            Err(err) => {
                println!("Error: {:?}", err);
                break;
            }
        }
    }
    Ok(())
}

enum Prompts {
    Angle,
    AngularRate,
    CartesianX,
    CartesianY,
    CartesianZ,
    Cmx,
    Cmy,
    Cmz,
    CylindricalA,
    CylindricalH,
    CylindricalR,
    Ixx,
    Iyy,
    Izz,
    Ixy,
    Ixz,
    Iyz,
    JointDamping,
    JointForce,
    JointSpring,
    Mass,
    Name,
    QuaternionW,
    QuaternionX,
    QuaternionY,
    QuaternionZ,
    SphericalR,
    SphericalA,
    SphericalI,
    Transform,
    TransformRotation,
    TransformTranslation,
}

impl Prompts {
    fn get_string(&self) -> &str {
        match self {
            Prompts::Angle => "Initial angle (units: rad, default: 0.0): ",
            Prompts::AngularRate => "Initial angular Rate (units: rad/sec, default: 0.0): ",
            Prompts::CartesianX => "Cartesian X (units: m, default: 0.0): ",
            Prompts::CartesianY => "Cartesian Y (units: m, default: 0.0): ",
            Prompts::CartesianZ => "Cartesian Z (units: m, default: 0.0): ",            
            Prompts::Cmx => "Center of Mass - X (units: m, default: 0.0): ",
            Prompts::Cmy => "Center of Mass - Y (units: m, default: 0.0): ",
            Prompts::Cmz => "Center of Mass - Z (units: m, default: 0.0): ",
            Prompts::CylindricalR => "Cylindrical radius (units: m, default: 0.0): ",
            Prompts::CylindricalA => "Cylindrical azimuth (units: rad, default: 0.0): ",
            Prompts::CylindricalH => "Cylindrical height (units: m, default: 0.0): ",
            Prompts::Ixx => "Ixx (units: kg-m^2, default: 1.0): ",
            Prompts::Iyy => "Iyy (units: kg-m^2, default: 1.0): ",
            Prompts::Izz => "Izz (units: kg-m^2, default: 1.0): ",
            Prompts::Ixy => "Ixy (units: kg-m^2, default: 0.0): ",
            Prompts::Ixz => "Ixz (units: kg-m^2, default: 0.0): ",
            Prompts::Iyz => "Iyz (units: kg-m^2, default: 0.0): ",
            Prompts::JointDamping => "Damping (units: None, default: 0.0): ",
            Prompts::JointForce => "Constant force (units: N, default: 0.0): ",
            Prompts::JointSpring => "Spring constant (units: N/m, default: 0.0): ",
            Prompts::Mass => "Mass (units: kg, default: 1.0): ",
            Prompts::Name => "Name: ",
            Prompts::QuaternionW => "Quaternion W (units: None, default: 1.0): ",
            Prompts::QuaternionX => "Quaternion X (units: None, default: 0.0): ",
            Prompts::QuaternionY => "Quaternion Y (units: None, default: 0.0): ",
            Prompts::QuaternionZ => "Quaternion Z (units: None, default: 0.0): ",
            Prompts::SphericalR => "Spherical radius (units: m, default: 0.0): ",
            Prompts::SphericalA => "Spherical azimuth (units: rad, default: 0.0): ",
            Prompts::SphericalI => "Spherical inclination (units: rad, default: 0.0): ",            
            Prompts::Transform => "Transform? ('i/identity','c/custom',  default: i): ",
            Prompts::TransformRotation => "Rotation? ['i' (identity), 'q' (quaternion), 'r' (rotation matrix), 'e' (euler angles), 'a' (aligned axes)]: ",
            Prompts::TransformTranslation => "Translation? ['z' (zero), 'cart' (cartesian), 'cyl' (cylindrical), 'sph' (spherical)]: ",
        }
    }

    fn validate_loop(&self) -> String {        
        loop {
            let s = self.prompt();
            match self.validate(&s) {
                Ok(_) => return s,
                Err(e) => {
                    eprintln!("{}",e);                    
                }
            }
        }
    }

    fn prompt(&self) -> String {
        let mut rl = DefaultEditor::new().unwrap();
        loop {
            let readline = rl.readline(self.get_string());
            match readline {
                Ok(name) => {
                    return name.trim().to_string();
                }
                Err(ReadlineError::Interrupted) | Err(ReadlineError::Eof) => {
                    println!("Operation cancelled.");
                    std::process::exit(1);
                }
                Err(err) => {
                    println!("Error: {:?}", err);
                    continue;
                }
            }
        }
    }

    fn validate(&self, str: &str) -> Result<(), InputErrors> {
        match self {
            // Non-numeric only
            Prompts::Angle
            | Prompts::AngularRate
            | Prompts::Cmx
            | Prompts::Cmy
            | Prompts::Cmz
            | Prompts::Ixy
            | Prompts::Ixz
            | Prompts::Iyz
            | Prompts::JointDamping
            | Prompts::JointForce
            | Prompts::JointSpring
            | Prompts::QuaternionW
            | Prompts::QuaternionX
            | Prompts::QuaternionY
            | Prompts::QuaternionZ => {
                if str.is_empty() {
                    //user must provide a default, but this is ok
                    return Ok(())
                }
                if str.parse::<f64>().is_err() {
                    return Err(InputErrors::NonNumeric);
                }
                Ok(())
            }
            // Non-numeric and > 0
            Prompts::Mass | Prompts::Ixx | Prompts::Iyy | Prompts::Izz => {
                if str.is_empty() {
                    //user must provide a default, but this is ok
                    return Ok(())
                }
                if str.parse::<f64>().is_err() {
                    return Err(InputErrors::NonNumeric);
                }
                if str.parse::<f64>().unwrap() <= 0.0 {
                    return Err(InputErrors::NotGreaterThanZero);
                }
                Ok(())
            }
            // Yes/No values
            Prompts::Transform => {
                if str.is_empty() {
                    //user must provide a default, but this is ok
                    return Ok(())
                }
                let possible_values = ["i", "c"];
                if !possible_values.contains(&(str.to_lowercase().as_str())) {
                    return Err(InputErrors::InvalidTransform);
                }
                Ok(())
            }
            // Rotation
            Prompts::TransformRotation => {
                if str.is_empty() {
                    //user must provide a default, but this is ok
                    return Ok(())
                }
                let possible_values = [
                    "i",                    
                    "q",                    
                    "r",                    
                    "e",
                    "a"                    
                ];
                if !possible_values.contains(&(str.to_lowercase().as_str())) {
                    return Err(InputErrors::InvalidRotation);
                }
                Ok(())
            }
            _ => Ok(()),
        }
    }
}

#[derive(Debug)]
pub enum InputErrors {
    InvalidRotation,
    InvalidTransform,
    NonNumeric,
    NotGreaterThanZero,
    NotYesOrNo,
}



impl std::fmt::Display for InputErrors {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            InputErrors::InvalidRotation => write!(f,"Error: input must be ['i' (identity), 'q' (quaternion), 'r' (rotation matrix), 'e' (euler angles), 'a' (aligned axes)]"),
            InputErrors::InvalidTransform => write!(f,"Error: input must be ['i' (identity), 'c' (custom)]"),
            InputErrors::NonNumeric => write!(f,"Error: input must be numeric"),
            InputErrors::NotGreaterThanZero => write!(f,"Error: input must be greater than 0"),
            InputErrors::NotYesOrNo => write!(f,"Error: input must be ['y' (yes), 'n' (no)]"),
            
        }
    }
}

fn prompt_transform() -> Transform {
    let transform = Prompts::Transform.validate_loop();    
    
    match transform.as_str() {
        "i" => Transform::IDENTITY,
        "c" => {            
            let rotation = prompt_rotation();
            let translation = prompt_translation();
            Transform::new(rotation,translation)
        }
        _ => panic!("shouldn't be possible. other characters caught in validation loop")
        }
    }


fn prompt_rotation() -> Rotation {
    let rotation = Prompts::TransformRotation.validate_loop();                        
    match rotation.as_str() {
        "i" => Rotation::IDENTITY,
        "q" => Rotation::from(prompt_quaternion()),    
        _ => panic!("shouldn't be possible. other characters caught in validation loop")       
    }
}

fn prompt_quaternion() -> Quaternion {

    let w = Prompts::QuaternionW.validate_loop();               
    let w = w.parse::<f64>().unwrap_or(1.0);

    let x = Prompts::QuaternionX.validate_loop();               
    let x = x.parse::<f64>().unwrap_or(0.0);

    let y = Prompts::QuaternionY.validate_loop();               
    let y = y.parse::<f64>().unwrap_or(0.0);

    let z = Prompts::QuaternionZ.validate_loop();               
    let z = z.parse::<f64>().unwrap_or(0.0);
    
    let q = Quaternion::new(x,y,z,w);
    println!("{:?}", q);
    q
}

fn prompt_translation() -> CoordinateSystem {
    let translation = Prompts::TransformTranslation.validate_loop();                        
    match translation.as_str() {
        "z" => CoordinateSystem::ZERO,
        "cart" => CoordinateSystem::from(prompt_cartesian()),    
        _ => panic!("shouldn't be possible. other characters caught in validation loop")       
    }
}

fn prompt_cartesian() -> Cartesian {
    let x = Prompts::CartesianX.validate_loop();               
    let x = x.parse::<f64>().unwrap_or(0.0);

    let y = Prompts::CartesianY.validate_loop();               
    let y = y.parse::<f64>().unwrap_or(0.0);

    let z = Prompts::CartesianZ.validate_loop();               
    let z = z.parse::<f64>().unwrap_or(0.0);

    let cartesian = Cartesian::new(x,y,z);
    println!("{:?}", cartesian);
    cartesian
}

