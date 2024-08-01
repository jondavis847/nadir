use aerospace::gravity::{Gravity, ConstantGravity, TwoBodyGravity};
use clap::{Parser, Subcommand, ValueEnum};
use colored::*;
use mass_properties::{CenterOfMass, Inertia, MassProperties};
use multibody::{
    aerospace::MultibodyGravity, base::Base, body::{Body, BodyErrors}, component::MultibodyComponent, joint::{
        revolute::{Revolute, RevoluteState},
        Joint, JointParameters,
    }, result::MultibodyResult, system::MultibodySystem, MultibodyTrait
};
use rotations::{Rotation, quaternion::Quaternion, rotation_matrix::RotationMatrix, euler_angles::{EulerAngles,EulerSequence},axes::AlignedAxes};
use coordinate_systems::{CoordinateSystem, cartesian::Cartesian};
use transforms::Transform;
use uuid::Uuid;

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
    View { sys_or_res: String , name: String,  component: Option<String>},
    /// Add a new Component to a MultibodySystem
    Add {
        system_name: String,
        #[arg(value_enum)]
        component: Components,
    },
    /// Simulate the MultibodySystem
    Simulate {
        system_name: String
    },
}

#[derive(ValueEnum, Clone, Debug)]
enum Components {
    Base,
    Body,
    Gravity,
    Revolute,
}

fn main() -> Result<(), ReadlineError> {
    let mut systems = HashMap::<String, MultibodySystem>::new();
    let mut results = HashMap::<String, MultibodyResult>::new();

    // Create an editor for reading lines from the user
    let mut rl = DefaultEditor::new()?;

    loop {
        let prompt = "Multibody>> ".to_string();
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
                        Commands::Add {
                            system_name,
                            component,
                        } => {
                            if let Some(system) = systems.get_mut(&system_name) {
                                match component {
                                    Components::Base => {                                        
                                           let base = prompt_base();                                           
                                           let name = base.get_name().to_string();
                                            let r = system.add_base(base);
                                            match r {
                                                Ok(_) => println!("{} added to {}", &name, system_name),
                                                Err(e) => eprintln!("{:?}",e)
                                            }
                                        
                                    }
                                    Components::Body => {
                                        let body = prompt_body();
                                        match body {
                                            Ok(body) => {
                                                let name = body.name.clone();
                                                let r = system.add_body(body); 
                                                match r {
                                                    Ok(_) => println!("{} added to {}", &name, system_name),
                                                    Err(e) => eprintln!("{:?}",e)
                                                }                                                
                                            }
                                            Err(e) => eprintln!("{:?}",e)
                                        }
                                    }
                                    Components::Gravity => {
                                        let gravity = prompt_gravity();
                                        let name = gravity.get_name().to_string();
                                        let r = system.add_gravity(gravity);
                                        match r {
                                            Ok(_) => println!("{} added to {}", &name, system_name),
                                            Err(e) => eprintln!("{:?}",e)
                                        }                                         
                                    }
                                    Components::Revolute => {                                        
                                            let revolute = prompt_revolute();
                                            let joint = Joint::Revolute(revolute);
                                            let name = joint.get_name().to_string();
                                            let r = system.add_joint(joint);
                                            match r {
                                                Ok(_) => println!("{} added to {}", &name, system_name),
                                                Err(e) => eprintln!("{:?}",e)
                                            }                                           
                                    }                                    
                                }
                            } else {
                                println!("System '{}' not found.", system_name);
                            }    
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
                        Commands::Create { name } => {
                            systems.insert((&name).into(), MultibodySystem::new());
                            println!("{} added to systems", name)
                        }
                        Commands::Exit => {
                            break;
                        }
                        Commands::View { sys_or_res, name, component,} => {
                            match sys_or_res.as_str() {
                                "system" => { 
                                    if let Some(sys) = systems.get(&name) {
                                        println!("{:#?}", sys);
                                    } else {
                                        eprintln!("Error: system '{}' not found ", name);
                                    }
                                }                
                                "result" => { 
                                    if let Some(res) = results.get(&name) {
                                        if let Some(component) = component {
                                            let res = res.get_component(&component);
                                            println!("{:#?}", res);
                                        } else {
                                            println!("{:#?}", res);
                                        }
                                    } else {
                                        eprintln!("Error: result '{}' not found ", name);
                                    }
                                }    
                                _ => println!("Error: invalid component - options are ['system','result']")         
                            }
                        }
                                                Commands::Simulate {system_name} => {
                            if let Some(system) = systems.get_mut(&system_name) {
                                match system.validate() {
                                    Ok(_)=> {
                                        let (name, start_time,stop_time, dt) = prompt_simulation();
                                        let result = system.simulate(name,start_time,stop_time,dt);
                                        match result {
                                            Ok(result) => {results.insert(result.name.clone(),result);},
                                            Err(e) => eprintln!("{:#?}",e)
                                        }
                                        
                                    }
                                    Err(e) => {
                                        eprintln!("{:#?}",e)
                                    }
                                }
                                
                            } else {
                                println!("System '{}' not found.", system_name);
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
    GravityType,
    GravityConstantX,
    GravityConstantY,
    GravityConstantZ,
    GravityTwoBodyMu,
    JointDamping,
    JointForce,
    JointSpring,
    Mass,
    Name,
    QuaternionW,
    QuaternionX,
    QuaternionY,
    QuaternionZ,
    SimulationDt,
    SimulationStart,
    SimulationStop,
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
            Prompts::GravityType => "Gravity type ['c' (constant), '2' (two body)]",
            Prompts::GravityConstantX => "Constant gravity X (m/sec^2) :",
            Prompts::GravityConstantY => "Constant gravity Y (m/sec^2) :",
            Prompts::GravityConstantZ => "Constant gravity Z (m/sec^2) :",
            Prompts::GravityTwoBodyMu => "Two body gravity mu (m^3/sec^2):",
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
            Prompts::SimulationDt => "dt (units: sec, default: 0.1)",
            Prompts::SimulationStart => "start_time (units: sec, default: 0.0)",
            Prompts::SimulationStop => "stop_time (units: sec, default: 10.0)",            
            Prompts::SphericalR => "Spherical radius (units: m, default: 0.0): ",
            Prompts::SphericalA => "Spherical azimuth (units: rad, default: 0.0): ",
            Prompts::SphericalI => "Spherical inclination (units: rad, default: 0.0): ",            
            Prompts::Transform => "Transform? ('i/identity','c/custom',  default: i): ",
            Prompts::TransformRotation => "Rotation? ['i' (identity), 'q' (quaternion), 'r' (rotation matrix), 'e' (euler angles), 'a' (aligned axes)]: ",
            Prompts::TransformTranslation => "Translation? ['z' (zero), 'cart' (cartesian), 'cyl' (cylindrical), 'sph' (spherical)]: ",
        }
    }

    fn validate_loop(&self, default: &str) -> String {        
        loop {
            let s = self.prompt();
            match self.validate(&s) {
                Ok(_) => {
                    if s.is_empty() {
                        return default.to_string()
                    } else {
                        return s
                    }
                },
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
            | Prompts::GravityConstantX
            | Prompts::GravityConstantY
            | Prompts::GravityConstantZ
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
            Prompts::GravityTwoBodyMu | Prompts::Mass | Prompts::Ixx | Prompts::Iyy | Prompts::Izz => {
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
            //Gravity
            Prompts::GravityType => {
                if str.is_empty() {
                    //user must provide a default, but this is ok
                    return Ok(())
                }
                let possible_values = ["c", "2"];
                if !possible_values.contains(&(str.to_lowercase().as_str())) {
                    return Err(InputErrors::InvalidGravity);
                }
                Ok(())
            }
            // Transform
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
    InvalidGravity,
    InvalidRotation,
    InvalidTransform,
    NonNumeric,
    NotGreaterThanZero,
    NotYesOrNo,
}



impl std::fmt::Display for InputErrors {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            InputErrors::InvalidGravity => write!(f,"Error: input must be ['c' (constant), '2' (two body)]"),
            InputErrors::InvalidRotation => write!(f,"Error: input must be ['i' (identity), 'q' (quaternion), 'r' (rotation matrix), 'e' (euler angles), 'a' (aligned axes)]"),
            InputErrors::InvalidTransform => write!(f,"Error: input must be ['i' (identity), 'c' (custom)]"),
            InputErrors::NonNumeric => write!(f,"Error: input must be numeric"),
            InputErrors::NotGreaterThanZero => write!(f,"Error: input must be greater than 0"),
            InputErrors::NotYesOrNo => write!(f,"Error: input must be ['y' (yes), 'n' (no)]"),
            
        }
    }
}

fn prompt_base() -> Base {
    let name = Prompts::Name.prompt();
    Base::new(&name)
}

fn prompt_body() -> Result<Body, BodyErrors> {
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
    Body::new(&name, mass_properties)
}

fn prompt_gravity() -> MultibodyGravity {
    let name = Prompts::Name.prompt();
    let gravity_type = Prompts::GravityType.validate_loop("c");
    let gravity = match gravity_type.as_str() {
        "c" => {
            let x = Prompts::GravityConstantX.validate_loop("0").parse::<f64>().unwrap_or(0.0);
            let y = Prompts::GravityConstantY.validate_loop("0").parse::<f64>().unwrap_or(0.0);
            let z = Prompts::GravityConstantZ.validate_loop("0").parse::<f64>().unwrap_or(0.0);
            Gravity::Constant(ConstantGravity::new(x,y,z))            
        }
        "2" => {
            let earth = aerospace::gravity::EARTH;
            let earth_string = earth.to_string();            
            let mu = Prompts::GravityTwoBodyMu.validate_loop(&earth_string).parse::<f64>().unwrap_or(earth);
            Gravity::TwoBody(TwoBodyGravity::new(mu))
        }
        _ => panic!("shouldn't be possible. other characters caught in validation loop")
    };
    MultibodyGravity::new(&name,gravity)

}

fn prompt_revolute() -> Revolute {
    let name = Prompts::Name.prompt();

    let angle = Prompts::Angle.validate_loop("0").parse::<f64>().unwrap_or(0.0);

    let angular_rate = Prompts::AngularRate.validate_loop("0")
        .parse::<f64>()
        .unwrap_or(0.0);

    let state = RevoluteState::new(angle, angular_rate);


    let force = Prompts::JointForce
        .validate_loop("0")
        .parse::<f64>()
        .unwrap_or(0.0);
    let spring = Prompts::JointSpring
        .validate_loop("0")
        .parse::<f64>()
        .unwrap_or(0.0);
    let damping = Prompts::JointDamping
        .validate_loop("0")
        .parse::<f64>()
        .unwrap_or(0.0);
    let parameters =
        JointParameters::new(force, damping, spring);

    Revolute::new(&name, parameters, state)
}

fn prompt_transform() -> Transform {
    let transform = Prompts::Transform.validate_loop("i");    
    
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
    let rotation = Prompts::TransformRotation.validate_loop("i");                        
    match rotation.as_str() {
        "i" => Rotation::IDENTITY,
        "q" => Rotation::from(prompt_quaternion()),    
        _ => panic!("shouldn't be possible. other characters caught in validation loop")       
    }
}

fn prompt_quaternion() -> Quaternion {

    let w = Prompts::QuaternionW.validate_loop("1");               
    let w = w.parse::<f64>().unwrap_or(1.0);

    let x = Prompts::QuaternionX.validate_loop("0");               
    let x = x.parse::<f64>().unwrap_or(0.0);

    let y = Prompts::QuaternionY.validate_loop("0");               
    let y = y.parse::<f64>().unwrap_or(0.0);

    let z = Prompts::QuaternionZ.validate_loop("0");               
    let z = z.parse::<f64>().unwrap_or(0.0);
    
    let q = Quaternion::new(x,y,z,w);
    println!("{:?}", q);
    q
}

fn prompt_translation() -> CoordinateSystem {
    let translation = Prompts::TransformTranslation.validate_loop("z");                        
    match translation.as_str() {
        "z" => CoordinateSystem::ZERO,
        "cart" => CoordinateSystem::from(prompt_cartesian()),    
        _ => panic!("shouldn't be possible. other characters caught in validation loop")       
    }
}

fn prompt_cartesian() -> Cartesian {
    let x = Prompts::CartesianX.validate_loop("0");               
    let x = x.parse::<f64>().unwrap_or(0.0);

    let y = Prompts::CartesianY.validate_loop("0");               
    let y = y.parse::<f64>().unwrap_or(0.0);

    let z = Prompts::CartesianZ.validate_loop("0");               
    let z = z.parse::<f64>().unwrap_or(0.0);

    let cartesian = Cartesian::new(x,y,z);
    println!("{:?}", cartesian);
    cartesian
}

fn prompt_simulation() -> (String, f64,f64,f64) {    

        let name = Prompts::Name.validate_loop(Uuid::new_v4().to_string().as_str());

        let start_time = Prompts::SimulationStart.validate_loop("0");
        let start_time = start_time.parse::<f64>().unwrap_or(0.0);

        let stop_time = Prompts::SimulationStop.validate_loop("10");
        let stop_time = stop_time.parse::<f64>().unwrap_or(10.0);
        
        let dt = Prompts::SimulationDt.validate_loop("0.1");
        let dt = dt.parse::<f64>().unwrap_or(0.1);
        (name, start_time,stop_time,dt)
}