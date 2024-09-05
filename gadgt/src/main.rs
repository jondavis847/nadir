#[cfg(feature = "dhat-heap")]
#[global_allocator]
static ALLOC: dhat::Alloc = dhat::Alloc;

use aerospace::gravity::{Gravity, ConstantGravity, TwoBodyGravity};
use clap::{Parser, Subcommand, ValueEnum};
use color::Color;
use coordinate_systems::{CoordinateSystem, cartesian::Cartesian};
use dirs_next::config_dir;
use gadgt_3d::{geometry::{cuboid::Cuboid, ellipsoid::{Ellipsoid, Ellipsoid16}, Geometry, GeometryState}, material::Material, mesh::Mesh};
use mass_properties::{CenterOfMass, Inertia, MassProperties};
use multibody::{
    aerospace::MultibodyGravity, base::Base, body::{Body, BodyErrors, BodyTrait}, component::MultibodyComponent, joint::{floating::{Floating, FloatingState}, joint_sim::JointSimTrait, joint_state::JointStates, prismatic::{Prismatic, PrismaticState}, revolute::{Revolute, RevoluteState}, Joint, JointParameters, JointTrait
    }, result::MultibodyResult, system::MultibodySystem, system_sim::MultibodySystemSim, MultibodyTrait
};
use nalgebra::Vector3;
use ratatui::{
    backend::CrosstermBackend,
    crossterm::{
        event::{self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode, KeyModifiers},
        execute,
        terminal::{EnterAlternateScreen, LeaveAlternateScreen},
    },        
    terminal::{Frame, Terminal},    
    widgets::{ Axis, Chart, Dataset, GraphType },
};
use reedline::{FileBackedHistory, Prompt, PromptEditMode, PromptHistorySearch, Reedline, Signal};
use ron::de::from_reader;
use ron::ser::{to_string_pretty, PrettyConfig};
use rotations::{Rotation, quaternion::Quaternion};
use std::borrow::Cow;
use std::collections::HashMap;
use std::fs::{self, File,OpenOptions};
use std::io::{self,Read, Write};
use std::path::{Path,PathBuf};
use std::process::{Command,Stdio};
use transforms::Transform;
use utilities::format_number;

#[derive(Debug, Parser)]
#[command(version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Option<Commands>,
}

#[derive(Debug, Subcommand)]
enum Commands {
    /// Sets the system as the active system
    Activate { system_name: String },
    /// Add a new Component to a MultibodySystem
    Add {        
        #[arg(value_enum)]
        component: Components,
    },
    /// Animate a multibody result
    Animate {
        result: String,
    },
    /// Make a connection from one component to another
    Connect {        
        from_name: String,
        to_name: String,
    },
    /// Create a new MultibodySystem
    Create { name: String },
    /// Delete a component    
    Delete {name: String},
    /// Disconnect two components, also deleting their transforms
    Disconnect {from_name: String, to_name: String},
    /// Edit a component
    Edit {name: String},    
    /// Exit the GADGT CLI
    Exit,    
    /// Load a saved MultibodySystem    
    Load { system: String },
    /// Plot a component by its name
    Plot { result: String , component: String, state: String},
    /// Run one cycle of the MultibodySystem (useful for debugging)
    Run,
    /// Save a MultibodySystem for future use
    Save,
    /// Simulate the MultibodySystem
    Simulate,
    /// View the MultibodySystem or a Component by its name
    View { sys_or_res: String , name: String,  component: Option<String>, state: Option<String>},
}

#[derive(ValueEnum, Clone, Debug)]
enum Components {
    Base,
    Body,
    Floating,
    Gravity,    
    Prismatic,
    Revolute,
}

fn main() {
    // just for profiling performance. use cargo run --features dhat-heap to profile
    #[cfg(feature = "dhat-heap")]
    let _profiler = dhat::Profiler::new_heap();

    //set up the command history
    let mut history_path = config_dir().unwrap_or_else(|| PathBuf::from("."));
    history_path.push("gadgt");
    history_path.push("command_history.txt");

    let history = Box::new(
        FileBackedHistory::with_file(100, history_path)
          .expect("Error configuring history with file"),
    );

    // set up the systems .ron file for loading and saving systems    
    const SYSTEMS_RON: &str = include_str!("../../multibody/resources/systems.ron");
    
    let mut systems_path = config_dir().unwrap_or_else(|| PathBuf::from("."));
    systems_path.push("gadgt"); 
    systems_path.push("systems.ron");
    if !Path::new(&systems_path).exists() {
        if let Some(parent) = systems_path.parent() {
            fs::create_dir_all(parent).expect("Failed to create config directory");
        }
        fs::write(&systems_path, SYSTEMS_RON).expect("Failed to write RON bytes to config file");
    }
    // set up the main prompts
    
    let mut rl = Reedline::create().with_history(history).with_ansi_colors(true);
    //let prompt_string = colored::Colorize::blue("GADGT").to_string();
    //let prompt = DefaultPrompt::new(DefaultPromptSegment::Basic(prompt_string), DefaultPromptSegment::Empty);  
    let prompt = Prompts::Main;  


    let mut active_system: Option<String> = None;

    let mut systems = HashMap::<String, MultibodySystem>::new();
    let mut results = HashMap::<String, MultibodyResult>::new();
    
    //rl.clear_screen().unwrap();    
    //let welcome = "Welcome to GADGT!".bright_blue();
    //println!("{}", welcome);    
    loop {
        let result = rl.read_line(&prompt);

        match result {
            Ok(signal) => {                
                match signal {
                    Signal::Success(input) => {
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
                                Commands::Activate {system_name} => {
                                    if systems.contains_key(&system_name) {
                                        active_system = Some(system_name.clone());
                                        let s = format!("'{}' set as the active system!", system_name);                                        
                                        success(&s);
                                    }else {
                                        error("System not found...");
                                    }
                                }
                                Commands::Add {                                    
                                    component,
                                } => {
                                    let system_name = match &active_system {
                                        Some(name) => name,
                                        None => {
                                            error("No active system. Create or load a system first.");
                                            continue
                                        }
                                    };
                                    if let Some(system) = systems.get_mut(system_name) {
                                        match component {
                                            Components::Base => {                                        
                                                let base = prompt_base();    
                                                match base {
                                                    Ok(base) => {
                                                        let name = base.get_name().to_string();
                                                        let r = system.add_base(base);
                                                        match r {                                                            
                                                            Ok(_) => success(format!("{} added to {}!", &name, system_name).as_str()),
                                                            Err(e) => eprintln!("{:?}",e)
                                                        }
                                                    }
                                                    Err(e) => match e {
                                                        InputErrors::CtrlC => continue,
                                                        _ => eprintln!("{:?}",e)
                                                    }
                                                }                                       
                                                
                                            }
                                            Components::Body => {
                                                let body = prompt_body();
                                                match body {
                                                    Ok(body) => {                                                        
                                                        let name = body.name.clone();
                                                        let r = system.add_body(body); 
                                                        match r {
                                                            Ok(_) => success(format!("{} added to {}!", &name, system_name).as_str()),
                                                            Err(e) => eprintln!("{:?}",e)
                                                        }                                                                                                        
                                                    }
                                                    Err(e) => match e {
                                                        InputErrors::CtrlC => continue,
                                                        _ => eprintln!("{:?}",e)
                                                    }
                                                }
                                            }
                                            Components::Gravity => {
                                                match prompt_gravity() {
                                                    Ok(gravity) => {
                                                        let name = gravity.get_name().to_string();
                                                        let r = system.add_gravity(gravity);
                                                        match r {
                                                            Ok(_) => success(format!("{} added to {}!", &name, system_name).as_str()),
                                                            Err(e) => eprintln!("{:?}",e)
                                                        }                                         
                                                    } 
                                                    Err(e) => match e {
                                                        InputErrors::CtrlC => continue,
                                                        _ => eprintln!("{:?}",e)
                                                    }
                                                };
                                            }
                                            Components::Floating => {                                        
                                                let floating = match prompt_floating() {
                                                    Ok(floating) => floating,
                                                    Err(e) => match e {
                                                        InputErrors::CtrlC => continue,
                                                        _ => {eprintln!("{:?}",e); continue}
                                                    }
                                                };
                                                
                                                let joint = Joint::Floating(floating);
                                                let name = joint.get_name().to_string();
                                                let r = system.add_joint(joint);
                                                match r {
                                                    Ok(_) => success(format!("{} added to {}!", &name, system_name).as_str()),
                                                    Err(e) => eprintln!("{:?}",e)
                                                }                                           
                                        }
                                            Components::Prismatic => {                                        
                                                    let prismatic = match prompt_prismatic() {
                                                        Ok(prismatic) => prismatic,
                                                        Err(e) => match e {
                                                            InputErrors::CtrlC => continue,
                                                            _ => {eprintln!("{:?}",e); continue}
                                                        }
                                                    };
                                                    
                                                    let joint = Joint::Prismatic(prismatic);
                                                    let name = joint.get_name().to_string();
                                                    let r = system.add_joint(joint);
                                                    match r {
                                                        Ok(_) => success(format!("{} added to {}!", &name, system_name).as_str()),
                                                        Err(e) => eprintln!("{:?}",e)
                                                    }                                           
                                            }
                                            Components::Revolute => {                                        
                                                let revolute = match prompt_revolute() {
                                                    Ok(revolute) => revolute,
                                                    Err(e) => match e {
                                                        InputErrors::CtrlC => continue,
                                                        _ => {eprintln!("{:?}",e); continue}
                                                    }
                                                };
                                                
                                                let joint = Joint::Revolute(revolute);
                                                let name = joint.get_name().to_string();
                                                let r = system.add_joint(joint);
                                                match r {
                                                    Ok(_) => success(format!("{} added to {}!", &name, system_name).as_str()),
                                                    Err(e) => eprintln!("{:?}",e)
                                                }                                           
                                        }                                        
                                        }
                                    } else {
                                        println!("Error: system '{}' not found.", system_name);
                                    }    
                                }
                                Commands::Animate { result } => {

                                    // get the result data, continue if it doesnt exist
                                    let result = match results.get(&result) {
                                        Some(result) => result,
                                        None => {
                                            let e = format!("could not find {result} in results...");
                                            error(&e);
                                            continue;
                                        }
                                    };

                                    // serialize the result data to be sent to the animation crate process
                                    let result_bytes = bincode::serialize(result).expect("Failed to serialize struct");

                                    let mut child  = Command::new("gadgt_animation")
                                            .stdin(Stdio::piped())
                                            .stdout(Stdio::piped())
                                            .stderr(Stdio::inherit())
                                            .spawn()
                                            .expect("Failed to execute animation");      

                                    
                                    // Get a handle to the child's stdin
                                    if let Some(mut stdin) = child.stdin.take() {
                                        // Send data to the animation process
                                        stdin.write_all(&result_bytes).expect("Failed to write to stdin");
                                        // Explicitly close the stdin to signal that no more data will be sent
                                        drop(stdin);
                                    }
                                },
                                Commands::Connect {                                    
                                    from_name,
                                    to_name,
                                } => {
                                    let sys_name = match &active_system {
                                        Some(name) => name,
                                        None => {
                                            error("No active system. Create or load a system first.");
                                            continue
                                        }
                                    };
                                    if let Some(sys) = systems.get_mut(sys_name) {
                                        let from_component_data = match sys.get_from_name(&from_name) {
                                            Some(component_data) => component_data,
                                            None => {
                                                error("Error: 'From' name not found");                                                
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
                                            ) => {
                                                let transform = prompt_transform();
                                                match transform {
                                                    Ok(transform) => Some(transform),
                                                    Err(e) => match e {
                                                        InputErrors::CtrlC => continue,
                                                        _ => {eprintln!("{:?}",e); continue}
                                                    }
                                                }
                                            },
                                            (
                                                MultibodyComponent::Joint,
                                                MultibodyComponent::Base | MultibodyComponent::Body,
                                            ) => {
                                                let transform = prompt_transform();
                                                match transform {
                                                    Ok(transform) => Some(transform),
                                                    Err(e) => match e {
                                                        InputErrors::CtrlC => continue,
                                                        _ => {eprintln!("{:?}",e); continue}
                                                    }
                                                }
                                            },
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

                                        match sys.connect(&from_name, &to_name, transform) {
                                           Ok(_) => success("components connected!"),
                                           Err(e) => {
                                            let e = format!("{:#?}", e);
                                            error(&e);
                                           }
                                        };
                                    } else {
                                        let e = colored::Colorize::red("System not found");
                                        println!("{}", e)
                                    }                                    
                                }
                                Commands::Create { name } => {                                    
                                    systems.insert((&name).into(), MultibodySystem::new()); 
                                    active_system = Some(name.clone());                                   
                                    let str = format!("{} added to systems!", &name);
                                    success(&str);                                    
                                }                       
                                Commands::Delete {name} => {
                                    let sys_name = match &active_system {
                                        Some(name) => name,
                                        None => {
                                            error("No active system. Create or load a system first.");
                                            continue
                                        }
                                    };

                                    if let Some(sys) = systems.get_mut(sys_name) {
                                        match sys.delete(&name) {
                                            Ok(_) => success(format!("'{}' deleted!", &name).as_str()),
                                            Err(e) => error(format!("{:#?}", e).as_str()),
                                        }
                                    }
                                }   
                                Commands::Disconnect {from_name, to_name} => {
                                    // check that a sys is active
                                    let sys_name = match &active_system {
                                        Some(name) => name,
                                        None => {
                                            error("No active system. Create or load a system first.");
                                            continue
                                        }
                                    };

                                    // get the components
                                    if let Some(sys) = systems.get_mut(sys_name) {
                                        let (from_type,from_id) = match sys.get_from_name(&from_name) {
                                            Some(component_data) => component_data,
                                            None => {
                                                error("Error: 'From' component not found");
                                                continue;
                                            }
                                        };
                                        let (to_type, to_id) = match sys.get_from_name(&to_name) {
                                            Some(component_data) => component_data,
                                            None => {
                                                eprintln!("Error: 'To' component not found");
                                                continue;
                                            }
                                        };

                                        match from_type {
                                            MultibodyComponent::Base => {
                                                if let Some(base) = &mut sys.base {
                                                    match to_type {                                                        
                                                        MultibodyComponent::Joint => {
                                                            if base.get_outer_joints().contains(&to_id) {
                                                                base.outer_joints.retain(|id| *id != to_id);
                                                                if let Some(joint) = sys.joints.get_mut(&to_id) {
                                                                    joint.delete_inner_body_id();
                                                                    success("Components disconnected!");                                                                    
                                                                } else {
                                                                    error("Joint not found...");
                                                                }
                                                            } else {
                                                                error("Components not connected...");
                                                            } 
                                                        }
                                                        _ => error("Invalid component combo...")
                                                    }
                                                }
                                            }
                                            MultibodyComponent::Body => {
                                                if let Some(body) = sys.bodies.get_mut(&from_id) {
                                                    match to_type {
                                                        MultibodyComponent::Joint => {
                                                            if body.outer_joints.contains(&to_id) {
                                                                body.outer_joints.retain(|id| *id != to_id);
                                                                if let Some(joint) = sys.joints.get_mut(&to_id) {
                                                                    joint.delete_inner_body_id();
                                                                    success("Components disconnected!");                                                                    
                                                                } else {
                                                                    error("Joint not found...");
                                                                }
                                                            } else {
                                                                error("Components not connected...");
                                                            } 
                                                        }
                                                        _ => error("Invalid component combo...")
                                                    }
                                                } else {
                                                    error("Body not found...")
                                                }
                                            }
                                            MultibodyComponent::Gravity => {                                                
                                                match to_type {
                                                    MultibodyComponent::Base => {
                                                        if let Some(base) = &mut sys.base {
                                                            if base.gravity.contains(&to_id) {
                                                                base.gravity.retain(|id| *id != to_id);
                                                                success("Components disconnected!");
                                                            } else {
                                                                error("Components not connected...");
                                                            }                                                            
                                                        }                                                                                                                
                                                    }
                                                    MultibodyComponent::Body => {
                                                        if let Some(body) = sys.bodies.get_mut(&to_id) {
                                                            if body.gravity.contains(&to_id) {
                                                                body.gravity.retain(|id| *id != to_id);
                                                                success("Components disconnected!");
                                                            } else {
                                                                error("Components not connected...");
                                                            }                                                            
                                                        }                                                                                                                
                                                    }
                                                    _ => error("Invalid component combo...")
                                                }
                                            }
                                            MultibodyComponent::Joint => {
                                                if let Some(joint) = sys.joints.get_mut(&from_id) {
                                                    match to_type {
                                                        MultibodyComponent::Body => {

                                                            // Does joint have an outer body id?
                                                            let outer_body_id = match joint.get_outer_body_id() {
                                                                Some(id) => id,
                                                                None => {
                                                                    error("Components not connected...");
                                                                    continue;
                                                                }
                                                            };

                                                            // Is the outer body id == to the to_id to disconnect?
                                                            if *outer_body_id != to_id {
                                                                error("Components not connected...");
                                                                continue;
                                                            }

                                                            // get the body
                                                            if let Some(body) = sys.bodies.get_mut(&to_id) {

                                                                // does body have an inner joint?
                                                                if let Some(inner_joint) = body.inner_joint {

                                                                    // is the inner joint the from id?
                                                                    if inner_joint == from_id {

                                                                        // all checks passed, disconnect
                                                                        body.inner_joint = None;
                                                                        joint.delete_outer_body_id();
                                                                        success("Components disconnected!");
                                                                    } else {

                                                                        // the bodies inner joint was some other joint
                                                                        error("Components not connected...");
                                                                    }                                                                    
                                                                    
                                                                }  else {

                                                                    // the body did not have an inner joint
                                                                    error("Components not connected...");
                                                                }                                                           
                                                            } else {

                                                                // body not found in sys
                                                                error("Body not found...");
                                                            }                                                                                                               
                                                        }
                                                        _ => error("Invalid component combo...")
                                                    }
                                                } else {
                                                    error("Joint not found...")
                                                }
                                            }
                                            

                                        }



                                    } else {
                                        error("Could not find system in systems"); // this should not be possible i believe
                                        continue
                                    }

                                }
                                Commands::Edit {name} => {
                                    // check that a sys is active
                                    let sys_name = match &active_system {
                                        Some(name) => name,
                                        None => {
                                            error("No active system. Create or load a system first.");
                                            continue
                                        }
                                    };

                                    // find the component
                                    if let Some(sys) = systems.get_mut(sys_name) {
                                        let (component_type, id) = match sys.get_from_name(&name) {
                                            Some(component_data) => component_data,
                                            None => {
                                                let s = format!("Error: '{}' not found in '{}'...", name,sys_name);
                                                error(&s);                                                
                                                continue;
                                            }
                                        };

                                        match component_type {
                                            MultibodyComponent::Base => {
                                                if let Some(old_base) = &mut sys.base {
                                                    // create a new base via prompt to get values for old base
                                                    let new_base = match prompt_base() {                                                    
                                                        Ok(base) => {
                                                            base
                                                        }
                                                        Err(e) => match e {
                                                            InputErrors::CtrlC => continue,
                                                            _ => {
                                                                eprintln!("{:?}",e);
                                                                continue;
                                                            }
                                                        }
                                                    };

                                                    // Set values for the old object from the new one, 
                                                    // maintaining id's and connections, then dropping new object
                                                    old_base.set_name(new_base.get_name().to_string());

                                                } else {
                                                    // i think this is impossible, since to find a base it has to exist
                                                    let s = format!("Error: '{}' not found as a base in '{}'...", name,sys_name);
                                                    error(&s);                                                
                                                    continue;
                                                }
                                            }
                                            MultibodyComponent::Body => {
                                                if let Some(old_body) = sys.bodies.get_mut(&id) {
                                                    // create a new object via prompt to get values for old object
                                                    let new_body = match prompt_body() {                                                    
                                                        Ok(body) => {
                                                            body
                                                        }
                                                        Err(e) => match e {
                                                            InputErrors::CtrlC => continue,
                                                            _ => {
                                                                eprintln!("{:?}",e);
                                                                continue;
                                                            }
                                                        }
                                                    };

                                                    // Set values for the old object from the new one, 
                                                    // maintaining id's and connections, then dropping new object
                                                    old_body.set_name(new_body.get_name().to_string());
                                                    old_body.mass_properties = new_body.mass_properties;
                                                    old_body.mesh = new_body.mesh;

                                                } else {
                                                    // i think this is impossible, since to find a base it has to exist
                                                    let s = format!("Error: '{}' not found as a body in '{}'...", name,sys_name);
                                                    error(&s);                                                
                                                    continue;
                                                }
                                            },
                                            MultibodyComponent::Joint => {
                                                if let Some(old_joint) = sys.joints.get_mut(&id) {
                                                    // create a new object via prompt to get values for old object
                                                    match old_joint {
                                                        Joint::Floating(old_joint) => {
                                                            let new_joint = match prompt_floating() {
                                                                Ok(floating) => {
                                                                    floating
                                                                }
                                                                Err(e) => match e {
                                                                    InputErrors::CtrlC => continue,
                                                                    _ => {
                                                                        eprintln!("{:?}",e);
                                                                        continue;
                                                                    }
                                                                }        
                                                            };
                                                            old_joint.set_name(new_joint.get_name().to_string());
                                                            old_joint.parameters = new_joint.parameters;
                                                            old_joint.state = new_joint.state;
                                                        }
                                                        Joint::Prismatic(old_joint) => {
                                                            let new_joint = match prompt_prismatic() {
                                                                Ok(prismatic) => {
                                                                    prismatic
                                                                }
                                                                Err(e) => match e {
                                                                    InputErrors::CtrlC => continue,
                                                                    _ => {
                                                                        eprintln!("{:?}",e);
                                                                        continue;
                                                                    }
                                                                }        
                                                            };
                                                            old_joint.set_name(new_joint.get_name().to_string());
                                                            old_joint.parameters = new_joint.parameters;
                                                            old_joint.state = new_joint.state;
                                                        }
                                                        Joint::Revolute(old_joint) => {
                                                            let new_joint = match prompt_revolute() {
                                                                Ok(revolute) => {
                                                                    revolute
                                                                }
                                                                Err(e) => match e {
                                                                    InputErrors::CtrlC => continue,
                                                                    _ => {
                                                                        eprintln!("{:?}",e);
                                                                        continue;
                                                                    }
                                                                }        
                                                            };
                                                            old_joint.set_name(new_joint.get_name().to_string());
                                                            old_joint.parameters = new_joint.parameters;
                                                            old_joint.state = new_joint.state;
                                                        }
                                                    }
                                                } else {
                                                    // i think this is impossible, since to find a base it has to exist
                                                    let s = format!("Error: '{}' not found as a body in '{}'...", name,sys_name);
                                                    error(&s);                                                
                                                    continue;
                                                }
                                            }
                                            MultibodyComponent::Gravity => {
                                                if let Some(old_gravity) = sys.gravities.get_mut(&id) {
                                                    // create a new object via prompt to get values for old object
                                                    let new_gravity = match prompt_gravity() {                                                    
                                                        Ok(g) => {
                                                            g
                                                        }
                                                        Err(e) => match e {
                                                            InputErrors::CtrlC => continue,
                                                            _ => {
                                                                eprintln!("{:?}",e);
                                                                continue;
                                                            }
                                                        }
                                                    };

                                                    // Set values for the old object from the new one, 
                                                    // maintaining id's and connections, then dropping new object
                                                    old_gravity.set_name(new_gravity.get_name().to_string());
                                                    old_gravity.gravity = new_gravity.gravity;

                                                } else {
                                                    // i think this is impossible, since to find a base it has to exist
                                                    let s = format!("Error: '{}' not found as a body in '{}'...", name,sys_name);
                                                    error(&s);                                                
                                                    continue;
                                                }
                                            },
                                        }
                                        success("Edit complete!");
                                    }

                                }                                 
                                Commands::Exit => {
                                    break;
                                },
                                Commands::Load {system} => {                                         

                                    let mut file = match File::open(&systems_path) {
                                        Ok(file) => file,
                                        Err(e) => {eprintln!("{}",e); continue},
                                    };
                                    let mut content = String::new();
                                    
                                    match file.read_to_string(&mut content) {
                                        Ok(_) => {},
                                        Err(e) => {eprintln!("{}",e); continue},
                                    }

                                    // Deserialize the RON content into a hashmap
                                    let saved_systems: HashMap<String, MultibodySystem> = match from_reader(content.as_bytes()) {
                                        Ok(s) => s,
                                        Err(e) => {eprintln!("{}",e); continue},
                                    };

                                    let sys = match saved_systems.get(&system) {
                                        Some(sys) => sys,
                                        None => {eprintln!("Error: system not found"); continue},
                                    };
                                    systems.insert(system.clone(),sys.clone());     
                                    active_system = Some(system.clone());                                               

                                    let p = colored::Colorize::green(format!("System '{}' successfully loaded!", system).as_str());
                                    println!("{}",p);
                                }

                                Commands::Plot { result, component, state} => {                                    

                                    if let Some(result) = results.get(&result) {
                                        let mut datasets = Vec::new();

                                        let df = result.get_component_state(&component, vec![&state]);
                                        let t = df.column("t").unwrap().f64().unwrap();
                                                                                                                        
                                        let data = match df.column(&state) {
                                            Ok(data) => data.f64().unwrap(),
                                            Err(e) => {                                
                                                error(&(e.to_string()));
                                                continue
                                            }
                                        };
                                        
                                        assert_eq!(t.len(), data.len());

                                        let x_bounds_0 = t.get(0).unwrap();
                                        let x_bounds_2 = t.get(t.len()-1).unwrap();
                                        let x_bounds_1 = (x_bounds_0 + x_bounds_2) /2.0;      

                                        // add an indicator for 0 if if exists in the y range
                                        let points = [(x_bounds_0, 0.0), (x_bounds_2,0.0)];
                                        datasets.push(Dataset::default()                                            
                                            .graph_type(GraphType::Line)
                                            .marker(ratatui::symbols::Marker::Braille)
                                            .style(ratatui::style::Style::default().fg(ratatui::style::Color::Gray))
                                            .data(&points));
                                        
                                        let points: Vec<(f64,f64)> = t
                                            .into_iter()
                                            .zip(data.into_iter())
                                            .map(|(x, y)| (x.unwrap_or_default(),y.unwrap_or_default())
                                            )
                                            .collect();
                
                                        datasets.push(Dataset::default()
                                            .name(state.clone())
                                            .graph_type(GraphType::Line)
                                            .marker(ratatui::symbols::Marker::Braille)
                                            .style(ratatui::style::Style::default().fg(ratatui::style::Color::Cyan))
                                            .data(&points));

                                                                         

                                        // Create the X axis and define its properties
                                        let x_title = "time (sec)".to_string();
                                        let x_axis = Axis::default()
                                            .title(x_title)
                                            .style(ratatui::style::Style::default())
                                            .bounds([x_bounds_0, x_bounds_2])
                                            .labels(vec![format_number(x_bounds_0).into(), format_number(x_bounds_1).into(), format_number(x_bounds_2).into()]);

                                        let y_bounds_0 = df.column(&state).unwrap().min().unwrap().unwrap();
                                        let y_bounds_2 = df.column(&state).unwrap().max().unwrap().unwrap();
                                        let y_bounds_1 = (y_bounds_0 + y_bounds_2) /2.0;                                        

                                        // Create the Y axis and define its properties
                                        //let y_title = colored::Colorize::red("Y Axis").to_string();
                                        let y_axis = Axis::default()
                                          //  .title(y_title)
                                            .style(ratatui::style::Style::default())
                                            .bounds([y_bounds_0, y_bounds_2])
                                            .labels(vec![format_number(y_bounds_0).into(), format_number(y_bounds_1).into(), format_number(y_bounds_2).into()]);

                                        

                                        // Create the chart and link all the parts together
                                        let chart = Chart::new(datasets)
                                            .block(ratatui::widgets::block::Block::new().title(format!("{}: {}", component,state)))
                                            .x_axis(x_axis.into())
                                            .y_axis(y_axis.into());

                                        
                                        // setup terminal                     
                                        //enable_raw_mode();                   
                                        let mut stdout = io::stdout();
                                        execute!(stdout, EnterAlternateScreen, EnableMouseCapture).unwrap();
                                        let backend = CrosstermBackend::new(stdout);
                                        let mut terminal = Terminal::new(backend).unwrap();

                                        loop {                                            
                                            if let Event::Key(key) = event::read().unwrap() {
                                                if key.code == KeyCode::Char('c') && key.modifiers == KeyModifiers::CONTROL {
                                                    break;
                                                }
                                            }
                                            
                                            // set up required closure
                                            let f = |frame: &mut Frame| {
                                                let area = frame.size();
                                                frame.render_widget(&chart,area);
                                            };                                        
                                            let _ = terminal.draw(f);
                                        }
                                        execute!(
                                        terminal.backend_mut(),
                                        LeaveAlternateScreen,
                                        DisableMouseCapture
                                        ).unwrap();                                        
                                        terminal.show_cursor().unwrap();
                                        
                                    }else {
                                        eprintln!("Error: system '{}' not found ", result);
                                    }
                                }
                                Commands::Run => {
                                    let system = match &active_system {
                                        Some(name) => name,
                                        None => {
                                            error("No active system. Create or load a system first.");
                                            continue
                                        }
                                    };

                                    if let Some(sys) = systems.get(system) {
                                        let mut sys_sim = match MultibodySystemSim::try_from(sys.clone()) {
                                            Ok(sys) => sys,
                                            Err(e) => {
                                                let e = format!("{:?}",e);
                                                error(&e);
                                                continue
                                            }
                                        };

                                        // Create a vec of JointStates
                                        let initial_joint_states = JointStates(sys_sim.joints.iter().map(|joint| joint.get_state()).collect());
                                        let mut final_joint_states = initial_joint_states.clone();

                                        sys_sim.run(&mut final_joint_states,&initial_joint_states,0.0);

                                        //dbg!(&sys_sim);
                                    }
                                }
                                Commands::Save => {
                                    let system = match &active_system {
                                        Some(name) => name,
                                        None => {
                                            error("No active system. Create or load a system first.");
                                            continue
                                        }
                                    };

                                    if let Some(sys) = systems.get(system) {

                                         // Open the file, creating it if it doesn't exist, or appending to it if it does exist
                                        let file = OpenOptions::new()
                                            .read(true)                                            
                                            .open(&systems_path);

                                        // Initialize or load the hashmap
                                        let mut systems: HashMap<String, MultibodySystem> = match file {
                                            Ok(mut file) => {
                                                let mut content = String::new();
                                                file.read_to_string(&mut content).unwrap();
                                                from_reader(content.as_bytes()).unwrap()
                                            },
                                            Err(_) => HashMap::new(),
                                        };                                           

                                        // Add a new entry to the hashmap                                        
                                        systems.insert(system.clone(), sys.clone());

                                        let ron_string = match to_string_pretty(&systems,PrettyConfig::new()) {
                                            Ok(string) => string,
                                            Err(e) => {eprintln!("{}", e); continue}
                                        };

                                        //handle result
                                        let mut file = match File::create(&systems_path) {
                                            Ok(file) => file,
                                            Err(e) => {eprintln!("{}", e); continue},
                                        };

                                        // Write the RON string to the file
                                        let res = file.write_all(ron_string.as_bytes());
                                        match res {
                                            Ok(_) => {},
                                            Err(e) => eprintln!("{}", e)
                                        }
                                    }
                                }

                                Commands::View { sys_or_res, name, component, state} => {
                                    match sys_or_res.as_str() {
                                        "system" => { 
                                            if let Some(sys) = systems.get(&name) {
                                                println!("{:#?}", sys);
                                            } else {
                                                let s = format!("{} not found in systems...", name);
                                                error(&s);
                                            }
                                        }                
                                        "result" => { 
                                            if let Some(res) = results.get(&name) {
                                                if let Some(component) = component {
                                                    let res = res.get_component(&component);
                                                    if let Some(state) = state {
                                                        match res.column(&state) {
                                                            Ok(res) => println!("{}", res),
                                                            Err(e) => eprintln!("{}", e),
                                                        }
                                                    } else {
                                                        println!("{:#?}", res);
                                                    }
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
                                Commands::Simulate  => {
                                    let system_name = match &active_system {
                                        Some(name) => name,
                                        None => {
                                            error("No active system. Create or load a system first.");
                                            continue
                                        }
                                    };
                                    if let Some(system) = systems.get_mut(system_name) {
                                        match system.validate() {
                                            Ok(_)=> {
                                                success("System validated!");                                                
                                                match prompt_simulation() {
                                                    Ok((name, start_time,stop_time, dt)) => {                                                        
                                                        let result = system.simulate(name.clone(),start_time,stop_time,dt);
                                                        match result {
                                                            Ok(result) => {
                                                                let s = format!("Simulation '{}' completed in {:#?}!", name, result.total_duration);
                                                                results.insert(result.name.clone(),result);                                                                
                                                                success(&s);
                                                            },
                                                            Err(e) => eprintln!("{:#?}",e)
                                                        }
                                                    }
                                                    Err(e) => {
                                                        match e {
                                                            InputErrors::CtrlC => continue,
                                                            _ => {eprintln!("{:?}",e); continue}
                                                        }
                                                    }
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
                    Signal::CtrlC | Signal::CtrlD => {
                        break
                    }
                }
                
            }
            Err(e) => eprintln!("{:#?}", e)            
        }
    }    
}

enum Prompts {
    //Algorithm,
    Angle,
    AngularRate,
    AngularRateX,
    AngularRateY,
    AngularRateZ,
    CartesianX,
    CartesianY,
    CartesianZ,
    Cmx,
    Cmy,
    Cmz,
    Color,
    ColorConstant,
    ColorR,
    ColorG,
    ColorB,
    ColorA,
    CuboidX,
    CuboidY,
    CuboidZ,
    //CylindricalA,
    //CylindricalH,
    //CylindricalR,
    Earth,
    EllipsoidRadiusX,
    EllipsoidRadiusY,
    EllipsoidRadiusZ,
    EllipsoidLatitudeBands,
    EllipsoidLongitudeBands,
    Ixx,
    Iyy,
    Izz,
    Ixy,
    Ixz,
    Iyz,
    Geometry,    
    GravityType,
    GravityConstantX,
    GravityConstantY,
    GravityConstantZ,
    GravityTwoBodyMu,
    JointDamping,
    JointDampingRotationX,
    JointDampingRotationY,
    JointDampingRotationZ,
    JointDampingTranslationX,
    JointDampingTranslationY,
    JointDampingTranslationZ,
    JointForce,
    JointForceRotationX,
    JointForceRotationY,
    JointForceRotationZ,
    JointForceTranslationX,
    JointForceTranslationY,
    JointForceTranslationZ,
    JointMechanics,
    JointSpring,
    JointSpringRotationX,
    JointSpringRotationY,
    JointSpringRotationZ,
    JointSpringTranslationX,
    JointSpringTranslationY,
    JointSpringTranslationZ,
    Main,
    Mass,
    Material,
    Mesh,
    Name,
    Position,
    PositionX,
    PositionY,
    PositionZ,
    QuaternionW,
    QuaternionX,
    QuaternionY,
    QuaternionZ,
    SimulationDt,
    SimulationStart,
    SimulationStop,
    //SphericalR,
    //SphericalA,
    //SphericalI,
    SpecularPower,
    Transform,
    TransformRotation,
    TransformTranslation,
    Velocity,
    VelocityX,
    VelocityY,
    VelocityZ,
}

impl Prompts {
    fn get_string(&self) -> &str {
        match self {
            //Prompts::Algorithm => "Algorithm ['aba', 'crb'] (default: crb)", 
            Prompts::Angle => "Initial angle (units: rad, default: 0.0)",
            Prompts::AngularRate => "Initial angular rate (units: rad/sec, default: 0.0)",
            Prompts::AngularRateX => "Initial angular rate X (units: rad/sec, default: 0.0)",
            Prompts::AngularRateY => "Initial angular rate Y (units: rad/sec, default: 0.0)",
            Prompts::AngularRateZ => "Initial angular rate Z (units: rad/sec, default: 0.0)",
            Prompts::CartesianX => "Cartesian [X] (units: m, default: 0.0)",
            Prompts::CartesianY => "Cartesian [Y] (units: m, default: 0.0)",
            Prompts::CartesianZ => "Cartesian [Z] (units: m, default: 0.0)",            
            Prompts::Cmx => "Center of mass [X] (units: m, default: 0.0)",
            Prompts::Cmy => "Center of mass [Y] (units: m, default: 0.0)",
            Prompts::Cmz => "Center of mass [Z] (units: m, default: 0.0)",
            Prompts::Color => "Color ['c' constant, 'r' rgba] (default: 'c')",
            Prompts::ColorConstant => "Color ['white','red','blue','green'] (default: 'white')",
            Prompts::ColorR => "Color Red [0-1] (default: 1)",
            Prompts::ColorG => "Color Green [0-1] (default: 1)",
            Prompts::ColorB => "Color Blue [0-1] (default: 1)",
            Prompts::ColorA => "Color Alpha [0-1] (default: 1)",
            Prompts::CuboidX => "Cuboid X (units: m, default: 1.0)",
            Prompts::CuboidY => "Cuboid Y (units: m, default: 1.0)",
            Prompts::CuboidZ => "Cuboid Z (units: m, default: 1.0)",
            //Prompts::CylindricalR => "Cylindrical radius (units: m, default: 0.0)",
            //Prompts::CylindricalA => "Cylindrical azimuth (units: rad, default: 0.0)",
            //Prompts::CylindricalH => "Cylindrical height (units: m, default: 0.0)",
            Prompts::EllipsoidLatitudeBands => "Number of latitude bands (default: 16)",
            Prompts::Earth => "Animate base as earth? (default: 'n')",
            Prompts::EllipsoidLongitudeBands => "Number of longitude bands (default: 16)",
            Prompts::EllipsoidRadiusX => "Ellipsoid radius X (default: 1.0)",
            Prompts::EllipsoidRadiusY => "Ellipsoid radius Y (default: 1.0)",
            Prompts::EllipsoidRadiusZ => "Ellipsoid radius Z (default: 1.0)",            
            Prompts::Geometry => "Geometry type ['c' cuboid,'e' ellipsoid] (default: 'c')",
            Prompts::GravityType => "Gravity type ['c' (constant), '2' (two body)]",
            Prompts::GravityConstantX => "Constant gravity X (m/sec^2, default: 0.0)",
            Prompts::GravityConstantY => "Constant gravity Y (m/sec^2), default: 0.0)",
            Prompts::GravityConstantZ => "Constant gravity Z (m/sec^2), default: 0.0)",
            Prompts::GravityTwoBodyMu => "Two body gravity mu (m^3/sec^2), default: Earth)",
            Prompts::Ixx => "Ixx (units: kg-m^2, default: 1.0)",
            Prompts::Iyy => "Iyy (units: kg-m^2, default: 1.0)",
            Prompts::Izz => "Izz (units: kg-m^2, default: 1.0)",
            Prompts::Ixy => "Ixy (units: kg-m^2, default: 0.0)",
            Prompts::Ixz => "Ixz (units: kg-m^2, default: 0.0)",
            Prompts::Iyz => "Iyz (units: kg-m^2, default: 0.0)",
            Prompts::JointDamping => "Damping (units: None, default: 0.0)",
            Prompts::JointDampingRotationX => "Rotational damping X (units: None, default: 0.0)",
            Prompts::JointDampingRotationY => "Rotational damping Y (units: None, default: 0.0)",
            Prompts::JointDampingRotationZ => "Rotational damping Z (units: None, default: 0.0)",
            Prompts::JointDampingTranslationX => "Translational damping X (units: None, default: 0.0)",
            Prompts::JointDampingTranslationY => "Translational damping Y (units: None, default: 0.0)",
            Prompts::JointDampingTranslationZ => "Translational damping Z (units: None, default: 0.0)",
            Prompts::JointForce => "Constant force (units: N, default: 0.0)",
            Prompts::JointForceRotationX => "Rotational constant force X (units: N, default: 0.0)",
            Prompts::JointForceRotationY => "Rotational constant force Y (units: N, default: 0.0)",
            Prompts::JointForceRotationZ => "Rotational constant force Z (units: N, default: 0.0)",
            Prompts::JointForceTranslationX => "Translational constant force X (units: N, default: 0.0)",
            Prompts::JointForceTranslationY => "Translational constant force X (units: N, default: 0.0)",
            Prompts::JointForceTranslationZ => "Translational constant force X (units: N, default: 0.0)",
            Prompts::JointMechanics => "Specify joint internal mechanics? ['y' (yes),'n' (no)] (default: n)",
            Prompts::JointSpring => "Spring constant (units: N/m, default: 0.0)",
            Prompts::JointSpringRotationX => "Rotational spring constant X (units: N/m, default: 0.0)",
            Prompts::JointSpringRotationY => "Rotational spring constant Y (units: N/m, default: 0.0)",
            Prompts::JointSpringRotationZ => "Rotational spring constant Z (units: N/m, default: 0.0)",
            Prompts::JointSpringTranslationX => "Translational spring constant X (units: N/m, default: 0.0)",
            Prompts::JointSpringTranslationY => "Translational spring constant Y (units: N/m, default: 0.0)",
            Prompts::JointSpringTranslationZ => "Translational spring constant Z (units: N/m, default: 0.0)",
            Prompts::Main => "GADGT",
            Prompts::Mass => "Mass (units: kg, default: 1.0)",
            Prompts::Material => "material type? ['b' (basic) 'p' (phong)] (default: 'b')",            
            Prompts::Mesh => "add mesh for animation? ['y','n'] (default: n)",
            Prompts::Name => "Name",
            Prompts::Position => "Initial position (units: m, default: 0.0)",
            Prompts::PositionX => "Initial position X (units: m, default: 0.0)",
            Prompts::PositionY => "Initial position Y (units: m, default: 0.0)",
            Prompts::PositionZ => "Initial position Z (units: m, default: 0.0)",
            Prompts::QuaternionW => "Quaternion W (units: None, default: 1.0)",
            Prompts::QuaternionX => "Quaternion X (units: None, default: 0.0)",
            Prompts::QuaternionY => "Quaternion Y (units: None, default: 0.0)",
            Prompts::QuaternionZ => "Quaternion Z (units: None, default: 0.0)",
            Prompts::SimulationDt => "dt (units: sec, default: 0.1)",
            Prompts::SimulationStart => "start_time (units: sec, default: 0.0)",
            Prompts::SimulationStop => "stop_time (units: sec, default: 10.0)", 
            Prompts::SpecularPower => "specular power (default: 32.0)",
            //Prompts::SphericalR => "Spherical radius (units: m, default: 0.0)",
            //Prompts::SphericalA => "Spherical azimuth (units: rad, default: 0.0)",
            //Prompts::SphericalI => "Spherical inclination (units: rad, default: 0.0)",            
            Prompts::Transform => "Transform ('i/identity','c/custom',  default: i)",
            Prompts::TransformRotation => "Rotation ['i' (identity), 'q' (quaternion), 'r' (rotation matrix), 'e' (euler angles), 'a' (aligned axes)]",
            Prompts::TransformTranslation => "Translation ['z' (zero), 'cart' (cartesian), 'cyl' (cylindrical), 'sph' (spherical)]",
            Prompts::Velocity => "Initial velocity (units: m/s, default: 0.0)",
            Prompts::VelocityX => "Initial velocity X (units: m/s, default: 0.0)",
            Prompts::VelocityY => "Initial velocity Y (units: m/s, default: 0.0)",
            Prompts::VelocityZ => "Initial velocity Z (units: m/s, default: 0.0)",
        }
    }

    fn validate_loop(&self, default: &str) -> Result<String, InputErrors> {        
        loop {
            let s = self.prompt()?;
            self.validate(&s)?;

            if s.is_empty() {
                return Ok(default.to_string())
            } else {
                return Ok(s)
            }
                
        }        
    }
    

    fn prompt(&self) -> Result<String,InputErrors> {
        let mut history_path = config_dir().unwrap_or_else(|| PathBuf::from("."));
        history_path.push("gadgt");
        history_path.push("command_history.txt");   
        
        let history = Box::new(
            FileBackedHistory::with_file(100, history_path)
              .expect("Error configuring history with file"),
          );

        let mut rl = Reedline::create().with_history(history).with_ansi_colors(true);
        //let prompt_string = colored::Colorize::cyan(self.get_string()).to_string();
        //let prompt = DefaultPrompt::new(DefaultPromptSegment::Basic(prompt_string), DefaultPromptSegment::Empty);    
        loop {
            let result = rl.read_line(self);
            match result {
                Ok(signal) => {
                    match signal {
                        Signal::Success(input) => {
                            return Ok(input)
                        }
                        Signal::CtrlC | Signal::CtrlD => return Err(InputErrors::CtrlC),
                    }
                }
                Err(e) => eprintln!("{:#?}", e)
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
            | Prompts::JointDampingRotationX
            | Prompts::JointDampingRotationY
            | Prompts::JointDampingRotationZ
            | Prompts::JointDampingTranslationX
            | Prompts::JointDampingTranslationY
            | Prompts::JointDampingTranslationZ
            | Prompts::JointForce
            | Prompts::JointForceRotationX
            | Prompts::JointForceRotationY
            | Prompts::JointForceRotationZ
            | Prompts::JointForceTranslationX
            | Prompts::JointForceTranslationY
            | Prompts::JointForceTranslationZ
            | Prompts::JointSpring
            | Prompts::JointSpringRotationX
            | Prompts::JointSpringRotationY
            | Prompts::JointSpringRotationZ
            | Prompts::JointSpringTranslationX
            | Prompts::JointSpringTranslationY
            | Prompts::JointSpringTranslationZ
            | Prompts::Position
            | Prompts::PositionX
            | Prompts::PositionY
            | Prompts::PositionZ
            | Prompts::QuaternionW
            | Prompts::QuaternionX
            | Prompts::QuaternionY
            | Prompts::QuaternionZ
            | Prompts::SpecularPower
            | Prompts::Velocity
            | Prompts::VelocityX
            | Prompts::VelocityY
            | Prompts::VelocityZ => {
                if str.is_empty() {
                    //leave empty to use default
                    return Ok(())
                }
                if str.parse::<f64>().is_err() {
                    return Err(InputErrors::NonNumeric);
                }
                Ok(())
            }
            Prompts::Color => {
                if str.is_empty() {
                    //leave empty to use default
                    return Ok(())
                }
                let possible_values = ["c", "r"];
                if !possible_values.contains(&(str.to_lowercase().as_str())) {
                    return Err(InputErrors::InvalidColor);
                }
                Ok(())
            }
            // numeric and > 0
            Prompts::GravityTwoBodyMu | Prompts::Mass | Prompts::Ixx | Prompts::Iyy | Prompts::Izz => {
                if str.is_empty() {
                    //leave empty to use default
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
            //Prompts::Algorithm => {
            //    if str.is_empty() {
            //        //leave empty to use default
            //        return Ok(())
            //    }
            //    let possible_values = ["aba", "crb"];
            //    if !possible_values.contains(&(str.to_lowercase().as_str())) {
            //        return Err(InputErrors::InvalidGravity);
            //    }
            //    Ok(())
            //}
            //GeometryType 
            Prompts::Geometry => {
                if str.is_empty() {
                    //leave empty to use default
                    return Ok(())
                }
                let possible_values = ["c", "e"];
                if !possible_values.contains(&(str.to_lowercase().as_str())) {
                    return Err(InputErrors::InvalidGeometry);
                }
                Ok(())
            }
            Prompts::Material => {
                if str.is_empty() {
                    //leave empty to use default
                    return Ok(())
                }
                let possible_values = ["b", "p"];
                if !possible_values.contains(&(str.to_lowercase().as_str())) {
                    return Err(InputErrors::InvalidMaterial);
                }
                Ok(())
            }
            //Gravity
            Prompts::GravityType => {
                if str.is_empty() {
                    //leave empty to use default
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
            // Yes or No
            Prompts::JointMechanics | Prompts::Earth | Prompts::Mesh => {
                if str.is_empty() {
                    //user must provide a default, but this is ok
                    return Ok(())
                }
                let possible_values = [
                    "y",                    
                    "n",                                        
                ];
                if !possible_values.contains(&(str.to_lowercase().as_str())) {
                    return Err(InputErrors::NotYesOrNo);
                }
                Ok(())
            }
            _ => Ok(()),
        }
    }
}

#[derive(Debug)]
pub enum InputErrors {
    Body(BodyErrors),
    CtrlC,
    InvalidColor,
    InvalidGeometry,
    InvalidGravity,
    InvalidMaterial,
    InvalidRotation,
    InvalidTransform,
    NonNumeric,
    NotGreaterThanZero,
    NotYesOrNo,
}

impl From<BodyErrors> for InputErrors {
    fn from(b: BodyErrors) -> InputErrors {
        InputErrors::Body(b)
    }
}

impl std::fmt::Display for InputErrors {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            InputErrors::Body(b) => write!(f,"{:?}", b), //TODO: implement Display for BodyErrors
            InputErrors::CtrlC => write!(f,"Error: ctrl+C pressed"),
            InputErrors::InvalidColor => write!(f,"Error: input must be ['c' (constant), 'r' (rgba)]"),
            InputErrors::InvalidGeometry => write!(f,"Error: input must be ['c' (cuboid), 'e' (ellipsoid)]"),
            InputErrors::InvalidGravity => write!(f,"Error: input must be ['c' (constant), '2' (two body)]"),
            InputErrors::InvalidMaterial => write!(f,"Error: input must be ['b' (basic), 'p' (phong)]"),
            InputErrors::InvalidRotation => write!(f,"Error: input must be ['i' (identity), 'q' (quaternion), 'r' (rotation matrix), 'e' (euler angles), 'a' (aligned axes)]"),
            InputErrors::InvalidTransform => write!(f,"Error: input must be ['i' (identity), 'c' (custom)]"),
            InputErrors::NonNumeric => write!(f,"Error: input must be numeric"),
            InputErrors::NotGreaterThanZero => write!(f,"Error: input must be greater than 0"),
            InputErrors::NotYesOrNo => write!(f,"Error: input must be ['y' (yes), 'n' (no)]"),
        }
    }
}

fn prompt_base() -> Result<Base,InputErrors> {
    let name = Prompts::Name.validate_loop("base")?;
    let earth = match Prompts::Earth.validate_loop("n")?.as_str() {
        "y" => true,
        "n" => false,
        _ => panic!("not possible")
    };
    
    Ok(Base::new(&name, earth))    
}

fn prompt_body() -> Result<Body, InputErrors> {
    let name = Prompts::Name.prompt()?;
    let mass = Prompts::Mass.validate_loop("1.0")?.parse::<f64>().unwrap_or(1.0);
    let cmx = Prompts::Cmx.validate_loop("0.0")?.parse::<f64>().unwrap_or(0.0);
    let cmy = Prompts::Cmy.validate_loop("0.0")?.parse::<f64>().unwrap_or(0.0);
    let cmz = Prompts::Cmz.validate_loop("0.0")?.parse::<f64>().unwrap_or(0.0);
    let ixx = Prompts::Ixx.validate_loop("1.0")?.parse::<f64>().unwrap_or(1.0);
    let iyy = Prompts::Iyy.validate_loop("1.0")?.parse::<f64>().unwrap_or(1.0);
    let izz = Prompts::Izz.validate_loop("1.0")?.parse::<f64>().unwrap_or(1.0);
    let ixy = Prompts::Ixy.validate_loop("0.0")?.parse::<f64>().unwrap_or(0.0);
    let ixz = Prompts::Ixz.validate_loop("0.0")?.parse::<f64>().unwrap_or(0.0);
    let iyz = Prompts::Iyz.validate_loop("0.0")?.parse::<f64>().unwrap_or(0.0);

    let mesh = Prompts::Mesh.validate_loop("n")?;
    let mesh = match mesh.as_str() {
        "y" => Some(prompt_mesh(name.clone())?),
        "n" => None,
        _ => panic!("shouldnt be possible, caught in prompt_geometry")
    };

    let com = CenterOfMass::new(cmx, cmy, cmz);
    let inertia =
        Inertia::new(ixx, iyy, izz, ixy, ixz, iyz).unwrap();
    let mass_properties =
        MassProperties::new(mass, com, inertia).unwrap();

    let body = match mesh {
        Some(mesh) => Body::new(&name, mass_properties)?.with_mesh(mesh),
        None => Body::new(&name, mass_properties)?
    };    
    Ok(body)    
}

fn prompt_color() -> Result<Color, InputErrors> {
    let color_type = Prompts::Color.validate_loop("c")?;
    let rgba = match color_type.as_str() {
        "c" => {
            
                let color = Prompts::ColorConstant.validate_loop("white")?;
                match color.as_str() {
                    "white" => Color::new(1.0,1.0,1.0,1.0),
                    "red" => Color::new(1.0,0.0,0.0,1.0),
                    "green" => Color::new(0.0,1.0,0.0,1.0),
                    "blue" => Color::new(0.0,0.0,1.0,1.0),
                    _ => panic!("invalid color, should have been caught by validate loop")
                }            
        },
        "r" => {
            let r = Prompts::ColorR.validate_loop("1.0")?.parse::<f32>().unwrap_or(1.0);
            let g = Prompts::ColorG.validate_loop("1.0")?.parse::<f32>().unwrap_or(1.0);
            let b = Prompts::ColorB.validate_loop("1.0")?.parse::<f32>().unwrap_or(1.0);
            let a = Prompts::ColorA.validate_loop("1.0")?.parse::<f32>().unwrap_or(1.0);
            Color::new(r,g,b,a)
        }
        _ => panic!("invalid option, should have been caught by validate loop")
    };
    Ok(rgba)
}

fn prompt_gravity() -> Result<MultibodyGravity, InputErrors> {
    
    let name = Prompts::Name.prompt()?;
    let gravity_type = Prompts::GravityType.validate_loop("c")?;

    let gravity = match gravity_type.trim() {
        "c" => {
            let x = Prompts::GravityConstantX.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);                
            let y = Prompts::GravityConstantY.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);                
            let z = Prompts::GravityConstantZ.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
            Gravity::Constant(ConstantGravity::new(x,y,z))            
        }
        "2" => {
            let earth = aerospace::gravity::EARTH;
            let earth_string = earth.to_string();            
            let mu = Prompts::GravityTwoBodyMu.validate_loop(&earth_string)?.parse::<f64>().unwrap_or(earth);                
            Gravity::TwoBody(TwoBodyGravity::new(mu))
        }
        _ => panic!("shouldn't be possible. other characters caught in validation loop")
    };
    Ok(MultibodyGravity::new(&name,gravity))

}

fn prompt_joint_mechanics() -> Result<bool,InputErrors> {
    let jm = Prompts::JointMechanics.validate_loop("n")?;    
    match jm.trim() {
        "y" => Ok(true),
        "n" => {            
            Ok(false)
        }
        _ => panic!("shouldn't be possible. other characters caught in validation loop")
    }
}

fn prompt_floating() -> Result<Floating, InputErrors> {

    let name = Prompts::Name.prompt()?;
    let qx = Prompts::QuaternionX.validate_loop("0.0")?.parse::<f64>().unwrap_or(0.0);
    let qy = Prompts::QuaternionY.validate_loop("0.0")?.parse::<f64>().unwrap_or(0.0);
    let qz = Prompts::QuaternionZ.validate_loop("0.0")?.parse::<f64>().unwrap_or(0.0);
    let qw = Prompts::QuaternionW.validate_loop("1.0")?.parse::<f64>().unwrap_or(1.0);
    let q = Quaternion::new(qx,qy,qz,qw).normalize();

    let wx = Prompts::AngularRateX.validate_loop("0.0")?.parse::<f64>().unwrap_or(0.0);
    let wy = Prompts::AngularRateY.validate_loop("0.0")?.parse::<f64>().unwrap_or(0.0);
    let wz = Prompts::AngularRateZ.validate_loop("0.0")?.parse::<f64>().unwrap_or(0.0);
    let w = Vector3::new(wx,wy,wz);

    let rx = Prompts::PositionX.validate_loop("0.0")?.parse::<f64>().unwrap_or(0.0);
    let ry = Prompts::PositionY.validate_loop("0.0")?.parse::<f64>().unwrap_or(0.0);
    let rz = Prompts::PositionZ.validate_loop("0.0")?.parse::<f64>().unwrap_or(0.0);
    let r = Vector3::new(rx,ry,rz);

    let vx = Prompts::VelocityX.validate_loop("0.0")?.parse::<f64>().unwrap_or(0.0);
    let vy = Prompts::VelocityY.validate_loop("0.0")?.parse::<f64>().unwrap_or(0.0);
    let vz = Prompts::VelocityZ.validate_loop("0.0")?.parse::<f64>().unwrap_or(0.0);
    let v = Vector3::new(vx,vy,vz);    
    
    let state = FloatingState::new(q,w,r,v);

    let mechanics = prompt_joint_mechanics()?;

    let parameters = match mechanics {
        false => [JointParameters::default();6],
        true => {
            let force_x_rotation = Prompts::JointForceRotationX.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
            let spring_x_rotation = Prompts::JointSpringRotationX.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
            let damping_x_rotation = Prompts::JointDampingRotationX.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
            let parameters_x_rotation = JointParameters::new(force_x_rotation,damping_x_rotation,spring_x_rotation);

            let force_y_rotation = Prompts::JointForceRotationY.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
            let spring_y_rotation = Prompts::JointSpringRotationY.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
            let damping_y_rotation = Prompts::JointDampingRotationY.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
            let parameters_y_rotation = JointParameters::new(force_y_rotation,damping_y_rotation,spring_y_rotation);

            let force_z_rotation = Prompts::JointForceRotationZ.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
            let spring_z_rotation = Prompts::JointSpringRotationZ.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
            let damping_z_rotation = Prompts::JointDampingRotationZ.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
            let parameters_z_rotation = JointParameters::new(force_z_rotation,damping_z_rotation,spring_z_rotation);

            let force_x_translation = Prompts::JointForceTranslationX.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
            let spring_x_translation = Prompts::JointSpringTranslationX.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
            let damping_x_translation = Prompts::JointDampingTranslationX.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
            let parameters_x_translation = JointParameters::new(force_x_translation,damping_x_translation,spring_x_translation);

            let force_y_translation = Prompts::JointForceTranslationY.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
            let spring_y_translation = Prompts::JointSpringTranslationY.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
            let damping_y_translation = Prompts::JointDampingTranslationY.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
            let parameters_y_translation = JointParameters::new(force_y_translation,damping_y_translation,spring_y_translation);

            let force_z_translation = Prompts::JointForceTranslationZ.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
            let spring_z_translation = Prompts::JointSpringTranslationZ.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
            let damping_z_translation = Prompts::JointDampingTranslationZ.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
            let parameters_z_translation = JointParameters::new(force_z_translation,damping_z_translation,spring_z_translation);

            [parameters_x_rotation, parameters_y_rotation, parameters_z_rotation, parameters_x_translation,parameters_y_translation,parameters_z_translation]            
        }
    };

    Ok(Floating::new(&name, parameters, state))
}

fn prompt_prismatic() -> Result<Prismatic, InputErrors> {
    let name = Prompts::Name.prompt()?;
    let position = Prompts::Position.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
    let velocity = Prompts::Velocity.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
    let state = PrismaticState::new(position, velocity);

    let force = Prompts::JointForce.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);            
    let spring = Prompts::JointSpring.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);        
    let damping = Prompts::JointDamping.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
    let parameters =
        JointParameters::new(force, damping, spring);

    Ok(Prismatic::new(&name, parameters, state))
}

fn prompt_revolute() -> Result<Revolute, InputErrors> {
    let name = Prompts::Name.prompt()?;
    let angle = Prompts::Angle.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
    let angular_rate = Prompts::AngularRate.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
    let state = RevoluteState::new(angle, angular_rate);

    let force = Prompts::JointForce.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);            
    let spring = Prompts::JointSpring.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);        
    let damping = Prompts::JointDamping.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
    let parameters =
        JointParameters::new(force, damping, spring);

    Ok(Revolute::new(&name, parameters, state))
}

fn prompt_transform() -> Result<Transform,InputErrors> {
    let transform = Prompts::Transform.validate_loop("i")?;    
    match transform.trim() {
        "i" => Ok(Transform::IDENTITY),
        "c" => {            
            let rotation = prompt_rotation()?;            
            let translation = prompt_translation()?;
            Ok(Transform::new(rotation,translation))
        }
        _ => panic!("shouldn't be possible. other characters caught in validation loop")
    }
}


fn prompt_rotation() -> Result<Rotation,InputErrors> {
    let rotation = Prompts::TransformRotation.validate_loop("i")?;                        
    match rotation.trim() {
        "i" => Ok(Rotation::IDENTITY),
        "q" => {
            let q = prompt_quaternion()?;
            Ok(Rotation::from(q))
        },    
        _ => panic!("shouldn't be possible. other characters caught in validation loop")       
    }
}

fn prompt_quaternion() -> Result<Quaternion, InputErrors> {

    let w = Prompts::QuaternionW.validate_loop("1")?.parse::<f64>().unwrap_or(1.0);        
    let x = Prompts::QuaternionX.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);        
    let y = Prompts::QuaternionY.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
    let z = Prompts::QuaternionZ.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);        
    
    let q = Quaternion::new(x,y,z,w);
    println!("{:?}", q);
    Ok(q)
}

fn prompt_translation() -> Result<CoordinateSystem,InputErrors> {
    let translation = Prompts::TransformTranslation.validate_loop("z")?;                        
    match translation.trim() {
        "z" => Ok(CoordinateSystem::ZERO),
        "cart" => {
            let cartesian = prompt_cartesian()?;
            Ok(CoordinateSystem::from(cartesian))
        },    
        _ => panic!("shouldn't be possible. other characters caught in validation loop")       
    }
}

fn prompt_cartesian() -> Result<Cartesian,InputErrors> {
    let x = Prompts::CartesianX.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
    let y = Prompts::CartesianY.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
    let z = Prompts::CartesianZ.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
    let cartesian = Cartesian::new(x,y,z);
    println!("{:?}", cartesian);
    Ok(cartesian)
}

fn prompt_simulation() -> Result<(String,f64,f64,f64),InputErrors> {   
        let name = Prompts::Name.validate_loop("temp")?;        
        let start_time = Prompts::SimulationStart.validate_loop("0")?.parse::<f64>().unwrap_or(0.0);
        let stop_time = Prompts::SimulationStop.validate_loop("10")?.parse::<f64>().unwrap_or(10.0);        
        let dt = Prompts::SimulationDt.validate_loop("0.1")?.parse::<f64>().unwrap_or(0.1);
        Ok((name, start_time,stop_time,dt))
}

fn prompt_cuboid() -> Result<Cuboid, InputErrors> {
    let x = Prompts::CuboidX.validate_loop("1.0")?.parse::<f32>().unwrap_or(1.0);
    let y = Prompts::CuboidY.validate_loop("1.0")?.parse::<f32>().unwrap_or(1.0);
    let z = Prompts::CuboidZ.validate_loop("1.0")?.parse::<f32>().unwrap_or(1.0);    
    Ok(Cuboid::new(x,y,z))
}

fn prompt_ellipsoid() -> Result<Geometry, InputErrors> {
    let x = Prompts::EllipsoidRadiusX.validate_loop("1.0")?.parse::<f32>().unwrap_or(1.0);
    let y = Prompts::EllipsoidRadiusY.validate_loop("1.0")?.parse::<f32>().unwrap_or(1.0);
    let z = Prompts::EllipsoidRadiusZ.validate_loop("1.0")?.parse::<f32>().unwrap_or(1.0);
    let lat = Prompts::EllipsoidLatitudeBands.validate_loop("16")?;
    
    let geometry = match lat.as_str() {
        "16" => Geometry::Ellipsoid16(Ellipsoid16::new(x,y,z)),
        "32" => Geometry::Ellipsoid16(Ellipsoid16::new(x,y,z)),
        "64" => Geometry::Ellipsoid16(Ellipsoid16::new(x,y,z)),
        _ => panic!("should not be possible, caught in validate loop")
    };
    Ok(geometry)
}

fn prompt_mesh(name: String) -> Result<Mesh, InputErrors> {
    let geometry = prompt_geometry()?;
    let material = prompt_material()?;
    Ok(Mesh {
        name,
        geometry,
        material,
        state: GeometryState::default(),
        texture: None,
    })
}

fn prompt_geometry() -> Result<Geometry, InputErrors> {
    let geometry_type = Prompts::Geometry.validate_loop("c")?;
    match geometry_type.as_str() {
        "c" => {
            let cuboid = prompt_cuboid()?;
            Ok(Geometry::Cuboid(cuboid))
        }
        "e" => {
            let ellipsoid = prompt_ellipsoid()?;
            Ok(ellipsoid)
        }
        _ => {
            Err(InputErrors::CtrlC)
        }
    }
}
fn prompt_material() -> Result<Material, InputErrors> {
    let material_type = Prompts::Material.validate_loop("b")?;
    match material_type.as_str() {
        "b" => {
            let color = prompt_color()?;
            Ok(Material::Basic{color})
        }
        "p" => {
            let color = prompt_color()?;
            let specular_power = Prompts::SpecularPower.validate_loop("32.0")?.parse::<f32>().unwrap_or(32.0);
            Ok(Material::Phong{color, specular_power})
        }
        _ => {
            Err(InputErrors::CtrlC)
        }
    }
}




fn success(s: &str) {
    println!("{}",colored::Colorize::green(s))
}

fn error(s: &str) {
    eprintln!("{}",colored::Colorize::red(s))
}

impl Prompt for Prompts {
    fn render_prompt_left(&self) -> Cow<str> {
        match self {
            Prompts::Main => {
                colored::Colorize::blue(self.get_string()).to_string().into()        
            }
            _ => colored::Colorize::cyan(self.get_string()).to_string().into()        
        }
        
    }

    fn render_prompt_right(&self) -> Cow<str> {
        "".into()
    }

    fn render_prompt_indicator(&self, _edit_mode: PromptEditMode) -> Cow<str> {
        colored::Colorize::blue(">").to_string().into()        
    }

    fn render_prompt_multiline_indicator(&self) -> Cow<str> {
        "".into()
    }

    fn render_prompt_history_search_indicator(
        &self,
        _history_search: PromptHistorySearch,
    ) -> Cow<str> {
        "".into()
    }
}