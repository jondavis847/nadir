#[cfg(feature = "dhat-heap")]
#[global_allocator]
static ALLOC: dhat::Alloc = dhat::Alloc;


use clap::{Parser, Subcommand, ValueEnum};
use cliclack::{intro,outro};
use colored::Colorize;
use dirs_next::config_dir;
use multibody::{
    base::BaseSystems, body::BodyTrait, component::MultibodyComponent, joint::{joint_sim::JointSimTrait, joint_state::JointStates, Joint, JointTrait
    }, result::MultibodyResult, system::MultibodySystem, system_sim::MultibodySystemSim, MultibodyTrait
};
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
use reedline::{DefaultPrompt, DefaultPromptSegment, FileBackedHistory, Reedline, Signal};
use ron::de::from_reader;
use ron::ser::{to_string_pretty, PrettyConfig};
use spice::Spice;
use std::collections::HashMap;
use std::fs::{self, File,OpenOptions};
use std::io::{self,Read, Write};
use std::path::{Path,PathBuf};
use std::process::{Command,Stdio};
use utilities::format_number;

mod prompts;
use prompts::*;

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
    Celestial,
    Floating,
    Gravity,    
    Prismatic,
    Revolute,
    Sensor,
}

fn main() {   

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
    let prompt_string = colored::Colorize::bright_blue("nadir").to_string();
    let prompt = DefaultPrompt::new(DefaultPromptSegment::Basic(prompt_string), DefaultPromptSegment::Empty);  
    //let prompt = PromptMain.prompt().unwrap();

    let mut active_system: Option<String> = None;
    let mut systems = HashMap::<String, MultibodySystem>::new();
    let mut results = HashMap::<String, MultibodyResult>::new();

    // load spice at the start
    let spice = match Spice::from_local() {
        Ok(spice) => spice,
        Err(_) => panic!("could not load spice. TODO: handle this more gracefully")
    };
    let spice = &mut Some(spice);
    
    rl.clear_screen().unwrap();        
    println!("{}",  "Welcome to NADIR!".bright_blue());

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
                                            Components::Base => {error("base is now permanent in system. don't need to add");continue},
                                            Components::Body => {
                                                let body = PromptBody.prompt();
                                                match body {
                                                    Ok(body) => {                                                        
                                                        let name = body.name.clone();
                                                        let r = system.add_body(body); 
                                                        match r {
                                                            Ok(_) => success(format!("{} added to {}!", &name, system_name).as_str()),
                                                            Err(e) => eprintln!("{:?}",e)
                                                        }                                                                                                        
                                                    }
                                                    Err(e) => {eprintln!("{:?}",e); continue}
                                                }
                                            }
                                            Components::Celestial => {                                                
                                                let celestial = PromptCelestial.prompt();
                                                match celestial {
                                                    Ok(celestial) => {                                                        
                                                        system.base.add_celestial_system(celestial);                                                                                                                           
                                                    }
                                                    Err(e) => eprintln!("{:?}",e)
                                                }                     
                                            }
                                            Components::Gravity => {
                                                match PromptGravity.prompt() {
                                                    Ok(gravity) => {                                                        
                                                        let r = system.add_gravity(gravity);
                                                        match r {
                                                            Ok(_) => success(format!("gravity added to {}!", system_name).as_str()),
                                                            Err(e) => eprintln!("{:?}",e)
                                                        }                                         
                                                    } 
                                                    Err(e) => {eprintln!("{:?}",e); continue}                                                    
                                                };
                                            }
                                            Components::Floating => {                                        
                                                let floating = match PromptJointFloating.prompt() {
                                                    Ok(floating) => floating,
                                                    Err(e) => {eprintln!("{:?}",e); continue}                                                    
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
                                                    let prismatic = match PromptJointPrismatic.prompt() {
                                                        Ok(prismatic) => prismatic,
                                                        Err(e) => {eprintln!("{:?}",e); continue}                                                    
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
                                                intro("New Revolute Joint").unwrap();
                                                let revolute = match PromptJointRevolute.prompt() {                                                    
                                                    Ok(revolute) => revolute,
                                                    Err(e) => {eprintln!("{:?}",e); continue}                                                    
                                                };                                                
                                                let joint = Joint::Revolute(revolute);
                                                let name = joint.get_name().to_string();
                                                let r = system.add_joint(joint);
                                                match r {
                                                    //Ok(_) => success(format!("{} added to {}!", &name, system_name).as_str()),
                                                    Ok(_) => outro(format!("{} added to {}!", &name, system_name).as_str()).unwrap(),
                                                    Err(e) => eprintln!("{:?}",e)
                                                }                                                   
                                            }  
                                            Components::Sensor => {
                                                let sensor = match PromptSensor.prompt() {
                                                    Ok(sensor) => sensor,
                                                    Err(e) => {eprintln!("{:?}",e); continue}                                                    
                                                };
                                                let name = sensor.get_name().to_string();
                                                let r = system.add_sensor(sensor);
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
                                                let transform = PromptTransform.prompt();
                                                match transform {
                                                    Ok(transform) => Some(transform),
                                                    Err(e) => {eprintln!("{:?}",e); continue}                                                    
                                                }
                                            },
                                            (
                                                MultibodyComponent::Joint,
                                                MultibodyComponent::Base | MultibodyComponent::Body,
                                            ) => {
                                                let transform =  PromptTransform.prompt();
                                                match transform {
                                                    Ok(transform) => Some(transform),
                                                    Err(e) => {eprintln!("{:?}",e); continue}                                                    
                                                }
                                            },                                            
                                            (
                                                MultibodyComponent::Sensor,
                                                MultibodyComponent::Body
                                            ) => {
                                                let transform =  PromptTransform.prompt();
                                                match transform {
                                                    Ok(transform) => Some(transform),
                                                    Err(e) => {eprintln!("{:?}",e); continue}                                                    
                                                }
                                            },
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
                                                match to_type {                                                        
                                                    MultibodyComponent::Joint => {
                                                        if sys.base.get_outer_joints().contains(&to_id) {
                                                            sys.base.outer_joints.retain(|id| *id != to_id);
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
                                            MultibodyComponent::Sensor => {                                                
                                                match to_type {                                                    
                                                    MultibodyComponent::Body => {
                                                        if let Some(body) = sys.bodies.get_mut(&to_id) {
                                                            if body.sensors.contains(&to_id) {
                                                                body.sensors.retain(|id| *id != to_id);
                                                                success("Components disconnected!");
                                                            } else {
                                                                error("Components not connected...");
                                                            }                                                            
                                                        }                                                                                                                
                                                    }
                                                    _ => error("Invalid component combo...")
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
                                                error("nothing to edit for base...");
                                                continue
                                            }
                                            MultibodyComponent::Body => {
                                                if let Some(old_body) = sys.bodies.get_mut(&id) {
                                                    // create a new object via prompt to get values for old object
                                                    let new_body = match PromptBody.prompt() {                                                    
                                                        Ok(body) => {
                                                            body
                                                        }
                                                        Err(e) => {eprintln!("{:?}",e); continue}
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
                                                            let new_joint = match PromptJointFloating.prompt() {
                                                                Ok(floating) => {
                                                                    floating
                                                                }
                                                                Err(e) => {eprintln!("{:?}",e); continue}        
                                                            };
                                                            old_joint.set_name(new_joint.get_name().to_string());
                                                            old_joint.parameters = new_joint.parameters;
                                                            old_joint.state = new_joint.state;
                                                        }
                                                        Joint::Prismatic(old_joint) => {
                                                            let new_joint = match PromptJointPrismatic.prompt() {
                                                                Ok(prismatic) => {
                                                                    prismatic
                                                                }
                                                                Err(e) => {eprintln!("{:?}",e); continue}       
                                                            };
                                                            old_joint.set_name(new_joint.get_name().to_string());
                                                            old_joint.parameters = new_joint.parameters;
                                                            old_joint.state = new_joint.state;
                                                        }
                                                        Joint::Revolute(old_joint) => {
                                                            let new_joint = match PromptJointRevolute.prompt() {
                                                                Ok(revolute) => {
                                                                    revolute
                                                                }
                                                                Err(e) => {eprintln!("{:?}",e); continue}       
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
                                            MultibodyComponent::Sensor => {
                                                if let Some(old_sensor) = sys.sensors.get_mut(&id) {
                                                    // create a new object via prompt to get values for old object
                                                    let new_sensor = match PromptSensor.prompt() {
                                                        Ok(s) => {
                                                            s
                                                        }
                                                        Err(e) => {eprintln!("{:?}",e); continue}
                                                    };

                                                    // Set values for the old object from the new one, 
                                                    // maintaining id's and connections, then dropping new object
                                                    old_sensor.set_name(new_sensor.get_name().to_string());
                                                    old_sensor.set_model(new_sensor.get_model().clone());
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
                                        let df = match df {
                                            Some(df) => df,
                                            None => {
                                                error("Error: component or state not found");
                                                continue
                                            }
                                        };
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

                                        let spice = match sys_sim.base.system {
                                            BaseSystems::Basic(_) => &mut None,
                                            BaseSystems::Celestial(_)=> spice
                                        };

                                        sys_sim.run(&mut final_joint_states,&initial_joint_states,0.0, spice);

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
                                                
                                                match file.read_to_string(&mut content) {
                                                    Ok(_) => {},
                                                    Err(e) => {eprintln!("{}", e); continue}        
                                                }
                                                match from_reader(content.as_bytes()) {
                                                    Ok(system) => system,
                                                    Err(e) => {eprintln!("{}", e); continue}        
                                                }
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
                                                    let res = match res {
                                                        Some(res) => res,
                                                        None => {
                                                            error("Error: component or state not found");
                                                            continue
                                                        }
                                                    };
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
                                                match PromptSimulation.prompt() {
                                                    Ok(sim) => {       

                                                        let spice = match system.base.system {
                                                            BaseSystems::Basic(_) => &mut None,
                                                            BaseSystems::Celestial(_)=> spice
                                                        };                                                 
                                                        let result = system.simulate(sim.name.clone(),sim.start,sim.stop,sim.dt,spice);
                                                        match result {
                                                            Ok(result) => {
                                                                let s = format!("Simulation '{}' completed in {:#?}!", sim.name, result.total_duration);
                                                                results.insert(result.name.clone(),result);                                                                
                                                                success(&s);
                                                            },
                                                            Err(e) => eprintln!("{:#?}",e)
                                                        }
                                                    }
                                                    Err(e) => {eprintln!("{:?}",e); continue}
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
                    Signal::CtrlD => {
                        break
                    }
                    Signal::CtrlC => {
                        continue
                    }
                }
                
            }
            Err(e) => eprintln!("{:#?}", e)            
        }
    }    
}

fn success(s: &str) {
    println!("{}", colored::Colorize::green(s))
}

fn error(s: &str) {
    eprintln!("{}", colored::Colorize::red(s))
}