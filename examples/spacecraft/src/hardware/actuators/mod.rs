use std::{fs::File, io::BufWriter, path::PathBuf};

use csv::Writer;
use multibody::{actuator::{Actuator, ActuatorModel, ActuatorSystem}, MultibodyResult};
use reaction_wheel::ReactionWheel;
use serde::{Deserialize, Serialize};

mod reaction_wheel;

#[derive(Debug,Clone,Deserialize,Serialize)]
pub struct SpacecraftActuators {
    pub rw: [Actuator<ReactionWheel>; 4],
}

impl ActuatorSystem for SpacecraftActuators {
    fn update(&mut self) {
        for rw in self.rw.iter_mut() {
            rw.model.process_command(command, &rw.connection.unwrap());
        }
    }

    fn initialize_writers(&self, path: &PathBuf) -> Vec<Writer<BufWriter<File>>>
    {
        let mut writers = Vec::new();
        for rw in self.rw.iter() {
            writers.push(rw.initialize_writer(path));
        }
        writers
    }

    fn write_result_files(&self, writers: &mut Vec<Writer<BufWriter<File>>>){        
        for i in 0..4 {
            self.rw[i].model.write_result_file(&mut writers[i]);
        }        
    }
}