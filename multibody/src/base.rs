use aerospace::gravity::Gravity;
use super::{
    body::{BodyErrors, BodyTrait},
    joint::JointTrait,
    MultibodyTrait,
};
use uuid::Uuid;

#[derive(Clone, Debug)]
pub struct Base {
    id: Uuid,
    name: String,
    outer_joints: Vec<Uuid>,    
    pub gravity: Vec<Uuid>,
}

impl Base {
    pub fn new(name: &str) -> Self {
        let mut name = name;
        if name.is_empty() {
            name = "base";
        }
        Self {
            id: Uuid::new_v4(),
            name: name.to_string(),
            outer_joints: Vec::new(),            
            gravity: Vec::new(),
        }
    }
}

impl BodyTrait for Base {
    fn connect_outer_joint<T:JointTrait>(&mut self, joint: &T) -> Result<(), BodyErrors> {
        let joint_id = joint.get_id();
        // Check if the joint already exists in outer_joints
        if self.outer_joints.iter().any(|id| id == joint_id) {
            return Err(BodyErrors::OuterJointExists);
        }

        // Push the new joint connection
        self.outer_joints.push(*joint_id);
        Ok(())
    }

    fn delete_outer_joint(&mut self, joint_id: &Uuid) {
        self.outer_joints.retain(|id| id != joint_id);
    }

    fn get_outer_joints(&self) -> &Vec<Uuid> {
        &self.outer_joints
    }
}
impl MultibodyTrait for Base {
    fn get_id(&self) -> &Uuid {
        &self.id
    }

    fn get_name(&self) -> &str {
        &self.name
    }

    fn set_name(&mut self, name: String) {
        self.name = name;
    }
}