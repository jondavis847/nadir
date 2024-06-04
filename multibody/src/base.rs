use uuid::Uuid;
use super::{MultibodyMeta, MultibodyTrait};

#[derive(Debug, Clone)]
pub struct Base {
    pub meta: MultibodyMeta,
}

impl Base {
    pub fn new(name: String) -> Self {
        let meta = MultibodyMeta::new(name);
        Self { meta }
    }
}

pub enum BaseErrors {}

impl MultibodyTrait for Base {
    fn connect_inner(&mut self, _id: Uuid) {
        //do nothing, nothing before base
    }

    fn connect_outer(&mut self, id: Uuid) {
        self.meta.id_outer.push(id);
    }
    fn delete_inner(&mut self) {
        //shouldn't matter, nothing before Base
    }
    fn delete_outer(&mut self, id: Uuid) {
        self.meta.id_outer.retain(|&outer_id| outer_id != id);
    }

    fn get_id(&self) -> Uuid {
        self.meta.id
    }

    fn get_inner_id(&self) -> Option<Uuid> {
        self.meta.id_inner
    }

    fn get_outer_id(&self) -> &Vec<Uuid> {
        &self.meta.id_outer
    }

    fn get_name(&self) -> &str {
        &self.meta.name
    }

    fn set_name(&mut self, name: String) {
        self.meta.name = name;
    }
}
