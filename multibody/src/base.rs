use super::{MultibodyMeta, MultibodyTrait};

#[derive(Debug, Clone)]
pub struct Base {
    pub meta: MultibodyMeta,
}

impl Base {
    pub fn new(meta: MultibodyMeta) -> Self {
        Self { meta }
    }
}

pub enum BaseErrors {}

impl MultibodyTrait for Base {
    fn connect_inner(&mut self, _id: usize) {
        //do nothing, nothing before base
    }

    fn connect_outer(&mut self, id: usize) {
        self.meta.id.outer.push(id);
    }
    fn delete_inner(&mut self) {
        //shouldn't matter, nothing before Base
    }
    fn delete_outer(&mut self, id: usize) {
        self.meta.id.outer.retain(|&outer_id| outer_id != id);
    }

    fn get_id(&self) -> Option<usize> {
        self.meta.id.component
    }

    fn get_inner_id(&self) -> Option<usize> {
        self.meta.id.inner
    }

    fn get_outer_id(&self) -> &Vec<usize> {
        &self.meta.id.outer
    }

    fn get_name(&self) -> &str {
        &self.meta.name
    }

    fn set_id(&mut self, id: usize) {
        self.meta.id.component = Some(id);
    }

    fn set_name(&mut self, name: String) {
        self.meta.name = name;
    }
}
