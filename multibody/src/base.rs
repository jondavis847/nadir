use super::{MultibodyMeta, MultibodyTrait};
use crate::ui::dummies::{DummyBase, DummyComponent, DummyErrors, DummyTrait};
use uuid::Uuid;

#[derive(Debug, Clone, Copy)]
pub enum BaseErrors {
    DummyErrors(DummyErrors),
}

#[derive(Debug, Clone)]
pub struct Base {
    pub meta: MultibodyMeta,
}

impl Base {
    pub fn from_dummy(
        component_id: Uuid,        
        dummy: &DummyBase,
        node_id: Uuid,
    ) -> Result<Self, BaseErrors> {
        if dummy.get_name().is_empty() {
            return Err(BaseErrors::DummyErrors(DummyErrors::NameIsEmpty));
        }

        let meta = MultibodyMeta::new(component_id, dummy.get_id(), dummy.get_name(), node_id);

        Ok(Self { meta })
    }
}

impl MultibodyTrait for Base {
    fn connect_from(&mut self, _id: Uuid) {
        //do nothing, nothing before base
    }

    fn connect_to(&mut self, id: Uuid) {
        self.meta.to_id.push(id);
    }
    fn delete_from(&mut self) {
        self.meta.from_id = None;
    }
    fn delete_to(&mut self, id: Uuid) {
        self.meta.to_id.retain(|&to_id| to_id != id);
    }

    fn get_component_id(&self) -> Uuid {
        self.meta.component_id
    }

    fn get_dummy_id(&self) -> Uuid {
        self.meta.dummy_id
    }

    fn get_from_id(&self) -> Option<Uuid> {
        self.meta.from_id
    }

    fn get_name(&self) -> &str {
        &self.meta.name
    }

    fn get_node_id(&self) -> Uuid {
        self.meta.node_id
    }

    fn get_to_id(&self) -> &Vec<Uuid> {
        &self.meta.to_id
    }

    fn inherit_from(&mut self, dummy: &DummyComponent) {
        match dummy {
            DummyComponent::Base(_) => {}
            _ => {} // error! must be dummy base
        }
    }

    fn set_component_id(&mut self, id: Uuid) {
        self.meta.component_id = id;
    }

    fn set_name(&mut self, name: String) {
        self.meta.name = name;
    }

    fn set_node_id(&mut self, id: Uuid) {
        self.meta.node_id = id;
    }

    fn set_system_id(&mut self, id: usize) {
        self.meta.system_id = Some(id);
    }
}
