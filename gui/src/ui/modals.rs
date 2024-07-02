use uuid::Uuid;

use crate::ui::dummies::DummyComponent;
#[derive(Debug, Clone, Copy)]
pub struct ActiveModal {
    pub dummy_type: DummyComponent,
    pub component_id: Option<Uuid>, // None if new component, id if editing component
}

impl ActiveModal {
    pub fn new(dummy_type: DummyComponent, component_id: Option<Uuid>) -> Self {
        Self {
            dummy_type,
            component_id,
        }
    }
}
