use uuid::Uuid;

use crate::multibody_ui::MultibodyComponent;
#[derive(Debug, Clone, Copy)]
pub struct ActiveModel(MultibodyComponent);

//pub struct ActiveModal {
    //pub dummy_component_id: Uuid,
    //pub graph_component_id: Option<Uuid>,    
//}

//impl ActiveModal {
  //  pub fn new(dummy_component_id: Uuid, graph_component_id: Option<Uuid>) -> Self {
    //    Self {dummy_component_id,graph_component_id}
    //}
//}