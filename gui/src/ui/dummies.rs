use crate::multibody_ui::MultibodyComponent;
use crate::ui::canvas::graph::Graph;
use multibody::{joint::Joint, MultibodyTrait};
use uuid::Uuid;

#[derive(Debug, Clone, Copy)]
pub enum DummyErrors {
    NameIsEmpty,
}

/// DummyComponents are like MultibodyComponents but with String fields
/// for editing in the text inputs rather than numeric values
#[derive(Debug, Clone)]
pub enum DummyComponent {
    Base(DummyBase),
    Body(DummyBody),
    Revolute(DummyRevolute),
}

pub trait DummyTrait {
    fn clear(&mut self);
    fn get_id(&self) -> Uuid;
    fn get_name(&self) -> String;
    fn inherit_from(&mut self, component_id: &Uuid, graph: &Graph); // these args suck, but has to be this way sicne names is stored separately in graph for performance
    fn set_name(&mut self, name: &str);
}

impl DummyTrait for DummyComponent {
    fn clear(&mut self) {
        match self {
            DummyComponent::Base(component) => component.clear(),
            DummyComponent::Body(component) => component.clear(),
            DummyComponent::Revolute(component) => component.clear(),
        }
    }

    fn get_id(&self) -> Uuid {
        match self {
            DummyComponent::Base(component) => component.get_id(),
            DummyComponent::Body(component) => component.get_id(),
            DummyComponent::Revolute(component) => component.get_id(),
        }
    }

    fn get_name(&self) -> String {
        match self {
            DummyComponent::Base(component) => component.get_name(),
            DummyComponent::Body(component) => component.get_name(),
            DummyComponent::Revolute(component) => component.get_name(),
        }
    }

    fn inherit_from(&mut self, component_id: &Uuid, graph: &Graph) {
        match self {
            DummyComponent::Base(dummy) => dummy.inherit_from(component_id, graph),
            DummyComponent::Body(dummy) => dummy.inherit_from(component_id, graph),
            DummyComponent::Revolute(dummy) => dummy.inherit_from(component_id, graph),
        }
    }

    fn set_name(&mut self, name: &str) {
        match self {
            DummyComponent::Base(component) => component.set_name(name),
            DummyComponent::Body(component) => component.set_name(name),
            DummyComponent::Revolute(component) => component.set_name(name),
        }
    }
}

#[derive(Debug, Default, Clone)]
pub struct DummyBase {
    id: Uuid,
    name: String,
}

impl DummyBase {
    pub fn new(id: Uuid) -> Self {
        Self {
            id: id,
            ..Default::default()
        }
    }
}

impl DummyTrait for DummyBase {
    fn clear(&mut self) {
        self.name = String::new();
    }

    fn get_id(&self) -> Uuid {
        self.id
    }

    fn get_name(&self) -> String {
        self.name.to_string()
    }

    fn inherit_from(&mut self, component_id: &Uuid, graph: &Graph) {
        if let Some(component) = graph.components.get(component_id) {
            match component {
                MultibodyComponent::Base(_) => {
                    self.set_name(&component.get_name());
                }
                _ => {} // TODO: error! must be a base
            }
        }
    }

    fn set_name(&mut self, name: &str) {
        self.name = name.to_string();
    }
}

#[derive(Default, Debug, Clone)]
pub struct DummyBody {
    id: Uuid,
    name: String,
    pub mass: String,
    pub cmx: String,
    pub cmy: String,
    pub cmz: String,
    pub ixx: String,
    pub iyy: String,
    pub izz: String,
    pub ixy: String,
    pub ixz: String,
    pub iyz: String,
}

impl DummyBody {
    pub fn new(id: Uuid) -> Self {
        Self {
            id: id,
            ..Default::default()
        }
    }
}

impl DummyTrait for DummyBody {
    fn clear(&mut self) {
        self.name = String::new();
        self.mass = String::new();
        self.cmx = String::new();
        self.cmy = String::new();
        self.cmz = String::new();
        self.ixx = String::new();
        self.iyy = String::new();
        self.izz = String::new();
        self.ixy = String::new();
        self.ixz = String::new();
        self.iyz = String::new();
    }

    fn get_id(&self) -> Uuid {
        self.id
    }

    fn get_name(&self) -> String {
        self.name.to_string()
    }

    fn inherit_from(&mut self, component_id: &Uuid, graph: &Graph) {
        if let Some(component) = graph.components.get(component_id) {
            match component {
                MultibodyComponent::Body(body) => {                       
                    self.set_name(&body.get_name());
                    let mp = body.get_mass_properties();
                    let com = mp.center_of_mass;
                    let inertia = mp.inertia;
                    self.mass = mp.mass.to_string();
                    self.cmx = com.get_cmx().to_string();
                    self.cmy = com.get_cmy().to_string();
                    self.cmz = com.get_cmz().to_string();
                    self.ixx = inertia.get_ixx().to_string();
                    self.iyy = inertia.get_iyy().to_string();
                    self.izz = inertia.get_izz().to_string();
                    self.ixy = inertia.get_ixy().to_string();
                    self.ixz = inertia.get_ixz().to_string();
                    self.iyz = inertia.get_iyz().to_string();
                }
                _ => {} // TODO: error! must be a base
            }
        }
    }

    fn set_name(&mut self, name: &str) {
        self.name = name.to_string();
    }
}

#[derive(Default, Debug, Clone)]
pub struct DummyRevolute {
    pub constant_force: String,
    pub dampening: String,
    id: Uuid,
    pub name: String,
    pub omega: String,
    pub spring_constant: String,
    pub theta: String,
}

impl DummyRevolute {
    pub fn new(id: Uuid) -> Self {
        Self {
            id: id,
            ..Default::default()
        }
    }
}

impl DummyTrait for DummyRevolute {
    fn clear(&mut self) {
        self.name = String::new();
    }

    fn get_id(&self) -> Uuid {
        self.id
    }

    fn get_name(&self) -> String {
        self.name.to_string()
    }

    fn inherit_from(&mut self, component_id: &Uuid, graph: &Graph) {
        if let Some(component) = graph.components.get(component_id) {
            match component {
                MultibodyComponent::Joint(joint) => match joint {
                    Joint::Revolute(revolute) => {
                        self.set_name(&revolute.get_name());
                        self.theta = revolute.state.theta.to_string();
                        self.omega = revolute.state.omega.to_string();
                        self.spring_constant = revolute.parameters.spring_constant.to_string();
                        self.dampening = revolute.parameters.dampening.to_string();
                        self.constant_force = revolute.parameters.constant_force.to_string();
                    } //_ => {} //TODO: error! must be a revolute
                },
                _ => {} // TODO: error! must be a joint
            }
        }
    }

    fn set_name(&mut self, name: &str) {
        self.name = name.to_string();
    }
}
