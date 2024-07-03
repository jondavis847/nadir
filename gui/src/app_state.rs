use iced::{mouse::Cursor, widget::canvas::Cache, Command, Size};
use std::time::{Duration, Instant};

use crate::multibody_ui::{BodyField, RevoluteField};
use crate::ui::{
    errors::Errors,
    mouse::MouseButtonReleaseEvents,
};
use crate::{
    ui::{
        canvas::{
            graph::{Graph, GraphMessage},
            nodebar::{Nodebar, NodebarMessage},
        },
        dummies::DummyComponent,
        modals::ActiveModal,
    },
    Message,
};
use multibody::{joint::Joint, MultibodyTrait};

#[derive(Debug)]
pub struct AppState {
    pub active_error: Option<Errors>,
    pub cache: Cache,
    pub counter_body: usize,
    pub counter_revolute: usize,
    pub graph: Graph,
    pub left_clicked_time_1: Option<Instant>,
    pub left_clicked_time_2: Option<Instant>,
    pub modal: Option<ActiveModal>,
    pub nodebar: Nodebar,
    pub theme: crate::ui::theme::Theme,
}

impl Default for AppState {
    fn default() -> Self {
        Self {
            active_error: None,
            cache: Cache::new(),
            counter_body: 0,
            counter_revolute: 0,
            left_clicked_time_1: None,
            left_clicked_time_2: None,
            graph: Graph::default(),
            modal: None,
            nodebar: Nodebar::default(),
            theme: crate::ui::theme::Theme::ORANGE,
        }
    }
}

impl AppState {
    pub fn close_error(&mut self) -> Command<Message> {
        self.active_error = None;
        Command::none()
    }

    pub fn close_modal(&mut self) -> Command<Message> {
        self.modal = None;
        Command::none()
    }

    pub fn cursor_moved(&mut self, cursor: Cursor) -> Command<Message> {
        let nodebar_redraw = self.nodebar.cursor_moved(cursor);
        let graph_redraw = self.graph.cursor_moved(cursor);

        // don't need to redraw just because mouse is moving
        if nodebar_redraw || graph_redraw {
            self.cache.clear();
        }
        Command::none()
    }

    pub fn delete_pressed(&mut self) -> Command<Message> {
        self.graph.delete_pressed();
        //self.nodebar.delete_pressed(); // no need for this, maybe ever?
        self.cache.clear();
        Command::none()
    }

    pub fn enter_pressed(&mut self) -> Command<Message> {
        // if the error modal is currently open, close it
        if self.active_error.is_some() {
            self.active_error = None;
        }
        //if a component modal is currently open, save it
        self.save_component()
    }

    pub fn left_button_pressed(&mut self, cursor: Cursor) -> Command<Message> {
        self.left_clicked_time_1 = self.left_clicked_time_2;
        self.left_clicked_time_2 = Some(Instant::now());

        self.nodebar.left_button_pressed(cursor);
        self.graph.left_button_pressed(cursor);
        self.cache.clear();
        Command::none()
    }

    pub fn left_button_released(&mut self, cursor: Cursor) -> Command<Message> {
        // Determine the type of mouse button release event
        let release_event = match (self.left_clicked_time_1, self.left_clicked_time_2) {
            (Some(clicked_time_1), Some(clicked_time_2)) => {
                let clicked_elapsed_time_1 = clicked_time_1.elapsed();
                let clicked_elapsed_time_2 = clicked_time_2.elapsed();

                if clicked_elapsed_time_1 <= Duration::from_millis(500) {
                    MouseButtonReleaseEvents::DoubleClick
                } else if clicked_elapsed_time_2 <= Duration::from_millis(300) {
                    MouseButtonReleaseEvents::SingleClick
                } else {
                    MouseButtonReleaseEvents::Held
                }
            }
            (None, Some(clicked_time_2)) => {
                if clicked_time_2.elapsed() <= Duration::from_millis(200) {
                    MouseButtonReleaseEvents::SingleClick
                } else {
                    MouseButtonReleaseEvents::Held
                }
            }
            _ => MouseButtonReleaseEvents::Nothing,
        };
        if let Some(NodebarMessage::NewComponent(active_modal)) =
            self.nodebar.left_button_released(&release_event)
        {
            // Only create a new component if the mouse is over the graph
            if cursor.is_over(self.graph.bounds) {
                match active_modal.dummy_type {
                    DummyComponent::Base => {
                        if self.graph.system.base.is_some() {
                            self.active_error = Some(Errors::TooManyBases);
                        } else {
                            self.modal = Some(active_modal);
                        }
                    }
                    DummyComponent::Body => {
                        self.modal = Some(active_modal);
                    }
                    DummyComponent::Revolute => {
                        self.modal = Some(active_modal);
                    }
                }
            }
        }
        if let Some(GraphMessage::EditComponent((component_type, component_id))) =
            self.graph.left_button_released(&release_event, cursor)
        {
            let dummy_type;
            match component_type {
                DummyComponent::Base => {
                    let base = self.graph.system.base.as_ref().unwrap();
                    self.nodebar.dummies.base.get_values_from(base);
                    dummy_type = DummyComponent::Base;
                }
                DummyComponent::Body => {
                    let body = self.graph.system.bodies.get(&component_id).unwrap();
                    self.nodebar.dummies.body.get_values_from(body);
                    dummy_type = DummyComponent::Body;
                }
                DummyComponent::Revolute => {
                    let joint = self.graph.system.joints.get(&component_id).unwrap();
                    match joint {
                        Joint::Revolute(revolute) => {
                            self.nodebar.dummies.revolute.get_values_from(revolute);
                            dummy_type = DummyComponent::Revolute;
                        }
                    }
                }
            }
            self.modal = Some(ActiveModal::new(dummy_type, Some(component_id)));
        }

        self.cache.clear();
        Command::none()
    }

    pub fn middle_button_pressed(&mut self, _cursor: Cursor) -> Command<Message> {
        //match self.graph.create_multibody_system() {
        //    Ok(system) => dbg!(system),
        //    Err(error) => {
        // TODO: handle error
        //        return Command::none();
        //    }
        // };
        Command::none()
    }

    pub fn right_button_pressed(&mut self, cursor: Cursor) -> Command<Message> {
        self.nodebar.right_button_pressed(cursor);
        self.graph.right_button_pressed(cursor);
        Command::none()
    }

    pub fn right_button_released(&mut self, cursor: Cursor) -> Command<Message> {
        self.graph.right_button_released(cursor);
        self.nodebar.right_button_released(cursor);
        self.cache.clear();
        Command::none()
    }

    pub fn save_component(&mut self) -> Command<Message> {
        // early return
        let modal = match self.modal {
            Some(ref modal) => modal,
            None => return Command::none(),
        };

        match modal.dummy_type {
            DummyComponent::Base => {
                if self.nodebar.dummies.base.name.is_empty() {
                    self.nodebar.dummies.base.name = "base".to_string();
                }
                match modal.component_id {
                    Some(_) => {
                        let base = self.graph.system.base.as_mut().unwrap();
                        //editing existing component
                        self.nodebar.dummies.base.set_values_for(base);
                    }
                    None => {
                        // saving a new base - hopefully error was caught somewhere that cant have too many bases
                        let base = self.nodebar.dummies.base.to_base();
                        let id = *base.get_id();
                        let label = base.get_name().to_string();
                        self.graph.system.add_base(base).unwrap();
                        self.graph
                            .save_component(&modal.dummy_type, id, label)
                            .unwrap();
                    }
                }
                self.nodebar.dummies.base.clear();
            }
            DummyComponent::Body => {
                if self.nodebar.dummies.body.name.is_empty() {
                    self.counter_body += 1;
                    self.nodebar.dummies.body.name =
                        format!("body{}", self.counter_body).to_string();
                }
                match modal.component_id {
                    Some(id) => {
                        //editing existing body
                        let body = self.graph.system.bodies.get_mut(&id).unwrap();
                        self.nodebar.dummies.body.set_values_for(body);
                    }
                    None => {
                        //creating new body
                        let body = self.nodebar.dummies.body.to_body();
                        let id = *body.get_id();
                        let label = body.get_name().to_string();
                        self.graph.system.add_body(body).unwrap();
                        self.graph
                            .save_component(&modal.dummy_type, id, label)
                            .unwrap();
                    }
                }
                self.nodebar.dummies.body.clear();
            }
            DummyComponent::Revolute => {
                if self.nodebar.dummies.revolute.name.is_empty() {
                    self.counter_revolute += 1;
                    self.nodebar.dummies.revolute.name =
                        format!("revolute{}", self.counter_revolute).to_string();
                }
                match modal.component_id {
                    Some(id) => {
                        //editing existing joint
                        let joint = self.graph.system.joints.get_mut(&id).unwrap();
                        match joint {
                            Joint::Revolute(revolute) => {
                                self.nodebar.dummies.revolute.set_values_for(revolute)
                            }
                        }
                    }
                    None => {
                        //create new joint
                        let joint = self.nodebar.dummies.revolute.to_joint();
                        let id = *joint.get_id();
                        let label = joint.get_name().to_string();
                        self.graph.system.add_joint(joint).unwrap();
                        self.graph
                            .save_component(&modal.dummy_type, id, label)
                            .unwrap();
                    }
                }

                self.nodebar.dummies.revolute.clear();
            }
        }

        //TODO: actually do something with the error/message
        //match graph_message {

        //}

        // Clear the modal and cache
        self.modal = None;
        self.cache.clear();
        Command::none()
    }

    pub fn tab_pressed(&mut self) -> Command<Message> {
        if self.modal.is_some() {
            Command::none()
            //iced::widget::focus_next() // not working right now
        } else {
            Command::none()
        }
    }

    pub fn update_body_field(&mut self, field: BodyField, value: &str) -> Command<Message> {
        let dummy_body = &mut self.nodebar.dummies.body;

        match field {
            BodyField::Name => dummy_body.name = value.to_string(),
            BodyField::Mass => dummy_body.mass = value.to_string(),
            BodyField::Cmx => dummy_body.cmx = value.to_string(),
            BodyField::Cmy => dummy_body.cmy = value.to_string(),
            BodyField::Cmz => dummy_body.cmz = value.to_string(),
            BodyField::Ixx => dummy_body.ixx = value.to_string(),
            BodyField::Iyy => dummy_body.iyy = value.to_string(),
            BodyField::Izz => dummy_body.izz = value.to_string(),
            BodyField::Ixy => dummy_body.ixy = value.to_string(),
            BodyField::Ixz => dummy_body.ixz = value.to_string(),
            BodyField::Iyz => dummy_body.iyz = value.to_string(),
        }

        Command::none()
    }

    pub fn update_revolute_field(&mut self, field: RevoluteField, value: &str) -> Command<Message> {
        let dummy_revolute = &mut self.nodebar.dummies.revolute;
        match field {
            RevoluteField::Name => dummy_revolute.name = value.to_string(),
            RevoluteField::ConstantForce => dummy_revolute.constant_force = value.to_string(),
            RevoluteField::damping => dummy_revolute.damping = value.to_string(),
            RevoluteField::Omega => dummy_revolute.omega = value.to_string(),
            RevoluteField::SpringConstant => dummy_revolute.spring_constant = value.to_string(),
            RevoluteField::Theta => dummy_revolute.theta = value.to_string(),
        }

        Command::none()
    }

    pub fn window_resized(&mut self, window_size: Size) -> Command<Message> {
        let graph_size = Size::new(
            window_size.width - self.nodebar.bounds.width,
            window_size.height,
        );
        self.graph.window_resized(graph_size);
        let nodebar_size = Size::new(self.nodebar.bounds.width, window_size.height);
        self.nodebar.window_resized(nodebar_size);
        self.cache.clear();
        Command::none()
    }
}
