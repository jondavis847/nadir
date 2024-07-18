use iced::{mouse::ScrollDelta, widget::canvas::Cache, Command, Point, Size};
use std::collections::HashMap;
use std::time::{Duration, Instant};
use utilities::{generate_unique_id, unique_strings};

use crate::multibody_ui::{BodyField, PrismaticField, RevoluteField};
use crate::ui::{
    animation_tab::AnimationTab,
    errors::Errors,
    mouse::MouseButtonReleaseEvents,
    plot_tab::PlotTab,
    tab_bar::{AppTabs, TabBar},
};
use crate::{
    ui::{
        dummies::DummyComponent,
        modals::ActiveModal,
        sim_tab::{
            canvas::{
                graph::{Graph, GraphMessage},
                nodebar::{Nodebar, NodebarMessage},
            },
            sim_div::SimDiv,
        },
    },
    Message,
};
use multibody::{joint::Joint, result::MultibodyResult, MultibodyTrait};

#[derive(Debug)]
pub struct AppState {
    pub active_error: Option<Errors>,
    pub animation_tab: AnimationTab,
    pub cache: Cache,
    pub counter_body: usize,
    pub counter_revolute: usize,
    pub counter_prismatic: usize,
    pub tab_bar: TabBar,
    pub graph: Graph,
    pub left_clicked_time_1: Option<Instant>,
    pub left_clicked_time_2: Option<Instant>,
    pub plot_tab: PlotTab,
    pub modal: Option<ActiveModal>,
    pub nodebar: Nodebar,
    pub results: HashMap<String, MultibodyResult>,
    pub simdiv: SimDiv,
    pub theme: crate::ui::theme::Theme,
}

impl Default for AppState {
    fn default() -> Self {
        Self {
            active_error: None,
            cache: Cache::new(),
            counter_body: 0,
            counter_revolute: 0,
            counter_prismatic: 0,
            tab_bar: TabBar::default(),
            left_clicked_time_1: None,
            left_clicked_time_2: None,
            graph: Graph::default(),
            animation_tab: AnimationTab::default(),
            plot_tab: PlotTab::default(),
            modal: None,
            nodebar: Nodebar::default(),
            results: HashMap::new(),
            simdiv: SimDiv::default(),
            theme: crate::ui::theme::Theme::ORANGE,
        }
    }
}

impl AppState {
    pub fn animation(&mut self, instant: iced::time::Instant) -> Command<Message> {
        if self.graph.animation(instant) {
            self.cache.clear()
        };
        Command::none()
    }

    pub fn close_error(&mut self) -> Command<Message> {
        self.active_error = None;
        Command::none()
    }

    pub fn close_modal(&mut self) -> Command<Message> {
        self.modal = None;
        Command::none()
    }

    pub fn cursor_moved(&mut self, canvas_cursor_position: Point) -> Command<Message> {
        match self.tab_bar.state.current_tab {
            AppTabs::Simulation => {
                let nodebar_redraw = self.nodebar.cursor_moved(canvas_cursor_position);
                let graph_redraw = self.graph.cursor_moved(canvas_cursor_position);

                // don't need to redraw just because mouse is moving
                if nodebar_redraw || graph_redraw {
                    self.cache.clear();
                }
            }
            _ => {}
        }

        Command::none()
    }

    pub fn delete_pressed(&mut self) -> Command<Message> {
        match self.tab_bar.state.current_tab {
            AppTabs::Simulation => {
                if self.modal.is_none() && self.active_error.is_none() {
                    //NOTE: . on num pad might also be delete, which will delete nodes when typing floats on modals
                    self.graph.delete_pressed();
                    self.cache.clear();
                }
            }
            _ => {}
        }

        //self.nodebar.delete_pressed(); // no need for this, maybe ever?

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

    pub fn left_button_pressed(&mut self, canvas_cursor_position: Point) -> Command<Message> {
        self.left_clicked_time_1 = self.left_clicked_time_2;
        self.left_clicked_time_2 = Some(Instant::now());

        match self.tab_bar.state.current_tab {
            AppTabs::Animation => self
                .animation_tab
                .left_button_pressed(canvas_cursor_position),
            AppTabs::Plot => self.plot_tab.left_button_pressed(canvas_cursor_position),
            AppTabs::Simulation => {
                //TODO: self.simulation_tab.left_button_pressed(canvas_cursor_position),
                self.nodebar.left_button_pressed(canvas_cursor_position);
                self.graph.left_button_pressed(canvas_cursor_position);
                self.cache.clear();
            }
        }

        Command::none()
    }

    pub fn left_button_released(&mut self, canvas_cursor_position: Point) -> Command<Message> {
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
            if self.graph.bounds.contains(canvas_cursor_position) {
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
                    DummyComponent::Prismatic => {
                        self.modal = Some(active_modal);
                    }
                }
            }
        }
        if let Some(GraphMessage::EditComponent((component_type, component_id))) = self
            .graph
            .left_button_released(&release_event, canvas_cursor_position)
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
                        _ => panic!("should not be possible to another joint"),
                    }
                }
                DummyComponent::Prismatic => {
                    let joint = self.graph.system.joints.get(&component_id).unwrap();
                    match joint {
                        Joint::Prismatic(prismatic) => {
                            self.nodebar.dummies.prismatic.get_values_from(prismatic);
                            dummy_type = DummyComponent::Prismatic;
                        }
                        _ => panic!("should not be possible to another joint"),
                    }
                }
            }
            self.modal = Some(ActiveModal::new(dummy_type, Some(component_id)));
        }

        self.cache.clear();
        Command::none()
    }

    pub fn middle_button_pressed(&mut self, _canvas_cursor_position: Point) -> Command<Message> {
        //match self.graph.create_multibody_system() {
        //    Ok(system) => dbg!(system),
        //    Err(error) => {
        // TODO: handle error
        //        return Command::none();
        //    }
        // };
        Command::none()
    }

    fn plot(&mut self) {
        let mut plot_data: Vec<(String, Vec<Point>)> = Vec::new();

        for sim_name in &self.plot_tab.selected_sims {
            let sim = self.results.get(sim_name).unwrap();

            for component_name in &self.plot_tab.selected_components {
                let component = sim.get_component(component_name);
                let t = component.column("t").unwrap().f64().unwrap();
                let column_names = component.get_column_names();
                for state_name in &self.plot_tab.selected_states {
                    if column_names.contains(&state_name.as_str()) {
                        let data = component.column(state_name).unwrap().f64().unwrap();
                        assert_eq!(t.len(), data.len());
                        // Create a Vec<iced::Point> by iterating over the Series
                        let points: Vec<Point> = t
                            .into_iter()
                            .zip(data.into_iter())
                            .map(|(x, y)| Point {
                                x: x.unwrap_or_default() as f32,
                                y: y.unwrap_or_default() as f32,
                            })
                            .collect();

                        let line_label = format!("{}:{}:{}", sim_name, component_name, state_name);
                        plot_data.push((line_label, points));
                    }
                }
            }
        }

        // Perform the plotting
        for (line_label, points) in plot_data {
            self.plot_tab.plot(line_label, points);
        }
    }

    pub fn plot_sim_selected(&mut self, sim_name: String) -> Command<Message> {
        self.plot_tab.sim_menu.option_selected(&sim_name);
        self.plot_tab.selected_sims = self.plot_tab.sim_menu.get_selected_options();

        let mut components = Vec::new();
        for sim in &self.plot_tab.selected_sims {
            let result = self.results.get(sim).unwrap();
            components = unique_strings(components, result.get_components());
        }

        self.plot_tab.component_menu.update_options(components);
        self.plot_tab.selected_components.clear();
        self.plot_tab.state_menu.update_options(Vec::new());
        self.plot_tab.selected_states.clear();

        Command::none()
    }

    pub fn plot_component_selected(&mut self, component_name: String) -> Command<Message> {
        self.plot_tab
            .component_menu
            .option_selected(&component_name);
        self.plot_tab.selected_components = self.plot_tab.component_menu.get_selected_options();

        // get the unique component states by looking all selected components
        let selected_sims = self.plot_tab.sim_menu.get_selected_options();
        let selected_components = self.plot_tab.component_menu.get_selected_options();
        let mut states = Vec::new();
        for sim in &selected_sims {
            let result = self.results.get(sim).unwrap();
            for component in &selected_components {
                let component_states = result.get_component_states(component);
                states = unique_strings(states, component_states);
            }
        }
        self.plot_tab.state_menu.update_options(states);
        self.plot_tab.selected_states.clear();
        Command::none()
    }

    pub fn plot_state_selected(&mut self, state_name: String) -> Command<Message> {
        dbg!(&state_name);
        self.plot_tab.state_menu.option_selected(&state_name);
        self.plot_tab.selected_states = self.plot_tab.state_menu.get_selected_options();

        self.plot();
        Command::none()
    }

    pub fn right_button_pressed(&mut self, canvas_cursor_position: Point) -> Command<Message> {
        self.nodebar.right_button_pressed(canvas_cursor_position);
        self.graph.right_button_pressed(canvas_cursor_position);
        Command::none()
    }

    pub fn right_button_released(&mut self, canvas_cursor_position: Point) -> Command<Message> {
        self.graph.right_button_released(canvas_cursor_position);
        self.nodebar.right_button_released(canvas_cursor_position);
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
                    Some(id) => {
                        let base = self.graph.system.base.as_mut().unwrap();
                        //editing existing component
                        self.nodebar.dummies.base.set_values_for(base);
                        // set the label for the node
                        let graph_node = self.graph.nodes.get_mut(&id).unwrap();
                        graph_node.node.label = base.get_name().to_string();
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
                        let graph_node = self.graph.nodes.get_mut(&id).unwrap();
                        graph_node.node.label = body.get_name().to_string();
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
                                self.nodebar.dummies.revolute.set_values_for(revolute);
                                let graph_node = self.graph.nodes.get_mut(&id).unwrap();
                                graph_node.node.label = revolute.get_name().to_string();
                            }
                            _ => panic!("this should not be possible"),
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
            DummyComponent::Prismatic => {
                if self.nodebar.dummies.prismatic.name.is_empty() {
                    self.counter_prismatic += 1;
                    self.nodebar.dummies.prismatic.name =
                        format!("prismatic{}", self.counter_prismatic).to_string();
                }
                match modal.component_id {
                    Some(id) => {
                        //editing existing joint
                        let joint = self.graph.system.joints.get_mut(&id).unwrap();
                        match joint {
                            Joint::Prismatic(prismatic) => {
                                self.nodebar.dummies.prismatic.set_values_for(prismatic);
                                let graph_node = self.graph.nodes.get_mut(&id).unwrap();
                                graph_node.node.label = prismatic.get_name().to_string();
                            }
                            _ => panic!("this should not be possible"),
                        }
                    }
                    None => {
                        //create new joint
                        let joint = self.nodebar.dummies.prismatic.to_joint();
                        let id = *joint.get_id();
                        let label = joint.get_name().to_string();
                        self.graph.system.add_joint(joint).unwrap();
                        self.graph
                            .save_component(&modal.dummy_type, id, label)
                            .unwrap();
                    }
                }

                self.nodebar.dummies.prismatic.clear();
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

    pub fn simulate(&mut self) -> Command<Message> {
        let sys = &self.graph.system;

        let SimDiv {
            name,
            start_time,
            stop_time,
            dt,
        } = &self.simdiv;

        // convert strings to floats
        let start_time = start_time.parse().unwrap_or(0.0);
        let stop_time = stop_time.parse().unwrap_or(10.0);
        let dt = dt.parse().unwrap_or(1.0);

        let mut name = name.clone();
        if name.is_empty() {
            name = format!("sim_{}", generate_unique_id());
        }

        let result = sys.simulate(name.clone(), start_time, stop_time, dt);
        self.results.insert(name.clone(), result);
        self.cache.clear();

        self.plot_tab.sim_menu.add_option(name.clone());

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

    pub fn update_revolute_field(
        &mut self,
        field: RevoluteField,
        string: String,
    ) -> Command<Message> {
        let dummy_revolute = &mut self.nodebar.dummies.revolute;
        match field {
            RevoluteField::Name => dummy_revolute.name = string,
            RevoluteField::ConstantForce => dummy_revolute.constant_force = string,
            RevoluteField::Damping => dummy_revolute.damping = string,
            RevoluteField::Omega => dummy_revolute.omega = string,
            RevoluteField::SpringConstant => dummy_revolute.spring_constant = string,
            RevoluteField::Theta => dummy_revolute.theta = string,
        }

        Command::none()
    }

    pub fn update_prismatic_field(
        &mut self,
        field: PrismaticField,
        string: String,
    ) -> Command<Message> {
        let dummy_prismatic = &mut self.nodebar.dummies.prismatic;
        match field {
            PrismaticField::Name => dummy_prismatic.name = string,
            PrismaticField::ConstantForce => dummy_prismatic.constant_force = string,
            PrismaticField::Damping => dummy_prismatic.damping = string,
            PrismaticField::Velocity => dummy_prismatic.velocity = string,
            PrismaticField::SpringConstant => dummy_prismatic.spring_constant = string,
            PrismaticField::Position => dummy_prismatic.position = string,
        }

        Command::none()
    }

    pub fn wheel_scrolled(&mut self, delta: ScrollDelta) -> Command<Message> {
        self.nodebar.wheel_scrolled(delta);
        self.graph.wheel_scrolled(delta);
        self.cache.clear();
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
