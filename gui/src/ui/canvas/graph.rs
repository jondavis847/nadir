use iced::{mouse::Cursor, Point, Rectangle, Size};
use multibody::joint::JointTrait;
use std::collections::HashMap;
use transforms::Transform;
use uuid::Uuid;

use super::edge::{Edge, EdgeConnection};
use super::node::Node;
use multibody::{system::MultibodySystem, MultibodyErrors};

use crate::ui::dummies::DummyComponent;
use crate::ui::mouse::{MouseButton, MouseButtonReleaseEvents};

pub enum GraphMessage {
    EditComponent((DummyComponent, Uuid)),
}

#[derive(Debug, Clone, Copy)]
pub enum GraphErrors {
    BodyInvalidId(Uuid),
    BodyMissingFrom(Uuid),
    IdNotFound(Uuid),
    NoBase,
    NoBaseConnections,
    JointMissingFrom(Uuid),
    JointMissingTo(Uuid),
    JointNoOuterBody(Uuid),
    Multibody(MultibodyErrors),
}

#[derive(Debug, Clone)]
pub struct GraphNode {
    pub component_id: Uuid,
    pub dummy_type: DummyComponent,
    pub edges: Vec<Uuid>,
    pub node: Node,
}

impl GraphNode {
    fn new(component_id: Uuid, dummy_type: DummyComponent, node: Node) -> Self {
        let edges = Vec::new();
        Self {
            component_id,
            dummy_type,
            edges,
            node,
        }
    }
}

#[derive(Debug)]
pub struct Graph {
    pub bounds: Rectangle,
    pub system: MultibodySystem,
    current_edge: Option<Uuid>,
    pub edges: HashMap<Uuid, Edge>,
    pub is_clicked: bool,
    last_cursor_position: Option<Point>,
    left_clicked_node: Option<Uuid>,
    pub nodes: HashMap<Uuid, GraphNode>,
    right_clicked_node: Option<Uuid>,
    selected_node: Option<Uuid>,
    //zoom: f32,
}

impl Default for Graph {
    fn default() -> Self {
        Self {
            bounds: Rectangle::new(Point::new(500.0, 0.0), Size::new(2000.0, 1000.0)),
            system: MultibodySystem::new(),
            current_edge: None,
            edges: HashMap::new(),
            is_clicked: false,
            last_cursor_position: None,
            left_clicked_node: None,
            nodes: HashMap::new(),
            right_clicked_node: None,
            selected_node: None,
            //zoom: 1.0,
        }
    }
}

impl Graph {
    pub fn cursor_moved(&mut self, cursor: Cursor) -> bool {
        let mut redraw = false;
        let cursor_position = cursor.position_in(self.bounds);

        // Handle left-clicked node dragging
        if let Some(clicked_node_id) = self.left_clicked_node {
            if let Some(graphnode) = self.nodes.get_mut(&clicked_node_id) {
                if let Some(position) = cursor_position {
                    graphnode.node.translate_to(position);
                    redraw = true;
                }
            }
        } else if self.is_clicked {
            // Handle graph translating
            if let Some(graph_cursor_position) = cursor_position {
                if let Some(last_position) = self.last_cursor_position {
                    let delta = graph_cursor_position - last_position;
                    self.nodes.iter_mut().for_each(|(_, graphnode)| {
                        graphnode.node.translate_by(delta);
                    });
                    redraw = true;
                }
            }
        }

        // Update last cursor position
        if let Some(position) = cursor_position {
            self.last_cursor_position = Some(position);

            // Handle right-clicked node for edge drawing
            if let Some(clicked_node_id) = self.right_clicked_node {
                if let Some(edge_id) = self.current_edge {
                    if let Some(edge) = self.edges.get_mut(&edge_id) {
                        edge.to = EdgeConnection::Point(position);
                    }
                } else {
                    let new_edge = Edge::new(
                        EdgeConnection::Node(clicked_node_id),
                        EdgeConnection::Point(position),
                    );
                    let new_edge_id = Uuid::new_v4();
                    self.edges.insert(new_edge_id, new_edge);
                    self.current_edge = Some(new_edge_id);

                    // Add the edge to the from node
                    if let Some(from_node) = self.nodes.get_mut(&clicked_node_id) {
                        from_node.edges.push(new_edge_id);
                    }
                }
                redraw = true;
            }
        }

        redraw
    }

    pub fn delete_pressed(&mut self) {
        if let Some(selected_node_id) = self.selected_node {
            if let Some(selected_node) = self.nodes.remove(&selected_node_id) {
                // Remove edges from all nodes and edges collection
                for edge_id in selected_node.edges {
                    for node in self.nodes.values_mut() {
                        node.edges.retain(|x| x != &edge_id);
                    }
                    self.edges.remove(&edge_id);
                }

                match selected_node.dummy_type {
                    DummyComponent::Base => self.system.base = None,
                    DummyComponent::Body => {
                        self.system.bodies.remove(&selected_node.component_id);
                    }
                    DummyComponent::Revolute => {
                        self.system.joints.remove(&selected_node.component_id);
                    }
                };
            }
        }
    }

    /// Finds a node within snapping distance of the cursor on the graph, if any.
    ///
    /// # Arguments
    ///
    /// * `cursor` - The current position of the cursor.
    /// * `nodes` - A reference to the hashmap containing all nodes in the graph.
    ///
    /// # Returns
    ///
    /// * `Option<Uuid>` - The UUID of the node under the cursor, if any.
    ///
    /// This function checks if the cursor is within the graph's bounds and, if so,
    /// iterates over the nodes to find the first node is in snapping distance of the cursor position.
    /// If such a node is found, its UUID is returned.
    fn get_snappable_node(&self, cursor: Cursor) -> Option<Uuid> {
        // Check if the cursor is within the graph's bounds
        if let Some(cursor_position) = cursor.position_in(self.bounds) {
            // Find the first node that the cursor is in snapping distance of
            return self
                .nodes
                .iter()
                .find(|(_, graphnode)| {
                    let node = &graphnode.node;
                    // Check if the cursor's x and y positions are within the node's bounds
                    cursor_position.x > node.bounds.x
                        && cursor_position.x < node.bounds.x + node.bounds.width
                        && cursor_position.y > node.bounds.y
                        && cursor_position.y < node.bounds.y + node.bounds.height
                })
                // If a node is found, return its UUID
                .map(|(id, _)| *id);
        }
        // If no node is found, return None
        None
    }

    pub fn left_button_pressed(&mut self, cursor: Cursor) {
        self.left_clicked_node = None;

        if let Some(cursor_position) = cursor.position_in(self.bounds) {
            self.is_clicked = true;
            self.last_cursor_position = Some(cursor_position);

            // Clear the nodes' selected flags and determine the clicked node
            for (id, graphnode) in &mut self.nodes {
                let node = &mut graphnode.node;
                node.is_selected = false;
                node.is_clicked(cursor_position, &MouseButton::Left);

                if node.is_left_clicked {
                    node.is_selected = true;
                    self.left_clicked_node = Some(*id);
                }
            }
        }

        // Update selected_node based on whether a node was clicked
        self.selected_node = self.left_clicked_node;
    }

    pub fn left_button_released(
        &mut self,
        release_event: &MouseButtonReleaseEvents,
        cursor: Cursor,
    ) -> Option<GraphMessage> {
        self.is_clicked = false;
        let mut message = None;

        if let Some(cursor_position) = cursor.position_in(self.bounds) {
            self.last_cursor_position = Some(cursor_position);
        }

        //clear the nodes selected flags, to be reapplied on click
        self.nodes.iter_mut().for_each(|(_, graphnode)| {
            graphnode.node.is_selected = false;
        });

        if let Some(clicked_node_id) = self.left_clicked_node {
            if let Some(graphnode) = self.nodes.get_mut(&clicked_node_id) {
                let clicked_node = &mut graphnode.node;
                match release_event {
                    MouseButtonReleaseEvents::DoubleClick => {
                        clicked_node.is_selected = true;

                        message = Some(GraphMessage::EditComponent((
                            graphnode.dummy_type,
                            graphnode.component_id,
                        )));
                    }
                    MouseButtonReleaseEvents::SingleClick => {
                        clicked_node.is_selected = true;
                    }
                    MouseButtonReleaseEvents::Held => {
                        clicked_node.is_selected = true;
                    }
                    MouseButtonReleaseEvents::Nothing => {}
                }
            }
        } else {
            // Clear all selections if no node was clicked
            self.nodes
                .values_mut()
                .for_each(|graphnode| graphnode.node.is_selected = false);
            self.selected_node = None;
        }
        self.left_clicked_node = None;
        message
    }

    pub fn right_button_pressed(&mut self, cursor: Cursor) {
        self.right_clicked_node = None;

        if let Some(cursor_position) = cursor.position_in(self.bounds) {
            for (id, graphnode) in &mut self.nodes {
                let node = &mut graphnode.node;
                node.is_clicked(cursor_position, &MouseButton::Right);
                if node.is_right_clicked {
                    self.right_clicked_node = Some(*id);
                }
            }
            self.last_cursor_position = Some(cursor_position);
        }
    }

    /// Handles the release of the right mouse button, finalizing or canceling an edge creation process.
    ///
    /// This function checks if there is an active edge creation process (stored in `self.current_edge`).
    /// If there is, it attempts to finalize the edge by connecting it to a node near the cursor.
    /// If any step in this process fails, it gracefully exits by removing the edge and resetting the state.
    ///
    /// # Arguments
    ///
    /// * `cursor` - The current position of the cursor.
    pub fn right_button_released(&mut self, cursor: Cursor) {
        // Get the current edge ID if it exists, return if it does not
        let edge_id = match self.current_edge {
            Some(id) => id,
            None => return,
        };

        // Helper function to clean up in case anything goes wrong
        let graceful_exit = |graph: &mut Self| {
            graph.edges.remove(&edge_id);
            graph.current_edge = None;
            graph.right_clicked_node = None;
        };

        // Get the edge if it exists, return if it does not
        let edge = match self.edges.get(&edge_id) {
            Some(edge) => edge,
            None => {
                graceful_exit(self);
                return;
            }
        };

        // Get the from_node_id, return if it is an EdgeConnection::Point
        let from_node_id = match edge.from {
            EdgeConnection::Node(id) => id,
            _ => {
                graceful_exit(self);
                return;
            }
        };

        // Get the component ID of the from node, return if it does not exist
        let from_node = match self.nodes.get(&from_node_id) {
            Some(id) => id,
            None => {
                graceful_exit(self);
                return;
            }
        };

        // Attempt to get the snappable node near the cursor, return if it does not exist
        let to_node_id = match self.get_snappable_node(cursor) {
            Some(id) => id,
            None => {
                graceful_exit(self);
                return;
            }
        };

        // Get the component ID of the to node, return if it does not exist
        let to_node = match self.nodes.get(&to_node_id) {
            Some(id) => id,
            None => {
                graceful_exit(self);
                return;
            }
        };

        // match valid conections and connect, other wise exit
        match (&from_node.dummy_type, &to_node.dummy_type) {
            (DummyComponent::Base, DummyComponent::Revolute) => {
                let base = self.system.base.as_mut().unwrap();
                let joint = self.system.joints.get_mut(&to_node.component_id).unwrap();
                joint
                    .connect_inner_body(base, Transform::default())
                    .unwrap();
            }
            (DummyComponent::Body, DummyComponent::Revolute) => {
                let body = self.system.bodies.get_mut(&from_node.component_id).unwrap();
                let joint = self.system.joints.get_mut(&to_node.component_id).unwrap();
                joint
                    .connect_inner_body(body, Transform::default())
                    .unwrap();
            }
            (DummyComponent::Revolute, DummyComponent::Body) => {
                let joint = self.system.joints.get_mut(&from_node.component_id).unwrap();
                let body = self.system.bodies.get_mut(&to_node.component_id).unwrap();
                joint
                    .connect_outer_body(body, Transform::default())
                    .unwrap();
            }
            _ => {
                graceful_exit(self);
                return;
            }
        }

        // now borrow mutably to push edges to to_node
        // from_node already has the edge at this point
        let to_node = match self.nodes.get_mut(&to_node_id) {
            Some(id) => id,
            None => {
                graceful_exit(self);
                return;
            }
        };
        to_node.edges.push(edge_id);

        // Update the edge to connect to the to_node, return if the edge does not exist
        let edge = match self.edges.get_mut(&edge_id) {
            Some(edge) => edge,
            None => {
                graceful_exit(self);
                return;
            }
        };
        edge.to = EdgeConnection::Node(to_node_id);

        // Clear the current edge and right-clicked node state
        self.current_edge = None;
        self.right_clicked_node = None;
    }

    pub fn save_component(
        &mut self,
        dummy_type: &DummyComponent,
        component_id: Uuid,
        label: String,
    ) -> Result<(), GraphErrors> {
        // only do this if we can save the node
        if let Some(last_cursor_position) = self.last_cursor_position {
            // Generate unique IDs for node
            let node_id = Uuid::new_v4();

            // Calculate the bounds for the new node
            let size = Size::new(100.0, 50.0); // TODO: make width dynamic based on name length
            let top_left = Point::new(
                last_cursor_position.x - size.width / 2.0,
                last_cursor_position.y - size.height / 2.0,
            );
            let bounds = Rectangle::new(top_left, size);

            // Create the new node
            let name = label;
            let new_node = Node::new(name, bounds);
            let graph_node = GraphNode::new(component_id, *dummy_type, new_node);

            self.nodes.insert(node_id, graph_node);
        }
        Ok(())
    }

    pub fn window_resized(&mut self, size: Size) {
        self.bounds.height = size.height;
        self.bounds.width = size.width;
    }
}
