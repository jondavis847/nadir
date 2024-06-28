use iced::{mouse::Cursor, Point, Rectangle, Size};
use std::collections::HashMap;
use uuid::Uuid;

use crate::ui::canvas::node::Node;
use crate::ui::dummies::{DummyBase, DummyBody, DummyComponent, DummyRevolute};
use crate::{MouseButton, MouseButtonReleaseEvents};

pub enum NodebarMessage {
    NewComponent(Uuid),
}

#[derive(Debug, Clone)]
pub struct NodebarNode {
    pub component_id: Uuid,
    pub home: Point,
    pub label: String, //TODO: make this an enum with function calls to get the labels?
    pub node: Node,
}

impl NodebarNode {
    pub fn new(component_id: Uuid, home: Point, label: String, node: Node) -> Self {
        Self {
            component_id,
            home,
            label,
            node,
        }
    }

    pub fn go_home(&mut self) {
        //send it home
        self.node.bounds.x = self.home.x;
        self.node.bounds.y = self.home.y;
        self.node.is_left_clicked = false;
        self.node.is_selected = false;
    }
}

#[derive(Debug, Clone)]
pub struct NodebarMap {
    pub base: Uuid,
    pub body: Uuid,
    pub revolute: Uuid,
}

#[derive(Debug, Clone)]
pub struct Nodebar {
    pub bounds: Rectangle,
    pub components: HashMap<Uuid, DummyComponent>,
    pub left_clicked_node: Option<Uuid>,
    pub map: NodebarMap,
    pub nodes: HashMap<Uuid, NodebarNode>,
}

impl Default for Nodebar {
    fn default() -> Self {
        let mut components = HashMap::new();
        let mut nodes = HashMap::new();

        let bounds = Rectangle::new(Point::new(0.0, 0.0), Size::new(130.0, 1000.0));
        let mut count: f32 = 1.0;

        let base_component_id = Uuid::new_v4();
        let base_component = DummyComponent::Base(DummyBase::new(base_component_id));
        components.insert(base_component_id, base_component);

        let base_node = create_default_node("+base", &mut count, base_component_id);
        let base_node_id = Uuid::new_v4();
        nodes.insert(base_node_id, base_node);

        let body_component_id = Uuid::new_v4();
        let body_component = DummyComponent::Body(DummyBody::new(body_component_id));
        components.insert(body_component_id, body_component);

        let body_node = create_default_node("+body", &mut count, body_component_id);
        let body_node_id = uuid::Uuid::new_v4();
        nodes.insert(body_node_id, body_node);

        let revolute_component_id = Uuid::new_v4();
        let revolute_component =
            DummyComponent::Revolute(DummyRevolute::new(revolute_component_id));
        components.insert(revolute_component_id, revolute_component);

        let revolute_node = create_default_node("+revolute", &mut count, revolute_component_id);
        let revolute_node_id = Uuid::new_v4();
        nodes.insert(revolute_node_id, revolute_node);

        let left_clicked_node = None;

        let map = NodebarMap {
            base: base_component_id,
            body: body_component_id,
            revolute: revolute_component_id,
        };

        Self {
            bounds,
            components,
            left_clicked_node,
            map,
            nodes,
        }
    }
}

impl Nodebar {
    pub fn cursor_moved(&mut self, cursor: Cursor) -> bool {
        let mut redraw = false;
        if let Some(clicked_node_id) = self.left_clicked_node {
            // a node is clicked and being dragged
            if let Some(nodebarnode) = self.nodes.get_mut(&clicked_node_id) {
                let clicked_node = &mut nodebarnode.node;
                if let Some(cursor_position) = cursor.position() {
                    clicked_node.translate_to(cursor_position);
                    redraw = true;
                }
            }
        }
        redraw
    }

    pub fn left_button_pressed(&mut self, cursor: Cursor) {
        self.left_clicked_node = None;

        if cursor.is_over(self.bounds) {
            if let Some(cursor_position) = cursor.position() {
                for (id, nodebarnode) in &mut self.nodes {
                    nodebarnode
                        .node
                        .is_clicked(cursor_position, &MouseButton::Left);

                    if nodebarnode.node.is_left_clicked {
                        self.left_clicked_node = Some(*id);
                    }
                }
            }
        }
    }

    pub fn left_button_released(
        &mut self,
        release_event: &MouseButtonReleaseEvents,
    ) -> Option<NodebarMessage> {
        let mut message = None;

        if let Some(clicked_node_id) = self.left_clicked_node {
            if let Some(nodebarnode) = self.nodes.get_mut(&clicked_node_id) {                
                match release_event {
                    MouseButtonReleaseEvents::DoubleClick => {}
                    MouseButtonReleaseEvents::SingleClick => {                        
                        // SingleClick is < 200 ms, which you can move and drop fast enough technically
                        if self.components.contains_key(&nodebarnode.component_id) {
                            message = Some(NodebarMessage::NewComponent(nodebarnode.component_id));
                        }
                        nodebarnode.go_home();
                    }
                    MouseButtonReleaseEvents::Held => {
                        if self.components.contains_key(&nodebarnode.component_id) {
                            message = Some(NodebarMessage::NewComponent(nodebarnode.component_id));
                        }
                        nodebarnode.go_home();
                    }
                    MouseButtonReleaseEvents::Nothing => {}
                }
            }
        } else {
            // Clear all selections if no node was clicked
            self.nodes
                .values_mut()
                .for_each(|nodebarnode| nodebarnode.node.is_selected = false);
        }
        self.left_clicked_node = None;
        message
    }

    pub fn right_button_pressed(&mut self, _cursor: Cursor) {
        //placeholder
    }

    pub fn right_button_released(&mut self, _cursor: Cursor) {
        //placeholder
    }

    pub fn window_resized(&mut self, size: Size) {
        self.bounds.height = size.height;
        self.bounds.width = size.width;
    }
}

fn create_default_node(label: &str, count: &mut f32, component_id: Uuid) -> NodebarNode {
    let padding = 15.0;
    let height = 50.0;
    let node_size = Size::new(100.0, height);
    let home = Point::new(padding, *count * padding + (*count - 1.0) * height);

    let node = Node::new(Rectangle::new(home, node_size)); //, label.to_string());

    *count += 1.0;
    NodebarNode::new(component_id, home, label.to_string(), node)
}
