use iced::{mouse::ScrollDelta, Point, Rectangle, Size};
use std::collections::HashMap;
use uuid::Uuid;

use super::node::Node;
use crate::ui::dummies::{Dummies, DummyComponent};
use crate::ui::modals::ActiveModal;
use crate::ui::mouse::{MouseButton, MouseButtonReleaseEvents};

pub enum NodebarMessage {
    NewComponent(ActiveModal),
}

#[derive(Debug, Clone)]
pub struct NodebarNode {
    pub component_type: DummyComponent,
    pub home: Point,
    pub node: Node,
}

impl NodebarNode {
    pub fn new(component_type: DummyComponent, home: Point, node: Node) -> Self {
        Self {
            component_type,
            home,
            node,
        }
    }

    pub fn go_home(&mut self) {
        //send it home
        self.node.rendered_bounds.x = self.home.x;
        self.node.rendered_bounds.y = self.home.y;
        self.node.is_left_clicked = false;
        self.node.is_selected = false;
    }
}

#[derive(Debug, Clone)]
pub struct Nodebar {
    pub bounds: Rectangle,
    pub dummies: Dummies,
    pub left_clicked_node: Option<Uuid>,
    pub nodes: HashMap<Uuid, NodebarNode>,
}

impl Default for Nodebar {
    fn default() -> Self {
        let mut nodes = HashMap::new();
        let x = 0.0;

        let bounds = Rectangle::new(Point::new(x, 0.0), Size::new(150.0, 1000.0));
        let mut count: f32 = 1.0;

        let base_node = create_default_node("+base", &mut count, DummyComponent::Base, x);
        let base_node_id = Uuid::new_v4();
        nodes.insert(base_node_id, base_node);

        let body_node = create_default_node("+body", &mut count, DummyComponent::Body, x);
        let body_node_id = uuid::Uuid::new_v4();
        nodes.insert(body_node_id, body_node);

        let prismatic_node =
            create_default_node("+prismatic", &mut count, DummyComponent::Prismatic, x);
        let prismatic_node_id = Uuid::new_v4();
        nodes.insert(prismatic_node_id, prismatic_node);

        let revolute_node =
            create_default_node("+revolute", &mut count, DummyComponent::Revolute, x);
        let revolute_node_id = Uuid::new_v4();
        nodes.insert(revolute_node_id, revolute_node);

        let left_clicked_node = None;

        let dummies = Dummies::default();

        Self {
            bounds,
            dummies,
            left_clicked_node,
            nodes,
        }
    }
}

impl Nodebar {
    pub fn cursor_moved(&mut self, canvas_cursor_position: Point) -> bool {
        let mut redraw = false;
        if let Some(clicked_node_id) = self.left_clicked_node {
            // a node is clicked and being dragged
            if let Some(nodebarnode) = self.nodes.get_mut(&clicked_node_id) {
                let clicked_node = &mut nodebarnode.node;
                clicked_node.translate_to(canvas_cursor_position);
                redraw = true;
            }
        }
        redraw
    }

    pub fn left_button_pressed(&mut self, canvas_cursor_position: Point) {
        self.left_clicked_node = None;
        if self.bounds.contains(canvas_cursor_position) {
            for (id, nodebarnode) in &mut self.nodes {
                nodebarnode
                    .node
                    .is_clicked(canvas_cursor_position, &MouseButton::Left);

                if nodebarnode.node.is_left_clicked {
                    self.left_clicked_node = Some(*id);
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
                        message = Some(NodebarMessage::NewComponent(ActiveModal::new(
                            nodebarnode.component_type,
                            None,
                        )));
                        nodebarnode.go_home();
                    }
                    MouseButtonReleaseEvents::Held => {
                        message = Some(NodebarMessage::NewComponent(ActiveModal::new(
                            nodebarnode.component_type,
                            None,
                        )));
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

    pub fn right_button_pressed(&mut self, _canvas_cursor_position: Point) {
        //placeholder
    }

    pub fn right_button_released(&mut self, _canvas_cursor_position: Point) {
        //placeholder
    }

    pub fn wheel_scrolled(&mut self, _delta: ScrollDelta) {
        //placeholder for nodebar scrolling
    }

    pub fn window_resized(&mut self, size: Size) {
        self.bounds.height = size.height;
        self.bounds.width = size.width;
    }
}

fn create_default_node(
    label: &str,
    count: &mut f32,
    component_type: DummyComponent,
    x: f32,
) -> NodebarNode {
    let padding = 25.0;
    let height = 50.0;
    let node_size = Size::new(100.0, height);
    let home = Point::new(x + padding, *count * padding + (*count - 1.0) * height);

    let node = Node::new(label.to_string(), Rectangle::new(home, node_size), 1.0); //, label.to_string());

    *count += 1.0;
    NodebarNode::new(component_type, home, node)
}
