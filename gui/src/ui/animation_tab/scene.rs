use iced::{
    advanced::Shell,
    mouse::{self, Cursor, Interaction},
    widget::{
        canvas::event::Status,
        shader::{wgpu, Event, Primitive, Program, Storage},
    },
    Color, Point, Rectangle, Size,
};

pub mod camera;
pub mod geometry;
pub mod pipeline;
pub mod vertex;

use geometry::cuboid::{Cuboid, CuboidRaw};

use camera::Camera;
use pipeline::{Pipeline, Uniforms};

use crate::Message;

#[derive(Debug, Clone)]
pub struct Scene {
    pub cuboids: Vec<Cuboid>,
    pub camera: Camera,
    pub light_color: Color,
}

impl Scene {
    pub fn new() -> Self {
        let cuboid = Cuboid::new(2.0, 1.0, 3.0, glam::Quat::IDENTITY, glam::Vec3::ZERO);

        let cuboid2 = Cuboid::new(
            1.0,
            1.0,
            1.0,
            glam::Quat::IDENTITY,
            glam::Vec3::new(0.5, 0.5, 0.5),
        );
        Self {
            cuboids: vec![cuboid, cuboid2],
            camera: Camera::default(),
            light_color: Color::WHITE,
        }
    }
}

#[derive(Default, Debug)]
pub struct SceneState {
    is_pressed: bool,
    last_mouse_position: Point,
}

#[derive(Debug)]
pub struct ScenePrimitive {
    pub uniforms: Uniforms,
    pub cuboids: Vec<CuboidRaw>,
}

impl ScenePrimitive {
    pub fn new(cuboids: &[Cuboid], camera: &Camera, bounds: Rectangle, light_color: Color) -> Self {
        let uniforms = pipeline::Uniforms::new(camera, bounds, light_color);

        Self {
            cuboids: cuboids
                .iter()
                .map(CuboidRaw::from_cuboid)
                .collect::<Vec<CuboidRaw>>(),
            uniforms,
        }
    }
}

impl Primitive for ScenePrimitive {
    fn prepare(
        &self,
        format: wgpu::TextureFormat,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _bounds: Rectangle,
        target_size: Size<u32>,
        _scale_factor: f32,
        storage: &mut Storage,
    ) {
        if !storage.has::<Pipeline>() {
            storage.store(Pipeline::new(device, queue, format, target_size));
        }

        let pipeline = storage.get_mut::<Pipeline>().unwrap();

        //upload data to GPU
        pipeline.update(
            device,
            queue,
            target_size,
            &self.uniforms,
            self.cuboids.len(),
            &self.cuboids,
        );
    }

    fn render(
        &self,
        storage: &Storage,
        target: &wgpu::TextureView,
        _target_size: Size<u32>,
        viewport: Rectangle<u32>,
        encoder: &mut wgpu::CommandEncoder,
    ) {
        //at this point our pipeline should always be initialized
        let pipeline = storage.get::<Pipeline>().unwrap();

        //render primitive
        pipeline.render(target, encoder, viewport, self.cuboids.len() as u32, false);
    }
}

impl Program<Message> for Scene {
    type State = SceneState;
    type Primitive = ScenePrimitive;

    // Required method
    fn draw(&self, state: &Self::State, cursor: Cursor, bounds: Rectangle) -> Self::Primitive {
        ScenePrimitive::new(&self.cuboids, &self.camera, bounds, self.light_color)
    }

    // Provided methods
    fn update(
        &self,
        state: &mut Self::State,
        event: Event,
        bounds: Rectangle,
        cursor: Cursor,
        _shell: &mut Shell<'_, Message>,
    ) -> (Status, Option<Message>) {
        if let Some(canvas_cursor_position) = cursor.position_in(bounds) {
            match event {
                Event::Mouse(mouse_event) => match mouse_event {
                    mouse::Event::ButtonPressed(mouse::Button::Left) => {
                        state.is_pressed = true;
                        state.last_mouse_position = canvas_cursor_position;
                        (
                            Status::Captured,
                            None, //Some(Message::LeftButtonPressed(canvas_cursor_position)),
                        )
                    }
                    mouse::Event::ButtonReleased(mouse::Button::Left) => {
                        state.is_pressed = false;
                        (Status::Captured, None)
                    }
                    mouse::Event::ButtonPressed(mouse::Button::Middle) => (
                        Status::Captured,
                        Some(Message::MiddleButtonPressed(canvas_cursor_position)),
                    ),
                    mouse::Event::ButtonPressed(mouse::Button::Right) => (
                        Status::Captured,
                        Some(Message::RightButtonPressed(canvas_cursor_position)),
                    ),
                    mouse::Event::ButtonReleased(mouse::Button::Right) => (
                        Status::Captured,
                        Some(Message::RightButtonReleased(canvas_cursor_position)),
                    ),
                    mouse::Event::CursorMoved { position: _ } => { //use canvas position instead of this position
                        let last_position = state.last_mouse_position;
                        state.last_mouse_position = canvas_cursor_position;
                        if state.is_pressed {                            
                            let delta = canvas_cursor_position - last_position;
                            (
                                Status::Captured,
                                Some(Message::TabAnimationCameraRotation(delta)),
                            )
                        } else {
                            (Status::Captured, None)
                        }
                    }
                    mouse::Event::WheelScrolled { delta } => {
                        (Status::Captured, Some(Message::WheelScrolled(delta)))
                    }
                    _ => (Status::Captured, None),
                },
                _ => (Status::Ignored, None),
            }
        } else {
            (Status::Ignored, None)
        }
    }
    fn mouse_interaction(
        &self,
        _state: &Self::State,
        _bounds: Rectangle,
        _cursor: Cursor,
    ) -> Interaction {
        Interaction::Pointer
    }
}
