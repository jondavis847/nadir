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
pub mod geometries;
pub mod pipeline;
pub mod vertex;

use geometries::{
    cuboid::{Cuboid, CuboidRaw},
    ellipsoid::{Ellipsoid, EllipsoidRaw},
};

use camera::Camera;
use pipeline::{Pipeline, Uniforms};

use crate::Message;

#[derive(Debug, Clone)]
pub struct Scene {
    pub cuboids: Vec<Cuboid>,
    pub ellipsoids: Vec<Ellipsoid>,
    pub camera: Camera,
    pub light_color: Color,
    pub light_pos: [f32; 3],
}

impl Default for Scene {
    fn default() -> Self {
        Self {
            cuboids: Vec::new(),
            ellipsoids: Vec::new(),
            camera: Camera::default(),
            light_color: Color::WHITE,
            light_pos: [10.0, 10.0, 10.0],
        }
    }
}
impl Scene {
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
    pub ellipsoids: Vec<EllipsoidRaw>,
}

impl ScenePrimitive {
    pub fn new(
        cuboids: &[Cuboid],
        ellipsoids: &[Ellipsoid],
        camera: &Camera,
        bounds: Rectangle,
        light_color: Color,
        light_pos: [f32; 3],
    ) -> Self {
        let uniforms = pipeline::Uniforms::new(camera, bounds, light_color, light_pos);

        Self {
            cuboids: cuboids
                .iter()
                .map(CuboidRaw::from_cuboid)
                .collect::<Vec<CuboidRaw>>(),
            ellipsoids: ellipsoids
                .iter()
                .map(EllipsoidRaw::from_ellipsoid)
                .collect::<Vec<EllipsoidRaw>>(),
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

          //upload data to GPU            
        let pipeline = storage.get_mut::<Pipeline>().unwrap();
        pipeline.update(
            device,
            queue,
            target_size,
            &self.uniforms,
            &self.cuboids,
            &self.ellipsoids,
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
        pipeline.render(
            target,
            encoder,
            viewport,
            self.cuboids.len() as u32,
            self.ellipsoids.len() as u32,
            false,
        );
    }
}

impl Program<Message> for Scene {
    type State = SceneState;
    type Primitive = ScenePrimitive;

    // Required method
    fn draw(&self, state: &Self::State, cursor: Cursor, bounds: Rectangle) -> Self::Primitive {
        ScenePrimitive::new(
            &self.cuboids,
            &self.ellipsoids,
            &self.camera,
            bounds,
            self.light_color,
            self.light_pos,
        )
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
                    mouse::Event::CursorMoved { position: _ } => {
                        //use canvas position instead of this position
                        let last_position = state.last_mouse_position;
                        state.last_mouse_position = canvas_cursor_position;
                        if state.is_pressed {
                            let delta = canvas_cursor_position - last_position;
                            (Status::Captured, Some(Message::CameraRotation(delta)))
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
