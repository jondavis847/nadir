use glam::Vec3;
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
    earth::Earth,
    ellipsoid::{Ellipsoid, EllipsoidRaw},
};

use camera::Camera;
use pipeline::{Pipeline, Uniforms};

use crate::Message;

#[derive(Debug, Clone)]
pub struct Scene {
    pub earth: Option<Earth>,
    pub cuboids: Vec<Cuboid>,
    pub ellipsoids: Vec<Ellipsoid>,
    pub camera: Camera,
    pub light_color: Color,
    pub light_pos: [f32; 3],
    pub world_target: Vec3,
}

impl Default for Scene {
    fn default() -> Self {
        Self {
            earth: None,
            cuboids: Vec::new(),
            ellipsoids: Vec::new(),
            camera: Camera::default(),
            light_color: Color::WHITE,
            light_pos: [0.0, 10.0, 0.0],
            world_target: Vec3::ZERO,
        }
    }
}
impl Scene {
    pub fn set_earth(&mut self, is_earth: bool) {
        self.earth = if is_earth {
            //TODO: calculate based on epoch
            self.light_pos = [0.0, 151.0e9, 0.0];           

            self.world_target = if !self.cuboids.is_empty() {                
                self.cuboids[0].position                
            } else if !self.ellipsoids.is_empty() {
                self.ellipsoids[0].position
            } else {
                todo!("no other shapes yet")
            };

            // convert all positions to target frame
            self.cuboids.iter_mut().for_each(|cuboid| cuboid.position -= self.world_target);
            self.ellipsoids.iter_mut().for_each(|ellipsoid| ellipsoid.position -= self.world_target);
            let unit = self.world_target.normalize();
            self.camera.set_position(10.0 * unit);
            self.camera.set_target(Vec3::ZERO);
            self.camera.set_far(1.0e12);
            
            let mut earth = Earth::default();
            earth.0.position -= self.world_target;
            Some(earth)
        } else {
            None
        };
    }
}

#[derive(Default, Debug)]
pub struct SceneState {
    is_pressed: bool,
    last_mouse_position: Point,
}

#[derive(Debug)]
pub struct ScenePrimitive {
    pub earth: Option<EllipsoidRaw>,
    pub uniforms: Uniforms,
    pub cuboids: Vec<CuboidRaw>,
    pub ellipsoids: Vec<EllipsoidRaw>,
}

impl ScenePrimitive {
    pub fn new(
        earth: &Option<Earth>,
        cuboids: &[Cuboid],
        ellipsoids: &[Ellipsoid],
        camera: &Camera,
        bounds: Rectangle,
        light_color: Color,
        light_pos: [f32; 3],
    ) -> Self {
        let uniforms = pipeline::Uniforms::new(camera, bounds, light_color, light_pos);
        let earth = if let Some(earth) = earth {
            Some(EllipsoidRaw::from_ellipsoid(&earth.0))
        } else {
            None
        };

        Self {
            earth,
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
            storage.store(Pipeline::new(
                device,
                queue,
                format,
                target_size,
                self.earth.is_some(),
            ));
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
            &self.earth,
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
            &self.earth,
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
