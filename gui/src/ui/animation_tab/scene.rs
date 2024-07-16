use crate::Message;

use iced::{
    advanced::Shell,
    mouse::{Cursor, Interaction},
    widget::{
        canvas::event::Status,
        shader::{wgpu, Event, Primitive, Program, Storage},
    },
    Color, Rectangle, Size,
};

pub mod camera;
pub mod pipeline;

use camera::Camera;
use pipeline::{
    cube::{Cube, Raw},
    Pipeline, Uniforms,
};

#[derive(Debug, Clone)]
pub struct Scene {    
    pub cubes: Vec<Cube>,
    pub camera: Camera,
    pub light_color: Color,
}

impl Scene {
    pub fn new() -> Self {
        let cube = Cube::new(0.2, glam::Quat::IDENTITY, glam::Vec3::ZERO);
        Self {            
            cubes: vec![cube],
            camera: Camera::default(),
            light_color: Color::WHITE,
        }
    }
}

#[derive(Default, Debug)]
pub struct SceneState {}

#[derive(Debug)]
pub struct ScenePrimitive {
    pub uniforms: Uniforms,
    pub cubes: Vec<Raw>,
}

impl ScenePrimitive {
    pub fn new(cubes: &[Cube], camera: &Camera, bounds: Rectangle, light_color: Color) -> Self {
        let uniforms = pipeline::Uniforms::new(camera, bounds, light_color);

        Self {
            cubes: cubes.iter().map(Raw::from_cube).collect::<Vec<Raw>>(),
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
        pipeline.update(device, queue, target_size, &self.uniforms, 1, &self.cubes);
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
        pipeline.render(target, encoder, viewport, self.cubes.len() as u32, false);
    }
}

impl<Message> Program<Message> for Scene {
    type State = SceneState;
    type Primitive = ScenePrimitive;

    // Required method
    fn draw(&self, state: &Self::State, cursor: Cursor, bounds: Rectangle) -> Self::Primitive {
        ScenePrimitive::new(&self.cubes, &self.camera, bounds, self.light_color)
    }

    // Provided methods
    fn update(
        &self,
        _state: &mut Self::State,
        _event: Event,
        _bounds: Rectangle,
        _cursor: Cursor,
        _shell: &mut Shell<'_, Message>,
    ) -> (Status, Option<Message>) {
        (Status::Captured, None)
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
