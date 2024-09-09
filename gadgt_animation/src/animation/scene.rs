use gadgt_3d::{
    earth::{Atmosphere, Earth},
    geometry::{
        cuboid::Cuboid,
        ellipsoid::{Ellipsoid16, Ellipsoid32, Ellipsoid64},
        Geometry,
    },
    mesh::{Mesh, MeshGpu, MeshPrimitive},
};
use glam::Vec3;
use iced::{
    advanced::Shell,
    mouse::{self, Cursor, Interaction},
    widget::{
        canvas::event::Status,
        shader::{
            wgpu::{self, util::DeviceExt},
            Event, Primitive, Program, Storage,
        },
    },
    Color, Point, Rectangle, Size,
};

use image::{load_from_memory, GenericImageView};

pub mod camera;
pub mod earth_pipeline;
pub mod pipeline;

use camera::Camera;
use earth_pipeline::{AtmospherePipeline, EarthBindGroup, EarthPipeline};
use pipeline::{
    uniforms::Uniforms, CuboidPipeline, Ellipsoid16Pipeline, Ellipsoid32Pipeline,
    Ellipsoid64Pipeline, Pipeline,
};

use crate::Message;

#[derive(Debug)]
pub struct Scene {
    pub camera: Camera,
    pub earth: Option<Earth>,
    light_color: Color,
    light_pos: [f32; 3],
    pub meshes: Vec<Mesh>,
    pub world_target: Vec3,
}

impl Default for Scene {
    fn default() -> Self {
        Self {
            earth: None,
            camera: Camera::default(),
            light_color: Color::WHITE,
            light_pos: [0.0, 10.0, 0.0],
            meshes: Vec::new(),
            world_target: Vec3::ZERO,
        }
    }
}
impl Scene {
    pub fn set_earth(&mut self, is_earth: bool) {
        self.earth = if is_earth {
            //TODO: calculate based on epoch
            self.light_pos = [0.0, 151.0e9, 0.0];
            self.light_color = Color::new(1.0, 1.0, 0.9, 1.0);

            self.world_target = if !self.meshes.is_empty() {
                self.meshes[0].state.position
            } else {
                Vec3::new(0.0, 0.0, 0.0)
            };

            // convert all positions to target frame
            self.meshes
                .iter_mut()
                .for_each(|mesh| mesh.set_position_from_target(self.world_target));

            let unit = self.world_target.normalize();
            self.camera.set_position(10.0 * unit);
            self.camera.set_target(Vec3::ZERO);
            self.camera.set_far(1.0e12);
            self.camera.set_fov(45.0);

            let mut earth = Earth::default();
            earth.0.set_position_from_target(self.world_target);
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
struct DepthView(wgpu::TextureView);

#[derive(Debug)]
struct MultisampleView(wgpu::TextureView);

#[derive(Debug)]
struct UniformBuffer(wgpu::Buffer);

#[derive(Debug)]
struct UniformBindGroup(wgpu::BindGroup);

#[derive(Debug)]
pub struct ScenePrimitive {
    earth: Option<MeshGpu>,
    earth_atmosphere: Option<MeshGpu>,
    meshes: Vec<MeshPrimitive>,
    uniforms: Uniforms,
}

impl ScenePrimitive {
    pub fn new(scene: &Scene, bounds: Rectangle) -> Self {
        let (earth, earth_atmosphere) = if let Some(earth) = &scene.earth {
            let atmosphere = Atmosphere::from(earth);
            (
                Some(MeshGpu::from(&earth.0)),
                Some(MeshGpu::from(&atmosphere.0)),
            )
        } else {
            (None, None)
        };

        let meshes: Vec<MeshPrimitive> = scene.meshes.iter().map(MeshPrimitive::from).collect();
        let uniforms = Uniforms::new(scene, bounds);

        Self {
            earth,
            earth_atmosphere,
            meshes,
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
        let uniform_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("uniform bind group layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            });

        //Create and store the uniform bindgroup
        if !storage.has::<UniformBindGroup>() {
            let uniforms = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("uniform buffer"),
                contents: bytemuck::bytes_of(&self.uniforms),
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            });

            let uniform_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("uniform and texture bind group"),
                layout: &uniform_bind_group_layout,
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: uniforms.as_entire_binding(),
                }],
            });
            storage.store(UniformBuffer(uniforms));
            storage.store(UniformBindGroup(uniform_bind_group));
        } else {
            if let Some(uniform_buffer) = storage.get::<UniformBuffer>() {
                queue.write_buffer(
                    &uniform_buffer.0,                  // Buffer to update
                    0,                                  // Offset (start at the beginning)
                    bytemuck::bytes_of(&self.uniforms), // New uniform data
                );
            }
        }

        // Create and store the depth_view
        if !storage.has::<DepthView>() {
            let depth_texture = device.create_texture(&wgpu::TextureDescriptor {
                label: Some("depth.texture"),
                size: wgpu::Extent3d {
                    width: target_size.width,
                    height: target_size.height,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Depth32Float,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                    | wgpu::TextureUsages::TEXTURE_BINDING,
                view_formats: &[],
            });

            let depth_view = depth_texture.create_view(&wgpu::TextureViewDescriptor::default());
            storage.store(DepthView(depth_view));
        }

        if !storage.has::<MultisampleView>() {
            // create the antialiasing textureview
            let multisampled_texture = device.create_texture(&wgpu::TextureDescriptor {
                label: Some("multisampled.color.texture"),
                size: wgpu::Extent3d {
                    width: target_size.width,
                    height: target_size.height,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1, // Set the multisample count here to match the pipeline
                dimension: wgpu::TextureDimension::D2,
                format,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
                view_formats: &[],
            });
            let multisampled_texture_view =
                multisampled_texture.create_view(&wgpu::TextureViewDescriptor::default());
            storage.store(MultisampleView(multisampled_texture_view));
        }

        // create all pipelines for each geometry present in meshes
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("pipeline layout"),
            bind_group_layouts: &[&uniform_bind_group_layout],
            push_constant_ranges: &[],
        });

        const MAIN_SHADER: &str = include_str!("scene/shaders/main.wgsl");

        //cuboids
        let cuboids: Vec<MeshGpu> = self
            .meshes
            .iter()
            .filter_map(|mesh| {
                if let Geometry::Cuboid(_) = mesh.geometry {
                    Some(mesh.mesh_gpu) // Collect mesh_gpu if it's an Ellipsoid
                } else {
                    None
                }
            })
            .collect();

        if !cuboids.is_empty() {
            if !storage.has::<CuboidPipeline>() {
                storage.store(CuboidPipeline(Pipeline::new(
                    device,
                    format,
                    &layout,
                    MAIN_SHADER,
                    "cuboid",
                    Cuboid::vertices(),
                    &cuboids,
                )));
            } else {
                if let Some(cuboid_pipeline) = storage.get_mut::<CuboidPipeline>() {
                    let pipeline = &mut cuboid_pipeline.0;
                    pipeline.update(queue, &cuboids);
                }
            }
        }

        //ellipsoid16
        let ellipsoid16s: Vec<MeshGpu> = self
            .meshes
            .iter()
            .filter_map(|mesh| {
                if let Geometry::Ellipsoid16(_) = mesh.geometry {
                    Some(mesh.mesh_gpu) // Collect mesh_gpu if it's an Ellipsoid
                } else {
                    None
                }
            })
            .collect();

        if !ellipsoid16s.is_empty() {
            if !storage.has::<Ellipsoid16Pipeline>() {
                storage.store(Ellipsoid16Pipeline(Pipeline::new(
                    device,
                    format,
                    &layout,
                    MAIN_SHADER,
                    "ellipsoid16",
                    Ellipsoid16::vertices(),
                    &ellipsoid16s,
                )));
            } else {
                if let Some(ellipsoid16_pipeline) = storage.get_mut::<Ellipsoid16Pipeline>() {
                    let pipeline = &mut ellipsoid16_pipeline.0;
                    pipeline.update(queue, &ellipsoid16s);
                }
            }
        }

        //ellipsoid32
        let ellipsoid32s: Vec<MeshGpu> = self
            .meshes
            .iter()
            .filter_map(|mesh| {
                if let Geometry::Ellipsoid32(_) = mesh.geometry {
                    Some(mesh.mesh_gpu) // Collect mesh_gpu if it's an Ellipsoid
                } else {
                    None
                }
            })
            .collect();

        if !ellipsoid32s.is_empty() {
            if !storage.has::<Ellipsoid32Pipeline>() {
                storage.store(Ellipsoid32Pipeline(Pipeline::new(
                    device,
                    format,
                    &layout,
                    MAIN_SHADER,
                    "ellipsoid32",
                    Ellipsoid32::vertices(),
                    &ellipsoid32s,
                )));
            } else {
                if let Some(ellipsoid32_pipeline) = storage.get_mut::<Ellipsoid32Pipeline>() {
                    let pipeline = &mut ellipsoid32_pipeline.0;
                    pipeline.update(queue, &ellipsoid32s);
                }
            }
        }

        //ellipsoid64
        let ellipsoid64s: Vec<MeshGpu> = self
            .meshes
            .iter()
            .filter_map(|mesh| {
                if let Geometry::Ellipsoid64(_) = mesh.geometry {
                    Some(mesh.mesh_gpu) // Collect mesh_gpu if it's an Ellipsoid
                } else {
                    None
                }
            })
            .collect();

        if !ellipsoid64s.is_empty() {
            if !storage.has::<Ellipsoid64Pipeline>() {
                storage.store(Ellipsoid64Pipeline(Pipeline::new(
                    device,
                    format,
                    &layout,
                    MAIN_SHADER,
                    "ellipsoid64",
                    Ellipsoid64::vertices(),
                    &ellipsoid64s,
                )));
            } else {
                if let Some(ellipsoid64_pipeline) = storage.get_mut::<Ellipsoid64Pipeline>() {
                    let pipeline = &mut ellipsoid64_pipeline.0;
                    pipeline.update(queue, &ellipsoid64s);
                }
            }
        }

        //earth
        if let Some(earth) = &self.earth {
            if !storage.has::<EarthPipeline>() {
                //const EARTH_COLOR: &[u8] = include_bytes!("../../resources/earth_color_4k.jpg");
                //const EARTH_COLOR: &[u8] = include_bytes!("../../resources/nasa_earth_color_5k.png");
                const EARTH_COLOR: &[u8] = include_bytes!("../../resources/earth_color_8K.tif");
                //const EARTH_COLOR: &[u8] = include_bytes!("../../resources/nasa_earth_day_4k.jpg");
                //const EARTH_COLOR: &[u8] = include_bytes!("../../resources/nasa_earth_day_21k.jpg");
                //const EARTH_NIGHT: &[u8] = include_bytes!("../../resources/earth_night_4k.jpg");
                const EARTH_NIGHT: &[u8] =
                    include_bytes!("../../resources/earth_nightlights_10K.tif");
                const EARTH_CLOUDS: &[u8] = include_bytes!("../../resources/earth_clouds_8K.tif");
                const EARTH_SPEC: &[u8] = include_bytes!("../../resources/earth_spec_4k.jpg");

                let earth_day = load_texture(device, queue, EARTH_COLOR, "earth_color");
                let earth_night = load_texture(device, queue, EARTH_NIGHT, "earth_night");
                let earth_spec = load_texture(device, queue, EARTH_SPEC, "earth_spec");
                let earth_clouds = load_texture(device, queue, EARTH_CLOUDS, "earth_clouds");

                let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
                    address_mode_u: wgpu::AddressMode::Repeat,
                    address_mode_v: wgpu::AddressMode::ClampToEdge,
                    address_mode_w: wgpu::AddressMode::ClampToEdge,
                    mag_filter: wgpu::FilterMode::Linear,
                    min_filter: wgpu::FilterMode::Linear,
                    mipmap_filter: wgpu::FilterMode::Linear,
                    anisotropy_clamp: 16,
                    ..Default::default()
                });

                let earth_bind_group_layout =
                    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                        label: Some("earth bind group layout"),
                        entries: &[
                            // sampler
                            wgpu::BindGroupLayoutEntry {
                                binding: 0,
                                visibility: wgpu::ShaderStages::FRAGMENT,
                                ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                                count: None,
                            },
                            //earth color
                            wgpu::BindGroupLayoutEntry {
                                binding: 1,
                                visibility: wgpu::ShaderStages::FRAGMENT,
                                ty: wgpu::BindingType::Texture {
                                    sample_type: wgpu::TextureSampleType::Float {
                                        filterable: true,
                                    },
                                    view_dimension: wgpu::TextureViewDimension::D2,
                                    multisampled: false,
                                },
                                count: None,
                            },
                            //earth night
                            wgpu::BindGroupLayoutEntry {
                                binding: 2,
                                visibility: wgpu::ShaderStages::FRAGMENT,
                                ty: wgpu::BindingType::Texture {
                                    sample_type: wgpu::TextureSampleType::Float {
                                        filterable: true,
                                    },
                                    view_dimension: wgpu::TextureViewDimension::D2,
                                    multisampled: false,
                                },
                                count: None,
                            },
                            //earth specular
                            wgpu::BindGroupLayoutEntry {
                                binding: 3,
                                visibility: wgpu::ShaderStages::FRAGMENT,
                                ty: wgpu::BindingType::Texture {
                                    sample_type: wgpu::TextureSampleType::Float {
                                        filterable: true,
                                    },
                                    view_dimension: wgpu::TextureViewDimension::D2,
                                    multisampled: false,
                                },
                                count: None,
                            },
                            //earth clouds
                            wgpu::BindGroupLayoutEntry {
                                binding: 4,
                                visibility: wgpu::ShaderStages::FRAGMENT,
                                ty: wgpu::BindingType::Texture {
                                    sample_type: wgpu::TextureSampleType::Float {
                                        filterable: true,
                                    },
                                    view_dimension: wgpu::TextureViewDimension::D2,
                                    multisampled: false,
                                },
                                count: None,
                            },
                        ],
                    });

                let earth_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("earth and texture bind group"),
                    layout: &earth_bind_group_layout,
                    entries: &[
                        wgpu::BindGroupEntry {
                            binding: 0,
                            resource: wgpu::BindingResource::Sampler(&sampler),
                        },
                        wgpu::BindGroupEntry {
                            binding: 1,
                            resource: wgpu::BindingResource::TextureView(&earth_day),
                        },
                        wgpu::BindGroupEntry {
                            binding: 2,
                            resource: wgpu::BindingResource::TextureView(&earth_night),
                        },
                        wgpu::BindGroupEntry {
                            binding: 3,
                            resource: wgpu::BindingResource::TextureView(&earth_spec),
                        },
                        wgpu::BindGroupEntry {
                            binding: 4,
                            resource: wgpu::BindingResource::TextureView(&earth_clouds),
                        },
                    ],
                });

                storage.store(EarthBindGroup(earth_bind_group));

                let earth_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                    label: Some("earth pipeline layout"),
                    bind_group_layouts: &[&uniform_bind_group_layout, &earth_bind_group_layout],
                    push_constant_ranges: &[],
                });

                storage.store(EarthPipeline::new(
                    device,
                    format,
                    &earth_layout,
                    &[*earth],
                    Ellipsoid64::vertices(),
                ));
            } else {
                if let Some(earth_pipeline) = storage.get_mut::<EarthPipeline>() {
                    earth_pipeline.update(queue, &[*earth]);
                }
            }
        }

        if let Some(atmosphere) = &self.earth_atmosphere {
            // atmosphere
            if !storage.has::<AtmospherePipeline>() {
                storage.store(AtmospherePipeline::new(
                    device,
                    format,
                    &layout,
                    &[*atmosphere],
                    Ellipsoid32::vertices(),
                ));
            } else {
                if let Some(atmosphere_pipeline) = storage.get_mut::<AtmospherePipeline>() {
                    atmosphere_pipeline.update(queue, &[*atmosphere]);
                }
            }
        }
    }

    fn render(
        &self,
        storage: &Storage,
        target: &wgpu::TextureView,
        _target_size: Size<u32>,
        viewport: Rectangle<u32>,
        encoder: &mut wgpu::CommandEncoder,
    ) {
        // unpack the depth_view and uniform_bind_group from storage
        let depth_view = &storage.get::<DepthView>().unwrap().0;
        let multisample_view = &storage.get::<MultisampleView>().unwrap().0;
        let uniform_bind_group = &storage.get::<UniformBindGroup>().unwrap().0;

        // set up the render pass
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("gadgt.pipeline.pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                //view: target,
                view: target,         //multisample_view,
                resolve_target: None, //Some(target),
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: depth_view,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
        });

        pass.set_scissor_rect(viewport.x, viewport.y, viewport.width, viewport.height);
        pass.set_bind_group(0, uniform_bind_group, &[]);

        //render cuboids
        if let Some(cuboid_pipeline) = storage.get::<CuboidPipeline>() {
            let pipeline = &cuboid_pipeline.0;
            pass.set_pipeline(&pipeline.pipeline);
            pass.set_vertex_buffer(0, pipeline.vertex_buffer.slice(..));
            pass.set_vertex_buffer(1, pipeline.instance_buffer.slice(..));
            pass.draw(0..pipeline.n_vertices, 0..pipeline.n_instances);
        }

        //render ellipsoid16s
        if let Some(ellipsoid_pipeline) = storage.get::<Ellipsoid16Pipeline>() {
            let pipeline = &ellipsoid_pipeline.0;
            pass.set_pipeline(&pipeline.pipeline);
            pass.set_vertex_buffer(0, pipeline.vertex_buffer.slice(..));
            pass.set_vertex_buffer(1, pipeline.instance_buffer.slice(..));
            pass.draw(0..pipeline.n_vertices, 0..pipeline.n_instances);
        }

        //render ellipsoid32s
        if let Some(ellipsoid_pipeline) = storage.get::<Ellipsoid32Pipeline>() {
            let pipeline = &ellipsoid_pipeline.0;
            pass.set_pipeline(&pipeline.pipeline);
            pass.set_vertex_buffer(0, pipeline.vertex_buffer.slice(..));
            pass.set_vertex_buffer(1, pipeline.instance_buffer.slice(..));
            pass.draw(0..pipeline.n_vertices, 0..pipeline.n_instances);
        }

        //render ellipsoid64s
        if let Some(ellipsoid_pipeline) = storage.get::<Ellipsoid64Pipeline>() {
            let pipeline = &ellipsoid_pipeline.0;
            pass.set_pipeline(&pipeline.pipeline);
            pass.set_vertex_buffer(0, pipeline.vertex_buffer.slice(..));
            pass.set_vertex_buffer(1, pipeline.instance_buffer.slice(..));
            pass.draw(0..pipeline.n_vertices, 0..pipeline.n_instances);
        }

        if let Some(pipeline) = storage.get::<EarthPipeline>() {
            if let Some(earth_bind_group) = storage.get::<EarthBindGroup>() {
                pass.set_bind_group(1, &earth_bind_group.0, &[]); // textures saved in bing group 1
                pass.set_pipeline(&pipeline.pipeline);
                pass.set_vertex_buffer(0, pipeline.vertex_buffer.slice(..));
                pass.set_vertex_buffer(1, pipeline.instance_buffer.slice(..));
                pass.draw(0..pipeline.n_vertices, 0..pipeline.n_instances);
            }
        }

        // need to render earth first so atmosphere pipeline knows how to blend
        if let Some(atmosphere_pipeline) = storage.get::<AtmospherePipeline>() {
            let pipeline = &atmosphere_pipeline.pipeline;
            pass.set_pipeline(pipeline);
            pass.set_vertex_buffer(0, atmosphere_pipeline.vertex_buffer.slice(..));
            pass.set_vertex_buffer(1, atmosphere_pipeline.instance_buffer.slice(..));
            pass.draw(
                0..atmosphere_pipeline.n_vertices,
                0..atmosphere_pipeline.n_instances,
            );
        }
    }
}

impl Program<Message> for Scene {
    type State = SceneState;
    type Primitive = ScenePrimitive;

    // Required method
    fn draw(&self, _state: &Self::State, _cursor: Cursor, bounds: Rectangle) -> Self::Primitive {
        ScenePrimitive::new(&self, bounds)
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

fn load_texture(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    texture_bytes: &[u8],
    label: &str,
) -> wgpu::TextureView {
    let img = load_from_memory(texture_bytes).expect("Failed to load image from embedded bytes");
    let rgba = img.to_rgba8();
    let dimensions = img.dimensions();

    let texture_size = wgpu::Extent3d {
        width: dimensions.0,
        height: dimensions.1,
        depth_or_array_layers: 1,
    };

    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some(label),
        size: texture_size,
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8UnormSrgb,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });

    queue.write_texture(
        wgpu::ImageCopyTexture {
            texture: &texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        &rgba,
        wgpu::ImageDataLayout {
            offset: 0,
            bytes_per_row: Some(4 * dimensions.0),
            rows_per_image: Some(dimensions.1),
        },
        texture_size,
    );

    let texture_view = texture.create_view(&wgpu::TextureViewDescriptor::default());

    texture_view
}
