use glam::{DVec3, Vec3};
use iced::{
    advanced::Shell,
    mouse::{self, Cursor, Interaction},
    widget::{
        canvas::event::Status,
        shader::{
            wgpu::{self, util::DeviceExt},
            Event, Primitive, Program, Storage, Viewport,
        },
    },
    Color, Point, Rectangle, Size,
};
use nadir_3d::{
    geometry::{
        cuboid::Cuboid,
        ellipsoid::{Ellipsoid16, Ellipsoid32, Ellipsoid64},
        Geometry,
    },
    mesh::{Mesh, MeshGpu, MeshPrimitive},
};

use image::{load_from_memory, GenericImageView};

pub mod camera;
pub mod earth_pipeline;
pub mod pipeline;
pub mod sun_pipeline;

use camera::Camera;
use earth_pipeline::{AtmospherePipeline, EarthBindGroup, EarthPipeline};
use pipeline::{
    uniforms::Uniforms, CuboidPipeline, Ellipsoid16Pipeline, Ellipsoid32Pipeline,
    Ellipsoid64Pipeline, Pipeline,
};
use sun_pipeline::{CoronaPipeline, SunPipeline};

use crate::{
    celestial::{CelestialAnimation, CelestialMeshes, CelestialPrimitives},
    Message,
};

#[derive(Debug)]
pub struct Scene {
    pub camera: Camera,
    light_color: Color,
    light_pos: [f64; 3],
    pub body_meshes: Vec<Mesh>,
    pub celestial: CelestialAnimation,
    pub world_target: Option<usize>, // Some(index into meshes), None is the origin
    pub sample_count: u32,
}

impl Default for Scene {
    fn default() -> Self {
        Self {
            camera: Camera::default(),
            light_color: Color::WHITE,
            light_pos: [0.0, 10.0, 0.0],
            body_meshes: Vec::new(),
            celestial: CelestialAnimation::default(),
            world_target: None,
            sample_count: 1,
        }
    }
}
impl Scene {
    pub fn set_celestial(&mut self) {
        self.world_target = if !self.body_meshes.is_empty() {
            Some(0)
        } else {
            None
        };
        let world_target = if let Some(index) = self.world_target {
            self.body_meshes[index].state.position
        } else {
            DVec3::ZERO
        };

        // convert all positions to target frame
        self.body_meshes
            .iter_mut()
            .for_each(|mesh| mesh.set_position_from_target(world_target));

        for (_, mesh) in &mut self.celestial.meshes {
            mesh.set_position_from_target(world_target);
        }

        // sun must be here for celestial system, so unwrap makes sense. need panic if it's not there
        let sun = self.celestial.meshes.get(&CelestialMeshes::Sun).unwrap();
        let sun_position = sun.state.position;

        self.light_pos = sun_position.into();
        self.light_color = Color::new(1.0, 1.0, 1.0, 1.0);

        let unit = world_target.normalize();
        let camera_position = Vec3::new(
            10.0 * unit[0] as f32,
            10.0 * unit[1] as f32,
            10.0 * unit[2] as f32,
        );
        self.camera.set_position(camera_position);
        self.camera.set_target(Vec3::ZERO);
        self.camera.set_far(1.1e13); //should cover to the heliopause,
        self.camera.set_fov(45.0);
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
struct UniformBindGroupLayout(wgpu::BindGroupLayout);

#[derive(Debug)]
struct PipelineLayout(wgpu::PipelineLayout);

#[derive(Debug)]
pub struct ScenePrimitive {
    meshes: Vec<MeshPrimitive>,
    celestial: CelestialPrimitives,
    uniforms: Uniforms,
    sample_count: u32,
}

impl ScenePrimitive {
    pub fn new(scene: &Scene, bounds: Rectangle) -> Self {
        let mut meshes = Vec::new();
        scene
            .body_meshes
            .iter()
            .for_each(|mesh| meshes.push(MeshPrimitive::from(mesh)));

        let celestial = CelestialPrimitives::from(&scene.celestial);
        let uniforms = Uniforms::new(scene, bounds);

        Self {
            meshes,
            celestial,
            uniforms,
            sample_count: scene.sample_count,
        }
    }
}

impl Primitive for ScenePrimitive {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        format: wgpu::TextureFormat,
        storage: &mut Storage,
        _bounds: &Rectangle,
        viewport: &Viewport,
    ) {
        // a new sceneprimitive is created each animation tick with updated values
        // this creates and stores bindgroups in storage on the first call to prepare
        // otherwise just updates the values in storage

        let current_viewport_size = viewport.physical_size();
        let resize = match storage.get::<Size<u32>>() {
            Some(size) if *size != current_viewport_size => {
                storage.store(current_viewport_size);
                true
            }
            None => {
                storage.store(current_viewport_size);
                false
            }
            _ => false,
        };

        // create and store the uniform bindgroup layout once
        // doesnt need to be updated but is used by other bindgroups
        if !storage.has::<UniformBindGroupLayout>() {
            let uniform_bind_group_layout =
                device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                    label: Some("uniform.bind.group.layout"),
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
            storage.store(UniformBindGroupLayout(uniform_bind_group_layout));
        }

        //Create and store the uniform bindgroup
        if !storage.has::<UniformBindGroup>() {
            let uniforms = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("uniform buffer"),
                contents: bytemuck::bytes_of(&self.uniforms),
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            });
            let uniform_bind_group_layout = &storage.get::<UniformBindGroupLayout>().unwrap().0;
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
        if !storage.has::<DepthView>() || resize {
            let depth_texture = device.create_texture(&wgpu::TextureDescriptor {
                label: Some("depth.texture"),
                size: wgpu::Extent3d {
                    width: current_viewport_size.width,
                    height: current_viewport_size.height,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: self.sample_count,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Depth32Float,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                    | wgpu::TextureUsages::TEXTURE_BINDING,
                view_formats: &[],
            });

            let depth_view = depth_texture.create_view(&wgpu::TextureViewDescriptor::default());
            storage.store(DepthView(depth_view));
        }

        if !storage.has::<MultisampleView>() || resize {
            // create the antialiasing textureview
            let multisampled_texture = device.create_texture(&wgpu::TextureDescriptor {
                label: Some("multisampled.color.texture"),
                size: wgpu::Extent3d {
                    width: current_viewport_size.width,
                    height: current_viewport_size.height,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: self.sample_count, // Set the multisample count here to match the pipeline
                dimension: wgpu::TextureDimension::D2,
                format,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
                view_formats: &[],
            });
            let multisampled_texture_view =
                multisampled_texture.create_view(&wgpu::TextureViewDescriptor::default());
            storage.store(MultisampleView(multisampled_texture_view));
        }

        const MAIN_SHADER: &str = include_str!("../scene/shaders/main.wgsl");

        if !storage.has::<PipelineLayout>() {
            let uniform_bind_group_layout = &storage.get::<UniformBindGroupLayout>().unwrap().0;
            // create all pipelines for each geometry present in meshes
            let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("pipeline layout"),
                bind_group_layouts: &[&uniform_bind_group_layout],
                push_constant_ranges: &[],
            });
            storage.store(PipelineLayout(layout));
        }

        if let Some(atmosphere) = self.celestial.meshes.get(&CelestialMeshes::EarthAtmosphere) {
            // atmosphere
            if !storage.has::<AtmospherePipeline>() {
                let layout = &storage.get::<PipelineLayout>().unwrap().0;
                storage.store(AtmospherePipeline::new(
                    device,
                    format,
                    &layout,
                    &[atmosphere.mesh_gpu],
                    Ellipsoid64::vertices(),
                    self.sample_count,
                ));
            } else {
                if let Some(atmosphere_pipeline) = storage.get_mut::<AtmospherePipeline>() {
                    atmosphere_pipeline.update(queue, &[atmosphere.mesh_gpu]);
                }
            }
        }

        //earth
        if let Some(earth) = self.celestial.meshes.get(&CelestialMeshes::Earth) {
            if !storage.has::<EarthPipeline>() {
                const EARTH_COLOR: &[u8] = include_bytes!("../../../resources/earth_color_8K.tif");
                const EARTH_NIGHT: &[u8] =
                    include_bytes!("../../../resources/earth_nightlights_10K.tif");
                const EARTH_CLOUDS: &[u8] =
                    include_bytes!("../../../resources/earth_clouds_8K.tif");
                const EARTH_SPEC: &[u8] = include_bytes!("../../../resources/earth_spec_8k.tif");
                // const EARTH_TOPO: &[u8] = include_bytes!("../../resources/earth_topography_5k.png");

                let earth_day = load_texture(device, queue, EARTH_COLOR, "earth_color");
                let earth_night = load_texture(device, queue, EARTH_NIGHT, "earth_night");
                let earth_spec = load_texture(device, queue, EARTH_SPEC, "earth_spec");
                let earth_clouds = load_texture(device, queue, EARTH_CLOUDS, "earth_clouds");
                // let earth_topography = load_texture(device, queue, EARTH_TOPO, "earth_topography");

                let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
                    address_mode_u: wgpu::AddressMode::Repeat,
                    address_mode_v: wgpu::AddressMode::ClampToEdge,
                    address_mode_w: wgpu::AddressMode::ClampToEdge,
                    mag_filter: wgpu::FilterMode::Linear,
                    min_filter: wgpu::FilterMode::Linear,
                    mipmap_filter: wgpu::FilterMode::Linear,
                    anisotropy_clamp: 2,
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
                            //earth topography
                            // wgpu::BindGroupLayoutEntry {
                            //     binding: 5,
                            //     visibility: wgpu::ShaderStages::FRAGMENT,
                            //     ty: wgpu::BindingType::Texture {
                            //         sample_type: wgpu::TextureSampleType::Float {
                            //             filterable: true,
                            //         },
                            //         view_dimension: wgpu::TextureViewDimension::D2,
                            //         multisampled: false,
                            //     },
                            //     count: None,
                            // },
                        ],
                    });

                let earth_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("earth.bind.group"),
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
                        // wgpu::BindGroupEntry {
                        //     binding: 5,
                        //     resource: wgpu::BindingResource::TextureView(&earth_topography),
                        // },
                    ],
                });

                storage.store(EarthBindGroup(earth_bind_group));

                let uniform_bind_group_layout = &storage.get::<UniformBindGroupLayout>().unwrap().0;
                let earth_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                    label: Some("earth.pipeline.layout"),
                    bind_group_layouts: &[&uniform_bind_group_layout, &earth_bind_group_layout],
                    push_constant_ranges: &[],
                });

                storage.store(EarthPipeline::new(
                    device,
                    format,
                    &earth_layout,
                    &[earth.mesh_gpu],
                    Ellipsoid64::vertices(),
                    self.sample_count,
                ));
            } else {
                if let Some(earth_pipeline) = storage.get_mut::<EarthPipeline>() {
                    earth_pipeline.update(queue, &[earth.mesh_gpu]);
                }
            }
        }

        if let Some(sun) = self.celestial.meshes.get(&CelestialMeshes::Sun) {
            if !storage.has::<SunPipeline>() {
                let layout = &storage.get::<PipelineLayout>().unwrap().0;
                storage.store(SunPipeline::new(
                    device,
                    format,
                    &layout,
                    &[sun.mesh_gpu],
                    Ellipsoid64::vertices(),
                    self.sample_count,
                ));
            } else {
                if let Some(sun_pipeline) = storage.get_mut::<SunPipeline>() {
                    sun_pipeline.update(queue, &[sun.mesh_gpu]);
                }
            }
        }

        if let Some(corona) = self.celestial.meshes.get(&CelestialMeshes::SunCorona) {
            if !storage.has::<CoronaPipeline>() {
                let layout = &storage.get::<PipelineLayout>().unwrap().0;
                storage.store(CoronaPipeline::new(
                    device,
                    format,
                    &layout,
                    &[corona.mesh_gpu],
                    Ellipsoid64::vertices(),
                    self.sample_count,
                ));
            } else {
                if let Some(corona_pipeline) = storage.get_mut::<CoronaPipeline>() {
                    corona_pipeline.update(queue, &[corona.mesh_gpu]);
                }
            }
        }

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
                let layout = &storage.get::<PipelineLayout>().unwrap().0;
                storage.store(CuboidPipeline(Pipeline::new(
                    device,
                    format,
                    &layout,
                    MAIN_SHADER,
                    "cuboid",
                    Cuboid::vertices(),
                    &cuboids,
                    self.sample_count,
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
                let layout = &storage.get::<PipelineLayout>().unwrap().0;
                storage.store(Ellipsoid16Pipeline(Pipeline::new(
                    device,
                    format,
                    &layout,
                    MAIN_SHADER,
                    "ellipsoid16",
                    Ellipsoid16::vertices(),
                    &ellipsoid16s,
                    self.sample_count,
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
                let layout = &storage.get::<PipelineLayout>().unwrap().0;
                storage.store(Ellipsoid32Pipeline(Pipeline::new(
                    device,
                    format,
                    &layout,
                    MAIN_SHADER,
                    "ellipsoid32",
                    Ellipsoid32::vertices(),
                    &ellipsoid32s,
                    self.sample_count,
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

        // push celestial objects as well to ellipsoid64s
        // for (body, mesh) in &self.celestial.meshes {
        //     match body {
        //         CelestialMeshes::Earth | CelestialMeshes::EarthAtmosphere => {
        //             //they have their own pipelines, dont add to ellipsoid64 pipeline
        //         }
        //         _ => ellipsoid64s.push(mesh.mesh_gpu),
        //     }
        // }

        if !ellipsoid64s.is_empty() {
            if !storage.has::<Ellipsoid64Pipeline>() {
                let layout = &storage.get::<PipelineLayout>().unwrap().0;
                storage.store(Ellipsoid64Pipeline(Pipeline::new(
                    device,
                    format,
                    &layout,
                    MAIN_SHADER,
                    "ellipsoid64",
                    Ellipsoid64::vertices(),
                    &ellipsoid64s,
                    self.sample_count,
                )));
            } else {
                if let Some(ellipsoid64_pipeline) = storage.get_mut::<Ellipsoid64Pipeline>() {
                    let pipeline = &mut ellipsoid64_pipeline.0;
                    pipeline.update(queue, &ellipsoid64s);
                }
            }
        }
    }

    fn render(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        storage: &Storage,
        target: &wgpu::TextureView,
        viewport: &Rectangle<u32>,
    ) {
        // unpack the depth_view and uniform_bind_group from storage
        let depth_view = &storage.get::<DepthView>().unwrap().0;
        let multisample_view = &storage.get::<MultisampleView>().unwrap().0;
        let uniform_bind_group = &storage.get::<UniformBindGroup>().unwrap().0;

        let (view, resolve_target) = match self.sample_count {
            1 => (target, None),
            4 => (multisample_view, Some(target)),
            _ => unreachable!("needs to be 1 or 4, error should be thrown earlier"),
        };

        // set up the render pass
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("nadir.pipeline.pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view,
                resolve_target,
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

        if let Some(corona_pipeline) = storage.get::<CoronaPipeline>() {
            let pipeline = &corona_pipeline.pipeline;
            pass.set_pipeline(pipeline);
            pass.set_vertex_buffer(0, corona_pipeline.vertex_buffer.slice(..));
            pass.set_vertex_buffer(1, corona_pipeline.instance_buffer.slice(..));
            pass.draw(
                0..corona_pipeline.n_vertices,
                0..corona_pipeline.n_instances,
            );
        }

        if let Some(sun_pipeline) = storage.get::<SunPipeline>() {
            let pipeline = &sun_pipeline.pipeline;
            pass.set_pipeline(pipeline);
            pass.set_vertex_buffer(0, sun_pipeline.vertex_buffer.slice(..));
            pass.set_vertex_buffer(1, sun_pipeline.instance_buffer.slice(..));
            pass.draw(0..sun_pipeline.n_vertices, 0..sun_pipeline.n_instances);
        }

        // render atmosphere first so its always covered
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

        if let Some(pipeline) = storage.get::<EarthPipeline>() {
            if let Some(earth_bind_group) = storage.get::<EarthBindGroup>() {
                pass.set_bind_group(1, &earth_bind_group.0, &[]); // textures saved in bing group 1
                pass.set_pipeline(&pipeline.pipeline);
                pass.set_vertex_buffer(0, pipeline.vertex_buffer.slice(..));
                pass.set_vertex_buffer(1, pipeline.instance_buffer.slice(..));
                pass.draw(0..pipeline.n_vertices, 0..pipeline.n_instances);
            }
        }

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
