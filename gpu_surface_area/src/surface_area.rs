use glam::Mat4;
use nadir_3d::vertex::SimpleVertex;
use std::{collections::HashMap, f64::NAN};

use crate::{
    GeometryId, GpuGeometryResources, SceneBounds, SharedUniforms,
    parallel_reduction::ParallelReduction,
};

pub struct SurfaceAreaInitialized {
    render_pipeline: wgpu::RenderPipeline,
    pub parallel_reduction: ParallelReduction,
}

pub struct SurfaceAreaCalculator {
    pub initialized: Option<SurfaceAreaInitialized>,
    pub aerodynamics_result: HashMap<GeometryId, AerodynamicsResult>,
}

#[derive(Debug)]
pub struct AerodynamicsResult {
    pub surface_area: f64,
    pub center_of_pressure: [f64; 3],
}

impl SurfaceAreaCalculator {
    pub fn new() -> Self {
        Self { initialized: None, aerodynamics_result: HashMap::new() }
    }

    pub fn initialize(
        &mut self,
        device: &wgpu::Device,
        shared_bind_group_layout: &wgpu::BindGroupLayout,
        geometry_bind_group_layout: &wgpu::BindGroupLayout,
        n_objects: usize,
        object_id_view: &wgpu::TextureView,
        position_view: &wgpu::TextureView,
    ) {
        const SHADER: &str = include_str!("surface_area.wgsl");
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Surface Area Shader"),
            source: wgpu::ShaderSource::Wgsl(SHADER.into()),
        });

        let render_pipeline_layout = device.create_pipeline_layout(
            &wgpu::PipelineLayoutDescriptor {
                label: Some("Surface Area Pipeline Layout"),
                bind_group_layouts: &[shared_bind_group_layout, geometry_bind_group_layout],
                push_constant_ranges: &[],
            },
        );

        const ATTRIBS: [wgpu::VertexAttribute; 1] = wgpu::vertex_attr_array![
            0 => Float32x3,
        ];

        let vertex_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<SimpleVertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &ATTRIBS,
        };

        let render_pipeline = device.create_render_pipeline(
            &wgpu::RenderPipelineDescriptor {
                label: Some("Surface Area Pipeline"),
                layout: Some(&render_pipeline_layout),
                vertex: wgpu::VertexState {
                    module: &shader,
                    entry_point: Some("vs_main"),
                    buffers: &[vertex_layout],
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                },
                fragment: Some(wgpu::FragmentState {
                    // object id texture
                    module: &shader,
                    entry_point: Some("fs_main"),
                    targets: &[
                        Some(wgpu::ColorTargetState {
                            format: wgpu::TextureFormat::R32Uint,
                            blend: None,
                            write_mask: wgpu::ColorWrites::ALL,
                        }),
                        Some(wgpu::ColorTargetState {
                            // Position texture
                            format: wgpu::TextureFormat::Rgba32Float,
                            blend: None,
                            write_mask: wgpu::ColorWrites::ALL,
                        }),
                    ],
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                }),
                primitive: wgpu::PrimitiveState {
                    topology: wgpu::PrimitiveTopology::TriangleList,
                    strip_index_format: None,
                    front_face: wgpu::FrontFace::Ccw,
                    cull_mode: Some(wgpu::Face::Back),
                    polygon_mode: wgpu::PolygonMode::Fill,
                    unclipped_depth: false,
                    conservative: false,
                },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: wgpu::TextureFormat::Depth32Float,
                    depth_write_enabled: true,
                    depth_compare: wgpu::CompareFunction::Less,
                    stencil: wgpu::StencilState::default(),
                    bias: wgpu::DepthBiasState::default(),
                }),
                multisample: wgpu::MultisampleState {
                    count: 1,
                    mask: !0,
                    alpha_to_coverage_enabled: false,
                },
                multiview: None,
                cache: None,
            },
        );

        let parallel_reduction = ParallelReduction::new(
            device,
            object_id_view,
            position_view,
            n_objects as u32,
        );

        self.initialized = Some(SurfaceAreaInitialized { render_pipeline, parallel_reduction });
    }

    pub fn calculate(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        geometry_resources: &HashMap<GeometryId, GpuGeometryResources>,
        scene_bounds: &SceneBounds,
        view_direction: &[f32; 3],
        resolution: u32,
        safety_factor: f32,
        shared_buffer: &wgpu::Buffer,
        shared_bindgroup: &wgpu::BindGroup,
        object_id_view: &wgpu::TextureView,
        position_view: &wgpu::TextureView,
        depth_view: &wgpu::TextureView,
    ) {
        // renders the geometry and stores calculated values as pixel data in the textures
        self.render(
            device,
            queue,
            geometry_resources,
            scene_bounds,
            view_direction,
            safety_factor,
            shared_buffer,
            shared_bindgroup,
            object_id_view,
            position_view,
            depth_view,
        );

        // uses parallel reduction to collect the texture values and reduce to per object values
        self.reduce(device, queue);

        // collects the final results and stores them in self.aerodynamics_result
        self.results(
            scene_bounds,
            resolution,
            safety_factor,
        );
    }

    pub fn calculate_area_per_pixel(
        scene_bounds: &SceneBounds,
        resolution: u32,
        safety_factor: f32,
    ) -> f32 {
        let world_width = (scene_bounds.max_y - scene_bounds.min_y) * safety_factor;
        let world_height = (scene_bounds.max_z - scene_bounds.min_z) * safety_factor;

        let pixel_world_width = world_width / resolution as f32;
        let pixel_world_height = world_height / resolution as f32;

        pixel_world_width * pixel_world_height
    }

    pub fn render(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        geometry_resources: &HashMap<GeometryId, GpuGeometryResources>,
        scene_bounds: &SceneBounds,
        view_direction: &[f32; 3],
        safety_factor: f32,
        shared_buffer: &wgpu::Buffer,
        shared_bindgroup: &wgpu::BindGroup,
        object_id_view: &wgpu::TextureView,
        position_view: &wgpu::TextureView,
        depth_view: &wgpu::TextureView,
    ) {
        if let Some(initialized) = &mut self.initialized {
            let projection_matrix = crate::create_orthographic_projection(
                scene_bounds,
                view_direction,
                safety_factor,
            );

            let shared_uniforms = SharedUniforms {
                projection_matrix: Mat4::from_cols_array_2d(&projection_matrix),
            };
            queue.write_buffer(
                shared_buffer,
                0,
                bytemuck::cast_slice(&[shared_uniforms]),
            );

            let mut encoder = device.create_command_encoder(
                &wgpu::CommandEncoderDescriptor { label: Some("Surface Area Render Encoder") },
            );

            {
                let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("Surface Area Render Pass"),
                    color_attachments: &[
                        Some(
                            wgpu::RenderPassColorAttachment {
                                view: object_id_view,
                                resolve_target: None,
                                ops: wgpu::Operations {
                                    load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                                    store: wgpu::StoreOp::Store,
                                },
                                depth_slice: None,
                            },
                        ),
                        Some(
                            wgpu::RenderPassColorAttachment {
                                view: position_view,
                                resolve_target: None,
                                ops: wgpu::Operations {
                                    load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                                    store: wgpu::StoreOp::Store,
                                },
                                depth_slice: None,
                            },
                        ),
                    ],
                    depth_stencil_attachment: Some(
                        wgpu::RenderPassDepthStencilAttachment {
                            view: depth_view,
                            depth_ops: Some(wgpu::Operations {
                                load: wgpu::LoadOp::Clear(1.0),
                                store: wgpu::StoreOp::Store,
                            }),
                            stencil_ops: None,
                        },
                    ),
                    timestamp_writes: None,
                    occlusion_query_set: None,
                });

                render_pass.set_pipeline(&initialized.render_pipeline);
                render_pass.set_bind_group(0, shared_bindgroup, &[]);

                for (_geometry_id, gpu_geometry) in geometry_resources {
                    render_pass.set_bind_group(
                        1,
                        &gpu_geometry.bind_group,
                        &[],
                    );
                    render_pass.set_vertex_buffer(
                        0,
                        gpu_geometry
                            .vertex_buffer
                            .slice(..),
                    );
                    render_pass.draw(
                        0..gpu_geometry.vertex_count,
                        0..1,
                    );
                }
            }
            queue.submit(Some(encoder.finish()));
            device
                .poll(wgpu::PollType::Wait)
                .unwrap();
        }
    }

    pub fn reduce(&mut self, device: &wgpu::Device, queue: &wgpu::Queue) {
        if let Some(initialized) = &mut self.initialized {
            initialized
                .parallel_reduction
                .dispatch(device, queue);
        }
    }

    pub fn results(&mut self, scene_bounds: &SceneBounds, resolution: u32, safety_factor: f32) {
        if let Some(init) = &self.initialized {
            let area_per_pixel = Self::calculate_area_per_pixel(
                scene_bounds,
                resolution,
                safety_factor,
            );

            for result in &init
                .parallel_reduction
                .result
            {
                let surface_area = (result.count as f32 * area_per_pixel) as f64;
                let cop = if result.count > 0 {
                    [
                        (result.pos[0] / result.count as f32) as f64,
                        (result.pos[1] / result.count as f32) as f64,
                        (result.pos[2] / result.count as f32) as f64,
                    ]
                } else {
                    [NAN; 3]
                };

                self.aerodynamics_result
                    .entry(GeometryId(result.id as usize))
                    .and_modify(|e| {
                        e.surface_area = surface_area;
                        e.center_of_pressure = cop;
                    })
                    .or_insert(AerodynamicsResult { surface_area, center_of_pressure: cop });
            }
        } else {
            panic!("SurfaceAreaCalculator failed to initialize");
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::GpuCalculator;
    use nadir_3d::geometry::{GeometryState, cuboid::Cuboid};
    use std::{f32::EPSILON, time::Instant};

    //    #[test]
    fn render_pixels_id() {
        let resolution = 64; // needs to be a multiple of 64 or bytes won't align (256 alignment - 64 u32 = 256)
        let mut gpu = GpuCalculator::new()
            .with_resolution(resolution)
            .with_surface_area();

        let cube = Cuboid::new(1.0, 1.0, 1.0).unwrap();
        let cube_state = GeometryState::default();
        gpu.add_geometry(cube.into(), &cube_state);

        gpu.initialize();
        gpu.calculate_scene_bounds();
        let view_direction = [-1.0, 0.0, 0.0];
        let start = Instant::now();
        if let Some(init) = &mut gpu.initialized {
            if let Some(area) = &mut gpu.surface_area {
                area.render(
                    &init.device,
                    &init.queue,
                    &init.geometry,
                    &gpu.scene_bounds,
                    &view_direction,
                    1.0,
                    &init.shared_uniform_buffer,
                    &init.shared_bind_group,
                    &init.object_id_texture,
                    &init.position_texture,
                    &init.depth_texture,
                );
            }
        }
        let stop = Instant::now();
        let duration = stop.duration_since(start);
        println!(
            "Surface area calculation time: {:?}",
            duration
        );

        // test specific gpu setup for copying results to cpu
        if let Some(init) = &gpu.initialized {
            let bytes_per_pixel = std::mem::size_of::<u32>() as u32;
            let staging_buffer = init
                .device
                .create_buffer(&wgpu::BufferDescriptor {
                    label: Some("test_staging"),
                    size: ((resolution * resolution) * bytes_per_pixel) as u64,
                    usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                    mapped_at_creation: false,
                });

            let mut encoder = init
                .device
                .create_command_encoder(
                    &wgpu::CommandEncoderDescriptor { label: Some("test encoder") },
                );

            encoder.copy_texture_to_buffer(
                wgpu::TexelCopyTextureInfoBase {
                    texture: init
                        .object_id_texture
                        .texture(),
                    mip_level: 0,
                    origin: wgpu::Origin3d::ZERO,
                    aspect: wgpu::TextureAspect::All,
                },
                wgpu::TexelCopyBufferInfoBase {
                    buffer: &staging_buffer,
                    layout: wgpu::TexelCopyBufferLayout {
                        offset: 0,
                        bytes_per_row: Some(resolution * bytes_per_pixel),
                        rows_per_image: Some(resolution),
                    },
                },
                wgpu::Extent3d {
                    width: resolution,
                    height: resolution,
                    depth_or_array_layers: 1,
                },
            );

            init.queue
                .submit(Some(encoder.finish()));
            init.device
                .poll(wgpu::PollType::Wait)
                .unwrap();

            let buffer_slice = staging_buffer.slice(..);
            let (tx, rx) = futures::channel::oneshot::channel();

            buffer_slice.map_async(
                wgpu::MapMode::Read,
                move |result| {
                    tx.send(result)
                        .unwrap();
                },
            );

            init.device
                .poll(wgpu::PollType::Wait)
                .unwrap();
            pollster::block_on(rx)
                .unwrap()
                .unwrap();

            let data = buffer_slice.get_mapped_range();
            let pixels: Vec<u32> = bytemuck::cast_slice(&data).to_vec();

            drop(data);
            staging_buffer.unmap();

            println!(
                "\nObject ID Texture Contents ({} x {}):",
                resolution, resolution
            );

            for i in 0..resolution as usize {
                for j in 0..resolution as usize {
                    let id = pixels[resolution as usize * i + j];
                    if id == 0 {
                        print!("{:>5} ", ".");
                    } else {
                        print!("{:>5} ", id);
                    };
                }
                println!("");
            }

            let mut id_counts = std::collections::HashMap::new();
            for &id in &pixels {
                *id_counts
                    .entry(id)
                    .or_insert(0) += 1;
            }

            println!("\nPixel Statistics:");
            for (id, count) in id_counts {
                println!(
                    "Object ID {}: {} pixels ({:.1}%)",
                    id,
                    count,
                    100.0 * count as f32 / (resolution * resolution) as f32
                );
            }
        }
    }

    //   #[test]
    fn render_pixels_position() {
        let resolution = 64; // needs to be a multiple of 64 or bytes won't align (256 alignment - 64 u32 = 256)
        let mut gpu = GpuCalculator::new()
            .with_resolution(resolution)
            .with_surface_area();

        let cube = Cuboid::new(1.0, 1.0, 1.0).unwrap();
        let cube_state = GeometryState::default();
        gpu.add_geometry(cube.into(), &cube_state);

        gpu.initialize();
        gpu.calculate_scene_bounds();
        let view_direction = [-1.0, 0.0, 0.0];
        let start = Instant::now();
        if let Some(init) = &mut gpu.initialized {
            if let Some(area) = &mut gpu.surface_area {
                area.render(
                    &init.device,
                    &init.queue,
                    &init.geometry,
                    &gpu.scene_bounds,
                    &view_direction,
                    1.0,
                    &init.shared_uniform_buffer,
                    &init.shared_bind_group,
                    &init.object_id_texture,
                    &init.position_texture,
                    &init.depth_texture,
                );
            }
        }
        let stop = Instant::now();
        let duration = stop.duration_since(start);
        println!(
            "Surface area calculation time: {:?}",
            duration
        );

        // test specific gpu setup for copying results to cpu
        if let Some(init) = &gpu.initialized {
            let bytes_per_pixel = std::mem::size_of::<[f32; 4]>() as u32;
            let staging_buffer = init
                .device
                .create_buffer(&wgpu::BufferDescriptor {
                    label: Some("test_staging"),
                    size: ((resolution * resolution) * bytes_per_pixel) as u64,
                    usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                    mapped_at_creation: false,
                });

            let mut encoder = init
                .device
                .create_command_encoder(
                    &wgpu::CommandEncoderDescriptor { label: Some("test encoder") },
                );

            encoder.copy_texture_to_buffer(
                wgpu::TexelCopyTextureInfoBase {
                    texture: init
                        .position_texture
                        .texture(),
                    mip_level: 0,
                    origin: wgpu::Origin3d::ZERO,
                    aspect: wgpu::TextureAspect::All,
                },
                wgpu::TexelCopyBufferInfoBase {
                    buffer: &staging_buffer,
                    layout: wgpu::TexelCopyBufferLayout {
                        offset: 0,
                        bytes_per_row: Some(resolution * bytes_per_pixel),
                        rows_per_image: Some(resolution),
                    },
                },
                wgpu::Extent3d {
                    width: resolution,
                    height: resolution,
                    depth_or_array_layers: 1,
                },
            );

            init.queue
                .submit(Some(encoder.finish()));
            init.device
                .poll(wgpu::PollType::Wait)
                .unwrap();

            let buffer_slice = staging_buffer.slice(..);
            let (tx, rx) = futures::channel::oneshot::channel();

            buffer_slice.map_async(
                wgpu::MapMode::Read,
                move |result| {
                    tx.send(result)
                        .unwrap();
                },
            );

            init.device
                .poll(wgpu::PollType::Wait)
                .unwrap();
            pollster::block_on(rx)
                .unwrap()
                .unwrap();

            let data = buffer_slice.get_mapped_range();
            let pixels: Vec<[f32; 4]> = bytemuck::cast_slice(&data).to_vec();

            drop(data);
            staging_buffer.unmap();

            for i in 0..resolution as usize {
                for j in 0..resolution as usize {
                    let pos_x = pixels[resolution as usize * i + j][0];
                    if pos_x.abs() < EPSILON {
                        print!("{:>5.2} ", ".");
                    } else {
                        print!("{:>5.2} ", pos_x);
                    };
                }
                println!("");
            }
            println!("");
            println!("");

            for i in 0..resolution as usize {
                for j in 0..resolution as usize {
                    let pos_y = pixels[resolution as usize * i + j][1];
                    if pos_y.abs() < EPSILON {
                        print!("{:>5.2} ", ".");
                    } else {
                        print!("{:>5.2} ", pos_y);
                    };
                }
                println!("");
            }

            println!("");
            println!("");

            for i in 0..resolution as usize {
                for j in 0..resolution as usize {
                    let pos_z = pixels[resolution as usize * i + j][2];
                    if pos_z.abs() < EPSILON {
                        print!("{:>5.2} ", ".");
                    } else {
                        print!("{:>5.2} ", pos_z);
                    };
                }
                println!("");
            }
        }
    }
}
