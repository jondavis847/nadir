use glam::Mat4;
use nadir_3d::vertex::SimpleVertex;
use std::{collections::HashMap, f64::NAN};

use crate::{
    GeometryId, GpuGeometryResources, SceneBounds, SharedUniforms,
    parallel_reduction::ParallelReduction,
};

pub struct SurfaceAreaInitialized {
    render_pipeline: wgpu::RenderPipeline,
    parallel_reduction: ParallelReduction,
}

pub struct SurfaceAreaCalculator {
    initialized: Option<SurfaceAreaInitialized>,
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
                    module: &shader,
                    entry_point: Some("fs_main"),
                    targets: &[Some(wgpu::ColorTargetState {
                        format: wgpu::TextureFormat::R32Uint,
                        blend: None,
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
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
                    color_attachments: &[Some(
                        wgpu::RenderPassColorAttachment {
                            view: object_id_view,
                            resolve_target: None,
                            ops: wgpu::Operations {
                                load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                                store: wgpu::StoreOp::Store,
                            },
                            depth_slice: None,
                        },
                    )],
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

            initialized
                .parallel_reduction
                .dispatch(device, queue);

            let results = &initialized
                .parallel_reduction
                .result;

            dbg!(results);
            let area_per_pixel = Self::calculate_area_per_pixel(
                scene_bounds,
                resolution,
                safety_factor,
            );

            for result in results {
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

    fn calculate_area_per_pixel(
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
}
