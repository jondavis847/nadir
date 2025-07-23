use glam::Mat4;
use nadir_3d::vertex::SimpleVertex;
use std::collections::HashMap;

use crate::{GeometryId, GpuGeometryResources, SceneBounds, SharedUniforms};

pub struct SurfaceAreaInitialized {
    render_pipeline: wgpu::RenderPipeline,
    result_buffer: wgpu::Buffer,
}

pub struct SurfaceAreaCalculator {
    initialized: Option<SurfaceAreaInitialized>,
}

impl SurfaceAreaCalculator {
    pub fn new() -> Self {
        Self { initialized: None }
    }

    pub fn initialize(
        &mut self,
        device: &wgpu::Device,
        shared_bind_group_layout: &wgpu::BindGroupLayout,
        geometry_bind_group_layout: &wgpu::BindGroupLayout,
        resolution: u32,
    ) {
        const SHADER: &str = include_str!("surface_area.wgsl");
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Surface Area Shader"),
            source: wgpu::ShaderSource::Wgsl(SHADER.into()),
        });

        // Add push constants to pipeline layout
        let render_pipeline_layout = device.create_pipeline_layout(
            &wgpu::PipelineLayoutDescriptor {
                label: Some("Surface Area Pipeline Layout"),
                bind_group_layouts: &[
                    shared_bind_group_layout,   // Bind group 0: shared data
                    geometry_bind_group_layout, // Bind group 1: per-geometry data
                ],
                push_constant_ranges: &[],
            },
        );

        // Calculate aligned buffer size
        let unaligned = resolution * 4; // 4 bytes per pixel
        let alignment = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
        let bytes_per_row = ((unaligned + alignment - 1) / alignment) * alignment;
        let buffer_size = (bytes_per_row * resolution) as u64;

        // Create buffer to read back results
        let result_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Surface Area Readback Buffer"),
            size: buffer_size,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        const ATTRIBS: [wgpu::VertexAttribute; 1] = wgpu::vertex_attr_array![
            //position
            0 => Float32x3,
        ];

        // created this here since geometry crate uses iced::wgpu which is a different version
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
                multisample: wgpu::MultisampleState::default(),
                multiview: None,
                cache: None,
            },
        );

        self.initialized = Some(SurfaceAreaInitialized { render_pipeline, result_buffer });
    }

    pub fn calculate(
        &mut self, // Changed to &mut since we need to initialize
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        geometry_resources: &HashMap<GeometryId, GpuGeometryResources>, // Use GPU resources
        scene_bounds: &SceneBounds,
        view_direction: &[f32; 3],
        resolution: u32,
        safety_factor: f32,
        shared_buffer: &wgpu::Buffer,
        shared_bindgroup: &wgpu::BindGroup,
        object_id_view: &wgpu::TextureView,
        depth_view: &wgpu::TextureView,
    ) -> Vec<f32> {
        if let Some(initialized) = &self.initialized {
            // Create orthographic projection matrix
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

            // Begin render pass
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
                                load: wgpu::LoadOp::Clear(wgpu::Color::BLACK), // Clear to 0 (no object)
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
                // Set shared bind group once
                render_pass.set_bind_group(0, shared_bindgroup, &[]);

                // Render each geometry with its resources
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

            // Copy render target to readable buffer
            let unaligned = resolution * 4;
            let alignment = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
            let bytes_per_row = ((unaligned + alignment - 1) / alignment) * alignment;

            encoder.copy_texture_to_buffer(
                wgpu::TexelCopyTextureInfo {
                    texture: object_id_view.texture(), // Fixed: use the texture from the view
                    mip_level: 0,
                    origin: wgpu::Origin3d::ZERO,
                    aspect: wgpu::TextureAspect::All,
                },
                wgpu::TexelCopyBufferInfo {
                    buffer: &initialized.result_buffer,
                    layout: wgpu::TexelCopyBufferLayout {
                        offset: 0,
                        bytes_per_row: Some(bytes_per_row),
                        rows_per_image: Some(resolution),
                    },
                },
                wgpu::Extent3d {
                    width: resolution,
                    height: resolution,
                    depth_or_array_layers: 1,
                },
            );

            queue.submit(Some(encoder.finish()));

            // Read back pixel data and calculate areas
            let pixel_data = self.read_buffer_data(
                device,
                &initialized.result_buffer,
                resolution,
            );

            // Print pixel data as a grid for debugging
            println!(
                "Pixel data as {}x{} grid:",
                resolution, resolution
            );
            println!("(0 = background, >0 = object ID)");
            println!();

            for y in 0..resolution {
                for x in 0..resolution {
                    let idx = (y * resolution + x) as usize;
                    if idx < pixel_data.len() {
                        print!("{:3} ", pixel_data[idx]);
                    } else {
                        print!("??? ");
                    }
                }
                println!(); // New line after each row
            }

            self.calculate_areas_from_pixels(
                pixel_data,
                geometry_resources.len(),
                *scene_bounds,
                resolution,
                safety_factor,
            )
        } else {
            panic!("SurfaceAreaCalculator failed to initialize");
        }
    }

    fn read_buffer_data(
        &self,
        device: &wgpu::Device,
        buffer: &wgpu::Buffer,
        resolution: u32,
    ) -> Vec<u32> {
        let buffer_slice = buffer.slice(..);

        // Set up a channel to receive mapping result
        let (sender, receiver) = futures::channel::oneshot::channel();

        // Map async, send result on channel when ready
        buffer_slice.map_async(
            wgpu::MapMode::Read,
            move |result| {
                sender
                    .send(result)
                    .unwrap();
            },
        );

        // Drive the device so the mapping can progress
        device
            .poll(wgpu::PollType::Wait)
            .expect("polling error");

        // Wait (blocking) until mapping is complete
        pollster::block_on(async {
            receiver
                .await
                .expect("failed to receive mapping result")
                .expect("failed to map buffer");
        });

        // Handle aligned rows properly
        let unaligned = resolution * 4; // 4 bytes per pixel
        let alignment = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
        let bytes_per_row = ((unaligned + alignment - 1) / alignment) * alignment;

        // Create a properly sized result without padding
        let mut result = Vec::with_capacity((resolution * resolution) as usize);

        let mapped_data = buffer_slice.get_mapped_range();
        let bytes = mapped_data.as_ref();

        // Extract just the actual pixels, skipping padding
        for y in 0..resolution {
            let row_start = (y * bytes_per_row) as usize; // bytes_per_row is in bytes, not u32s
            for x in 0..resolution {
                let pixel_offset = row_start + (x * 4) as usize; // 4 bytes per u32 pixel

                // Convert 4 bytes to u32 (little-endian)
                let pixel_bytes = [
                    bytes[pixel_offset],
                    bytes[pixel_offset + 1],
                    bytes[pixel_offset + 2],
                    bytes[pixel_offset + 3],
                ];
                let pixel_value = u32::from_le_bytes(pixel_bytes);
                result.push(pixel_value);
            }
        }

        drop(mapped_data);
        buffer.unmap();

        result
    }

    fn calculate_areas_from_pixels(
        &self,
        pixels: Vec<u32>,
        num_objects: usize,
        scene_bounds: SceneBounds,
        resolution: u32,
        scale_factor: f32,
    ) -> Vec<f32> {
        let mut pixel_counts = vec![0u32; num_objects];

        // Count pixels per object ID
        for pixel in pixels {
            if pixel > 0 && pixel <= num_objects as u32 {
                pixel_counts[(pixel - 1) as usize] += 1;
            }
        }

        // Convert pixel counts to surface area
        let area_per_pixel = Self::calculate_area_per_pixel(
            scene_bounds,
            resolution,
            scale_factor,
        );

        pixel_counts
            .iter()
            .map(|&count| count as f32 * area_per_pixel)
            .collect()
    }

    fn calculate_area_per_pixel(
        scene_bounds: SceneBounds,
        resolution: u32,
        scale_factor: f32,
    ) -> f32 {
        // Calculate the world space area that each pixel represents
        let world_width = (scene_bounds.max_x - scene_bounds.min_x) * scale_factor;
        let world_height = (scene_bounds.max_y - scene_bounds.min_y) * scale_factor;

        // Each pixel represents this much world area
        let pixel_world_width = world_width / resolution as f32;
        let pixel_world_height = world_height / resolution as f32;

        pixel_world_width * pixel_world_height
    }
}
