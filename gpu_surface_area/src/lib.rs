use glam::{Mat4, Vec3};
use nadir_3d::{
    geometry::{Geometry, GeometryTrait},
    vertex::{SimpleVertex, simple_vertices},
};
use wgpu::util::DeviceExt;

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct Uniforms {
    projection_matrix: [[f32; 4]; 4],
    object_id: u32,
    _padding: [u32; 3], // Align to 16 bytes
}
pub struct SurfaceAreaCalculator {
    render_pipeline: wgpu::RenderPipeline,
    texture: wgpu::Texture,
    texture_view: wgpu::TextureView,
    depth_texture: wgpu::Texture,
    depth_view: wgpu::TextureView,
    buffer: wgpu::Buffer,
    uniform_bind_group_layout: wgpu::BindGroupLayout,
    resolution: u32,
    scale_factor: f32,
}

impl SurfaceAreaCalculator {
    pub fn new(device: &wgpu::Device, _queue: &wgpu::Queue, resolution: u32) -> Self {
        // Create render target texture (R32Uint format for object IDs)
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("Surface Area Render Target"),
            size: wgpu::Extent3d {
                width: resolution,
                height: resolution,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::R32Uint,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });

        let texture_view = texture.create_view(&wgpu::TextureViewDescriptor::default());

        // Create depth texture
        let depth_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("Depth Texture"),
            size: wgpu::Extent3d {
                width: resolution,
                height: resolution,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth32Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });

        let depth_view = depth_texture.create_view(&wgpu::TextureViewDescriptor::default());

        // Calculate aligned buffer size
        let unaligned = resolution * 4; // 4 bytes per pixel
        let alignment = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
        let bytes_per_row = ((unaligned + alignment - 1) / alignment) * alignment;
        let buffer_size = (bytes_per_row * resolution) as u64;

        // Create buffer to read back results
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Surface Area Readback Buffer"),
            size: buffer_size,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        // Create bind group layout for uniforms
        let uniform_bind_group_layout = device.create_bind_group_layout(
            &wgpu::BindGroupLayoutDescriptor {
                label: Some("Uniform Bind Group Layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            },
        );

        // Create render pipeline
        let render_pipeline = Self::create_render_pipeline(
            device,
            &uniform_bind_group_layout,
        );

        Self {
            render_pipeline,
            texture,
            texture_view,
            depth_texture,
            depth_view,
            buffer,
            uniform_bind_group_layout,
            resolution,
            scale_factor: 1.1,
        }
    }

    fn create_render_pipeline(
        device: &wgpu::Device,
        uniform_bind_group_layout: &wgpu::BindGroupLayout,
    ) -> wgpu::RenderPipeline {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Surface Area Shader"),
            source: wgpu::ShaderSource::Wgsl(SHADER_SOURCE.into()),
        });

        let render_pipeline_layout = device.create_pipeline_layout(
            &wgpu::PipelineLayoutDescriptor {
                label: Some("Surface Area Pipeline Layout"),
                bind_group_layouts: &[uniform_bind_group_layout],
                push_constant_ranges: &[],
            },
        );

        const ATTRIBS: [wgpu::VertexAttribute; 1] = wgpu::vertex_attr_array![
            //position
            0 => Float32x3,
        ];

        // created this hear since geometry crate uses iced::wgpu which is a different version
        let vertex_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<SimpleVertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &ATTRIBS,
        };

        device.create_render_pipeline(
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
        )
    }

    pub fn calculate_surface_areas(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        geometries: &[Geometry],
        view_direction: [f32; 3],
        scene_bounds: SceneBounds,
    ) -> Vec<f32> {
        // Create orthographic projection matrix
        let projection_matrix = Self::create_orthographic_projection(
            view_direction,
            scene_bounds,
            self.scale_factor,
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
                        view: &self.texture_view,
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
                        view: &self.depth_view,
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

            render_pass.set_pipeline(&self.render_pipeline);

            // Render each geometry with its object ID
            for (object_id, geometry) in geometries
                .iter()
                .enumerate()
            {
                let vertices = simple_vertices(&geometry.get_vertices());
                let vertex_buffer = device.create_buffer_init(
                    &wgpu::util::BufferInitDescriptor {
                        label: Some(&format!(
                            "Vertex Buffer {}",
                            object_id
                        )),
                        contents: bytemuck::cast_slice(&vertices),
                        usage: wgpu::BufferUsages::VERTEX,
                    },
                );

                // Create uniforms
                let uniforms = Uniforms {
                    projection_matrix,
                    object_id: (object_id + 1) as u32, // +1 because 0 is background
                    _padding: [0; 3],
                };

                let uniform_buffer = device.create_buffer_init(
                    &wgpu::util::BufferInitDescriptor {
                        label: Some(&format!(
                            "Uniform Buffer {}",
                            object_id
                        )),
                        contents: bytemuck::cast_slice(&[uniforms]),
                        usage: wgpu::BufferUsages::UNIFORM,
                    },
                );

                let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some(&format!(
                        "Bind Group {}",
                        object_id
                    )),
                    layout: &self.uniform_bind_group_layout,
                    entries: &[wgpu::BindGroupEntry {
                        binding: 0,
                        resource: uniform_buffer.as_entire_binding(),
                    }],
                });

                render_pass.set_bind_group(0, &bind_group, &[]);
                render_pass.set_vertex_buffer(0, vertex_buffer.slice(..));
                render_pass.draw(0..vertices.len() as u32, 0..1);
            }
        }

        // Copy render target to readable buffer
        let unaligned = self.resolution * 4;
        let alignment = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
        let bytes_per_row = ((unaligned + alignment - 1) / alignment) * alignment;

        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture: &self.texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &self.buffer,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(bytes_per_row),
                    rows_per_image: Some(self.resolution),
                },
            },
            wgpu::Extent3d {
                width: self.resolution,
                height: self.resolution,
                depth_or_array_layers: 1,
            },
        );

        queue.submit(Some(encoder.finish()));

        // Read back pixel data and calculate areas
        let pixel_data = self.read_buffer_data(device);

        //After getting pixel_data, add this debugging:
        // println!(
        //     "Pixel data as {}x{} grid:",
        //     self.resolution, self.resolution
        // );
        // for y in 0..self.resolution {
        //     for x in 0..self.resolution {
        //         let idx = (y * self.resolution + x) as usize;
        //         print!("{:2} ", pixel_data[idx]);
        //     }
        //     println!();
        // }

        self.calculate_areas_from_pixels(
            pixel_data,
            geometries.len(),
            scene_bounds,
            self.resolution,
            self.scale_factor,
        )
    }

    fn read_buffer_data(&self, device: &wgpu::Device) -> Vec<u32> {
        let buffer_slice = self
            .buffer
            .slice(..);

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
        let unaligned = self.resolution * 4; // 4 bytes per pixel
        let alignment = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
        let bytes_per_row = ((unaligned + alignment - 1) / alignment) * alignment;

        // Create a properly sized result without padding
        let mut result = Vec::with_capacity((self.resolution * self.resolution) as usize);

        let mapped_data = buffer_slice.get_mapped_range();
        let bytes = mapped_data.as_ref();

        // Extract just the actual pixels, skipping padding
        for y in 0..self.resolution {
            let row_start = (y * bytes_per_row) as usize; // bytes_per_row is in bytes, not u32s
            for x in 0..self.resolution {
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
        self.buffer
            .unmap();

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

    fn create_orthographic_projection(
        view_direction: [f32; 3],
        scene_bounds: SceneBounds,
        scale_factor: f32,
    ) -> [[f32; 4]; 4] {
        // Extract bounds
        let min = Vec3::new(
            scene_bounds.min_x,
            scene_bounds.min_y,
            scene_bounds.min_z,
        );
        let max = Vec3::new(
            scene_bounds.max_x,
            scene_bounds.max_y,
            scene_bounds.max_z,
        );

        // Calculate center of bounds and size
        let center = (min + max) * 0.5;
        let size = max - min;

        // Add buffer for projection (ensure whole geometry is visible)
        let buffer_factor = 1.2; // Use 1.2 for 20% extra space around object

        // Store this for scaling area calculations later
        // (Store this as a static or member variable for use in calculate_area_per_pixel)

        // Convert view direction to Vec3 and normalize
        let dir = Vec3::from_array(view_direction).normalize();

        // Calculate the up vector (perpendicular to view direction)
        let up = if dir
            .y
            .abs()
            > 0.9
        {
            Vec3::Z
        } else {
            Vec3::Y
        };

        // Position camera OUTSIDE the object, looking toward center
        let view_distance = size.length();
        let eye = center - dir * view_distance;

        // Create view matrix
        let view_matrix = Mat4::look_at_rh(eye, center, up);

        // Create orthographic projection with buffer
        let ortho_size = size.max_element() * 0.5 * scale_factor;
        let ortho_matrix = Mat4::orthographic_rh(
            -ortho_size,
            ortho_size,
            -ortho_size,
            ortho_size,
            0.1,
            view_distance * 2.0,
        );

        // Combine matrices and return
        let view_proj = ortho_matrix * view_matrix;
        view_proj.to_cols_array_2d()
    }

    fn calculate_area_per_pixel(
        scene_bounds: SceneBounds,
        resolution: u32,
        scale_factor: f32,
    ) -> f32 {
        // Simple scaling formula: area per pixel * scaling factor squared
        (scene_bounds.max_x - scene_bounds.min_x) * (scene_bounds.max_y - scene_bounds.min_y)
            / ((resolution * resolution) as f32)
            * (scale_factor * scale_factor)
    }
}

#[derive(Copy, Clone)]
pub struct SceneBounds {
    min_x: f32,
    max_x: f32,
    min_y: f32,
    max_y: f32,
    min_z: f32,
    max_z: f32,
}

const SHADER_SOURCE: &str = r#"
struct Uniforms {
    projection_matrix: mat4x4<f32>,
    object_id: u32,
}

@group(0) @binding(0) var<uniform> uniforms: Uniforms;

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
}

@vertex
fn vs_main(@location(0) position: vec3<f32>) -> VertexOutput {
    var out: VertexOutput;
    out.clip_position = uniforms.projection_matrix * vec4<f32>(position, 1.0);
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) u32 {
    return uniforms.object_id;
}
"#;

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{DQuat, Quat};
    use nadir_3d::geometry::{GeometryState, cuboid::Cuboid};

    #[test]
    fn test_cube_area() {
        // WGPU Setup (use default adapter)
        let instance = wgpu::Instance::default();
        let adapter = pollster::block_on(
            instance.request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::LowPower,
                compatible_surface: None,
                force_fallback_adapter: false,
            }),
        )
        .expect("Failed to find a GPU adapter");
        let (device, queue) =
            pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default()))
                .expect("Failed to get device");

        // Build test geometry (unit cube)
        let cube = Cuboid::new(1.0, 1.0, 1.0).unwrap();
        // Provide dummy geometry state if needed by your trait/struct impl
        let geometry = cube;
        let geometry_vec: Vec<Geometry> = vec![geometry.into()];

        // Scene bounds that tightly include the cube
        let bounds = SceneBounds {
            min_x: -1.0,
            max_x: 1.0,
            min_y: -1.0,
            max_y: 1.0,
            min_z: -1.0,
            max_z: 1.0,
        };

        // Compute surface area using calculator
        let resolution = 32;
        let calc = SurfaceAreaCalculator::new(&device, &queue, resolution);

        // Front-on (Z) view
        let view_direction = [0.0, 0.0, -1.0];

        // Your geometry should implement GeometryTrait (get_vertices)
        let areas = calc.calculate_surface_areas(
            &device,
            &queue,
            &geometry_vec,
            view_direction,
            bounds,
        );

        // Only one object, expect just front face: area should be close to 1.0
        let calculated_area = areas[0];
        println!(
            "Calculated area: {}",
            calculated_area
        );
        assert!(
            (calculated_area - 1.0).abs() < 0.02, // Allow small error due to resolution
            "Expected ~1.0, got {}",
            calculated_area
        );
    }

    #[test]
    fn test_two_cubes_one_rotated() {
        // WGPU Setup
        let instance = wgpu::Instance::default();
        let adapter = pollster::block_on(
            instance.request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::LowPower,
                compatible_surface: None,
                force_fallback_adapter: false,
            }),
        )
        .expect("Failed to find a GPU adapter");
        let (device, queue) =
            pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default()))
                .expect("Failed to get device");

        // Create two cubes
        let cube1 = Cuboid::new(1.0, 1.0, 1.0).unwrap();
        let cube2 = Cuboid::new(1.0, 1.0, 1.0).unwrap();

        // Create geometry states - one normal, one rotated 90 degrees around Y axis
        let state1 = GeometryState {
            position: Vec3::new(-1.5, 0.0, 0.0).into(), // Position first cube to the left
            rotation: DQuat::IDENTITY,                  // No rotation
        };

        let state2 = GeometryState {
            position: Vec3::new(1.5, 0.0, 0.0).into(), // Position second cube to the right
            rotation: DQuat::from_rotation_y(std::f64::consts::PI / 4.0), // 90° rotation around Y
        };

        // Apply transformations and create geometries
        let transform1 = cube1.get_transform(&state1);
        let transform2 = cube2.get_transform(&state2);

        let mut vertices1 = cube1.get_vertices();
        let mut vertices2 = cube2.get_vertices();

        // Apply transformations to vertices
        for vertex in &mut vertices1 {
            let pos = transform1.transformation_matrix
                * vertex
                    .pos
                    .extend(1.0);
            vertex.pos = pos.truncate();
            vertex.normal = transform1.normal_matrix * vertex.normal;
        }

        for vertex in &mut vertices2 {
            let pos = transform2.transformation_matrix
                * vertex
                    .pos
                    .extend(1.0);
            vertex.pos = pos.truncate();
            vertex.normal = transform2.normal_matrix * vertex.normal;
        }

        // Create geometry objects with transformed vertices
        let geometry1 = Geometry::from_vertices(vertices1);
        let geometry2 = Geometry::from_vertices(vertices2);
        let geometry_vec = vec![geometry1, geometry2];

        // Expanded scene bounds to contain both cubes
        let bounds = SceneBounds {
            min_x: -3.0,
            max_x: 3.0,
            min_y: -1.5,
            max_y: 1.5,
            min_z: -1.5,
            max_z: 1.5,
        };

        // Calculate surface areas
        let resolution = 64; // Higher resolution for better accuracy with 2 objects
        let calc = SurfaceAreaCalculator::new(&device, &queue, resolution);

        // View from positive Z direction
        let view_direction = [0.0, 0.0, 1.0];

        let areas = calc.calculate_surface_areas(
            &device,
            &queue,
            &geometry_vec,
            view_direction,
            bounds,
        );

        println!(
            "Cube 1 (unrotated) area: {}",
            areas[0]
        );
        println!(
            "Cube 2 (rotated 90°) area: {}",
            areas[1]
        );

        // Both cubes should have the same apparent surface area from this view direction
        // The rotation shouldn't change the projected area when viewed along Z
        assert!(
            (areas[0] - 1.0).abs() < 0.05,
            "Expected cube 1 area ~1.0, got {}",
            areas[0]
        );
        assert!(
            (areas[1] - 1.0).abs() < 0.05,
            "Expected cube 2 area ~1.0, got {}",
            areas[1]
        );

        // Areas should be approximately equal
        assert!(
            (areas[0] - areas[1]).abs() < 0.02,
            "Expected similar areas, got {} and {}",
            areas[0],
            areas[1]
        );

        println!("Test passed: Both cubes have similar projected areas!");
    }
}
