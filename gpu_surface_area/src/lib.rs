use std::collections::HashMap;

use glam::Mat4;
use nadir_3d::{
    geometry::{Geometry, GeometryState, GeometryTrait},
    vertex::{SimpleVertexKey, simple_vertices},
};
use wgpu::{
    BindGroupLayoutDescriptor, BindGroupLayoutEntry, BindingType, BufferBindingType,
    BufferDescriptor, BufferUsages, ComputePipelineDescriptor, Device, PipelineLayoutDescriptor,
    Queue, ShaderModuleDescriptor, ShaderSource, ShaderStages, util::BufferInitDescriptor,
    util::DeviceExt,
};

pub struct GpuSurfaceArea {
    geometries: Vec<Geometry>,
    transforms: Vec<Mat4>,
    initialized: Option<Initialized>,
}

struct Initialized {
    bindgroup: wgpu::BindGroup,
    compute_pipeline: wgpu::ComputePipeline,
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    direction_buffer: wgpu::Buffer,
    result_buffer: wgpu::Buffer,
    staging_buffer: wgpu::Buffer,
    n_vertices: usize,
    n_triangles: usize, // for determining workgroup size
    meta: Vec<GeometryMetadata>,
}

#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct GeometryMetadata {
    index_offset: u32,   // Where this geometry's indices start in index buffer
    index_count: u32,    // How many indices this geometry has
    triangle_count: u32, // How many triangles (index_count / 3)
    _padding: u32,       // Padding for alignment
}

impl GpuSurfaceArea {
    pub fn new() -> Self {
        Self {
            geometries: Vec::new(),
            transforms: Vec::new(),
            initialized: None,
        }
    }

    pub fn add_body(&mut self, geometry: Geometry, state: GeometryState) -> usize {
        self.transforms
            .push(
                geometry
                    .get_transform(&state)
                    .transformation_matrix,
            );
        self.geometries
            .push(geometry);

        self.geometries
            .len()
    }

    pub fn update_body(&mut self, index: usize, state: &GeometryState) {
        self.transforms[index] = self.geometries[index]
            .get_transform(&state)
            .transformation_matrix;
    }

    pub fn initialize(&mut self, device: &Device) {
        // Create buffers with deduplication
        let num_geometries = self
            .geometries
            .len();
        let mut meta = Vec::new();
        let mut unique_vertices = Vec::new();
        let mut collected_indices = Vec::new();
        let mut vertex_map: HashMap<SimpleVertexKey, u32> = HashMap::new();

        for geometry in &self.geometries {
            let vertices = simple_vertices(&geometry.get_vertices());
            let mut geometry_indices = Vec::new();

            // Process each vertex in this geometry
            for vertex in vertices {
                let key = SimpleVertexKey::from_vertex(&vertex, 1000.0); // 10000 = tolerance of 0.1 mm

                let vertex_index = match vertex_map.get(&key) {
                    Some(&existing_index) => existing_index, // Reuse existing vertex
                    None => {
                        // Add new unique vertex
                        let new_index = unique_vertices.len() as u32;
                        unique_vertices.push(vertex);
                        vertex_map.insert(key, new_index);
                        new_index
                    }
                };

                geometry_indices.push(vertex_index);
            }

            // Store metadata
            meta.push(GeometryMetadata {
                index_offset: collected_indices.len() as u32,
                index_count: geometry_indices.len() as u32,
                triangle_count: (geometry_indices.len() / 3) as u32,
                _padding: 0,
            });

            collected_indices.extend(geometry_indices);
        }

        // Create buffers with deduplicated data
        let vertex_buffer = device.create_buffer_init(&BufferInitDescriptor {
            label: Some("Vertex Buffer"),
            contents: bytemuck::cast_slice(&unique_vertices),
            usage: BufferUsages::STORAGE,
        });

        let index_buffer = device.create_buffer_init(&BufferInitDescriptor {
            label: Some("Index Buffer"),
            contents: bytemuck::cast_slice(&collected_indices),
            usage: BufferUsages::STORAGE,
        });

        let n_vertices = unique_vertices.len();
        let n_triangles = collected_indices.len() / 3;

        // Define bind group layout
        let bind_group_layout = device.create_bind_group_layout(&BindGroupLayoutDescriptor {
            label: Some("Surface Area Bind Group Layout"),
            entries: &[
                // Vertex buffer
                BindGroupLayoutEntry {
                    binding: 0,
                    visibility: ShaderStages::COMPUTE,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // Index buffer
                BindGroupLayoutEntry {
                    binding: 1,
                    visibility: ShaderStages::COMPUTE,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // Parameters (direction, counts)
                BindGroupLayoutEntry {
                    binding: 2,
                    visibility: ShaderStages::COMPUTE,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // Result buffer
                BindGroupLayoutEntry {
                    binding: 3,
                    visibility: ShaderStages::COMPUTE,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Storage { read_only: false },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        });

        // Create pipeline layout
        let pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
            label: Some("Surface Area Pipeline Layout"),
            bind_group_layouts: &[&bind_group_layout],
            push_constant_ranges: &[],
        });

        // Create shader module
        let shader_module = device.create_shader_module(ShaderModuleDescriptor {
            label: Some("Surface Area Compute Shader"),
            source: ShaderSource::Wgsl(std::borrow::Cow::Borrowed(
                include_str!("area.wgsl"), // Read file at compile time
            )),
        });

        // Create compute pipeline
        let compute_pipeline = device.create_compute_pipeline(&ComputePipelineDescriptor {
            label: Some("Surface Area Pipeline"),
            layout: Some(&pipeline_layout),
            module: &shader_module,
            entry_point: Some("main"),
            compilation_options: Default::default(),
            cache: None,
        });

        let direction_buffer = device.create_buffer(&BufferDescriptor {
            label: Some("Direction Buffer"),
            size: 3 * 32, // [f32;3]
            usage: BufferUsages::UNIFORM | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let result_buffer = device.create_buffer(&BufferDescriptor {
            label: Some("Result Buffer"),
            size: num_geometries as u64 * std::mem::size_of::<f32>() as u64,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });

        let staging_buffer = device.create_buffer(&BufferDescriptor {
            label: Some("Staging Buffer"),
            size: std::mem::size_of::<f32>() as u64,
            usage: BufferUsages::COPY_DST | BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        // Create a bind group for this calculation
        let bindgroup = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: vertex_buffer.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: index_buffer.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 2, resource: direction_buffer.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 3, resource: result_buffer.as_entire_binding() },
            ],
            label: Some("Surface Area Bind Group"),
        });

        self.initialized = Some(Initialized {
            bindgroup,
            compute_pipeline,
            vertex_buffer,
            index_buffer,
            direction_buffer,
            result_buffer,
            staging_buffer,
            n_triangles,
            n_vertices,
            meta,
        });
    }

    pub fn calculate_surface_area(
        &self,
        geometry_index: usize,
        direction: [f32; 3],
        device: &Device,
        queue: &Queue,
    ) -> Result<f32, Box<dyn std::error::Error>> {
        // Ensure initialized
        if let Some(initialized) = &self.initialized {
            // Update direction buffer
            queue.write_buffer(
                &initialized.direction_buffer,
                0,
                bytemuck::cast_slice(&[direction]),
            );

            // Create command encoder
            let mut encoder = device.create_command_encoder(
                &wgpu::CommandEncoderDescriptor { label: Some("Surface Area Encoder") },
            );

            // Compute pass
            {
                let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                    label: Some("Surface Area Compute Pass"),
                    timestamp_writes: None,
                });

                compute_pass.set_pipeline(&initialized.compute_pipeline);
                compute_pass.set_bind_group(0, &initialized.bindgroup, &[]);

                // Calculate workgroup size based on triangle count
                let workgroup_size = 64; // Must match your shader's @workgroup_size                
                let workgroup_count =
                    (initialized.n_triangles + workgroup_size - 1) / workgroup_size;

                compute_pass.dispatch_workgroups(workgroup_count as u32, 1, 1);
            }

            // Copy result to staging buffer
            encoder.copy_buffer_to_buffer(
                &initialized.result_buffer,
                (geometry_index * std::mem::size_of::<f32>()) as u64,
                &initialized.staging_buffer,
                0,
                std::mem::size_of::<f32>() as u64,
            );

            // Submit command buffer
            queue.submit(std::iter::once(
                encoder.finish(),
            ));

            // Read result
            let mut result = [0.0f32];
            let buffer_slice = initialized
                .staging_buffer
                .slice(..);

            // Map the buffer
            let (tx, rx) = std::sync::mpsc::channel();
            buffer_slice.map_async(
                wgpu::MapMode::Read,
                move |v| {
                    tx.send(v)
                        .unwrap()
                },
            );
            device
                .poll(wgpu::PollType::Wait)
                .unwrap();

            rx.recv()
                .unwrap()
                .unwrap();

            // Get the mapped range and read the result
            let range = buffer_slice.get_mapped_range();
            result.copy_from_slice(bytemuck::cast_slice(
                &range[0..4],
            ));

            // Unmap
            drop(range);
            initialized
                .staging_buffer
                .unmap();
            Ok(result[0])
        } else {
            panic!("GPU was not initialized. Make sure you run GpuSurfaceArea .initialize()")
        }
    }
}

#[cfg(test)]
mod tests {
    use glam::{DQuat, Vec3};
    use nadir_3d::geometry::cuboid::Cuboid;

    use super::*;

    #[test]
    fn create_cube() {
        let cube = Cuboid::new(2.0, 2.0, 2.0).unwrap();
        let state = GeometryState { position: Vec3::ZERO, rotation: DQuat::IDENTITY };
        let mut sa = GpuSurfaceArea::new();
        let index = sa.add_body(cube.into(), state);
        sa.initialize(device);

        sa.calculate_surface_area(
            index,
            [1.0, 0.0, 0.0],
            device,
            queue,
        );
    }
}
