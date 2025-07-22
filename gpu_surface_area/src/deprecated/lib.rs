use std::{collections::HashMap, f32};

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

use crate::bvh::{BVH, GeometryBVHInfo};

pub mod bvh;

pub struct GpuSurfaceArea {
    geometries: Vec<Geometry>,
    transforms: Vec<Mat4>,
    initialized: Option<Initialized>,
}

#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct Uniforms {
    direction: [f32; 3],
    origin: [f32; 3],
    grid_x_size: f32,
    grid_y_size: f32,
    grid_z_distance: f32,
    grid_nrays_x: u32,
    grid_nrays_y: u32,
    ray_area: f32,
}

struct Initialized {
    bindgroup: wgpu::BindGroup,
    compute_pipeline: wgpu::ComputePipeline,
    uniforms: Uniforms,
    result: Vec<f32>,
    n_triangles: u32,
    meta: Vec<GeometryMetadata>,
    uniform_buffer: wgpu::Buffer,
    result_buffer: wgpu::Buffer,
    staging_buffer: wgpu::Buffer,
    transform_buffer: wgpu::Buffer,
}

#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct RayHit {
    closest_t: f32,      // Distance to closest intersection (f32::INFINITY if no hit)
    geometry_id: u32,    // Which geometry was hit (u32::MAX if no hit)
    projected_area: f32, // Area contribution (uniforms.ray_area if hit, 0.0 if miss)
    _padding: u32,       // Alignment padding
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
            - 1
    }

    pub fn update_body(&mut self, index: usize, state: &GeometryState) {
        self.transforms[index] = self.geometries[index]
            .get_transform(&state)
            .transformation_matrix;
    }

    pub fn initialize(&mut self, device: &Device) {
        let num_geometries = self
            .geometries
            .len();
        let mut meta = Vec::new();
        let mut unique_vertices = Vec::new();
        let mut collected_indices = Vec::new();
        let mut vertex_map: HashMap<SimpleVertexKey, u32> = HashMap::new();
        let mut max_dimension = 0.0;
        let mut all_bvh_nodes = Vec::new();
        let mut geometry_bvhs = Vec::new();

        // For each geometry:
        // 1. Get the vertices
        // 2. Add the vertices to the unique vertices, using hashing to make them unique
        // 3. Create the triangles by adding the 3 indices into the unique_vertices
        // 4. Add a BVH node per triangle from this geometry
        for geometry in &self.geometries {
            let vertices = simple_vertices(&geometry.get_vertices());
            let mut geometry_indices = Vec::new();
            // Process each vertex in this geometry
            for vertex in vertices {
                // Hashsing for creating unique vertices
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
                // Add this vertex index to indeces
                geometry_indices.push(vertex_index);

                // Determine what the max vertex value is for all triangles to help determine ray grid size
                if vertex
                    .pos
                    .x
                    .abs()
                    > max_dimension
                {
                    max_dimension = vertex
                        .pos
                        .x
                        .abs();
                }
                if vertex
                    .pos
                    .y
                    .abs()
                    > max_dimension
                {
                    max_dimension = vertex
                        .pos
                        .y
                        .abs();
                }
                if vertex
                    .pos
                    .z
                    .abs()
                    > max_dimension
                {
                    max_dimension = vertex
                        .pos
                        .z
                        .abs();
                }
            }

            // Store metadata
            meta.push(GeometryMetadata {
                index_offset: collected_indices.len() as u32,
                index_count: geometry_indices.len() as u32,
                triangle_count: (geometry_indices.len() / 3) as u32,
                _padding: 0,
            });

            // Extended all indices by this geometries indices
            collected_indices.extend(geometry_indices);

            // Build BVH
            let bvh = BVH::new(
                &unique_vertices,
                &collected_indices,
                8, // 8 triangles per leaf
            );

            // Store metadata about this geometry's BVH
            geometry_bvhs.push(GeometryBVHInfo {
                root_node_index: all_bvh_nodes.len() as u32,
                node_count: bvh
                    .nodes
                    .len() as u32,
            });

            // Add all nodes to combined array
            all_bvh_nodes.extend_from_slice(&bvh.nodes);
        }

        let n_triangles = (collected_indices.len() / 3) as u32;
        let result = vec![0.0; num_geometries];

        let grid_x_size = 2.0 * max_dimension;
        let grid_y_size = 2.0 * max_dimension;
        let grid_z_distance = 5.0 * max_dimension;

        let uniforms = Uniforms {
            direction: [1.0, 0.0, 0.0],
            origin: [0.0, 0.0, 1.0],
            grid_x_size,
            grid_y_size,
            grid_nrays_x: 100,
            grid_nrays_y: 100,
            grid_z_distance,
            ray_area: grid_x_size / 100 as f32 * grid_y_size / 100 as f32,
        };

        let total_rays = uniforms.grid_nrays_x * uniforms.grid_nrays_y;

        // Initialize with "no hit" values
        let init_data: Vec<RayHit> = (0..total_rays)
            .map(|_| RayHit {
                closest_t: f32::INFINITY,
                geometry_id: u32::MAX, // Invalid geometry ID
                projected_area: 0.0,
                _padding: 0,
            })
            .collect();

        // Create ray hits buffer - one entry per ray
        let ray_hits_buffer = device.create_buffer_init(&BufferInitDescriptor {
            label: Some("Ray Hits Buffer"),
            contents: bytemuck::cast_slice(&init_data),
            usage: BufferUsages::STORAGE | BufferUsages::COPY_SRC | BufferUsages::COPY_DST,
        });

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

        let transform_buffer = device.create_buffer_init(&BufferInitDescriptor {
            label: Some("Transform Buffer"),
            contents: bytemuck::cast_slice(&self.transforms),
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
        });

        // Create BVH buffers
        let bvh_nodes_buffer = device.create_buffer_init(&BufferInitDescriptor {
            label: Some("BVH Nodes Buffer"),
            contents: bytemuck::cast_slice(&all_bvh_nodes),
            usage: BufferUsages::STORAGE,
        });

        let geometry_bvh_buffer = device.create_buffer_init(&BufferInitDescriptor {
            label: Some("Geometry BVH Info Buffer"),
            contents: bytemuck::cast_slice(&geometry_bvhs),
            usage: BufferUsages::STORAGE,
        });

        // Create inverse transform buffer (needed for BVH traversal)
        let inverse_transforms: Vec<Mat4> = self
            .transforms
            .iter()
            .map(|t| t.inverse())
            .collect();

        let inverse_transform_buffer = device.create_buffer_init(&BufferInitDescriptor {
            label: Some("Inverse Transform Buffer"),
            contents: bytemuck::cast_slice(&inverse_transforms),
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
        });

        let uniform_buffer = device.create_buffer(&BufferDescriptor {
            label: Some("Direction Buffer"),
            size: std::mem::size_of::<Uniforms>() as u64,
            usage: BufferUsages::UNIFORM | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let result_buffer = device.create_buffer(&BufferDescriptor {
            label: Some("Result Buffer"),
            size: total_rays as u64 * std::mem::size_of::<RayHit>() as u64,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });

        let staging_buffer = device.create_buffer(&BufferDescriptor {
            label: Some("Staging Buffer"),
            size: total_rays as u64 * std::mem::size_of::<RayHit>() as u64,
            usage: BufferUsages::COPY_DST | BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        // Create metadata buffer
        let metadata_buffer = device.create_buffer_init(&BufferInitDescriptor {
            label: Some("Geometry Metadata Buffer"),
            contents: bytemuck::cast_slice(&meta),
            usage: BufferUsages::STORAGE,
        });

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
                // Transform buffer
                BindGroupLayoutEntry {
                    binding: 2,
                    visibility: ShaderStages::COMPUTE,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // Uniforms
                BindGroupLayoutEntry {
                    binding: 3,
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
                    binding: 4,
                    visibility: ShaderStages::COMPUTE,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Storage { read_only: false },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // MetaData
                BindGroupLayoutEntry {
                    binding: 5,
                    visibility: ShaderStages::COMPUTE,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // BVH nodes
                BindGroupLayoutEntry {
                    binding: 6,
                    visibility: ShaderStages::COMPUTE,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // Geometry BVH info
                BindGroupLayoutEntry {
                    binding: 7,
                    visibility: ShaderStages::COMPUTE,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // Inverse transforms
                BindGroupLayoutEntry {
                    binding: 8,
                    visibility: ShaderStages::COMPUTE,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // Ray hits buffer
                BindGroupLayoutEntry {
                    binding: 9,
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

        // Create a bind group for this calculation
        let bindgroup = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: vertex_buffer.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: index_buffer.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 2, resource: transform_buffer.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 3, resource: uniform_buffer.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 4, resource: result_buffer.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 5, resource: metadata_buffer.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 6, resource: bvh_nodes_buffer.as_entire_binding() },
                wgpu::BindGroupEntry {
                    binding: 7,
                    resource: geometry_bvh_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 8,
                    resource: inverse_transform_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry { binding: 9, resource: ray_hits_buffer.as_entire_binding() },
            ],
            label: Some("Surface Area Bind Group"),
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

        self.initialized = Some(Initialized {
            bindgroup,
            compute_pipeline,
            transform_buffer,
            uniforms,
            uniform_buffer,
            result_buffer,
            staging_buffer,
            result,
            n_triangles,
            meta,
        });
    }

    pub fn calculate_surface_area(
        &mut self,
        geometry_index: usize,
        direction: [f32; 3],
        device: &Device,
        queue: &Queue,
    ) -> Result<f32, Box<dyn std::error::Error>> {
        if let Some(initialized) = &mut self.initialized {
            // Update uniforms

            // Get the mean origin of all of the geometries
            let mut mean_origin = [0.0; 3];
            for transform in &self.transforms {
                let translation = transform.w_axis;
                mean_origin[0] += translation[0];
                mean_origin[1] += translation[1];
                mean_origin[2] += translation[2];
            }

            mean_origin[0] = mean_origin[0]
                / self
                    .geometries
                    .len() as f32;
            mean_origin[1] = mean_origin[1]
                / self
                    .geometries
                    .len() as f32;
            mean_origin[2] = mean_origin[2]
                / self
                    .geometries
                    .len() as f32;

            initialized
                .uniforms
                .direction = direction;
            initialized
                .uniforms
                .origin = mean_origin;
            initialized
                .uniforms
                .ray_area = initialized
                .uniforms
                .grid_x_size
                / initialized
                    .uniforms
                    .grid_nrays_x as f32
                * initialized
                    .uniforms
                    .grid_y_size
                / initialized
                    .uniforms
                    .grid_nrays_y as f32;

            queue.write_buffer(
                &initialized.uniform_buffer,
                0,
                bytemuck::cast_slice(&[initialized.uniforms]),
            );

            // Update transforms
            queue.write_buffer(
                &initialized.transform_buffer,
                0,
                bytemuck::cast_slice(&self.transforms),
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

                // TODO
                // for now we put triangles on x and rays on y, but maybe decide which maxes more sense
                // that is if x and y max workgroups are different
                // actually, we have to hardcode workgroup size in the shader, so just choose 64 for now i suppose
                // maybe have 2 shaders, one for 64 and one for 256?
                let max_workgroup_size = 64;
                let triangle_workgroup =
                    (initialized.n_triangles + max_workgroup_size - 1) / max_workgroup_size;
                let ray_workgroup = (initialized
                    .uniforms
                    .grid_nrays_x
                    * initialized
                        .uniforms
                        .grid_nrays_y
                    + max_workgroup_size
                    - 1)
                    / max_workgroup_size;
                compute_pass.dispatch_workgroups(
                    triangle_workgroup as u32,
                    ray_workgroup as u32,
                    1,
                );
            }

            let meta = &initialized.meta[geometry_index];
            let offset_bytes = meta.index_offset as u64 / 3 * std::mem::size_of::<f32>() as u64;
            let triangle_count = meta.triangle_count as usize;
            let byte_size = triangle_count as u64 * std::mem::size_of::<f32>() as u64;

            encoder.copy_buffer_to_buffer(
                &initialized.result_buffer,
                offset_bytes,
                &initialized.staging_buffer,
                0,
                byte_size,
            );

            // Submit command buffer
            queue.submit(std::iter::once(
                encoder.finish(),
            ));

            let buffer_slice = initialized
                .staging_buffer
                .slice(0..byte_size);

            // Map the buffer
            buffer_slice.map_async(
                wgpu::MapMode::Read,
                |result| {
                    if let Err(e) = result {
                        eprintln!(
                            "Failed to map buffer: {:?}",
                            e
                        );
                    }
                },
            );

            // Poll until mapping is complete
            device.poll(wgpu::PollType::Wait)?;

            // Get the mapped range and read the result
            let range = buffer_slice.get_mapped_range();
            let result_data: &[f32] = bytemuck::cast_slice(&range);
            println!(
                "Read {} triangle areas for geometry {}",
                result_data.len(),
                geometry_index
            );
            dbg!(result_data);
            if result_data.is_empty() {
                return Err("No data in buffer".into());
            }
            let surface_area: f32 = result_data
                .iter()
                .copied()
                .sum();

            // Update the result vector
            initialized.result[geometry_index] = surface_area;

            // Unmap the buffer
            drop(range);
            initialized
                .staging_buffer
                .unmap();

            Ok(surface_area)
        } else {
            Err("GPU was not initialized. Make sure you run GpuSurfaceArea .initialize()".into())
        }
    }
}
