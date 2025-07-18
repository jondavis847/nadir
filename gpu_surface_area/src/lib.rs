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
    result: Vec<f32>,
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
            - 1
    }

    pub fn update_body(&mut self, index: usize, state: &GeometryState) {
        self.transforms[index] = self.geometries[index]
            .get_transform(&state)
            .transformation_matrix;
    }

    pub fn initialize(&mut self, device: &Device) {
        // let limits = device.limits();

        // println!(
        //     "Max compute workgroup size X: {}",
        //     limits.max_compute_workgroup_size_x
        // );
        // println!(
        //     "Max compute workgroup size Y: {}",
        //     limits.max_compute_workgroup_size_y
        // );
        // println!(
        //     "Max compute workgroup size Z: {}",
        //     limits.max_compute_workgroup_size_z
        // );
        // println!(
        //     "Max compute workgroup storage size: {}",
        //     limits.max_compute_workgroup_storage_size
        // );
        // println!(
        //     "Max compute invocations per workgroup: {}",
        //     limits.max_compute_invocations_per_workgroup
        // );

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
                // MetaData
                BindGroupLayoutEntry {
                    binding: 4,
                    visibility: ShaderStages::COMPUTE,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Storage { read_only: true },
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
            size: std::mem::size_of::<[f32; 3]>() as u64,
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
            size: num_geometries as u64 * std::mem::size_of::<f32>() as u64,
            usage: BufferUsages::COPY_DST | BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        // Create metadata buffer
        let metadata_buffer = device.create_buffer_init(&BufferInitDescriptor {
            label: Some("Geometry Metadata Buffer"),
            contents: bytemuck::cast_slice(&meta),
            usage: BufferUsages::STORAGE,
        });

        // Create a bind group for this calculation
        let bindgroup = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: vertex_buffer.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: index_buffer.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 2, resource: direction_buffer.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 3, resource: result_buffer.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 4, resource: metadata_buffer.as_entire_binding() },
            ],
            label: Some("Surface Area Bind Group"),
        });

        let result = vec![0.0; num_geometries];

        self.initialized = Some(Initialized {
            bindgroup,
            compute_pipeline,
            vertex_buffer,
            index_buffer,
            direction_buffer,
            result_buffer,
            staging_buffer,
            result,
            n_triangles,
            n_vertices,
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
        // Return the result directly
        // Ensure initialized
        if let Some(initialized) = &mut self.initialized {
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

                let workgroup_size = 64;
                let workgroup_count =
                    (initialized.n_triangles + workgroup_size - 1) / workgroup_size;
                compute_pass.dispatch_workgroups(workgroup_count as u32, 1, 1);
            }

            // Copy result to staging buffer (only the specific geometry's result)
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

            // Read result - only read the single f32 we copied
            let buffer_slice = initialized
                .staging_buffer
                .slice(0..std::mem::size_of::<f32>() as u64);

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

            if result_data.is_empty() {
                return Err("No data in buffer".into());
            }

            let surface_area = result_data[0];

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

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{DQuat, DVec3};
    use nadir_3d::geometry::cuboid::Cuboid;

    async fn create_wgpu_context() -> (Device, Queue) {
        // Create instance
        let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });

        // Request adapter
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: None,
                force_fallback_adapter: false,
            })
            .await
            .expect("Failed to find a graphics adapter");

        // Request device and queue
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: Some("Main Device"),
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::default(),
                memory_hints: wgpu::MemoryHints::default(),
                trace: wgpu::Trace::Off,
            })
            .await
            .expect("Failed to create device");

        (device, queue)
    }

    #[test]
    fn test_cube_loading_metadata() {
        let (device, _queue) = pollster::block_on(create_wgpu_context());

        let cube = Cuboid::new(2.0, 2.0, 2.0).unwrap();
        let state = GeometryState { position: DVec3::ZERO, rotation: DQuat::IDENTITY };

        let mut sa = GpuSurfaceArea::new();
        let _index = sa.add_body(cube.into(), state);
        sa.initialize(&device);

        // Verify initialization worked
        assert!(
            sa.initialized
                .is_some(),
            "GPU surface area should be initialized"
        );

        let initialized = sa
            .initialized
            .as_ref()
            .unwrap();

        // A cube should have 8 vertices and 12 triangles (2 per face * 6 faces)
        // But after deduplication, it should have exactly 8 unique vertices
        assert_eq!(
            initialized.n_vertices, 8,
            "Cube should have 8 unique vertices after deduplication"
        );
        assert_eq!(
            initialized.n_triangles, 12,
            "Cube should have 12 triangles"
        );

        // Test metadata
        assert_eq!(
            initialized
                .meta
                .len(),
            1,
            "Should have metadata for 1 geometry"
        );

        let cube_meta = &initialized.meta[0];
        assert_eq!(
            cube_meta.index_offset, 0,
            "First geometry should start at index 0"
        );
        assert_eq!(
            cube_meta.index_count, 36,
            "Cube should have 36 indices (12 triangles * 3 vertices each)"
        );
        assert_eq!(
            cube_meta.triangle_count, 12,
            "Cube should have 12 triangles"
        );
    }

    #[test]
    fn test_multiple_cubes_deduplication() {
        let (device, _queue) = pollster::block_on(create_wgpu_context());

        let cube1 = Cuboid::new(1.0, 1.0, 1.0).unwrap();
        let cube2 = Cuboid::new(2.0, 2.0, 2.0).unwrap();
        let cube3 = Cuboid::new(2.0, 2.0, 2.0).unwrap();

        let state1 = GeometryState {
            position: DVec3::new(0.0, 0.0, 0.0),
            rotation: DQuat::IDENTITY,
        };
        let state2 = GeometryState {
            position: DVec3::new(5.0, 0.0, 0.0),
            rotation: DQuat::IDENTITY,
        };

        let state3 = GeometryState {
            position: DVec3::new(0.0, 0.0, 0.0),
            rotation: DQuat::IDENTITY,
        };

        let mut sa = GpuSurfaceArea::new();
        sa.add_body(cube1.into(), state1);
        sa.add_body(cube2.into(), state2);
        sa.add_body(cube3.into(), state3);
        sa.initialize(&device);

        let initialized = sa
            .initialized
            .as_ref()
            .unwrap();

        // Two different-sized cubes should have 16 unique vertices total
        assert_eq!(
            initialized.n_vertices, 16,
            "Three cubes where 2 are same geometry should have 16 unique vertices"
        );
        assert_eq!(
            initialized.n_triangles, 36,
            "Three cubes should have 36 triangles total"
        );

        // Test metadata for both geometries
        assert_eq!(
            initialized
                .meta
                .len(),
            3,
            "Should have metadata for 3 geometries"
        );

        let cube1_meta = &initialized.meta[0];
        let cube2_meta = &initialized.meta[1];
        let cube3_meta = &initialized.meta[2];

        assert_eq!(
            cube1_meta.index_offset, 0,
            "First cube should start at index 0"
        );
        assert_eq!(
            cube1_meta.triangle_count, 12,
            "Each cube should have 12 triangles"
        );

        assert_eq!(
            cube2_meta.index_offset, 36,
            "Second cube should start at index 36"
        );
        assert_eq!(
            cube2_meta.triangle_count, 12,
            "Each cube should have 12 triangles"
        );

        assert_eq!(
            cube3_meta.index_offset, 72,
            "Third cube should start at index 72"
        );
        assert_eq!(
            cube2_meta.triangle_count, 12,
            "Each cube should have 12 triangles"
        );
    }

    #[test]
    fn test_cube_surface_area() {
        let (device, queue) = pollster::block_on(create_wgpu_context());

        // 1. Create a 2x2x2 cube (dimensions 2.0 in each direction)
        let cube = Cuboid::new(2.0, 2.0, 2.0).unwrap();
        let state = GeometryState { position: DVec3::ZERO, rotation: DQuat::IDENTITY };

        let mut sa = GpuSurfaceArea::new();
        let index = sa.add_body(cube.into(), state);
        sa.initialize(&device);

        // 2. Calculate surface area along each axis
        sa.calculate_surface_area(
            index,
            [1.0, 0.0, 0.0],
            &device,
            &queue,
        )
        .unwrap();
        let area_x = sa
            .initialized
            .as_ref()
            .unwrap()
            .result[index];

        sa.calculate_surface_area(
            index,
            [0.0, 1.0, 0.0],
            &device,
            &queue,
        )
        .unwrap();
        let area_y = sa
            .initialized
            .as_ref()
            .unwrap()
            .result[index];

        sa.calculate_surface_area(
            index,
            [0.0, 0.0, 1.0],
            &device,
            &queue,
        )
        .unwrap();
        let area_z = sa
            .initialized
            .as_ref()
            .unwrap()
            .result[index];

        // 3. For a 2x2x2 cube, projected area along any axis should be 4.0
        println!(
            "Calculated surface areas - X: {}, Y: {}, Z: {}",
            area_x, area_y, area_z
        );

        // Check with reasonable epsilon for floating-point errors
        const EPSILON: f32 = 0.001;
        assert!(
            (area_x - 4.0).abs() < EPSILON,
            "X-axis surface area should be 4.0, got {}",
            area_x
        );
        assert!(
            (area_y - 4.0).abs() < EPSILON,
            "Y-axis surface area should be 4.0, got {}",
            area_y
        );
        assert!(
            (area_z - 4.0).abs() < EPSILON,
            "Z-axis surface area should be 4.0, got {}",
            area_z
        );
    }

    //#[test]
    fn test_rotated_cube_surface_area() {
        let (device, queue) = pollster::block_on(create_wgpu_context());

        // 1. Create a 2x2x2 cube and rotate it 45 degrees around Y axis
        let cube = Cuboid::new(2.0, 2.0, 2.0).unwrap();
        let state = GeometryState {
            position: DVec3::ZERO,
            rotation: DQuat::from_rotation_y(std::f64::consts::PI / 4.0), // 45 degrees
        };

        let mut sa = GpuSurfaceArea::new();
        let index = sa.add_body(cube.into(), state);
        sa.initialize(&device);

        // 2. Calculate surface area along X and Z axes
        sa.calculate_surface_area(
            index,
            [1.0, 0.0, 0.0],
            &device,
            &queue,
        )
        .unwrap();
        let area_x = sa
            .initialized
            .as_ref()
            .unwrap()
            .result[index];

        sa.calculate_surface_area(
            index,
            [0.0, 0.0, 1.0],
            &device,
            &queue,
        )
        .unwrap();
        let area_z = sa
            .initialized
            .as_ref()
            .unwrap()
            .result[index];

        // 3. For a 45-degree rotated cube along Y, projected area should be √2 * 2 * 2 = 5.66
        // (Because we see the cube corner-on, which increases the projected area)
        println!(
            "Rotated cube surface areas - X: {}, Z: {}",
            area_x, area_z
        );

        const EPSILON: f32 = 0.1;
        let expected_area = 2.0 * 2.0 * std::f32::consts::SQRT_2;
        assert!(
            (area_x - expected_area).abs() < EPSILON,
            "Rotated X-axis surface area should be ~{}, got {}",
            expected_area,
            area_x
        );
        assert!(
            (area_z - expected_area).abs() < EPSILON,
            "Rotated Z-axis surface area should be ~{}, got {}",
            expected_area,
            area_z
        );
    }

    //#[test]
    fn test_rectangular_cuboid_surface_area() {
        let (device, queue) = pollster::block_on(create_wgpu_context());

        // 1. Create a rectangular cuboid with different dimensions
        let cuboid = Cuboid::new(1.0, 2.0, 3.0).unwrap();
        let state = GeometryState { position: DVec3::ZERO, rotation: DQuat::IDENTITY };

        let mut sa = GpuSurfaceArea::new();
        let index = sa.add_body(cuboid.into(), state);
        sa.initialize(&device);

        // 2. Calculate surface areas along each axis
        sa.calculate_surface_area(
            index,
            [1.0, 0.0, 0.0],
            &device,
            &queue,
        )
        .unwrap();
        let area_x = sa
            .initialized
            .as_ref()
            .unwrap()
            .result[index];

        sa.calculate_surface_area(
            index,
            [0.0, 1.0, 0.0],
            &device,
            &queue,
        )
        .unwrap();
        let area_y = sa
            .initialized
            .as_ref()
            .unwrap()
            .result[index];

        sa.calculate_surface_area(
            index,
            [0.0, 0.0, 1.0],
            &device,
            &queue,
        )
        .unwrap();
        let area_z = sa
            .initialized
            .as_ref()
            .unwrap()
            .result[index];

        // 3. Expected areas:
        // X-axis: 2.0 * 3.0 = 6.0 (height * depth)
        // Y-axis: 1.0 * 3.0 = 3.0 (width * depth)
        // Z-axis: 1.0 * 2.0 = 2.0 (width * height)
        println!(
            "Rectangular cuboid surface areas - X: {}, Y: {}, Z: {}",
            area_x, area_y, area_z
        );

        const EPSILON: f32 = 0.001;
        assert!(
            (area_x - 6.0).abs() < EPSILON,
            "X-axis surface area should be 6.0, got {}",
            area_x
        );
        assert!(
            (area_y - 3.0).abs() < EPSILON,
            "Y-axis surface area should be 3.0, got {}",
            area_y
        );
        assert!(
            (area_z - 2.0).abs() < EPSILON,
            "Z-axis surface area should be 2.0, got {}",
            area_z
        );
    }

    //#[test]
    fn test_multiple_geometries() {
        let (device, queue) = pollster::block_on(create_wgpu_context());

        // Create two different cubes
        let cube1 = Cuboid::new(1.0, 1.0, 1.0).unwrap();
        let cube2 = Cuboid::new(2.0, 2.0, 2.0).unwrap();

        let state1 = GeometryState {
            position: DVec3::new(-2.0, 0.0, 0.0), // Position first cube on left
            rotation: DQuat::IDENTITY,
        };

        let state2 = GeometryState {
            position: DVec3::new(2.0, 0.0, 0.0), // Position second cube on right
            rotation: DQuat::IDENTITY,
        };

        let mut sa = GpuSurfaceArea::new();
        let index1 = sa.add_body(cube1.into(), state1);
        let index2 = sa.add_body(cube2.into(), state2);
        sa.initialize(&device);

        // Calculate areas
        sa.calculate_surface_area(
            index1,
            [0.0, 0.0, 1.0],
            &device,
            &queue,
        )
        .unwrap();
        let area1 = sa
            .initialized
            .as_ref()
            .unwrap()
            .result[index1];

        sa.calculate_surface_area(
            index2,
            [0.0, 0.0, 1.0],
            &device,
            &queue,
        )
        .unwrap();
        let area2 = sa
            .initialized
            .as_ref()
            .unwrap()
            .result[index2];

        // Expected values
        // First cube (1x1x1): area = 1.0
        // Second cube (2x2x2): area = 4.0
        println!(
            "Multiple geometries - Cube1 (1x1x1): {}, Cube2 (2x2x2): {}",
            area1, area2
        );

        const EPSILON: f32 = 0.001;
        assert!(
            (area1 - 1.0).abs() < EPSILON,
            "Cube1 area should be 1.0, got {}",
            area1
        );
        assert!(
            (area2 - 4.0).abs() < EPSILON,
            "Cube2 area should be 4.0, got {}",
            area2
        );
    }

    //#[test]
    fn test_diagonal_projection() {
        let (device, queue) = pollster::block_on(create_wgpu_context());

        // Create a cube
        let cube = Cuboid::new(2.0, 2.0, 2.0).unwrap();
        let state = GeometryState { position: DVec3::ZERO, rotation: DQuat::IDENTITY };

        let mut sa = GpuSurfaceArea::new();
        let index = sa.add_body(cube.into(), state);
        sa.initialize(&device);

        // Project along diagonal direction
        let diagonal = [1.0, 1.0, 1.0]; // Direction vector
        let magnitude = ((diagonal[0] * diagonal[0]
            + diagonal[1] * diagonal[1]
            + diagonal[2] * diagonal[2]) as f32)
            .sqrt();
        let normalized =
            [diagonal[0] / magnitude, diagonal[1] / magnitude, diagonal[2] / magnitude];

        sa.calculate_surface_area(
            index, normalized, &device, &queue,
        )
        .unwrap();
        let area = sa
            .initialized
            .as_ref()
            .unwrap()
            .result[index];

        // For a 2x2x2 cube projected along the (1,1,1) diagonal,
        // the projected area should be 2*2*sqrt(3) = 6.93
        println!(
            "Diagonal projection area: {}",
            area
        );

        const EPSILON: f32 = 0.1;
        let expected_area = 2.0 * 2.0 * (3.0_f32.sqrt()) / 3.0;
        assert!(
            (area - expected_area).abs() < EPSILON,
            "Diagonal projection area should be ~{}, got {}",
            expected_area,
            area
        );
    }

    //#[test]
    fn test_update_body_state() {
        let (device, queue) = pollster::block_on(create_wgpu_context());

        // Create a rectangular cuboid
        let cuboid = Cuboid::new(1.0, 2.0, 3.0).unwrap();
        let initial_state = GeometryState { position: DVec3::ZERO, rotation: DQuat::IDENTITY };

        let mut sa = GpuSurfaceArea::new();
        let index = sa.add_body(cuboid.into(), initial_state);
        sa.initialize(&device);

        // Get initial area
        sa.calculate_surface_area(
            index,
            [1.0, 0.0, 0.0],
            &device,
            &queue,
        )
        .unwrap();
        let initial_area = sa
            .initialized
            .as_ref()
            .unwrap()
            .result[index];

        // Update state - rotate 90 degrees around Z
        let rotated_state = GeometryState {
            position: DVec3::ZERO,
            rotation: DQuat::from_rotation_z(std::f64::consts::PI / 2.0),
        };

        sa.update_body(index, &rotated_state);

        // Calculate area again
        sa.calculate_surface_area(
            index,
            [1.0, 0.0, 0.0],
            &device,
            &queue,
        )
        .unwrap();
        let rotated_area = sa
            .initialized
            .as_ref()
            .unwrap()
            .result[index];

        // Before rotation: projecting along X gives height*depth = 2*3 = 6
        // After rotation: height and width swap, so we get width*depth = 1*3 = 3
        println!(
            "Initial area: {}, After rotation: {}",
            initial_area, rotated_area
        );

        const EPSILON: f32 = 0.1;
        assert!(
            (initial_area - 6.0).abs() < EPSILON,
            "Initial area should be 6.0, got {}",
            initial_area
        );
        assert!(
            (rotated_area - 3.0).abs() < EPSILON,
            "Rotated area should be 3.0, got {}",
            rotated_area
        );
    }
}
