use std::collections::HashMap;

use bytemuck::{Pod, Zeroable};
use glam::{Mat4, Vec3, Vec4};
use nadir_3d::{
    geometry::{Geometry, GeometryState, GeometryTrait},
    vertex::simple_vertices,
};
use wgpu::util::DeviceExt;

use crate::surface_area::SurfaceAreaCalculator;

pub mod atomic_accumulation;
pub mod parallel_reduction;
pub mod surface_area;

pub enum GpuCalculatorMethod {
    Rasterization {
        resolution: u32,
        safety_factor: f32,
    },
    RayTracing {
        nrays_x: u32,
        nrays_y: u32,
    },
}

#[derive(Eq, PartialEq, Hash, Copy, Clone, Debug)]
pub struct GeometryId(usize);

#[derive(Debug)]
pub struct GeometryData {
    geometry: Geometry,
    uniforms: GeometryUniforms,
    max_vertex_mag: f32,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct SceneBounds {
    min_x: f32,
    max_x: f32,
    min_y: f32,
    max_y: f32,
    min_z: f32,
    max_z: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Zeroable, Pod)]
struct SharedUniforms {
    projection_matrix: Mat4,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Zeroable, Pod)]
struct GeometryUniforms {
    world_transform: Mat4,
    id: u32,
    _padding: [u32; 3], // Align to 16 bytes
}

pub struct GpuGeometryResources {
    bind_group: wgpu::BindGroup,
    uniform_buffer: wgpu::Buffer,
    vertex_buffer: wgpu::Buffer,
    vertex_count: u32,
}

pub struct GpuInitialized {
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    // object_id_texture:
    /// Stores a unique integer ID for each visible fragment's geometry.
    /// Each object gets a unique ID (1, 2, 3, ...), and 0 means background.
    /// Used to count visible pixels per object (e.g., for surface area calculation).
    object_id_texture: wgpu::TextureView,
    /// position_texture:
    /// Stores the interpolated body-space or world-space position of each visible fragment.
    /// Used to compute the center of pressure by summing (area * position) per fragment.
    /// Also useful for lighting, normals, or SRP direction-based force calculations.
    position_texture: wgpu::TextureView,
    /// depth_texture:
    /// Stores depth values for fragments and enables depth testing during rasterization.
    /// Ensures only the nearest (visible) fragments are drawn when objects overlap.
    /// Prevents back surfaces and occluded geometry from contributing to area/CoP/SRP.
    depth_texture: wgpu::TextureView,
    geometry: HashMap<GeometryId, GpuGeometryResources>,
    geometry_uniform_bindgroup_layout: wgpu::BindGroupLayout,
    //reduction_buffers: ReductionBuffers,
    shared_uniform_buffer: wgpu::Buffer,
    shared_bind_group: wgpu::BindGroup,
    shared_bind_group_layout: wgpu::BindGroupLayout,
    resolution: u32,
}

pub struct GpuCalculator {
    method: GpuCalculatorMethod,
    geometry: HashMap<GeometryId, GeometryData>,
    surface_area: Option<SurfaceAreaCalculator>,
    //center_of_pressure: Option<CenterOfPressureCalculator>,
    //solar_radiation_pressure: Option<SrpCalculator>,
    initialized: Option<GpuInitialized>,
    scene_bounds: SceneBounds,
}

impl GpuCalculator {
    pub fn new() -> Self {
        Self {
            method: GpuCalculatorMethod::Rasterization { resolution: 1024, safety_factor: 1.0 },
            geometry: HashMap::new(),
            surface_area: None,
            initialized: None,
            scene_bounds: SceneBounds::default(),
        }
    }

    pub fn add_geometry(&mut self, geometry: Geometry, state: &GeometryState) -> GeometryId {
        let id = GeometryId(
            self.geometry
                .len()
                + 1,
        );

        let world_transform = geometry
            .get_transform(state)
            .transformation_matrix;

        // Create a transform without translation (only scale + rotation)
        let mut scale_rotation_transform = world_transform;
        scale_rotation_transform.w_axis = Vec4::new(0.0, 0.0, 0.0, 1.0); // Remove translation

        let vertices = geometry.get_vertices();
        let mut max_vertex_mag = 0.0;
        for vertex in vertices {
            // Transform vertex with scale + rotation only
            let transformed_pos = scale_rotation_transform
                * vertex
                    .pos
                    .extend(1.0);
            let transformed_pos_3d = transformed_pos.truncate();

            let mag = transformed_pos_3d.length();
            if mag > max_vertex_mag {
                max_vertex_mag = mag;
            }
        }

        self.geometry
            .insert(
                id,
                GeometryData {
                    geometry,
                    uniforms: GeometryUniforms {
                        world_transform,
                        id: id.0 as u32,
                        _padding: [0; 3],
                    },
                    max_vertex_mag,
                },
            );

        id
    }

    pub fn calculate_scene_bounds(&mut self) {
        // for scene bounds calculations
        let mut scene_bounds = SceneBounds {
            min_x: std::f32::INFINITY,
            max_x: -std::f32::INFINITY,
            min_y: std::f32::INFINITY,
            max_y: -std::f32::INFINITY,
            min_z: std::f32::INFINITY,
            max_z: -std::f32::INFINITY,
        };

        for (_, geometry) in &self.geometry {
            // calculate bounding box
            let world_position = geometry
                .uniforms
                .world_transform
                .col(3);

            let geometry_bounds = SceneBounds {
                min_x: world_position.x - geometry.max_vertex_mag,
                max_x: world_position.x + geometry.max_vertex_mag,
                min_y: world_position.y - geometry.max_vertex_mag,
                max_y: world_position.y + geometry.max_vertex_mag,
                min_z: world_position.z - geometry.max_vertex_mag,
                max_z: world_position.z + geometry.max_vertex_mag,
            };
            scene_bounds.min_x = scene_bounds
                .min_x
                .min(geometry_bounds.min_x);
            scene_bounds.max_x = scene_bounds
                .max_x
                .max(geometry_bounds.max_x);
            scene_bounds.min_y = scene_bounds
                .min_y
                .min(geometry_bounds.min_y);
            scene_bounds.max_y = scene_bounds
                .max_y
                .max(geometry_bounds.max_y);
            scene_bounds.min_z = scene_bounds
                .min_z
                .min(geometry_bounds.min_z);
            scene_bounds.max_z = scene_bounds
                .max_z
                .max(geometry_bounds.max_z);
        }
        self.scene_bounds = scene_bounds;
    }

    pub fn calculate_surface_area(&mut self, view_direction: &[f32; 3]) {
        // recalculate the scene bounds with new transform information
        // FIXME: if we do this once per calc (aero, srp, cop) that seems wasteful
        self.calculate_scene_bounds();

        if let Some(initialized) = &mut self.initialized {
            let surface_area_calc = self
                .surface_area
                .as_mut()
                .unwrap();

            let safety_factor = match self.method {
                GpuCalculatorMethod::Rasterization { safety_factor, .. } => safety_factor,
                GpuCalculatorMethod::RayTracing { .. } => todo!(),
            };

            surface_area_calc.calculate(
                &initialized.device,
                &initialized.queue,
                &initialized.geometry,
                &self.scene_bounds,
                view_direction,
                initialized.resolution,
                safety_factor,
                &initialized.shared_uniform_buffer,
                &initialized.shared_bind_group,
                &initialized.object_id_texture,
                &initialized.position_texture,
                &initialized.depth_texture,
            );
        } else {
            panic!("GpuCalculator not initialized");
        }
    }

    // don't initialize until all geometries have been added
    // if geometries are added after initilization, can i just re call this, or do i need to drop gpu resources first somehow?
    pub fn initialize(&mut self) {
        let (resolution, _) = match self.method {
            GpuCalculatorMethod::Rasterization { resolution, safety_factor } => {
                (resolution, safety_factor)
            }
            _ => panic!("Ray tracing method not implemented yet"),
        };

        // Create WGPU device and queue
        let instance = wgpu::Instance::default();
        let adapter = pollster::block_on(
            instance.request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: None,
                force_fallback_adapter: false,
            }),
        )
        .expect("Failed to find a GPU adapter");

        let (device, queue) = pollster::block_on(
            adapter.request_device(&wgpu::DeviceDescriptor {
                label: Some("GPU Device"),
                ..Default::default()
            }),
        )
        .expect("Failed to get device");

        // Create textures
        let size = wgpu::Extent3d {
            width: resolution,
            height: resolution,
            depth_or_array_layers: 1,
        };

        let object_id_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("Object ID Texture"),
            size,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::R32Uint,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                | wgpu::TextureUsages::TEXTURE_BINDING
                | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });

        let position_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("Position Texture"),
            size,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba32Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                | wgpu::TextureUsages::TEXTURE_BINDING
                | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });

        let depth_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("Depth Texture"),
            size,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth32Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });

        let mut gpu_geometries = HashMap::new();

        // Create shared bind group layout (for projection matrix)
        let shared_bind_group_layout = device.create_bind_group_layout(
            &wgpu::BindGroupLayoutDescriptor {
                label: Some("Shared Bind Group Layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            },
        );

        // Create the uniform bind group layout
        let geometry_uniform_bindgroup_layout = device.create_bind_group_layout(
            &wgpu::BindGroupLayoutDescriptor {
                label: Some("Geometry Uniform Bind Group Layout"),
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

        // Create shared uniform buffer
        let shared_uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Shared Uniform Buffer"),
            size: std::mem::size_of::<SharedUniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        // Create shared bind group
        let shared_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Shared Bind Group"),
            layout: &shared_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: shared_uniform_buffer.as_entire_binding(),
            }],
        });

        for (id, geometry) in &self.geometry {
            // Create uniform buffer per geometry
            let uniform_buffer = device.create_buffer_init(
                &wgpu::util::BufferInitDescriptor {
                    label: Some(&format!(
                        "Uniform Buffer {}",
                        id.0
                    )),
                    contents: bytemuck::cast_slice(&[geometry.uniforms]),
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                },
            );

            // Create bind group using the layout and buffer
            let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some(&format!(
                    "Bind Group {}",
                    id.0
                )),
                layout: &geometry_uniform_bindgroup_layout,
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: uniform_buffer.as_entire_binding(),
                }],
            });

            // Create vertex buffers for each geometry
            let vertices = simple_vertices(
                &geometry
                    .geometry
                    .get_vertices(),
            );
            let vertex_buffer = device.create_buffer_init(
                &wgpu::util::BufferInitDescriptor {
                    label: Some(&format!(
                        "Vertex Buffer {}",
                        id.0
                    )),
                    contents: bytemuck::cast_slice(&vertices),
                    usage: wgpu::BufferUsages::VERTEX,
                },
            );

            let gpu_geometry = GpuGeometryResources {
                bind_group,
                uniform_buffer,
                vertex_buffer,
                vertex_count: vertices.len() as u32,
            };

            gpu_geometries.insert(*id, gpu_geometry);
        }

        // let reduction_buffers = ReductionBuffers::new(
        //     &device,
        //     self.geometry
        //         .len(),
        // );

        self.initialized = Some(GpuInitialized {
            device,
            queue,
            object_id_texture: object_id_texture.create_view(&Default::default()),
            position_texture: position_texture.create_view(&Default::default()),
            depth_texture: depth_texture.create_view(&Default::default()),
            resolution,
            geometry: gpu_geometries,
            geometry_uniform_bindgroup_layout,
            // reduction_buffers,
            shared_uniform_buffer,
            shared_bind_group,
            shared_bind_group_layout,
        });

        // if let Some(gpu) = &self.initialized {
        //     // initialize center of pressure area
        //     if let Some(surface_area) = &mut self.surface_area {
        //         surface_area.initialize(
        //             &gpu.device,
        //             &gpu.geometry_uniform_bindgroup_layout,
        //             resolution,
        //         );
        //     }
        // }

        // if let Some(gpu) = &self.initialized {
        //     // initialize solar radiation pressure
        //     if let Some(surface_area) = &mut self.surface_area {
        //         surface_area.initialize(
        //             &gpu.device,
        //             &gpu.geometry_uniform_bindgroup_layout,
        //             resolution,
        //         );
        //     }
        // }

        if let Some(gpu) = &self.initialized {
            // initialize surface area
            if let Some(surface_area) = &mut self.surface_area {
                surface_area.initialize(
                    &gpu.device,
                    &gpu.shared_bind_group_layout,
                    &gpu.geometry_uniform_bindgroup_layout,
                    gpu.geometry
                        .len(),
                    &gpu.object_id_texture,
                    &gpu.position_texture,
                );
            }
        }
    }

    pub fn update_geometry(&mut self, id: GeometryId, state: &GeometryState) {
        if let Some(geometry_data) = self
            .geometry
            .get_mut(&id)
        {
            // Update the CPU-side uniforms
            geometry_data
                .uniforms
                .world_transform = geometry_data
                .geometry
                .get_transform(state)
                .transformation_matrix;

            // Update the GPU uniform buffer
            if let Some(initialized) = &mut self.initialized {
                if let Some(gpu_geometry) = initialized
                    .geometry
                    .get(&id)
                {
                    // Write the updated uniforms to the GPU buffer
                    initialized
                        .queue
                        .write_buffer(
                            &gpu_geometry.uniform_buffer,
                            0, // offset
                            bytemuck::cast_slice(&[geometry_data.uniforms]),
                        );
                }
            }
        } else {
            panic!(
                "Geometry with ID {:?} not found",
                id
            );
        }
    }

    pub fn with_resolution(mut self, new_resolution: u32) -> Self {
        match &mut self.method {
            GpuCalculatorMethod::Rasterization { resolution, .. } => {
                *resolution = new_resolution;
            }
            _ => {}
        }
        self
    }

    pub fn with_surface_area(mut self) -> Self {
        self.surface_area = Some(SurfaceAreaCalculator::new());
        self
    }
}

fn create_orthographic_projection(
    bounds: &SceneBounds,
    view_direction: &[f32; 3],
    safety_factor: f32,
) -> [[f32; 4]; 4] {
    let min = Vec3::new(
        bounds.min_x,
        bounds.min_y,
        bounds.min_z,
    );
    let max = Vec3::new(
        bounds.max_x,
        bounds.max_y,
        bounds.max_z,
    );
    let center = (min + max) * 0.5;

    // Calculate half-extents, not full size
    let half_extents = (max - min) * 0.5 * safety_factor;

    let dir = Vec3::from(*view_direction).normalize();

    let world_up = Vec3::Z;

    let right = if dir
        .cross(world_up)
        .length()
        > 0.001
    {
        dir.cross(world_up)
            .normalize()
    } else {
        // View direction is parallel to Z, use Y as fallback
        Vec3::Y
    };
    let up = right
        .cross(dir)
        .normalize();

    let eye = center - dir * half_extents.length() * 2.0;

    //let view = Mat4::look_at_rh(eye, center, up);
    //Build view matrix manually instead of using look_at_rh
    // sinec glams frame is x right y up z out versus out frame of x out y right z up
    let view = Mat4::from_cols(
        Vec4::new(right.x, up.x, -dir.x, 0.0), // Column 0
        Vec4::new(right.y, up.y, -dir.y, 0.0), // Column 1
        Vec4::new(right.z, up.z, -dir.z, 0.0), // Column 2
        Vec4::new(
            -right.dot(eye),
            -up.dot(eye),
            dir.dot(eye),
            1.0,
        ), // Column 3
    );

    // Use half-extents for orthographic bounds
    let ortho = Mat4::orthographic_rh(
        -half_extents.y,
        half_extents.y,
        -half_extents.z,
        half_extents.z,
        0.01 * half_extents.length(),
        half_extents.length() * 4.0,
    );

    (ortho * view).to_cols_array_2d()
}

#[cfg(test)]
mod tests {
    use std::{f64::consts::PI, time::Instant};

    use super::*;
    use glam::{DQuat, DVec3};
    use nadir_3d::geometry::{GeometryState, cuboid::Cuboid};

    #[test]
    fn test_cube_area() {
        // Create a GPU calculator with surface area capability
        let mut gpu_calc = GpuCalculator::new().with_surface_area();

        // Build test geometry (unit cube)
        let cube_geometry = Cuboid::new(1.0, 1.0, 1.0).unwrap();
        let geometry_state = GeometryState::default(); // Assuming default state is identity transform

        // Add geometry to the calculator
        let cube_id = gpu_calc.add_geometry(
            cube_geometry.into(),
            &geometry_state,
        );

        // Initialize the GPU resources
        gpu_calc.initialize();

        // Front-on (Z) view - looking at the cube from the front
        let view_direction = [0.0, 0.0, -1.0];

        // Calculate surface area using the new framework
        gpu_calc.calculate_surface_area(&view_direction);

        let result = gpu_calc
            .surface_area
            .unwrap()
            .aerodynamics_result;

        // Only one object, expect just front face: area should be close to 1.0
        assert_eq!(
            result.len(),
            1,
            "Should have one geometry"
        );
        let calculated_area = result
            .get(&cube_id)
            .unwrap();

        println!(
            "Calculated area: {:?}",
            calculated_area.surface_area
        );

        // For a unit cube viewed from the front, we should see approximately 1.0 square units
        assert!(
            (calculated_area.surface_area - 1.0).abs() < 0.05, // Allow small error due to rasterization
            "Expected ~1.0, got {:?}",
            calculated_area
        );
    }

    #[test]
    fn test_cube_area_multiple_views() {
        let mut gpu_calc = GpuCalculator::new().with_surface_area();

        let cube_geometry = Cuboid::new(2.0, 2.0, 2.0).unwrap(); // 2x2x2 cube
        let geometry_state = GeometryState::default();

        let cube_id = gpu_calc.add_geometry(
            cube_geometry.into(),
            &geometry_state,
        );
        gpu_calc.initialize();

        // Test different viewing directions
        let test_cases = [
            ([0.0, 0.0, -1.0], "front"),  // Front face
            ([0.0, 0.0, 1.0], "back"),    // Back face
            ([1.0, 0.0, 0.0], "right"),   // Right face
            ([-1.0, 0.0, 0.0], "left"),   // Left face
            ([0.0, 1.0, 0.0], "top"),     // Top face
            ([0.0, -1.0, 0.0], "bottom"), // Bottom face
        ];

        for (view_direction, name) in test_cases {
            gpu_calc.calculate_surface_area(&view_direction);

            let result = gpu_calc
                .surface_area
                .as_ref()
                .unwrap();

            let calculated_area = result
                .aerodynamics_result
                .get(&cube_id)
                .unwrap();
            println!(
                "{} view calculated area: {:?}",
                name, calculated_area.surface_area
            );

            // Each face of a 2x2 cube should have area 4.0
            assert!(
                (calculated_area.surface_area - 4.0).abs() < 1.0,
                "{} view: Expected ~4.0, got {:?}",
                name,
                calculated_area
            );
        }
    }

    #[test]
    fn test_rotated_cube_area() {
        // Create a GPU calculator with surface area capability
        let mut gpu_calc = GpuCalculator::new().with_surface_area();

        // Build test geometry (unit cube)
        let cube_geometry = Cuboid::new(1.0, 1.0, 1.0).unwrap();
        let geometry_state = GeometryState {
            position: DVec3::ZERO,
            rotation: DQuat::from_euler(
                glam::EulerRot::YXZ,
                PI / 4.0,
                PI / 4.0,
                0.0,
            ),
        };

        // Add geometry to the calculator
        let cube_id = gpu_calc.add_geometry(
            cube_geometry.into(),
            &geometry_state,
        );

        // Initialize the GPU resources
        gpu_calc.initialize();

        // Front-on (X) view - looking at the cube from the front
        let view_direction = [-1.0, 0.0, 0.0];

        // Calculate surface area using the new framework
        let start = Instant::now();
        gpu_calc.calculate_surface_area(&view_direction);
        let stop = Instant::now();
        let duration = stop.duration_since(start);
        dbg!(duration);

        let result = gpu_calc
            .surface_area
            .as_ref()
            .unwrap();
        // Only one object, expect just front face: area should be close to 1.0
        assert_eq!(
            result
                .aerodynamics_result
                .len(),
            1,
            "Should have one geometry"
        );
        dbg!(cube_id);
        dbg!(&result.aerodynamics_result);
        let calculated_area = result
            .aerodynamics_result
            .get(&cube_id)
            .unwrap();

        println!(
            "Calculated area: {:?}",
            calculated_area.surface_area
        );

        // For a unit cube viewed from the front, we should see approximately 1.0 square units
        assert!(
            (calculated_area.surface_area - (3.0 as f64).sqrt()).abs() < 0.05, // Allow small error due to rasterization
            "Expected ~1.0, got {:?}",
            calculated_area
        );
    }

    #[test]
    fn test_2_cubes_area() {
        // Create a GPU calculator with surface area capability
        let mut gpu_calc = GpuCalculator::new()
            .with_surface_area()
            .with_resolution(1024);

        // Build test geometry (unit cube)
        let cube_geometry = Cuboid::new(1.0, 1.0, 1.0).unwrap();
        let state1 = GeometryState {
            position: DVec3 { x: 0.0, y: 0.5, z: 0.5 },
            rotation: DQuat::IDENTITY,
        };

        let state2 = GeometryState {
            position: DVec3 { x: 0.0, y: -0.5, z: -0.5 },
            rotation: DQuat::IDENTITY,
        };

        let cube1_id = gpu_calc.add_geometry(cube_geometry.into(), &state1);
        let cube2_id = gpu_calc.add_geometry(cube_geometry.into(), &state2);

        gpu_calc.initialize();

        let view_direction = [-1.0, 0.0, 0.0];

        // Calculate surface area using the new framework
        gpu_calc.calculate_surface_area(&view_direction);

        let result = gpu_calc
            .surface_area
            .as_ref()
            .unwrap();
        // Only one object, expect just front face: area should be close to 1.0
        assert_eq!(
            result
                .aerodynamics_result
                .len(),
            2,
            "Should have two geometry"
        );
        let calculated_area = result
            .aerodynamics_result
            .get(&cube1_id)
            .unwrap()
            .surface_area;

        println!(
            "Cube1 calculated area: {:?}",
            calculated_area
        );

        // For a unit cube viewed from the front, we should see approximately 1.0 square units
        assert!(
            (calculated_area - 1.0).abs() < 0.05, // Allow small error due to rasterization
            "Expected ~1.0, got {:?}",
            calculated_area
        );

        let calculated_area = result
            .aerodynamics_result
            .get(&cube2_id)
            .unwrap()
            .surface_area;

        println!(
            "Calculated area: {:?}",
            calculated_area
        );

        // For a unit cube viewed from the front, we should see approximately 1.0 square units
        assert!(
            (calculated_area - 1.0).abs() < 0.05, // Allow small error due to rasterization
            "Expected ~1.0, got {:?}",
            calculated_area
        );
    }
}
