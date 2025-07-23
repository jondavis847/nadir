use std::collections::HashMap;

use glam::{Mat4, Vec3};
use nadir_3d::{
    geometry::{Geometry, GeometryState, GeometryTrait, GeometryTransform},
    vertex::simple_vertices,
};
use wgpu::util::DeviceExt;

use crate::surface_area::SurfaceAreaCalculator;

pub mod surface_area;

pub struct GpuInitialized {
    device: wgpu::Device,
    queue: wgpu::Queue,
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
    vertex_buffers: Vec<wgpu::Buffer>,
    transform_buffer: wgpu::Buffer,
    resolution: u32,
}

pub struct GpuCalculator {
    method: GpuCalculatorMethod,
    geometry: HashMap<GeometryId, GeometryData>,
    surface_area: Option<SurfaceAreaCalculator>,
    //center_of_pressure: Option<CenterOfPressureCalculator>,
    //solar_radiation_pressure: Option<SrpCalculator>,
    initialized: Option<GpuInitialized>,
}

impl GpuCalculator {
    pub fn new() -> Self {
        Self {
            method: GpuCalculatorMethod::Rasterization { resolution: 1024, safety_factor: 1.1 },
            geometry: HashMap::new(),
            surface_area: None,
            initialized: None,
        }
    }

    pub fn add_geometry(&mut self, geometry: Geometry, state: &GeometryState) -> GeometryId {
        let id = GeometryId(
            self.geometry
                .len(),
        );
        let transform = geometry.get_transform(state);
        self.geometry
            .insert(
                id,
                GeometryData { geometry, transform },
            );
        id
    }

    pub fn update_geometry(&mut self, id: GeometryId, state: &GeometryState) {
        if let Some(geometry) = self
            .geometry
            .get_mut(&id)
        {
            geometry.transform = geometry
                .geometry
                .get_transform(state);
        } else {
            panic!(
                "Geometry with ID {:?} not found",
                id
            );
        }
    }

    // don't initialize until all geometries have been added
    // if geometries are added after initilization, can i just re call this, or do i need to drop gpu resources first somehow?
    pub fn initialize(&mut self) {
        let (resolution, safety_factor) = match self.method {
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

        // Create transform buffer
        let transform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Transform Buffer"),
            size: 0, // to be resized later
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        // Create vertex buffers for each geometry
        let mut vertex_buffers = Vec::new();
        for geometry in self
            .geometry
            .values()
        {
            // Create vertex buffers for each geometry
            let vertices = simple_vertices(
                &geometry
                    .geometry
                    .get_vertices(),
            );
            let vertex_buffer = device.create_buffer_init(
                &wgpu::util::BufferInitDescriptor {
                    label: Some("Vertex Buffer"),
                    contents: bytemuck::cast_slice(&vertices),
                    usage: wgpu::BufferUsages::VERTEX,
                },
            );
            vertex_buffers.push(vertex_buffer);
        }

        self.initialized = Some(GpuInitialized {
            device,
            queue,
            object_id_texture: object_id_texture.create_view(&Default::default()),
            position_texture: position_texture.create_view(&Default::default()),
            depth_texture: depth_texture.create_view(&Default::default()),
            transform_buffer,
            resolution,
            vertex_buffers,
        });
    }
}

fn create_orthographic_projection(
    bounds: &SceneBounds,
    view_direction: &[f32; 3],
    scale_factor: f32,
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
    let size = (max - min) * scale_factor;

    let dir = Vec3::from(*view_direction).normalize();
    let up = if dir
        .y
        .abs()
        > 0.9
    {
        Vec3::Z
    } else {
        Vec3::Y
    };
    let eye = center - dir * size.length();
    let view = Mat4::look_at_rh(eye, center, up);
    let ortho = Mat4::orthographic_rh(
        -size.x,
        size.x,
        -size.y,
        size.y,
        0.1,
        size.length() * 2.0,
    );
    (ortho * view).to_cols_array_2d()
}
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

pub struct GeometryData {
    geometry: Geometry,
    transform: GeometryTransform,
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
}
