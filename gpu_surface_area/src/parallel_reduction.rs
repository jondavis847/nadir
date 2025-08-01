pub mod stage1;
use stage1::Stage1;
use wgpu::util::DeviceExt;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Uniforms {
    num_objects: u32,
    num_workgroups: u32,
    _pad0: u32,
    _pad1: u32,
}

impl Uniforms {
    pub fn new(num_objects: u32, num_workgroups: u32) -> Self {
        Self { num_objects, num_workgroups, _pad0: 0, _pad1: 0 }
    }
}

pub struct ParallelReduction {
    uniforms: Uniforms,
    uniform_bind_group_layout: wgpu::BindGroupLayout,
    uniform_bind_group: wgpu::BindGroup,
    uniform_buffer: wgpu::Buffer,
    stage1: Stage1,
}

impl ParallelReduction {
    const WORKGROUP_WIDTH: u32 = 16;
    const WORKGROUP_HEIGHT: u32 = 16;

    pub fn new(
        device: &wgpu::Device,
        object_id_texture: &wgpu::TextureView,
        position_texture: &wgpu::TextureView,
        num_objects: u32,
    ) -> Self {
        // Get dimenions of the textures, make sure they match
        let object_id_size = object_id_texture
            .texture()
            .size();
        let position_size = position_texture
            .texture()
            .size();
        assert!(
            object_id_size.width == position_size.width
                && object_id_size.height == position_size.height,
            "Object ID and Position textures must have the same dimensions",
        );
        let texture_width = object_id_size.width;
        let texture_height = object_id_size.height;

        // Calculate the number of workgroups required for the calculations
        let wg_x = (texture_width + Self::WORKGROUP_WIDTH - 1) / Self::WORKGROUP_WIDTH;
        let wg_y = (texture_height + Self::WORKGROUP_HEIGHT - 1) / Self::WORKGROUP_HEIGHT;
        let num_workgroups = wg_x * wg_y;

        let uniforms = Uniforms::new(num_objects, num_workgroups);
        let uniform_bind_group_layout = Self::bind_group_layout(device);
        let uniform_buffer = device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("Parallel Reduction Uniform Buffer"),
                contents: bytemuck::bytes_of(&uniforms),
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            },
        );
        let uniform_bind_group = Self::bind_group(
            device,
            &uniform_bind_group_layout,
            &uniform_buffer,
        );

        let stage1 = Stage1::new(
            device,
            &uniforms,
            &uniform_bind_group_layout,
            object_id_texture.clone(),
            position_texture.clone(),
        );

        Self {
            uniforms,
            uniform_bind_group,
            uniform_bind_group_layout,
            uniform_buffer,
            stage1,
        }
    }

    fn bind_group_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
        device.create_bind_group_layout(
            &wgpu::BindGroupLayoutDescriptor {
                label: Some("Parallel Reduction Uniform BindGroupLayout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            },
        )
    }

    fn bind_group(
        device: &wgpu::Device,
        layout: &wgpu::BindGroupLayout,
        uniform_buffer: &wgpu::Buffer,
    ) -> wgpu::BindGroup {
        device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Parallel Reduction Uniform Bind Group"),
            layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buffer.as_entire_binding(),
            }],
        })
    }
}
