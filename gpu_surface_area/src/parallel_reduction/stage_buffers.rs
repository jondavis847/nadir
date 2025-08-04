#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable, Debug)]
pub struct WorkgroupResult {
    pub id: u32,
    pub count: u32,
    pub pos: glam::Vec3,
    _padding: glam::Vec3,
}

// The buffers will store the summed results per workgroup
pub struct StageBuffers {
    pub length: u32,
    pub result_buffer: wgpu::Buffer,
}

impl StageBuffers {
    pub fn new(device: &wgpu::Device, buffer_length: u32) -> Self {
        let buffer_size = std::mem::size_of::<WorkgroupResult>() as u32 * buffer_length;
        let result_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("workgroup_results"),
            size: buffer_size as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: true,
        });
        // initialize to zero to prevent garbage data
        {
            let mut buffer_view = result_buffer
                .slice(..)
                .get_mapped_range_mut();
            buffer_view.fill(0);
        }
        result_buffer.unmap();

        Self { length: buffer_length, result_buffer }
    }
}
