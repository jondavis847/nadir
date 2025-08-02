pub mod stage1;
pub mod stage2;
pub mod stage_buffers;
use stage1::Stage1;
use stage2::Stage2;
use wgpu::util::DeviceExt;

use crate::parallel_reduction::stage_buffers::WorkgroupResult;

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
    stage2: Vec<Stage2>,
    result_buffer: wgpu::Buffer,
    pub result: Vec<WorkgroupResult>,
    texture_width: u32,
    texture_height: u32,
}

impl ParallelReduction {
    const WORKGROUP_WIDTH: u32 = 16;
    const WORKGROUP_HEIGHT: u32 = 16;
    const WORKGROUP_SIZE: u32 = 256;

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

        let mut stage2 = Vec::new();
        if stage1
            .buffers
            .length
            > Self::WORKGROUP_SIZE
        {
            stage2.push(Stage2::new(
                device,
                &uniform_bind_group_layout,
                &stage1.buffers,
            ));

            while stage2
                .last()
                .unwrap()
                .buffers
                .length
                > Self::WORKGROUP_SIZE
            {
                let last_stage = stage2
                    .last()
                    .unwrap();
                let next_stage = Stage2::new(
                    device,
                    &uniform_bind_group_layout,
                    &last_stage.buffers,
                );
                stage2.push(next_stage);
            }
        }

        let result_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Parallel Reduction Result Buffer"),
            size: std::mem::size_of::<WorkgroupResult>() as u64 * num_objects as u64,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let result = Vec::with_capacity(num_objects as usize);

        Self {
            uniforms,
            uniform_bind_group,
            uniform_bind_group_layout,
            uniform_buffer,
            stage1,
            stage2,
            result,
            result_buffer,
            texture_height,
            texture_width,
        }
    }

    pub fn dispatch(&mut self, device: &wgpu::Device, queue: &wgpu::Queue) {
        let mut encoder = device.create_command_encoder(
            &wgpu::CommandEncoderDescriptor { label: Some("ParallelReduction Command Encoder") },
        );

        // Stage 1 - Process texture data and perform initial reduction
        {
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("ParallelReduction Stage1 Pass"),
                timestamp_writes: None,
            });

            compute_pass.set_pipeline(
                &self
                    .stage1
                    .pipeline,
            );
            compute_pass.set_bind_group(
                0,
                &self.uniform_bind_group,
                &[],
            );
            compute_pass.set_bind_group(
                1,
                &self
                    .stage1
                    .bind_group,
                &[],
            );

            let dispatch_x =
                (self.texture_width + Self::WORKGROUP_WIDTH - 1) / Self::WORKGROUP_WIDTH;
            let dispatch_y =
                (self.texture_height + Self::WORKGROUP_HEIGHT - 1) / Self::WORKGROUP_HEIGHT;

            compute_pass.dispatch_workgroups(dispatch_x, dispatch_y, 1);
        }

        // Stage 2 - Continue to reduce results from previous stages until final result is achieved
        for (i, stage) in self
            .stage2
            .iter()
            .enumerate()
        {
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some(&format!(
                    "ParallelReduction Stage2 Pass {}",
                    i
                )),
                timestamp_writes: None,
            });

            compute_pass.set_pipeline(&stage.pipeline);
            compute_pass.set_bind_group(
                0,
                &self.uniform_bind_group,
                &[],
            );
            compute_pass.set_bind_group(1, &stage.bind_group, &[]);

            let dispatch_x = ceil_div(
                stage
                    .buffers
                    .length,
                Self::WORKGROUP_SIZE,
            );
            compute_pass.dispatch_workgroups(dispatch_x, 1, 1);
        }

        // --- Copy final stage buffer to result buffer ---
        if let Some(final_stage) = self
            .stage2
            .last()
        {
            let byte_size = std::mem::size_of::<WorkgroupResult>() as u64
                * self
                    .uniforms
                    .num_objects as u64;
            encoder.copy_buffer_to_buffer(
                &final_stage
                    .buffers
                    .result_buffer,
                0,
                &self.result_buffer,
                0,
                byte_size,
            );
        } else {
            let byte_size = std::mem::size_of::<WorkgroupResult>() as u64
                * self
                    .uniforms
                    .num_objects as u64;
            encoder.copy_buffer_to_buffer(
                &self
                    .stage1
                    .buffers
                    .result_buffer,
                0,
                &self.result_buffer,
                0,
                byte_size,
            );
        }

        // --- Submit command buffer ---
        let command_buffer = encoder.finish();
        queue.submit(Some(command_buffer));

        // Wait for completion (especially useful before mapping)
        device
            .poll(wgpu::PollType::Wait)
            .expect("poll error");

        self.read_results(device);
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

    fn read_results(&mut self, device: &wgpu::Device) {
        let slice = self
            .result_buffer
            .slice(..);

        let (tx, rx) = futures::channel::oneshot::channel();

        slice.map_async(
            wgpu::MapMode::Read,
            move |res| {
                // ignore send errors: receiver will always be alive here
                let _ = tx.send(res);
            },
        );

        device
            .poll(wgpu::PollType::Wait)
            .expect("poll error");

        pollster::block_on(rx)
            .unwrap()
            .expect("Mapping failed");

        let data = slice.get_mapped_range();

        let slice: &[WorkgroupResult] = bytemuck::cast_slice(&data);

        self.result
            .clear();
        self.result
            .extend_from_slice(slice);

        drop(data);
        self.result_buffer
            .unmap();
    }
}

// helper ---
#[inline]
pub fn ceil_div(n: u32, d: u32) -> u32 {
    (n + d - 1) / d
}
