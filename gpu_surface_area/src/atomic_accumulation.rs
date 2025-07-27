use std::num::NonZeroU64;

use wgpu::PipelineCompilationOptions;

pub struct AtomicAccumulation {
    pipeline: wgpu::ComputePipeline,
    zero_pipeline: wgpu::ComputePipeline,
    bind_group_layout: wgpu::BindGroupLayout,
    bind_group: wgpu::BindGroup,
    staging_buffer: wgpu::Buffer,
    result_buffer: wgpu::Buffer,
}

impl AtomicAccumulation {
    pub fn new(
        device: &wgpu::Device,
        num_objects: usize,
        object_id_texture: &wgpu::TextureView,
    ) -> Self {
        let result_size = (num_objects * std::mem::size_of::<u32>()) as u64;

        let result_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Atomic Accumulation Result Buffer"),
            size: result_size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });

        let staging_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Atomic Accumulation Staging Buffer"),
            size: result_size,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let bind_group_layout = device.create_bind_group_layout(
            &wgpu::BindGroupLayoutDescriptor {
                label: Some("Atomic Accumulation Bind Group Layout"),
                entries: &[
                    // texture binding
                    wgpu::BindGroupLayoutEntry {
                        binding: 0,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Texture {
                            sample_type: wgpu::TextureSampleType::Uint,
                            view_dimension: wgpu::TextureViewDimension::D2,
                            multisampled: false,
                        },
                        count: None,
                    },
                    // storage buffer
                    wgpu::BindGroupLayoutEntry {
                        binding: 1,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: NonZeroU64::new(std::mem::size_of::<u32>() as u64),
                        },
                        count: None,
                    },
                ],
            },
        );

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Atomic Accumulation Bind Group"),
            layout: &bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(object_id_texture),
                },
                wgpu::BindGroupEntry { binding: 1, resource: result_buffer.as_entire_binding() },
            ],
        });

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Atomic Accumulation Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("atomic_accumulation.wgsl").into()),
        });

        let pipeline_layout = device.create_pipeline_layout(
            &wgpu::PipelineLayoutDescriptor {
                label: Some("Atomic Accumulation Pipeline Layout"),
                bind_group_layouts: &[&bind_group_layout],
                push_constant_ranges: &[],
            },
        );

        let pipeline = device.create_compute_pipeline(
            &wgpu::ComputePipelineDescriptor {
                label: Some("Atomic Accumulation Pipeline"),
                layout: Some(&pipeline_layout),
                module: &shader,
                entry_point: Some("main"),
                cache: None,
                compilation_options: PipelineCompilationOptions::default(),
            },
        );

        let zero_pipeline = device.create_compute_pipeline(
            &wgpu::ComputePipelineDescriptor {
                label: Some("Atomic Zero Pipeline"),
                layout: Some(&pipeline_layout),
                module: &shader,
                entry_point: Some("zero_buffer"),
                cache: None,
                compilation_options: PipelineCompilationOptions::default(),
            },
        );

        Self {
            pipeline,
            zero_pipeline,
            bind_group_layout,
            bind_group,
            staging_buffer,
            result_buffer,
        }
    }

    pub fn calculate(
        &self,
        object_id_texture: &wgpu::TextureView,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        resolution: u32,
    ) -> Vec<u32> {
        let mut encoder = device.create_command_encoder(
            &wgpu::CommandEncoderDescriptor { label: Some("Atomic Accumulation Encoder") },
        );

        self.clear(
            &mut encoder,
            (self
                .staging_buffer
                .size()
                / 4) as u32,
        );

        {
            let mut cpass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("Atomic Accumulation Pass"),
                timestamp_writes: None,
            });

            cpass.set_pipeline(&self.pipeline);
            cpass.set_bind_group(0, &self.bind_group, &[]);
            cpass.dispatch_workgroups(
                (resolution + 7) / 8,
                (resolution + 7) / 8,
                1,
            );
        }

        // Copy result buffer to staging buffer for CPU read
        encoder.copy_buffer_to_buffer(
            &self.result_buffer,
            0,
            &self.staging_buffer,
            0,
            self.staging_buffer
                .size(),
        );

        // Submit GPU work
        queue.submit(Some(encoder.finish()));

        // Wait and map buffer for read
        let buffer_slice = self
            .staging_buffer
            .slice(..);
        let (tx, rx) = futures::channel::oneshot::channel();
        buffer_slice.map_async(
            wgpu::MapMode::Read,
            move |v| {
                tx.send(v)
                    .unwrap()
            },
        );
        device
            .poll(wgpu::PollType::Wait)
            .expect("Failed to poll device");
        pollster::block_on(async {
            rx.await
                .unwrap()
                .unwrap();
        });

        let data = buffer_slice.get_mapped_range();
        let result: Vec<u32> = bytemuck::cast_slice(&data).to_vec();
        drop(data);
        self.staging_buffer
            .unmap();

        result
    }

    pub fn clear(&self, encoder: &mut wgpu::CommandEncoder, n_objects: u32) {
        let mut cpass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("Atomic Clear Pass"),
            timestamp_writes: None,
        });

        cpass.set_pipeline(&self.zero_pipeline);
        cpass.set_bind_group(0, &self.bind_group, &[]);
        cpass.dispatch_workgroups((n_objects + 63) / 64, 1, 1);
    }
}
