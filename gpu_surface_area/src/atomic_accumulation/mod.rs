use std::num::NonZeroU64;

use wgpu::PipelineCompilationOptions;

pub struct AtomicAccumulationResult {
    pub pixel_count: u32,
    pub position_sum: [f32; 3],
}

pub struct AtomicAccumulation {
    pipeline: wgpu::ComputePipeline,
    zero_pipeline: wgpu::ComputePipeline,
    bind_group: wgpu::BindGroup,
    per_object_id_sum: wgpu::Buffer,
    per_object_pos_sum_x: wgpu::Buffer,
    per_object_pos_sum_y: wgpu::Buffer,
    per_object_pos_sum_z: wgpu::Buffer,
    staging_id_sum: wgpu::Buffer,
    staging_pos_sum_x: wgpu::Buffer,
    staging_pos_sum_y: wgpu::Buffer,
    staging_pos_sum_z: wgpu::Buffer,
}

impl AtomicAccumulation {
    pub fn new(
        device: &wgpu::Device,
        num_objects: usize,
        object_id_texture: &wgpu::TextureView,
    ) -> Self {
        let result_size = (num_objects * std::mem::size_of::<u32>()) as u64;

        let per_object_id_sum = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Per Object Id Sum Buffer"),
            size: result_size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });

        let per_object_pos_sum_x = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Per Object Position X Sum Buffer"),
            size: result_size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });

        let per_object_pos_sum_y = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Per Object Position Y Sum Buffer"),
            size: result_size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });

        let per_object_pos_sum_z = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Per Object Position Z Sum Buffer"),
            size: result_size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });

        let staging_id_sum = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Per Object Id Staging Buffer"),
            size: result_size,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let staging_pos_sum_x = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Per Object Position X Staging Buffer"),
            size: result_size,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let staging_pos_sum_y = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Per Object Position Y Staging Buffer"),
            size: result_size,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let staging_pos_sum_z = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Per Object Position Z Staging Buffer"),
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
                    // per_obect_id_sum buffer
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
                    // per_obect_pos_sum_x buffer
                    wgpu::BindGroupLayoutEntry {
                        binding: 2,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: NonZeroU64::new(std::mem::size_of::<f32>() as u64),
                        },
                        count: None,
                    },
                    // per_obect_pos_sum_y buffer
                    wgpu::BindGroupLayoutEntry {
                        binding: 3,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: NonZeroU64::new(std::mem::size_of::<f32>() as u64),
                        },
                        count: None,
                    },
                    // per_obect_pos_sum_z buffer
                    wgpu::BindGroupLayoutEntry {
                        binding: 4,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: NonZeroU64::new(std::mem::size_of::<f32>() as u64),
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
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: per_object_id_sum.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: per_object_pos_sum_x.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: per_object_pos_sum_y.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: per_object_pos_sum_z.as_entire_binding(),
                },
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
            bind_group,
            per_object_id_sum,
            per_object_pos_sum_x,
            per_object_pos_sum_y,
            per_object_pos_sum_z,
            staging_id_sum,
            staging_pos_sum_x,
            staging_pos_sum_y,
            staging_pos_sum_z,
        }
    }

    pub fn calculate(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        resolution: u32,
        num_objects: usize,
    ) -> Vec<AtomicAccumulationResult> {
        let mut encoder = device.create_command_encoder(
            &wgpu::CommandEncoderDescriptor { label: Some("Atomic Accumulation Encoder") },
        );

        self.clear(
            &mut encoder,
            (self
                .per_object_id_sum
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

        // Copy all GPU results to staging buffers
        encoder.copy_buffer_to_buffer(
            &self.per_object_id_sum,
            0,
            &self.staging_id_sum,
            0,
            self.staging_id_sum
                .size(),
        );
        encoder.copy_buffer_to_buffer(
            &self.per_object_pos_sum_x,
            0,
            &self.staging_pos_sum_x,
            0,
            self.staging_pos_sum_x
                .size(),
        );
        encoder.copy_buffer_to_buffer(
            &self.per_object_pos_sum_y,
            0,
            &self.staging_pos_sum_y,
            0,
            self.staging_pos_sum_y
                .size(),
        );
        encoder.copy_buffer_to_buffer(
            &self.per_object_pos_sum_z,
            0,
            &self.staging_pos_sum_z,
            0,
            self.staging_pos_sum_z
                .size(),
        );

        // Submit GPU work
        queue.submit(Some(encoder.finish()));

        let pixel_counts = Self::read_pixel_counts(device, &self.staging_id_sum);
        let position_sums = Self::read_position_sums(
            device,
            &self.staging_pos_sum_x,
            &self.staging_pos_sum_y,
            &self.staging_pos_sum_z,
            num_objects,
        );

        let result: Vec<AtomicAccumulationResult> = pixel_counts
            .into_iter()
            .zip(position_sums)
            .map(|(px, sum)| AtomicAccumulationResult { pixel_count: px, position_sum: sum })
            .collect();

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

    fn read_pixel_counts(device: &wgpu::Device, staging_buffer: &wgpu::Buffer) -> Vec<u32> {
        let slice = staging_buffer.slice(..);
        let (tx, rx) = futures::channel::oneshot::channel();
        slice.map_async(
            wgpu::MapMode::Read,
            move |v| {
                tx.send(v)
                    .unwrap();
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

        let mapped = slice.get_mapped_range();
        let data: Vec<u32> = bytemuck::cast_slice(&mapped).to_vec();
        drop(mapped);
        staging_buffer.unmap();
        data
    }

    fn read_position_sums(
        device: &wgpu::Device,
        staging_x: &wgpu::Buffer,
        staging_y: &wgpu::Buffer,
        staging_z: &wgpu::Buffer,
        num_objects: usize,
    ) -> Vec<[f32; 3]> {
        fn map_buffer_f32(device: &wgpu::Device, buffer: &wgpu::Buffer) -> Vec<f32> {
            let buffer_slice = buffer.slice(..);
            let (tx, rx) = futures::channel::oneshot::channel();

            buffer_slice.map_async(
                wgpu::MapMode::Read,
                move |v| {
                    tx.send(v)
                        .unwrap();
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

            let mapped = buffer_slice.get_mapped_range();
            let data: Vec<f32> = bytemuck::cast_slice(&mapped).to_vec();
            drop(mapped);
            buffer.unmap();
            data
        }

        let x_vals = map_buffer_f32(device, staging_x);
        let y_vals = map_buffer_f32(device, staging_y);
        let z_vals = map_buffer_f32(device, staging_z);

        assert_eq!(x_vals.len(), num_objects);
        assert_eq!(y_vals.len(), num_objects);
        assert_eq!(z_vals.len(), num_objects);

        (0..num_objects)
            .map(|i| [x_vals[i], y_vals[i], z_vals[i]])
            .collect()
    }
}
