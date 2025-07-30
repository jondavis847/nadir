pub struct ParallelReduction {
    count_pipeline: wgpu::ComputePipeline,
    reduce_pipeline: wgpu::ComputePipeline,
    clear_pipeline: wgpu::ComputePipeline,
    count_bind_group: wgpu::BindGroup,
    reduce_bind_group: wgpu::BindGroup,

    group_counts: wgpu::Buffer,
    group_sum_x: wgpu::Buffer,
    group_sum_y: wgpu::Buffer,
    group_sum_z: wgpu::Buffer,

    total_counts: wgpu::Buffer,
    total_sum_x: wgpu::Buffer,
    total_sum_y: wgpu::Buffer,
    total_sum_z: wgpu::Buffer,

    staging_buffer: wgpu::Buffer, // For final mapped readback
}

impl ParallelReduction {
    pub fn new(
        device: &wgpu::Device,
        object_id_texture: &wgpu::TextureView,
        position_texture: &wgpu::TextureView,
        num_objects: usize,
        num_workgroups: usize,
    ) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Parallel Reduction Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("parallel_reduction.wgsl").into()),
        });

        // Layouts
        let count_bind_group_layout = device.create_bind_group_layout(
            &wgpu::BindGroupLayoutDescriptor {
                label: Some("Count Bind Group Layout"),
                entries: &[
                    // object_id_tex, position_tex
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
                    wgpu::BindGroupLayoutEntry {
                        binding: 1,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Texture {
                            sample_type: wgpu::TextureSampleType::Float { filterable: false },
                            view_dimension: wgpu::TextureViewDimension::D2,
                            multisampled: false,
                        },
                        count: None,
                    },
                    // output buffers
                    wgpu::BindGroupLayoutEntry {
                        binding: 2,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 3,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 4,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 5,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                ],
            },
        );

        let reduce_bind_group_layout = device.create_bind_group_layout(
            &wgpu::BindGroupLayoutDescriptor {
                label: Some("Reduce Bind Group Layout"),
                entries: &[
                    // inputs
                    wgpu::BindGroupLayoutEntry {
                        binding: 0,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 1,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 2,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 3,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    // outputs
                    wgpu::BindGroupLayoutEntry {
                        binding: 4,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 5,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 6,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 7,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                ],
            },
        );

        // === Pipelines ===

        let clear_pipeline = device.create_compute_pipeline(
            &wgpu::ComputePipelineDescriptor {
                label: Some("Clear Buffers Pipeline"),
                layout: Some(
                    &device.create_pipeline_layout(
                        &wgpu::PipelineLayoutDescriptor {
                            label: Some("Clear Buffers Layout"),
                            bind_group_layouts: &[&reduce_bind_group_layout],
                            push_constant_ranges: &[],
                        },
                    ),
                ),
                module: &shader,
                entry_point: Some("clear_buffers"), // <-- must match WGSL function name
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                cache: None,
            },
        );

        let count_pipeline = device.create_compute_pipeline(
            &wgpu::ComputePipelineDescriptor {
                label: Some("Count Pipeline"),
                layout: Some(
                    &device.create_pipeline_layout(
                        &wgpu::PipelineLayoutDescriptor {
                            label: Some("Count Layout"),
                            bind_group_layouts: &[&count_bind_group_layout],
                            push_constant_ranges: &[],
                        },
                    ),
                ),
                module: &shader,
                entry_point: Some("count_pixels_and_positions"),
                cache: None,
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
        );

        let reduce_pipeline = device.create_compute_pipeline(
            &wgpu::ComputePipelineDescriptor {
                label: Some("Reduce Pipeline"),
                layout: Some(
                    &device.create_pipeline_layout(
                        &wgpu::PipelineLayoutDescriptor {
                            label: Some("Reduce Layout"),
                            bind_group_layouts: &[&reduce_bind_group_layout],
                            push_constant_ranges: &[],
                        },
                    ),
                ),
                module: &shader,
                entry_point: Some("reduce_histograms"),
                cache: None,
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
        );

        // === Buffers ===
        let obj_count = num_objects as u64;
        let group_count = num_workgroups as u64;
        let buffer_size_u32 = (group_count * obj_count * 4) as wgpu::BufferAddress; // 4 bytes per u32
        let buffer_size_f32 = (group_count * obj_count * 4) as wgpu::BufferAddress; // 4 bytes per f32

        let make_buf = |label: &str, size, usage| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size,
                usage,
                mapped_at_creation: false,
            })
        };

        let group_counts = make_buf(
            "group_counts",
            buffer_size_u32,
            wgpu::BufferUsages::STORAGE,
        );
        let group_sum_x = make_buf(
            "group_sum_x",
            buffer_size_f32,
            wgpu::BufferUsages::STORAGE,
        );
        let group_sum_y = make_buf(
            "group_sum_y",
            buffer_size_f32,
            wgpu::BufferUsages::STORAGE,
        );
        let group_sum_z = make_buf(
            "group_sum_z",
            buffer_size_f32,
            wgpu::BufferUsages::STORAGE,
        );

        let total_counts = make_buf(
            "total_counts",
            obj_count * 4,
            wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        );
        let total_sum_x = make_buf(
            "total_sum_x",
            obj_count * 4,
            wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        );
        let total_sum_y = make_buf(
            "total_sum_y",
            obj_count * 4,
            wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        );
        let total_sum_z = make_buf(
            "total_sum_z",
            obj_count * 4,
            wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        );

        let staging_buffer = make_buf(
            "staging",
            obj_count * 4,
            wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        );

        let count_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Parallel Count Bind Group"),
            layout: &count_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(object_id_texture),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::TextureView(position_texture),
                },
                wgpu::BindGroupEntry { binding: 2, resource: group_counts.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 3, resource: group_sum_x.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 4, resource: group_sum_y.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 5, resource: group_sum_z.as_entire_binding() },
            ],
        });

        let reduce_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Parallel Reduce Bind Group"),
            layout: &reduce_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: group_counts.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: group_sum_x.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 2, resource: group_sum_y.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 3, resource: group_sum_z.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 4, resource: total_counts.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 5, resource: total_sum_x.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 6, resource: total_sum_y.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 7, resource: total_sum_z.as_entire_binding() },
            ],
        });

        Self {
            clear_pipeline,
            count_pipeline,
            reduce_pipeline,
            count_bind_group,
            reduce_bind_group,
            group_counts,
            group_sum_x,
            group_sum_y,
            group_sum_z,
            total_counts,
            total_sum_x,
            total_sum_y,
            total_sum_z,
            staging_buffer,
        }
    }

    pub fn reduce(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        resolution: u32,
        num_objects: usize,
    ) -> Vec<ReductionResult> {
        let mut encoder = device.create_command_encoder(
            &wgpu::CommandEncoderDescriptor { label: Some("Parallel Reduction Encoder") },
        );
        {
            let mut clear_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("Clear Buffers Pass"),
                timestamp_writes: None,
            });
            clear_pass.set_pipeline(&self.clear_pipeline);
            clear_pass.set_bind_group(
                0,
                &self.reduce_bind_group,
                &[],
            );
            clear_pass.dispatch_workgroups(num_objects as u32, 1, 1);
        }
        {
            let mut cpass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("Count Compute Pass"),
                timestamp_writes: None,
            });
            cpass.set_pipeline(&self.count_pipeline);
            cpass.set_bind_group(0, &self.count_bind_group, &[]);

            let workgroups_x = (resolution + 7) / 8;
            let workgroups_y = (resolution + 7) / 8;
            cpass.dispatch_workgroups(workgroups_x, workgroups_y, 1);
        }

        {
            let mut rpass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("Reduce Compute Pass"),
                timestamp_writes: None,
            });
            rpass.set_pipeline(&self.reduce_pipeline);
            rpass.set_bind_group(
                0,
                &self.reduce_bind_group,
                &[],
            );
            rpass.dispatch_workgroups(num_objects as u32, 1, 1);
        }

        // === 3. Copy results to staging buffer ===
        let result_size = (num_objects * std::mem::size_of::<ReductionResult>()) as u64;

        // Create a temporary buffer to hold packed result structs
        let packed_result_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Packed Results"),
            size: result_size,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        // Pack the separate component buffers into the final struct buffer using copy_buffer_to_buffer
        encoder.copy_buffer_to_buffer(
            &self.total_counts,
            0,
            &packed_result_buffer,
            0,
            (num_objects * 4) as u64,
        );
        encoder.copy_buffer_to_buffer(
            &self.total_sum_x,
            0,
            &packed_result_buffer,
            (num_objects * 4) as u64,
            (num_objects * 4) as u64,
        );
        encoder.copy_buffer_to_buffer(
            &self.total_sum_y,
            0,
            &packed_result_buffer,
            (num_objects * 8) as u64,
            (num_objects * 4) as u64,
        );
        encoder.copy_buffer_to_buffer(
            &self.total_sum_z,
            0,
            &packed_result_buffer,
            (num_objects * 12) as u64,
            (num_objects * 4) as u64,
        );

        // Submit and wait
        queue.submit(Some(encoder.finish()));
        device.poll(wgpu::PollType::Wait);

        // Map result buffer
        let buffer_slice = packed_result_buffer.slice(..);
        let (sender, receiver) = futures::channel::oneshot::channel();
        buffer_slice.map_async(
            wgpu::MapMode::Read,
            move |res| {
                sender
                    .send(res)
                    .unwrap();
            },
        );
        pollster::block_on(async {
            receiver
                .await
                .unwrap()
                .unwrap();
        });

        // Read mapped data
        let data = buffer_slice.get_mapped_range();
        let results: Vec<ReductionResult> = bytemuck::cast_slice(&data).to_vec();

        drop(data);
        packed_result_buffer.unmap();

        results
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Zeroable, bytemuck::Pod)]
pub struct ReductionResult {
    pub pixel_count: u32,
    pub sum_x: f32,
    pub sum_y: f32,
    pub sum_z: f32,
}
