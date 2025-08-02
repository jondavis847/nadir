use crate::parallel_reduction::{ParallelReduction, stage_buffers::StageBuffers};

pub struct Stage2 {
    pub buffers: StageBuffers,
    bind_group_layout: wgpu::BindGroupLayout,
    pub bind_group: wgpu::BindGroup,
    pub pipeline: wgpu::ComputePipeline,
}

impl Stage2 {
    pub fn new(
        device: &wgpu::Device,
        uniform_bind_group_layout: &wgpu::BindGroupLayout,
        previous_buffers: &StageBuffers,
    ) -> Self {
        let bind_group_layout = Self::bind_group_layout(device);
        let output_buffer_length = (previous_buffers.length + ParallelReduction::WORKGROUP_SIZE
            - 1)
            / ParallelReduction::WORKGROUP_SIZE;
        let current_buffers = StageBuffers::new(device, output_buffer_length);
        let bind_group = Self::bind_group(
            device,
            &bind_group_layout,
            previous_buffers,
            &current_buffers,
        );
        let pipeline = Self::pipeline(
            device,
            uniform_bind_group_layout,
            &bind_group_layout,
        );
        Self {
            buffers: current_buffers,
            bind_group_layout,
            bind_group,
            pipeline,
        }
    }

    fn bind_group_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
        device.create_bind_group_layout(
            &wgpu::BindGroupLayoutDescriptor {
                label: Some("Stage2 BGL"),
                entries: &[
                    // 0: stage1 result_buffer
                    wgpu::BindGroupLayoutEntry {
                        binding: 0,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: true },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    // 1: stage2 result_buffer
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
                ],
            },
        )
    }

    fn bind_group(
        device: &wgpu::Device,
        layout: &wgpu::BindGroupLayout,
        previous_buffers: &StageBuffers,
        current_buffers: &StageBuffers,
    ) -> wgpu::BindGroup {
        device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Parallel Reduction Stage 2 Bind Group"),
            layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: previous_buffers
                        .result_buffer
                        .as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: current_buffers
                        .result_buffer
                        .as_entire_binding(),
                },
            ],
        })
    }

    pub fn pipeline(
        device: &wgpu::Device,
        uniform_bind_group_layout: &wgpu::BindGroupLayout,
        stage1_bind_group_layout: &wgpu::BindGroupLayout,
    ) -> wgpu::ComputePipeline {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Parallel Reduction Stage 2 Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("stage2.wgsl").into()),
        });

        device.create_compute_pipeline(
            &wgpu::ComputePipelineDescriptor {
                label: Some("Stage2 Per-Workgroup Reduction"),
                layout: Some(
                    &device.create_pipeline_layout(
                        &wgpu::PipelineLayoutDescriptor {
                            label: Some("Stage2 Layout"),
                            bind_group_layouts: &[
                                uniform_bind_group_layout,
                                &stage1_bind_group_layout,
                            ],
                            push_constant_ranges: &[],
                        },
                    ),
                ),
                module: &shader,
                entry_point: Some("main"),
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                cache: None,
            },
        )
    }
}
