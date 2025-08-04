use crate::parallel_reduction::{ParallelReduction, stage_buffers::StageBuffers};

pub struct Stage2 {
    pub buffers: StageBuffers,
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
        Self { buffers: current_buffers, bind_group, pipeline }
    }

    fn bind_group_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
        device.create_bind_group_layout(
            &wgpu::BindGroupLayoutDescriptor {
                label: Some("Stage2 BGL"),
                entries: &[
                    // 0: previous result_buffer
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
                    // 1: current result_buffer
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

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use nadir_3d::geometry::{GeometryState, cuboid::Cuboid};

    use crate::{
        GpuCalculator, parallel_reduction::stage_buffers::WorkgroupResult,
        surface_area::SurfaceAreaCalculator,
    };

    #[test]
    fn parallel_reduction_stage2() {
        // 1024 * 1024  pixels/ 256 threads per workgroup = 4096 workgroups
        let resolution = 1024; // needs to be a multiple of 64 or bytes won't align (256 alignment - 64 u32 = 256)
        let mut gpu = GpuCalculator::new()
            .with_resolution(resolution)
            .with_surface_area();

        let cube = Cuboid::new(1.0, 1.0, 1.0).unwrap();
        let cube_state = GeometryState::default();
        gpu.add_geometry(cube.into(), &cube_state);

        gpu.initialize();
        gpu.calculate_scene_bounds();
        let view_direction = [-1.0, 0.0, 0.0];
        let start = Instant::now();
        if let Some(init) = &mut gpu.initialized {
            if let Some(area) = &mut gpu.surface_area {
                area.render(
                    &init.device,
                    &init.queue,
                    &init.geometry,
                    &gpu.scene_bounds,
                    &view_direction,
                    1.0,
                    &init.shared_uniform_buffer,
                    &init.shared_bind_group,
                    &init.object_id_texture,
                    &init.position_texture,
                    &init.depth_texture,
                );
            }
        }
        let stop = Instant::now();
        let duration = stop.duration_since(start);
        println!("Render time: {:?}", duration);

        // dig down a bit until we can run stage1
        if let Some(area) = &mut gpu.surface_area {
            if let Some(aero_init) = &mut area.initialized {
                if let Some(gpu_init) = &gpu.initialized {
                    let mut encoder = gpu_init
                        .device
                        .create_command_encoder(
                            &wgpu::CommandEncoderDescriptor { label: Some("test encoder") },
                        );

                    let start = Instant::now();
                    aero_init
                        .parallel_reduction
                        .stage1
                        .dispatch(
                            aero_init
                                .parallel_reduction
                                .n_workgroups_x,
                            aero_init
                                .parallel_reduction
                                .n_workgroups_y,
                            &aero_init
                                .parallel_reduction
                                .uniform_bind_group,
                            &mut encoder,
                        );
                    let stop = Instant::now();
                    let duration = stop.duration_since(start);
                    println!("Reduce time: {:?}", duration);

                    let staging_buffer = gpu_init
                        .device
                        .create_buffer(&wgpu::BufferDescriptor {
                            label: Some("test_staging"),
                            size: (aero_init
                                .parallel_reduction
                                .stage1
                                .buffers
                                .length
                                * std::mem::size_of::<WorkgroupResult>() as u32)
                                as u64,
                            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                            mapped_at_creation: false,
                        });

                    encoder.copy_buffer_to_buffer(
                        &aero_init
                            .parallel_reduction
                            .stage1
                            .buffers
                            .result_buffer,
                        0,
                        &staging_buffer,
                        0,
                        aero_init
                            .parallel_reduction
                            .stage1
                            .buffers
                            .length as u64
                            * std::mem::size_of::<WorkgroupResult>() as u64,
                    );

                    gpu_init
                        .queue
                        .submit(Some(encoder.finish()));
                    gpu_init
                        .device
                        .poll(wgpu::PollType::Wait)
                        .unwrap();

                    let buffer_slice = staging_buffer.slice(..);
                    let (tx, rx) = futures::channel::oneshot::channel();

                    buffer_slice.map_async(
                        wgpu::MapMode::Read,
                        move |result| {
                            tx.send(result)
                                .unwrap();
                        },
                    );

                    gpu_init
                        .device
                        .poll(wgpu::PollType::Wait)
                        .unwrap();
                    pollster::block_on(rx)
                        .unwrap()
                        .unwrap();

                    let data = buffer_slice.get_mapped_range();
                    let results: Vec<WorkgroupResult> = bytemuck::cast_slice(&data).to_vec();

                    assert!(
                        results.len() == 4096,
                        "Result length for this test should be 64 * 64 pixels * 1 object / 256 workers = 16 workgroup elements"
                    );

                    let area_per_pixel = SurfaceAreaCalculator::calculate_area_per_pixel(
                        &gpu.scene_bounds,
                        resolution,
                        1.0,
                    );
                    let mut area = 0.0;
                    for result in results {
                        area += result.count as f32 * area_per_pixel;
                    }
                    println!("Surface Area: {:?} m^2", area);
                    assert!(
                        1.0 - area < 0.07,
                        "Calculated area should be 1.0 +/- 1%"
                    );

                    drop(data);
                    staging_buffer.unmap();
                }
            }
        }
    }
}
