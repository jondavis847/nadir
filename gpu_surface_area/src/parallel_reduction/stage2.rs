use crate::parallel_reduction::{ParallelReduction, stage_buffers::StageBuffers};

use super::ceil_div;

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
        let output_buffer_length = ceil_div(
            previous_buffers.length,
            ParallelReduction::WORKGROUP_SIZE,
        );

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

    pub fn dispatch(
        &self,
        previous_buffers: &StageBuffers,
        encoder: &mut wgpu::CommandEncoder,
        uniform_bind_group: &wgpu::BindGroup,
    ) {
        let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some(&format!(
                "ParallelReduction Stage2 Pass",
            )),
            timestamp_writes: None,
        });

        compute_pass.set_pipeline(&self.pipeline);
        compute_pass.set_bind_group(0, uniform_bind_group, &[]);
        compute_pass.set_bind_group(1, &self.bind_group, &[]);

        let dispatch_x = ceil_div(
            previous_buffers.length,
            ParallelReduction::WORKGROUP_SIZE,
        );
        dbg!(dispatch_x);
        compute_pass.dispatch_workgroups(dispatch_x, 1, 1);

        drop(compute_pass);
    }
}

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use glam::{DQuat, DVec3};
    use nadir_3d::geometry::{GeometryState, cuboid::Cuboid};

    use crate::{
        GpuCalculator,
        parallel_reduction::{ceil_div, stage_buffers::WorkgroupResult},
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
                let parallel_reduction = &mut aero_init.parallel_reduction;
                if let Some(gpu_init) = &gpu.initialized {
                    let mut encoder = gpu_init
                        .device
                        .create_command_encoder(
                            &wgpu::CommandEncoderDescriptor { label: Some("test encoder") },
                        );

                    let start = Instant::now();
                    parallel_reduction
                        .stage1
                        .dispatch(
                            parallel_reduction.n_workgroups_x,
                            parallel_reduction.n_workgroups_y,
                            &parallel_reduction.uniform_bind_group,
                            &mut encoder,
                        );
                    let stop = Instant::now();
                    let duration = stop.duration_since(start);
                    println!("Reduce time: {:?}", duration);

                    //now run stage2
                    assert!(
                        parallel_reduction
                            .stage2
                            .len()
                            > 1,
                        "expected stage2 to have 2 entries, 4096->16, 16->1",
                    );
                    let previous_buffers = &parallel_reduction
                        .stage1
                        .buffers;
                    parallel_reduction.stage2[0].dispatch(
                        previous_buffers,
                        &mut encoder,
                        &parallel_reduction.uniform_bind_group,
                    );

                    let staging_buffer = gpu_init
                        .device
                        .create_buffer(&wgpu::BufferDescriptor {
                            label: Some("test_staging"),
                            size: (parallel_reduction.stage2[0]
                                .buffers
                                .length
                                * std::mem::size_of::<WorkgroupResult>() as u32)
                                as u64,
                            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                            mapped_at_creation: false,
                        });

                    encoder.copy_buffer_to_buffer(
                        &parallel_reduction.stage2[0]
                            .buffers
                            .result_buffer,
                        0,
                        &staging_buffer,
                        0,
                        parallel_reduction.stage2[0]
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
                        results.len() == 16,
                        "Result length for this test should be 4096/256 workers = 16 workgroup elements"
                    );
                    let area_per_pixel = SurfaceAreaCalculator::calculate_area_per_pixel(
                        &gpu.scene_bounds,
                        resolution,
                        1.0,
                    );
                    let mut count_sum = 0;
                    let mut x_sum = 0.0;
                    let mut y_sum = 0.0;
                    let mut z_sum = 0.0;
                    for result in &results {
                        count_sum += result.count;
                        x_sum += result.pos[0];
                        y_sum += result.pos[1];
                        z_sum += result.pos[2];
                    }
                    let area = count_sum as f32 * area_per_pixel;
                    let cop_x = x_sum / count_sum as f32;
                    let cop_y = y_sum / count_sum as f32;
                    let cop_z = z_sum / count_sum as f32;
                    assert!(
                        area - 1.0 < 0.01,
                        "area was not within 1% of expectation"
                    );
                    assert!(
                        cop_x - 0.5 < 0.005,
                        "cop_x was not within 1% of expecation"
                    );
                    assert!(
                        cop_y < 0.0005,
                        "cop_y was not within 1% of expecation"
                    );
                    assert!(
                        cop_z < 0.0005,
                        "cop_z was not within 1% of expecation"
                    );

                    //run stage2 again for final 16 elements
                    let mut encoder = gpu_init
                        .device
                        .create_command_encoder(
                            &wgpu::CommandEncoderDescriptor { label: Some("test encoder") },
                        );

                    let previous_buffers = &parallel_reduction.stage2[0].buffers;
                    parallel_reduction.stage2[1].dispatch(
                        previous_buffers,
                        &mut encoder,
                        &parallel_reduction.uniform_bind_group,
                    );

                    let staging_buffer2 = gpu_init
                        .device
                        .create_buffer(&wgpu::BufferDescriptor {
                            label: Some("test_staging2"),
                            size: (aero_init
                                .parallel_reduction
                                .stage2[1]
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
                            .stage2[1]
                            .buffers
                            .result_buffer,
                        0,
                        &staging_buffer2,
                        0,
                        aero_init
                            .parallel_reduction
                            .stage2[1]
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

                    let buffer_slice = staging_buffer2.slice(..);
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
                        results.len() == 1,
                        "Result length for this test should be 16/256 workers = 1 workgroup elements"
                    );
                    let result = results[0];
                    let area_per_pixel = SurfaceAreaCalculator::calculate_area_per_pixel(
                        &gpu.scene_bounds,
                        resolution,
                        1.0,
                    );
                    let area = result.count as f32 * area_per_pixel;
                    let cop_x = result.pos[0] / result.count as f32;
                    let cop_y = result.pos[1] / result.count as f32;
                    let cop_z = result.pos[2] / result.count as f32;
                    assert!(
                        area - 1.0 < 0.01,
                        "area was not within 1% of expectation"
                    );
                    assert!(
                        cop_x - 0.5 < 0.005,
                        "cop_x was not within 1% of expecation"
                    );
                    assert!(
                        cop_y < 0.0005,
                        "cop_y was not within 1% of expecation"
                    );
                    assert!(
                        cop_z < 0.0005,
                        "cop_z was not within 1% of expecation"
                    );
                }
            }
        }
    }

    #[test]
    fn parallel_reduction_stage2_2objects() {
        // 1024 * 1024  pixels/ 256 threads per workgroup = 4096 workgroups
        let resolution = 1024; // needs to be a multiple of 64 or bytes won't align (256 alignment - 64 u32 = 256)
        let mut gpu = GpuCalculator::new()
            .with_resolution(resolution)
            .with_surface_area();

        let cube1 = Cuboid::new(1.0, 1.0, 1.0).unwrap();
        let cube2 = Cuboid::new(2.0, 2.0, 2.0).unwrap();
        let cube1_state = GeometryState::default();
        let cube2_state = GeometryState {
            position: DVec3::new(0.0, 2.0, 0.0),
            rotation: DQuat::IDENTITY,
        };
        gpu.add_geometry(cube1.into(), &cube1_state);
        gpu.add_geometry(cube2.into(), &cube2_state);
        gpu.initialize();

        gpu.calculate_scene_bounds();
        let view_direction = [-1.0, 0.0, 0.0];
        let start = Instant::now();
        if let Some(init) = &mut gpu.initialized {
            if let Some(area) = &mut gpu.surface_area {
                if let Some(area_init) = &area.initialized {
                    assert!(
                        area_init
                            .parallel_reduction
                            .stage1
                            .buffers
                            .length
                            == ceil_div(1024 * 1024 * 2, 256),
                        "incorrect stage1 buffer length"
                    );
                    assert!(
                        area_init
                            .parallel_reduction
                            .stage2[0]
                            .buffers
                            .length
                            == ceil_div(
                                area_init
                                    .parallel_reduction
                                    .stage1
                                    .buffers
                                    .length,
                                256
                            ),
                        "incorrect stage2 buffer length"
                    );
                    assert!(
                        area_init
                            .parallel_reduction
                            .stage2[1]
                            .buffers
                            .length
                            == ceil_div(
                                area_init
                                    .parallel_reduction
                                    .stage2[0]
                                    .buffers
                                    .length,
                                256
                            ),
                        "incorrect stage2 buffer length"
                    );
                }
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
                let parallel_reduction = &mut aero_init.parallel_reduction;
                if let Some(gpu_init) = &gpu.initialized {
                    let mut encoder = gpu_init
                        .device
                        .create_command_encoder(
                            &wgpu::CommandEncoderDescriptor { label: Some("test encoder") },
                        );

                    let start = Instant::now();
                    parallel_reduction
                        .stage1
                        .dispatch(
                            parallel_reduction.n_workgroups_x,
                            parallel_reduction.n_workgroups_y,
                            &parallel_reduction.uniform_bind_group,
                            &mut encoder,
                        );
                    let stop = Instant::now();
                    let duration = stop.duration_since(start);
                    println!("Reduce time: {:?}", duration);

                    //now run stage2
                    assert!(
                        parallel_reduction
                            .stage2
                            .len()
                            > 1,
                        "expected stage2 to have 2 entries, 8192->32, 32->1",
                    );
                    let previous_buffers = &parallel_reduction
                        .stage1
                        .buffers;
                    parallel_reduction.stage2[0].dispatch(
                        previous_buffers,
                        &mut encoder,
                        &parallel_reduction.uniform_bind_group,
                    );

                    let staging_buffer = gpu_init
                        .device
                        .create_buffer(&wgpu::BufferDescriptor {
                            label: Some("test_staging"),
                            size: (parallel_reduction.stage2[0]
                                .buffers
                                .length
                                * std::mem::size_of::<WorkgroupResult>() as u32)
                                as u64,
                            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                            mapped_at_creation: false,
                        });

                    encoder.copy_buffer_to_buffer(
                        &parallel_reduction.stage2[0]
                            .buffers
                            .result_buffer,
                        0,
                        &staging_buffer,
                        0,
                        parallel_reduction.stage2[0]
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
                        results.len() == 32,
                        "Result length for this test should be 4096*2 objects/256 workers = 32 workgroup elements, got {:?}",
                        results.len()
                    );
                    let area_per_pixel = SurfaceAreaCalculator::calculate_area_per_pixel(
                        &gpu.scene_bounds,
                        resolution,
                        1.0,
                    );
                    let mut count_sum1 = 0;
                    let mut x_sum1 = 0.0;
                    let mut y_sum1 = 0.0;
                    let mut z_sum1 = 0.0;
                    let mut count_sum2 = 0;
                    let mut x_sum2 = 0.0;
                    let mut y_sum2 = 0.0;
                    let mut z_sum2 = 0.0;
                    for result in &results {
                        if result.id == 1 {
                            count_sum1 += result.count;
                            x_sum1 += result.pos[0];
                            y_sum1 += result.pos[1];
                            z_sum1 += result.pos[2];
                        }
                        if result.id == 2 {
                            count_sum2 += result.count;
                            x_sum2 += result.pos[0];
                            y_sum2 += result.pos[1];
                            z_sum2 += result.pos[2];
                        }
                    }
                    let area1 = count_sum1 as f32 * area_per_pixel;
                    let cop_x1 = x_sum1 / count_sum1 as f32;
                    let cop_y1 = y_sum1 / count_sum1 as f32;
                    let cop_z1 = z_sum1 / count_sum1 as f32;
                    let area2 = count_sum2 as f32 * area_per_pixel;
                    let cop_x2 = x_sum2 / count_sum2 as f32;
                    let cop_y2 = y_sum2 / count_sum2 as f32;
                    let cop_z2 = z_sum2 / count_sum2 as f32;
                    assert!(
                        area1 - 1.0 < 0.01,
                        "area1 was not within 1% of expectation, got {:?}",
                        area1
                    );
                    assert!(
                        cop_x1 - 0.5 < 0.005,
                        "cop_x1 was not within 1% of expecation, got {:?}",
                        cop_x1
                    );
                    assert!(
                        cop_y1 < 0.001,
                        "cop_y1 was not within 1% of expecation, got {:?}",
                        cop_y1
                    );
                    assert!(
                        cop_z1 < 0.001,
                        "cop_z1 was not within 1% of expecation, got {:?}",
                        cop_z1
                    );

                    assert!(
                        area2 - 4.0 < 0.04,
                        "area was not within 1% of expectation, got {:?}",
                        area2
                    );
                    assert!(
                        cop_x2 - 1.0 < 0.005,
                        "cop_x2 was not within 1% of expecation, got {:?}",
                        cop_x2
                    );
                    assert!(
                        cop_y2 - 2.0 < 0.02,
                        "cop_y2 was not within 1% of expecation, got {:?}",
                        cop_y2
                    );
                    assert!(
                        cop_z2 < 0.001,
                        "cop_z2 was not within 1% of expecation, got {:?}",
                        cop_z2
                    );

                    //run stage2 again for final 16 elements
                    let mut encoder = gpu_init
                        .device
                        .create_command_encoder(
                            &wgpu::CommandEncoderDescriptor { label: Some("test encoder") },
                        );

                    let previous_buffers = &parallel_reduction.stage2[0].buffers;
                    parallel_reduction.stage2[1].dispatch(
                        previous_buffers,
                        &mut encoder,
                        &parallel_reduction.uniform_bind_group,
                    );

                    let staging_buffer2 = gpu_init
                        .device
                        .create_buffer(&wgpu::BufferDescriptor {
                            label: Some("test_staging2"),
                            size: (aero_init
                                .parallel_reduction
                                .stage2[1]
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
                            .stage2[1]
                            .buffers
                            .result_buffer,
                        0,
                        &staging_buffer2,
                        0,
                        aero_init
                            .parallel_reduction
                            .stage2[1]
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

                    let buffer_slice = staging_buffer2.slice(..);
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
                        results.len() == 1,
                        "Result length for this test should be 16/256 workers = 1 workgroup elements"
                    );
                    let result = results[0];
                    let area_per_pixel = SurfaceAreaCalculator::calculate_area_per_pixel(
                        &gpu.scene_bounds,
                        resolution,
                        1.0,
                    );
                    let area = result.count as f32 * area_per_pixel;
                    let cop_x = result.pos[0] / result.count as f32;
                    let cop_y = result.pos[1] / result.count as f32;
                    let cop_z = result.pos[2] / result.count as f32;
                    assert!(
                        area - 1.0 < 0.01,
                        "area was not within 1% of expectation"
                    );
                    assert!(
                        cop_x - 0.5 < 0.005,
                        "cop_x was not within 1% of expectation"
                    );
                    assert!(
                        cop_y < 0.0005,
                        "cop_y was not within 1% of expectation"
                    );
                    assert!(
                        cop_z < 0.0005,
                        "cop_z was not within 1% of expectation"
                    );
                }
            }
        }
    }
}
