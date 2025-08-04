// Stage 1 of the parallel reduction process takes pixel data from the fragment shader stored in the object ID and position textures
// and compute per-object counts and sums of positions.
// This is done in parallel per workgroup, storing results per object per workgroup in a shared memory array

use wgpu::CommandEncoder;

use crate::parallel_reduction::{Uniforms, stage_buffers::StageBuffers};

pub struct Stage1 {
    pub buffers: StageBuffers,
    pub bind_group: wgpu::BindGroup,
    pub pipeline: wgpu::ComputePipeline,
}

struct Stage1Textures {
    object_id_texture: wgpu::TextureView,
    position_texture: wgpu::TextureView,
}

impl Stage1 {
    pub fn new(
        device: &wgpu::Device,
        uniforms: &Uniforms,
        uniform_bind_group_layout: &wgpu::BindGroupLayout,
        object_id_texture: wgpu::TextureView,
        position_texture: wgpu::TextureView,
    ) -> Self {
        let bind_group_layout = Self::bind_group_layout(device);
        let buffer_length = uniforms.num_workgroups * uniforms.num_objects;
        let buffers = StageBuffers::new(device, buffer_length);
        let textures = Stage1Textures { object_id_texture, position_texture };
        let bind_group = Self::bind_group(
            device,
            &bind_group_layout,
            &buffers,
            &textures,
        );
        let pipeline = Self::pipeline(
            device,
            uniform_bind_group_layout,
            &bind_group_layout,
        );
        Self { buffers, bind_group, pipeline }
    }

    fn bind_group_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
        device.create_bind_group_layout(
            &wgpu::BindGroupLayoutDescriptor {
                label: Some("Stage1 BGL"),
                entries: &[
                    // 0: object_id_texture
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
                    // 1: position_texture
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
                    // 2: workgroup_result
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
                ],
            },
        )
    }

    fn bind_group(
        device: &wgpu::Device,
        layout: &wgpu::BindGroupLayout,
        buffers: &StageBuffers,
        textures: &Stage1Textures,
    ) -> wgpu::BindGroup {
        device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Parallel Count Bind Group"),
            layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&textures.object_id_texture),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::TextureView(&textures.position_texture),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: buffers
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
            label: Some("Parallel Reduction Stage 1 Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("stage1.wgsl").into()),
        });

        device.create_compute_pipeline(
            &wgpu::ComputePipelineDescriptor {
                label: Some("Stage1 Per-Workgroup Reduction"),
                layout: Some(
                    &device.create_pipeline_layout(
                        &wgpu::PipelineLayoutDescriptor {
                            label: Some("Stage1 Layout"),
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
        n_workgroups_x: u32,
        n_workgroups_y: u32,
        uniform_bind_group: &wgpu::BindGroup,
        encoder: &mut CommandEncoder,
    ) {
        {
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("ParallelReduction Stage1 Pass"),
                timestamp_writes: None,
            });

            compute_pass.set_pipeline(&self.pipeline);
            compute_pass.set_bind_group(0, uniform_bind_group, &[]);
            compute_pass.set_bind_group(1, &self.bind_group, &[]);

            compute_pass.dispatch_workgroups(
                n_workgroups_x,
                n_workgroups_y,
                1,
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use nadir_3d::geometry::{GeometryState, cuboid::Cuboid};

    use crate::{GpuCalculator, parallel_reduction::stage_buffers::WorkgroupResult};

    #[test]
    fn parallel_reduction_stage1() {
        // 64 * 64  pixels/ 256 threads per workgroup = 16 workgroups
        let resolution = 64; // needs to be a multiple of 64 or bytes won't align (256 alignment - 64 u32 = 256)
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
                    println!("=== BUFFER DEBUG INFO ===");
                    println!("Resolution: {}", resolution);
                    println!("Workgroup size: 16x16 = 256 threads");
                    println!(
                        "Workgroups X: {}",
                        (resolution + 15) / 16
                    );
                    println!(
                        "Workgroups Y: {}",
                        (resolution + 15) / 16
                    );
                    println!(
                        "Total workgroups: {}",
                        ((resolution + 15) / 16) * ((resolution + 15) / 16)
                    );
                    println!("Num objects: 1");
                    println!(
                        "Expected buffer entries: {}",
                        ((resolution + 15) / 16) * ((resolution + 15) / 16) * 1
                    );
                    println!(
                        "WorkgroupResult size: {} bytes",
                        std::mem::size_of::<WorkgroupResult>()
                    );
                    println!(
                        "Buffer size: {} bytes",
                        aero_init
                            .parallel_reduction
                            .stage1
                            .buffers
                            .length as usize
                            * std::mem::size_of::<WorkgroupResult>()
                    );

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
                    // Add this right after reading the results
                    println!("=== RAW MEMORY DEBUG ===");
                    let raw_bytes = bytemuck::cast_slice::<WorkgroupResult, u8>(&results[0..3]);
                    println!("First 96 bytes (3 structs worth):");
                    for (i, chunk) in raw_bytes
                        .chunks(4)
                        .enumerate()
                    {
                        let value = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
                        let float_value =
                            f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
                        println!(
                            "  Offset {}: 0x{:08X} = {} (as u32) = {:.3} (as f32)",
                            i * 4,
                            value,
                            value,
                            float_value
                        );
                    }
                    println!("=== RESULTS ANALYSIS ===");
                    println!(
                        "Results length: {}",
                        results.len()
                    );
                    for (i, result) in results
                        .iter()
                        .enumerate()
                        .take(20)
                    {
                        // Show first 20
                        println!(
                            "Entry {}: id={}, count={}, pos=({:.3}, {:.3}, {:.3})",
                            i, result.id, result.count, result.pos[0], result.pos[1], result.pos[2]
                        );
                    }
                    drop(data);
                    staging_buffer.unmap();

                    dbg!(&results);
                }
            }
        }
    }
}
