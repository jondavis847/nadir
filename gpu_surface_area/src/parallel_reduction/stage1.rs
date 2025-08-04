// Stage 1 of the parallel reduction process takes pixel data from the fragment shader stored in the object ID and position textures
// and compute per-object counts and sums of positions.
// This is done in parallel per workgroup, storing results per object per workgroup in a shared memory array

use crate::parallel_reduction::{Uniforms, stage_buffers::StageBuffers};

pub struct Stage1 {
    pub buffers: StageBuffers,
    bind_group_layout: wgpu::BindGroupLayout,
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
        Self { buffers, bind_group_layout, bind_group, pipeline }
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
}

#[cfg(test)]
mod tests {
    use crate::parallel_reduction::{ceil_div, stage_buffers::WorkgroupResult};

    use super::*;
    use wgpu::util::DeviceExt;

    #[inline]
    fn align_up_256(n: usize) -> usize {
        (n + 255) & !255
    }

    //     #[test]
    //     fn stage1_reduce_per_tile_per_object() {
    //         pollster::block_on(async {
    //             // ---- 1) Device/Queue ----
    //             let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor::default());
    //             let adapter = instance
    //                 .request_adapter(&wgpu::RequestAdapterOptions {
    //                     power_preference: wgpu::PowerPreference::HighPerformance,
    //                     compatible_surface: None,
    //                     force_fallback_adapter: false,
    //                 })
    //                 .await
    //                 .expect("No suitable adapter");

    //             let (device, queue) = adapter
    //                 .request_device(&wgpu::DeviceDescriptor {
    //                     label: Some("stage1-test-device"),
    //                     required_features: wgpu::Features::empty(),
    //                     ..Default::default()
    //                 })
    //                 .await
    //                 .expect("Device creation failed");

    //             // ---- 2) Build textures (16x16) and upload data ----
    //             let width: u32 = 16;
    //             let height: u32 = 16;

    //             // OBJECT ID TEXTURE: R32Uint
    //             let object_id_tex = device.create_texture(&wgpu::TextureDescriptor {
    //                 label: Some("object_id_tex"),
    //                 size: wgpu::Extent3d { width, height, depth_or_array_layers: 1 },
    //                 mip_level_count: 1,
    //                 sample_count: 1,
    //                 dimension: wgpu::TextureDimension::D2,
    //                 format: wgpu::TextureFormat::R32Uint,
    //                 usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
    //                 view_formats: &[],
    //             });
    //             let object_id_view = object_id_tex.create_view(&wgpu::TextureViewDescriptor::default());

    //             // Build padded buffer: tight row = width * 4B; pad to multiple of 256B
    //             let row_bytes_obj = (width as usize) * 4;
    //             let padded_row_bytes_obj = align_up_256(row_bytes_obj);
    //             let mut obj_pixels_padded = vec![0u8; padded_row_bytes_obj * (height as usize)];
    //             for y in 0..(height as usize) {
    //                 let row_start = y * padded_row_bytes_obj;
    //                 for x in 0..(width as usize) {
    //                     let id: u32 = if x < (width as usize / 2) {
    //                         1
    //                     } else {
    //                         2
    //                     };
    //                     let off = row_start + x * 4;
    //                     obj_pixels_padded[off..off + 4].copy_from_slice(&id.to_le_bytes());
    //                 }
    //             }
    //             queue.write_texture(
    //                 wgpu::TexelCopyTextureInfo {
    //                     texture: &object_id_tex,
    //                     mip_level: 0,
    //                     origin: wgpu::Origin3d::ZERO,
    //                     aspect: wgpu::TextureAspect::All,
    //                 },
    //                 &obj_pixels_padded,
    //                 wgpu::TexelCopyBufferLayout {
    //                     offset: 0,
    //                     bytes_per_row: Some(padded_row_bytes_obj as u32),
    //                     rows_per_image: Some(height),
    //                 },
    //                 wgpu::Extent3d { width, height, depth_or_array_layers: 1 },
    //             );

    //             // POSITION TEXTURE: Rgba32Float
    //             let position_tex = device.create_texture(&wgpu::TextureDescriptor {
    //                 label: Some("position_tex"),
    //                 size: wgpu::Extent3d { width, height, depth_or_array_layers: 1 },
    //                 mip_level_count: 1,
    //                 sample_count: 1,
    //                 dimension: wgpu::TextureDimension::D2,
    //                 format: wgpu::TextureFormat::Rgba32Float,
    //                 usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
    //                 view_formats: &[],
    //             });
    //             let position_view = position_tex.create_view(&wgpu::TextureViewDescriptor::default());

    //             // Build padded buffer: tight row = width * 16B; pad to multiple of 256B
    //             let row_bytes_pos = (width as usize) * 16;
    //             let padded_row_bytes_pos = align_up_256(row_bytes_pos);
    //             let mut pos_pixels = vec![0u8; padded_row_bytes_pos * (height as usize)];
    //             for y in 0..(height as usize) {
    //                 let row_start = y * padded_row_bytes_pos;
    //                 for x in 0..(width as usize) {
    //                     let off = row_start + x * 16;
    //                     let vals = [10.0f32, 20.0, 30.0, 0.0];
    //                     pos_pixels[off..off + 16].copy_from_slice(bytemuck::cast_slice(&vals));
    //                 }
    //             }
    //             queue.write_texture(
    //                 wgpu::TexelCopyTextureInfo {
    //                     texture: &position_tex,
    //                     mip_level: 0,
    //                     origin: wgpu::Origin3d::ZERO,
    //                     aspect: wgpu::TextureAspect::All,
    //                 },
    //                 &pos_pixels,
    //                 wgpu::TexelCopyBufferLayout {
    //                     offset: 0,
    //                     bytes_per_row: Some(padded_row_bytes_pos as u32),
    //                     rows_per_image: Some(height),
    //                 },
    //                 wgpu::Extent3d { width, height, depth_or_array_layers: 1 },
    //             );

    //             // ---- 3) Uniforms (group 0) ----
    //             // One 16x16 workgroup covers the image -> num_workgroups = 1
    //             let wg_x = ceil_div(width, 16);
    //             let wg_y = ceil_div(height, 16);
    //             assert_eq!(wg_x, 1);
    //             assert_eq!(wg_y, 1);
    //             let num_workgroups = wg_x * wg_y;
    //             let num_objects: u32 = 2;

    //             // This Uniforms must match your crate::parallel_reduction::Uniforms
    //             let uniforms = Uniforms { num_objects, num_workgroups, _pad0: 0, _pad1: 0 };

    //             let uniform_bgl = device.create_bind_group_layout(
    //                 &wgpu::BindGroupLayoutDescriptor {
    //                     label: Some("uniforms-bgl"),
    //                     entries: &[wgpu::BindGroupLayoutEntry {
    //                         binding: 0,
    //                         visibility: wgpu::ShaderStages::COMPUTE,
    //                         ty: wgpu::BindingType::Buffer {
    //                             ty: wgpu::BufferBindingType::Uniform,
    //                             has_dynamic_offset: false,
    //                             min_binding_size: None,
    //                         },
    //                         count: None,
    //                     }],
    //                 },
    //             );
    //             let uniform_buf = device.create_buffer_init(
    //                 &wgpu::util::BufferInitDescriptor {
    //                     label: Some("uniforms"),
    //                     contents: bytemuck::bytes_of(&uniforms),
    //                     usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
    //                 },
    //             );
    //             let uniform_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
    //                 label: Some("uniforms-bg"),
    //                 layout: &uniform_bgl,
    //                 entries: &[wgpu::BindGroupEntry {
    //                     binding: 0,
    //                     resource: uniform_buf.as_entire_binding(),
    //                 }],
    //             });

    //             // ---- 4) Build Stage 1 (group 1 + pipeline) ----
    //             let stage1 = Stage1::new(
    //                 &device,
    //                 &uniforms,
    //                 &uniform_bgl,           // this becomes group(0) in the pipeline layout
    //                 object_id_view.clone(), // group(1) bindings 0..1
    //                 position_view.clone(),
    //             );

    //             // ---- 5) Dispatch Stage 1 ----
    //             let mut encoder = device.create_command_encoder(
    //                 &wgpu::CommandEncoderDescriptor { label: Some("stage1-encoder") },
    //             );
    //             {
    //                 let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
    //                     label: Some("stage1-pass"),
    //                     timestamp_writes: None,
    //                 });
    //                 pass.set_pipeline(&stage1.pipeline);
    //                 pass.set_bind_group(0, &uniform_bg, &[]);
    //                 pass.set_bind_group(1, &stage1.bind_group, &[]);
    //                 pass.dispatch_workgroups(wg_x, wg_y, 1);
    //             }

    //             // Prepare readback buffers for the 4 storage outputs.
    //             let elem_count = (num_workgroups * num_objects) as usize; // 1 * 2 = 2
    //             let bytes_u32 = (elem_count * 4) as u64;
    //             let bytes_f32 = (elem_count * 4) as u64;

    //             let read_counts = device.create_buffer(&wgpu::BufferDescriptor {
    //                 label: Some("read_counts"),
    //                 size: bytes_u32,
    //                 usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
    //                 mapped_at_creation: false,
    //             });
    //             let read_sum_x = device.create_buffer(&wgpu::BufferDescriptor {
    //                 label: Some("read_sum_x"),
    //                 size: bytes_f32,
    //                 usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
    //                 mapped_at_creation: false,
    //             });
    //             let read_sum_y = device.create_buffer(&wgpu::BufferDescriptor {
    //                 label: Some("read_sum_y"),
    //                 size: bytes_f32,
    //                 usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
    //                 mapped_at_creation: false,
    //             });
    //             let read_sum_z = device.create_buffer(&wgpu::BufferDescriptor {
    //                 label: Some("read_sum_z"),
    //                 size: bytes_f32,
    //                 usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
    //                 mapped_at_creation: false,
    //             });

    //             // Copy Stage1 storage buffers -> readback buffers
    //             encoder.copy_buffer_to_buffer(
    //                 &stage1
    //                     .buffers
    //                     .result_buffer,
    //                 0,
    //                 &read_counts,
    //                 0,
    //                 (std::mem::size_of::<WorkgroupResult>() * elem_count) as u64,
    //             );

    //             // Submit and wait
    //             queue.submit(Some(encoder.finish()));
    //             device
    //                 .poll(wgpu::PollType::Wait)
    //                 .unwrap();

    //             // ---- 6) Map & verify results ----
    //             // Expected: one tile, two objects, 128 pixels each half.
    //             let expected_count = 128u32;
    //             let expected_x = 128.0f32 * 10.0;
    //             let expected_y = 128.0f32 * 20.0;
    //             let expected_z = 128.0f32 * 30.0;

    //             fn read_buffer<T: bytemuck::Pod>(
    //                 buffer: &wgpu::Buffer,
    //                 device: &wgpu::Device,
    //             ) -> Vec<T> {
    //                 let slice = buffer.slice(..);

    //                 let (sender, receiver) = futures::channel::oneshot::channel();

    //                 slice.map_async(
    //                     wgpu::MapMode::Read,
    //                     move |res| {
    //                         sender
    //                             .send(res)
    //                             .unwrap();
    //                     },
    //                 );

    //                 device
    //                     .poll(wgpu::PollType::Wait)
    //                     .unwrap();
    //                 pollster::block_on(async {
    //                     receiver
    //                         .await
    //                         .unwrap()
    //                         .unwrap(); // unwrap the Result<(), BufferAsyncError>
    //                 });

    //                 let data = slice.get_mapped_range();
    //                 let result = bytemuck::cast_slice(&data).to_vec();
    //                 drop(data);
    //                 buffer.unmap();
    //                 result
    //             }

    //             let counts: Vec<u32> = read_buffer(&read_counts, &device);
    //             let sum_x: Vec<f32> = read_buffer(&read_sum_x, &device);
    //             let sum_y: Vec<f32> = read_buffer(&read_sum_y, &device);
    //             let sum_z: Vec<f32> = read_buffer(&read_sum_z, &device);

    //             // Exactly 2 entries (num_workgroups * num_objects) in each buffer.
    //             assert_eq!(counts.len(), 2);
    //             assert_eq!(sum_x.len(), 2);
    //             assert_eq!(sum_y.len(), 2);
    //             assert_eq!(sum_z.len(), 2);

    //             // Object 0 (left half)
    //             assert_eq!(counts[0], expected_count);
    //             assert!((sum_x[0] - expected_x).abs() < 1e-5);
    //             assert!((sum_y[0] - expected_y).abs() < 1e-5);
    //             assert!((sum_z[0] - expected_z).abs() < 1e-5);

    //             // Object 1 (right half)
    //             assert_eq!(counts[1], expected_count);
    //             assert!((sum_x[1] - expected_x).abs() < 1e-5);
    //             assert!((sum_y[1] - expected_y).abs() < 1e-5);
    //             assert!((sum_z[1] - expected_z).abs() < 1e-5);
    //         });
    //     }
}
