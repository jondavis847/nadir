mod buffer;
mod uniforms;

pub use uniforms::Uniforms;

use super::geometries::cuboid::{Cuboid, CuboidRaw};
use super::geometries::ellipsoid::{Ellipsoid, EllipsoidRaw};
use super::vertex::Vertex;
use buffer::DynamicBuffer;

use iced::widget::shader::wgpu::{self, util::DeviceExt};

use iced::{Rectangle, Size};

pub struct Pipeline {
    pipeline_cuboid: wgpu::RenderPipeline,
    pipeline_ellipsoid: wgpu::RenderPipeline,
    vertex_cuboid: wgpu::Buffer,
    vertex_ellipsoid: wgpu::Buffer,
    instance_cuboids: DynamicBuffer,
    instance_ellipsoids: DynamicBuffer,
    uniforms: wgpu::Buffer,
    uniform_bind_group: wgpu::BindGroup,
}

impl Pipeline {
    pub fn new(
        device: &wgpu::Device,
        _queue: &wgpu::Queue,
        format: wgpu::TextureFormat,
        _target_size: Size<u32>,
    ) -> Self {
        let cuboid_raw = CuboidRaw::from_cuboid(&Cuboid::default());

        //vertices of one cube
        let vertex_cuboid: wgpu::Buffer =
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("cuboid vertex buffer"),
                contents: bytemuck::cast_slice(&cuboid_raw.vertices()),
                usage: wgpu::BufferUsages::VERTEX,
            });

        //cube instance data
        let instance_cuboids = DynamicBuffer::new(
            device,
            "cuboid instance buffer",
            std::mem::size_of::<CuboidRaw>() as u64,
            wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        );

        let ellipsoid_raw = EllipsoidRaw::from_ellipsoid(&Ellipsoid::default());

        //vertices of one ellipsoid
        let vertex_ellipsoid = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("ellipsoid vertex buffer"),
            contents: bytemuck::cast_slice(&ellipsoid_raw.vertices()),
            usage: wgpu::BufferUsages::VERTEX,
        });

        //ellipsoid instance data
        let instance_ellipsoids = DynamicBuffer::new(
            device,
            "ellipsoid instance buffer",
            std::mem::size_of::<EllipsoidRaw>() as u64,
            wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        );

        //uniforms for all objects
        let uniforms = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("uniform buffer"),
            size: std::mem::size_of::<Uniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let uniform_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("cuboid uniform bind group layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            });

        let uniform_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("uniform bind group"),
            layout: &uniform_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniforms.as_entire_binding(),
            }],
        });

        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("pipeline layout"),
            bind_group_layouts: &[&uniform_bind_group_layout],
            push_constant_ranges: &[],
        });

        let cuboid_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("cuboid shader"),
            source: wgpu::ShaderSource::Wgsl(std::borrow::Cow::Borrowed(include_str!(
                "shaders/cuboid.wgsl"
            ))),
        });       

        let pipeline_cuboid = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("cuboid pipeline"),
            layout: Some(&layout),
            vertex: wgpu::VertexState {
                module: &cuboid_shader,
                entry_point: "vs_main",
                buffers: &[Vertex::desc(), CuboidRaw::desc()],
            },
            primitive: wgpu::PrimitiveState::default(),
            depth_stencil: None,
            multisample: wgpu::MultisampleState {
                count: 1,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            fragment: Some(wgpu::FragmentState {
                module: &cuboid_shader,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: Some(wgpu::BlendState {
                        color: wgpu::BlendComponent {
                            src_factor: wgpu::BlendFactor::SrcAlpha,
                            dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                            operation: wgpu::BlendOperation::Add,
                        },
                        alpha: wgpu::BlendComponent {
                            src_factor: wgpu::BlendFactor::One,
                            dst_factor: wgpu::BlendFactor::One,
                            operation: wgpu::BlendOperation::Max,
                        },
                    }),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            multiview: None,
        });

        let ellipsoid_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("ellipsoid shader"),
            source: wgpu::ShaderSource::Wgsl(std::borrow::Cow::Borrowed(include_str!(
                "shaders/ellipsoid.wgsl"
            ))),
        });

        let pipeline_ellipsoid = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("ellipsoid pipeline"),
            layout: Some(&layout),
            vertex: wgpu::VertexState {
                module: &ellipsoid_shader,
                entry_point: "vs_main",
                buffers: &[Vertex::desc(), EllipsoidRaw::desc()],
            },
            primitive: wgpu::PrimitiveState::default(),
            depth_stencil: None,
            multisample: wgpu::MultisampleState {
                count: 1,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            fragment: Some(wgpu::FragmentState {
                module: &ellipsoid_shader,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: Some(wgpu::BlendState {
                        color: wgpu::BlendComponent {
                            src_factor: wgpu::BlendFactor::SrcAlpha,
                            dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                            operation: wgpu::BlendOperation::Add,
                        },
                        alpha: wgpu::BlendComponent {
                            src_factor: wgpu::BlendFactor::One,
                            dst_factor: wgpu::BlendFactor::One,
                            operation: wgpu::BlendOperation::Max,
                        },
                    }),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            multiview: None,
        });

        Self {
            pipeline_cuboid,
            pipeline_ellipsoid,
            vertex_cuboid,
            vertex_ellipsoid,
            instance_cuboids,
            instance_ellipsoids,
            uniforms,
            uniform_bind_group,
        }
    }

    pub fn update(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _target_size: Size<u32>,
        uniforms: &Uniforms,        
        cuboids: &[CuboidRaw],
        ellipsoids: &[EllipsoidRaw],
    ) {
        //dbg!(&ellipsoids);
        // update uniforms
        queue.write_buffer(&self.uniforms, 0, bytemuck::bytes_of(uniforms));

        //resize cuboid vertex buffer if cubes amount changed
        let n_cuboids = cuboids.len();
        let cuboid_size = n_cuboids * std::mem::size_of::<CuboidRaw>();
        self.instance_cuboids.resize(device, cuboid_size as u64);        
        queue.write_buffer(&self.instance_cuboids.raw, 0, bytemuck::cast_slice(cuboids));

        //resize ellipsoid vertex buffer if cubes amount changed
        let n_ellipsoids = ellipsoids.len();
        let ellipsoid_size = n_ellipsoids * std::mem::size_of::<EllipsoidRaw>();
        self.instance_ellipsoids.resize(device, ellipsoid_size as u64);
        queue.write_buffer(&self.instance_ellipsoids.raw, 0, bytemuck::cast_slice(ellipsoids));
        
    }

    pub fn render(
        &self,
        target: &wgpu::TextureView,
        encoder: &mut wgpu::CommandEncoder,
        viewport: Rectangle<u32>,
        num_cuboids: u32,
        num_ellipsoids: u32,
        _show_depth: bool,
    ) {
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("gadgt.pipeline.pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: target,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Load,
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
            });

            pass.set_scissor_rect(viewport.x, viewport.y, viewport.width, viewport.height);
            pass.set_bind_group(0, &self.uniform_bind_group, &[]);
        
            // Render Cuboids
            pass.set_pipeline(&self.pipeline_cuboid); // Set the cuboid pipeline
            pass.set_vertex_buffer(0, self.vertex_cuboid.slice(..));
            pass.set_vertex_buffer(1, self.instance_cuboids.raw.slice(..));
            pass.draw(0..36, 0..num_cuboids);
        
            // Render Ellipsoids
            pass.set_pipeline(&self.pipeline_ellipsoid); // Switch to the ellipsoid pipeline
            pass.set_vertex_buffer(0, self.vertex_ellipsoid.slice(..)); // Use slot 0 for vertex data
            pass.set_vertex_buffer(1, self.instance_ellipsoids.raw.slice(..)); // Use slot 1 for instance data
        
            // Assuming n_vertices is correctly calculated based on latitude and longitude bands
            let n_vertices = 1440; // Adjust as necessary for dynamic vertex count
            pass.draw(0..n_vertices, 0..num_ellipsoids);
        }
    }
}
