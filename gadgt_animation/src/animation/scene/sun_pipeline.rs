use gadgt_3d::{mesh::MeshGpu, vertex::Vertex};
use iced::widget::shader::wgpu::{self, util::DeviceExt, PipelineLayout};

// earth and earth atmosphere require their own pipelines since they have their own custom shaders

#[derive(Debug)]
pub struct SunPipeline {
    pub pipeline: wgpu::RenderPipeline,
    pub vertex_buffer: wgpu::Buffer,
    pub instance_buffer: wgpu::Buffer,
    pub n_vertices: u32,
    pub n_instances: u32,
}

impl SunPipeline {
    pub fn new(
        device: &wgpu::Device,
        format: wgpu::TextureFormat,
        layout: &PipelineLayout,
        meshes: &[MeshGpu],
        vertices: Vec<Vertex>,
        sample_count: u32,
    ) -> Self {
        const VERTEX_LABEL: &str = "sun.vertex.buffer";
        const INSTANCE_LABEL: &str = "sun.instance.buffer";
        const SHADER_LABEL: &str = "sun.shader";
        const PIPELINE_LABEL: &str = "sun.pipeline";
        const SHADER_FILE: &str = include_str!("shaders/sun.wgsl");

        // Create the constant vertex buffer
        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(VERTEX_LABEL),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        // Create an instance buffer with just 1 set of data, of which we will later overwrite with more instances of the geometry
        let instance_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(INSTANCE_LABEL),
            contents: bytemuck::cast_slice(&meshes),
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        });

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some(SHADER_LABEL),
            source: wgpu::ShaderSource::Wgsl(std::borrow::Cow::Borrowed(SHADER_FILE)),
        });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some(PIPELINE_LABEL),
            layout: Some(layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                buffers: &[Vertex::desc(), MeshGpu::desc()],
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: Some(wgpu::Face::Back), // Cull backfaces for performance and artifact reduction
                front_face: wgpu::FrontFace::Ccw,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState {
                count: sample_count,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            multiview: None,
        });
        let n_vertices = vertices.len() as u32;
        let n_instances = meshes.len() as u32;

        Self {
            pipeline,
            vertex_buffer,
            instance_buffer,
            n_vertices,
            n_instances,
        }
    }

    pub fn update(&mut self, queue: &wgpu::Queue, meshes: &[MeshGpu]) {
        queue.write_buffer(&self.instance_buffer, 0, bytemuck::cast_slice(meshes));
    }
}

#[derive(Debug)]
pub struct CoronaPipeline {
    pub pipeline: wgpu::RenderPipeline,
    pub vertex_buffer: wgpu::Buffer,
    pub instance_buffer: wgpu::Buffer,
    pub n_vertices: u32,
    pub n_instances: u32,
}

impl CoronaPipeline {
    pub fn new(
        device: &wgpu::Device,
        format: wgpu::TextureFormat,
        layout: &PipelineLayout,
        meshes: &[MeshGpu],
        vertices: Vec<Vertex>,
        sample_count: u32,
    ) -> Self {
        const VERTEX_LABEL: &str = "corona.vertex.buffer";
        const INSTANCE_LABEL: &str = "corona.instance.buffer";
        const PIPELINE_LABEL: &str = "corona.pipeline";
        const SHADER_LABEL: &str = "corona.shader";
        const SHADER_FILE: &str = include_str!("shaders/corona.wgsl");

        // Create the constant vertex buffer
        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(VERTEX_LABEL),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        // Create an instance buffer with just 1 set of data, of which we will later overwrite with more instances of the geometry
        let instance_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(INSTANCE_LABEL),
            contents: bytemuck::cast_slice(&meshes),
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        });

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some(SHADER_LABEL),
            source: wgpu::ShaderSource::Wgsl(std::borrow::Cow::Borrowed(SHADER_FILE)),
        });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some(PIPELINE_LABEL),
            layout: Some(layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                buffers: &[Vertex::desc(), MeshGpu::desc()],
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: Some(wgpu::Face::Back), // Cull backfaces for performance and artifact reduction
                front_face: wgpu::FrontFace::Ccw,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: false,
                depth_compare: wgpu::CompareFunction::Always,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState {
                count: sample_count,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
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
                            dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                            operation: wgpu::BlendOperation::Add,
                        },
                    }),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            multiview: None,
        });
        let n_vertices = vertices.len() as u32;
        let n_instances = meshes.len() as u32;

        Self {
            pipeline,
            vertex_buffer,
            instance_buffer,
            n_vertices,
            n_instances,
        }
    }

    pub fn update(&mut self, queue: &wgpu::Queue, meshes: &[MeshGpu]) {
        queue.write_buffer(&self.instance_buffer, 0, bytemuck::cast_slice(meshes));
    }
}
