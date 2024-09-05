mod buffer;
mod uniforms;
use std::fs;

use gadgt_3d::geometry::{Geometry, GeometryTrait};
use iced::widget::shader::wgpu::PipelineLayout;
pub use uniforms::Uniforms;

use gadgt_3d::{mesh::MeshGpu, vertex::Vertex};

use iced::widget::shader::wgpu::{self, util::DeviceExt};
use iced::{Rectangle, Size};
use image::{load_from_memory, GenericImageView};

pub struct CuboidPipeline(pub Pipeline);
pub struct Ellipsoid16Pipeline(pub Pipeline);
pub struct Ellipsoid32Pipeline(pub Pipeline);
pub struct Ellipsoid64Pipeline(pub Pipeline);

#[derive(Debug)]
pub struct Pipeline {
    pub pipeline: wgpu::RenderPipeline,
    pub vertex_buffer: wgpu::Buffer,
    pub instance_buffer: wgpu::Buffer,
    pub n_vertices: u32,
    pub n_instances: u32,
}

impl Pipeline {
    pub fn new(
        device: &wgpu::Device,
        format: wgpu::TextureFormat,
        layout: &PipelineLayout,
        shader_file: &str,
        label: &str,
        vertices: Vec<Vertex>,
        meshes: &[MeshGpu],
    ) -> Self {        
        let vertex_label = format!("{label}.vertex.buffer");
        let instance_label = format!("{label}.instance.buffer");
        let shader_label = format!("{label}.shader");
        let pipeline_label = format!("{label}.pipeline");

        // Create the constant vertex buffer
        let vertex_buffer: wgpu::Buffer =
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(&vertex_label),
                contents: bytemuck::cast_slice(&vertices),
                usage: wgpu::BufferUsages::VERTEX,
            });

        // Create an instance buffer with just 1 set of data, of which we will later overwrite with more instances of the geometry
        let instance_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(&instance_label),
            contents: bytemuck::cast_slice(&meshes),
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,            
        });

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some(&shader_label),
            source: wgpu::ShaderSource::Wgsl(std::borrow::Cow::Borrowed(&shader_file)),
        });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some(&pipeline_label),
            layout: Some(layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                buffers: &[Vertex::desc(), MeshGpu::desc()],
            },
            primitive: wgpu::PrimitiveState::default(),            
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),            
            multisample: wgpu::MultisampleState {
                count: 4,
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
                            dst_factor: wgpu::BlendFactor::One,
                            operation: wgpu::BlendOperation::Max,
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

    pub fn update(
        &mut self,        
        queue: &wgpu::Queue,        
        meshes: &[MeshGpu],        
    ) {        
        queue.write_buffer(
            &self.instance_buffer,
            0,
            bytemuck::cast_slice(meshes),
        );
    }

    pub fn render<'a>(
        &'a self,
        pass: &'a mut wgpu::RenderPass<'a>,
    ) {
        {
            pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
            pass.set_vertex_buffer(1, self.instance_buffer.slice(..));
            pass.draw(0..self.n_vertices, 0..self.n_instances);
        }
    }
}

fn load_texture(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    texture_bytes: &[u8],
    label: &str,
) -> wgpu::TextureView {
    let img = load_from_memory(texture_bytes).expect("Failed to load image from embedded bytes");
    let rgba = img.to_rgba8();
    let dimensions = img.dimensions();

    let texture_size = wgpu::Extent3d {
        width: dimensions.0,
        height: dimensions.1,
        depth_or_array_layers: 1,
    };

    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some(label),
        size: texture_size,
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8UnormSrgb,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });

    queue.write_texture(
        wgpu::ImageCopyTexture {
            texture: &texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        &rgba,
        wgpu::ImageDataLayout {
            offset: 0,
            bytes_per_row: Some(4 * dimensions.0),
            rows_per_image: Some(dimensions.1),
        },
        texture_size,
    );

    let texture_view = texture.create_view(&wgpu::TextureViewDescriptor::default());

    texture_view
}
