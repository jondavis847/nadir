mod buffer;
mod uniforms;

pub use uniforms::Uniforms;

use super::geometries::cuboid::{Cuboid, CuboidRaw};
use super::geometries::earth::Earth;
use super::geometries::ellipsoid::{Ellipsoid, EllipsoidRaw};
use super::vertex::Vertex;
use buffer::DynamicBuffer;

use iced::widget::shader::wgpu::{self, util::DeviceExt};
use iced::{Rectangle, Size};
use image::{load_from_memory, GenericImageView};

pub struct Pipeline {
    depth_view: wgpu::TextureView,
    pipeline_cuboid: wgpu::RenderPipeline,
    pipeline_earth: Option<wgpu::RenderPipeline>,
    pipeline_ellipsoid: wgpu::RenderPipeline,
    vertex_cuboid: wgpu::Buffer,
    vertex_earth: Option<wgpu::Buffer>,
    vertex_ellipsoid: wgpu::Buffer,
    instance_cuboids: DynamicBuffer,
    instance_earth: Option<DynamicBuffer>,
    instance_ellipsoids: DynamicBuffer,
    uniforms: wgpu::Buffer,
    uniform_bind_group: wgpu::BindGroup,
    earth_bind_group: Option<wgpu::BindGroup>,
}

impl Pipeline {
    pub fn new(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        format: wgpu::TextureFormat,
        target_size: Size<u32>,
        earth: bool,
    ) -> Self {
        //depth buffer
        let depth_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("cubes depth texture"),
            size: wgpu::Extent3d {
                width: target_size.width,
                height: target_size.height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth32Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        });

        let depth_view = depth_texture.create_view(&wgpu::TextureViewDescriptor::default());

        // cuboids
        let cuboid_raw = CuboidRaw::from_cuboid(&Cuboid::default());

        //vertices of one cube
        let vertex_cuboid: wgpu::Buffer =
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("cuboid vertex buffer"),
                contents: bytemuck::cast_slice(&cuboid_raw.vertices()),
                usage: wgpu::BufferUsages::VERTEX,
            });

        // cube instance data
        let instance_cuboids = DynamicBuffer::new(
            device,
            "cuboid instance buffer",
            std::mem::size_of::<CuboidRaw>() as u64,
            wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        );

        // ellipsoids
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
                label: Some("uniform bind group layout"),
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
            label: Some("uniform and texture bind group"),
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
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
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
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
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

        // earth
        let (vertex_earth, instance_earth, pipeline_earth, earth_bind_group) = if earth {
            let earth_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("earth shader"),
                source: wgpu::ShaderSource::Wgsl(std::borrow::Cow::Borrowed(include_str!(
                    "shaders/earth.wgsl"
                ))),
            });

            //vertex buffer
            let vertex_earth = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("earth vertex buffer"),
                contents: bytemuck::cast_slice(&ellipsoid_raw.vertices()),
                usage: wgpu::BufferUsages::VERTEX,
            });

            //ellipsoid instance data
            let instance_earth = DynamicBuffer::new(
                device,
                "earth instance buffer",
                std::mem::size_of::<EllipsoidRaw>() as u64,
                wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            );

            const EARTH_COLOR: &[u8] = include_bytes!("../../../resources/earth_color_4k.jpg");
            const EARTH_NIGHT: &[u8] = include_bytes!("../../../resources/earth_night_4k.jpg");
            const EARTH_SPEC: &[u8] = include_bytes!("../../../resources/earth_spec_4k.jpg");

            let earth_day = load_texture(device, queue, EARTH_COLOR, "earth_color");
            let earth_night = load_texture(device, queue, EARTH_NIGHT, "earth_night");
            let earth_spec = load_texture(device, queue, EARTH_SPEC, "earth_spec");

            let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
                address_mode_u: wgpu::AddressMode::Repeat,
                address_mode_v: wgpu::AddressMode::ClampToEdge,
                address_mode_w: wgpu::AddressMode::ClampToEdge,
                mag_filter: wgpu::FilterMode::Linear,
                min_filter: wgpu::FilterMode::Linear,
                mipmap_filter: wgpu::FilterMode::Linear,
                anisotropy_clamp: 16,
                ..Default::default()
            });

            let earth_bind_group_layout =
                device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                    label: Some("earth bind group layout"),
                    entries: &[
                        // sampler
                        wgpu::BindGroupLayoutEntry {
                            binding: 0,
                            visibility: wgpu::ShaderStages::FRAGMENT,
                            ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                            count: None,
                        },
                        //earth color
                        wgpu::BindGroupLayoutEntry {
                            binding: 1,
                            visibility: wgpu::ShaderStages::FRAGMENT,
                            ty: wgpu::BindingType::Texture {
                                sample_type: wgpu::TextureSampleType::Float { filterable: true },
                                view_dimension: wgpu::TextureViewDimension::D2,
                                multisampled: false,
                            },
                            count: None,
                        },
                        //earth night
                        wgpu::BindGroupLayoutEntry {
                            binding: 2,
                            visibility: wgpu::ShaderStages::FRAGMENT,
                            ty: wgpu::BindingType::Texture {
                                sample_type: wgpu::TextureSampleType::Float { filterable: true },
                                view_dimension: wgpu::TextureViewDimension::D2,
                                multisampled: false,
                            },
                            count: None,
                        },
                        //earth specular
                        wgpu::BindGroupLayoutEntry {
                            binding: 3,
                            visibility: wgpu::ShaderStages::FRAGMENT,
                            ty: wgpu::BindingType::Texture {
                                sample_type: wgpu::TextureSampleType::Float { filterable: true },
                                view_dimension: wgpu::TextureViewDimension::D2,
                                multisampled: false,
                            },
                            count: None,
                        },
                    ],
                });

            let earth_bindgroup = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("earth and texture bind group"),
                layout: &earth_bind_group_layout,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: wgpu::BindingResource::Sampler(&sampler),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: wgpu::BindingResource::TextureView(&earth_day),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: wgpu::BindingResource::TextureView(&earth_night),
                    },
                    wgpu::BindGroupEntry {
                        binding: 3,
                        resource: wgpu::BindingResource::TextureView(&earth_spec),
                    },
                ],
            });

            let earth_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("earth pipeline layout"),
                bind_group_layouts: &[&uniform_bind_group_layout, &earth_bind_group_layout],
                push_constant_ranges: &[],
            });

            let pipeline_earth = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some("earth pipeline"),
                layout: Some(&earth_layout),
                vertex: wgpu::VertexState {
                    module: &earth_shader,
                    entry_point: "vs_main",
                    buffers: &[Vertex::desc(), EllipsoidRaw::desc()],
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
                    count: 1,
                    mask: !0,
                    alpha_to_coverage_enabled: false,
                },
                fragment: Some(wgpu::FragmentState {
                    module: &earth_shader,
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
            (
                Some(vertex_earth),
                Some(instance_earth),
                Some(pipeline_earth),
                Some(earth_bindgroup),
            )
        } else {
            (None, None, None, None)
        };

        Self {
            depth_view,
            pipeline_cuboid,
            pipeline_ellipsoid,
            pipeline_earth,
            vertex_cuboid,
            vertex_earth,
            vertex_ellipsoid,
            instance_cuboids,
            instance_earth,
            instance_ellipsoids,
            uniforms,
            uniform_bind_group,
            earth_bind_group,
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
        earth: &Option<EllipsoidRaw>,
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
        self.instance_ellipsoids
            .resize(device, ellipsoid_size as u64);
        queue.write_buffer(
            &self.instance_ellipsoids.raw,
            0,
            bytemuck::cast_slice(ellipsoids),
        );

        if let Some(earth) = earth {
            if let Some(instance_earth) = &self.instance_earth {                
                queue.write_buffer(&instance_earth.raw, 0, bytemuck::bytes_of(earth));
            }
        }
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
                        //load: wgpu::LoadOp::Load,
                        load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });

            pass.set_scissor_rect(viewport.x, viewport.y, viewport.width, viewport.height);
            pass.set_bind_group(0, &self.uniform_bind_group, &[]);


            if let Some(earth_bind_group) = &self.earth_bind_group {
                pass.set_bind_group(1, earth_bind_group, &[]);
            }

            // Render Earth
            if let Some(pipeline_earth) = &self.pipeline_earth {
                pass.set_pipeline(pipeline_earth); 
                if let Some(vertex_earth) = &self.vertex_earth {
                    pass.set_vertex_buffer(0, vertex_earth.slice(..));
                }
                if let Some(instance_earth) = &self.instance_earth {
                    pass.set_vertex_buffer(1, instance_earth.raw.slice(..));
                    
                }
                pass.draw(0..1440, 0..1);
            }

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
