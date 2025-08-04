// Bind group 0: Shared uniforms (projection matrix)
struct SharedUniforms {
    projection_matrix: mat4x4<f32>,
}
@group(0) @binding(0) var<uniform> shared_uniforms: SharedUniforms;

// Bind group 1: Per-geometry uniforms (world transform + object ID)
struct GeometryUniforms {
    world_transform: mat4x4<f32>,
    object_id: u32,
}
@group(1) @binding(0) var<uniform> geometry_uniforms: GeometryUniforms;

struct VertexInput {
    @location(0) position: vec3<f32>,
}

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,    
    @location(0) object_id: u32,
    @location(1) world_position: vec4<f32>,
}

@vertex
fn vs_main(vertex: VertexInput) -> VertexOutput {
    // Transform vertex position: local -> world -> clip space
    let world_pos = geometry_uniforms.world_transform * vec4<f32>(vertex.position, 1.0);
    let clip_pos = shared_uniforms.projection_matrix * world_pos;

    return VertexOutput(
        clip_pos,        
        geometry_uniforms.object_id,
        world_pos,
    );
}

struct FragmentOutput {
    @location(0) object_id: u32,
    @location(1) position: vec4<f32>,
}


@fragment
fn fs_main(in: VertexOutput) -> FragmentOutput {
    return FragmentOutput(
        in.object_id,
        in.world_position,
    );
}