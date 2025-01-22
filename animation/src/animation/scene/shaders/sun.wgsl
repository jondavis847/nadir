struct Uniforms {
    projection: mat4x4<f32>,
    camera_pos: vec4<f32>,    
    light_color: vec4<f32>,       
    light_pos: vec3<f32>,     
}

@group(0) @binding(0) var<uniform> uniforms: Uniforms;

struct Vertex {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) tangent: vec3<f32>,
    @location(3) uv: vec2<f32>,
}

struct Instance {
    @location(4) matrix_0: vec4<f32>,    
    @location(5) matrix_1: vec4<f32>,    
    @location(6) matrix_2: vec4<f32>,    
    @location(7) matrix_3: vec4<f32>,    
    @location(8) normal_matrix_0: vec3<f32>,    
    @location(9) normal_matrix_1: vec3<f32>,    
    @location(10) normal_matrix_2: vec3<f32>,  
    @location(11) color: vec4<f32>,  
    @location(12) material_type: u32,
    @location(13) specular_power: f32,
}

struct VertexOutput {
  @builtin(position) clip_pos: vec4<f32>,
    @location(0) uv: vec2<f32>,
    @location(1) world_pos: vec3<f32>,
    @location(2) normal: vec3<f32>,
    @location(3) color: vec4<f32>,
    @location(4) material_type: u32,
    @location(5) specular_power: f32,
}

@vertex
fn vs_main(vertex: Vertex, instance: Instance) -> VertexOutput {

    let transformation_matrix = mat4x4<f32>(
        instance.matrix_0,
        instance.matrix_1,
        instance.matrix_2,
        instance.matrix_3,
    );

    let normal_matrix = mat3x3<f32>(
        instance.normal_matrix_0,
        instance.normal_matrix_1,
        instance.normal_matrix_2,
    );

    let world_pos = transformation_matrix * vec4<f32>(vertex.position, 1.0);
    let normal = normalize(normal_matrix * vertex.normal);
    
    // Compute the logarithmic depth
    let far_plane = 1e12; // Adjust this value according to your far plane distance
    let view_depth = length(uniforms.camera_pos.xyz - world_pos.xyz);
    let log_depth = log2(view_depth + 1.0) / log2(far_plane + 1.0);

    var out: VertexOutput;
    out.clip_pos = uniforms.projection * world_pos;
    out.uv = vertex.uv;
    out.world_pos = world_pos.xyz;
    out.normal = normal;
    out.color = instance.color;
    out.material_type = instance.material_type;
    out.specular_power = instance.specular_power;

    return out;
}
struct FragmentOutput {
    @location(0) color: vec4<f32>,
    @builtin(frag_depth) depth: f32,
}

@fragment
fn fs_main(in: VertexOutput) -> FragmentOutput {    
    let sun_color = vec4<f32>(1.0, 0.9804, 0.8627,1.0);
   
    var out: FragmentOutput;
    out.color = vec4<f32>(sun_color);   
    return out;    
}