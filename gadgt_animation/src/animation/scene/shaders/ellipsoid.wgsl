struct Uniforms {
    projection: mat4x4<f32>,
    camera_pos: vec4<f32>,
    light_pos: vec3<f32>,
    light_color: vec4<f32>,    
}

@group(0) @binding(0) var<uniform> uniforms: Uniforms;

struct Vertex {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) tangent: vec3<f32>,
    @location(3) uv: vec2<f32>,
}

struct Ellipsoid {
    @location(4) matrix_0: vec4<f32>,    
    @location(5) matrix_1: vec4<f32>,    
    @location(6) matrix_2: vec4<f32>,    
    @location(7) matrix_3: vec4<f32>,    
    @location(8) normal_matrix_0: vec3<f32>,    
    @location(9) normal_matrix_1: vec3<f32>,    
    @location(10) normal_matrix_2: vec3<f32>,    
    @location(11) color: vec4<f32>, // RGBA color for the ellipsoid
}

struct Output {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) uv: vec2<f32>,
    @location(1) tangent_pos: vec3<f32>,
    @location(2) tangent_camera_pos: vec3<f32>,
    @location(3) tangent_light_pos: vec3<f32>,
    @location(4) color: vec4<f32>,
}

@vertex
fn vs_main(vertex: Vertex, ellipsoid: Ellipsoid) -> Output {     

    let ellipsoid_matrix = mat4x4<f32>(
         ellipsoid.matrix_0,
         ellipsoid.matrix_1,
         ellipsoid.matrix_2,
         ellipsoid.matrix_3,
     );

    let normal_matrix = mat3x3<f32>(
        ellipsoid.normal_matrix_0,
        ellipsoid.normal_matrix_1,
        ellipsoid.normal_matrix_2,
    );

    // Convert to tangent space to calculate lighting in the same coordinate space as normal map sample
    let tangent = normalize(normal_matrix * vertex.tangent);
    let normal = normalize(normal_matrix * vertex.normal);
    let bitangent = cross(tangent, normal);

    // Shift everything into tangent space
    let tbn = transpose(mat3x3<f32>(tangent, bitangent, normal));

    let world_pos = ellipsoid_matrix * vec4<f32>(vertex.position, 1.0);

    var out: Output;
    out.clip_pos = uniforms.projection * world_pos;
    out.uv = vertex.uv;
    out.tangent_pos = tbn * world_pos.xyz;
    out.tangent_camera_pos = tbn * uniforms.camera_pos.xyz;
    out.tangent_light_pos = tbn * uniforms.light_pos;
    out.color = ellipsoid.color;

    return out;
}

@fragment
fn fs_main(in: Output) -> @location(0) vec4<f32> {
    // Use the color from the ellipsoid data
    return in.color;
}
