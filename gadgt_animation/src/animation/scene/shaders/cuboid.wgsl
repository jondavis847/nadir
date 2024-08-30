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

struct Cuboid {
    @location(4) matrix_0: vec4<f32>,    
    @location(5) matrix_1: vec4<f32>,    
    @location(6) matrix_2: vec4<f32>,    
    @location(7) matrix_3: vec4<f32>,    
    @location(8) normal_matrix_0: vec3<f32>,    
    @location(9) normal_matrix_1: vec3<f32>,    
    @location(10) normal_matrix_2: vec3<f32>,  
    @location(11) color: vec4<f32>,  
}

struct Output {
  @builtin(position) clip_pos: vec4<f32>,
    @location(0) uv: vec2<f32>,
    @location(1) world_pos: vec3<f32>,
    @location(2) normal: vec3<f32>,
    @location(3) color: vec4<f32>,
}

@vertex
fn vs_main(vertex: Vertex, cube: Cuboid) -> Output {

    let cube_matrix = mat4x4<f32>(
        cube.matrix_0,
        cube.matrix_1,
        cube.matrix_2,
        cube.matrix_3,
    );

    let normal_matrix = mat3x3<f32>(
        cube.normal_matrix_0,
        cube.normal_matrix_1,
        cube.normal_matrix_2,
    );

    let world_pos = cube_matrix * vec4<f32>(vertex.position, 1.0);
    let normal = normalize(normal_matrix * vertex.normal);

    var out: Output;
    out.clip_pos = uniforms.projection * world_pos;
    out.uv = vertex.uv;
    out.world_pos = world_pos.xyz;
    out.normal = normal;
    out.color = cube.color;

    return out;
}
struct FragOutput {
    @location(0) color: vec4<f32>,
    @builtin(frag_depth) depth: f32,
}

@fragment
fn fs_main(in: Output) -> FragOutput {
    let light_dir = normalize(uniforms.light_pos - in.world_pos);
    let view_dir = normalize(uniforms.camera_pos.xyz - in.world_pos);

    let ambient = 0.05 * in.color.rgb;

    // Diffuse lighting (Lambertian reflectance)
    let diff = max(dot(in.normal, light_dir), 0.0);
    let diffuse = diff * in.color.rgb * uniforms.light_color.rgb;

    // Specular lighting (Phong reflection model)
    let light_reflect = normalize(reflect(-light_dir, in.normal));
    var specular_factor = dot(view_dir, light_reflect);    
    var specular = vec3<f32>(0.0);

    if (specular_factor > 0.0) {
        specular_factor = pow(specular_factor, 32.0);
        specular = uniforms.light_color.rgb * specular_factor;
    } 

    // Combine results
    let result = ambient + diffuse + specular;

    // Compute the logarithmic depth
    let far_plane = 1e12; // Adjust this value according to your far plane distance
    let view_depth = length(uniforms.camera_pos.xyz - in.world_pos);
    let log_depth = log2(view_depth + 1.0) / log2(far_plane + 1.0);


    var out: FragOutput;
    out.color = vec4<f32>(result, 1.0);
    out.depth = log_depth;
    return out;    
}