struct Uniforms {
    projection: mat4x4<f32>,
    camera_pos: vec4<f32>,
    light_color: vec4<f32>,
}

const LIGHT_POS: vec3<f32> = vec3<f32>(0.0, 3.0, 3.0);

@group(0) @binding(0) var<uniform> uniforms: Uniforms;

struct Vertex {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) tangent: vec3<f32>,
    @location(3) uv: vec2<f32>,
}

struct Cube {
    @location(4) matrix_0: vec4<f32>,
    @location(5) matrix_1: vec4<f32>,
    @location(6) matrix_2: vec4<f32>,
    @location(7) matrix_3: vec4<f32>,
    @location(8) normal_matrix_0: vec3<f32>,
    @location(9) normal_matrix_1: vec3<f32>,
    @location(10) normal_matrix_2: vec3<f32>,
}

struct Output {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) uv: vec2<f32>,
    @location(1) tangent_pos: vec3<f32>,
    @location(2) tangent_camera_pos: vec3<f32>,
    @location(3) tangent_light_pos: vec3<f32>,
}

@vertex
fn vs_main(vertex: Vertex, cube: Cube) -> Output {
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

    //convert to tangent space to calculate lighting in same coordinate space as normal map sample
    let tangent = normalize(normal_matrix * vertex.tangent);
    let normal = normalize(normal_matrix * vertex.normal);
    let bitangent = cross(tangent, normal);

    //shift everything into tangent space
    let tbn = transpose(mat3x3<f32>(tangent, bitangent, normal));

    let world_pos = cube_matrix * vec4<f32>(vertex.position, 1.0);

    var out: Output;
    out.clip_pos = uniforms.projection * world_pos;
    out.uv = vertex.uv;
    out.tangent_pos = tbn * world_pos.xyz;
    out.tangent_camera_pos = tbn * uniforms.camera_pos.xyz;
    out.tangent_light_pos = tbn * LIGHT_POS;

    return out;
}

//cube properties
const CUBE_BASE_COLOR: vec4<f32> = vec4<f32>(0.294118, 0.462745, 0.611765, 0.6);

@fragment
fn fs_main(in: Output) -> @location(0) vec4<f32> {
    return CUBE_BASE_COLOR;
}