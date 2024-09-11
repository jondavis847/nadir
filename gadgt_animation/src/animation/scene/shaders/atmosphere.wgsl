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
    let light_dir = normalize(uniforms.light_pos - in.world_pos);
    let view_dir = normalize(uniforms.camera_pos.xyz - in.world_pos);

    let light_angle = abs(dot(in.normal, light_dir));
    let night_angle = max(dot(in.normal,-light_dir),0.0);
    let view_angle = dot(view_dir, in.normal);    

    let scatter_factor = 0.1; // controls flat scattering intensity
    let scatter_color = normalize(vec3<f32>(0.4, 0.5, 1.0));
    
    let light_angle_factor = 0.5; // controls how much the angle towards the sun affects lighting
    let view_angle_factor = 5.0; // controls how much the angle towards the camera affects the color
    let night_angle_factor = 5.0; // controls how much to reduce atmosphere color on the night side

    // fade out values with very small normals so that the atmosphere "blurs" into space
    let fade_start = 0.0;
    let fade_end = 0.2;
    let fade_strength = 3.0;

    var fade_factor: f32;
    if view_angle < fade_start {
        fade_factor = 0.0;
    } else if view_angle > fade_end {
        fade_factor = 1.0;
    } else {
        let t = (view_angle - fade_start) / (fade_end - fade_start);
        fade_factor = pow(t, fade_strength);
    }

    let scattering = pow(1.0 - light_angle,light_angle_factor) * pow(1.0 - view_angle, view_angle_factor) * pow(1.0 - night_angle,night_angle_factor) * fade_factor * scatter_factor * scatter_color;    
    
    let final_color = vec3<f32>(scattering);
    
    // Compute logarithmic depth
    let far_plane = 1e12;
    let view_depth = length(uniforms.camera_pos.xyz - in.world_pos);
    let log_depth = log2(view_depth + 1.0) / log2(far_plane + 1.0);

    var out: FragmentOutput;
    out.color = vec4<f32>(final_color, 1.0);    
    out.depth = log_depth;

    return out;
}
