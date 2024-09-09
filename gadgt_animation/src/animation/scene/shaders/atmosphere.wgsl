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

// Rayleigh scattering constants for RGB
const PI: f32 = 3.14159265;
const EARTH_RADIUS: f32 = 6378137.0;
const ATMOSPHERE_HEIGHT: f32 = 100000.0; //100km
const ATMOSPHERE_DENSITY: f32 = 0.0001;
const RAYLEIGH_COEFFICIENT: vec3<f32> = vec3<f32>(5.5e-6, 1.35e-5, 3.31e-5);
const MIE_COEFFICIENT: vec3<f32> = vec3<f32>(2.0e-6);
const MIE_SCATTERING: f32 = 0.9; // Mie forward scattering factor

@fragment
fn fs_main(in: VertexOutput) -> FragmentOutput {
    let light_dir = normalize(uniforms.light_pos - in.world_pos);
    let view_dir = normalize(uniforms.camera_pos.xyz - in.world_pos);

    // Calculate the dot product between the normal and the light direction
    var light_angle = dot(in.normal, light_dir);
    if light_angle <= 0.0 {
        light_angle = dot(in.normal, -light_dir);
    }

    let view_angle = dot(view_dir, in.normal);


    // Smooth the transition between day and night
    //let atmosphere_intensity = smoothstep(0.0, 1.0, light_angle);
    let atmosphere_intensity = (1.0-view_angle) * light_angle; // strongest at horizons, not when looking straight down

    // Apply Rayleigh and Mie scattering, modulated by the atmosphere intensity
    let rayleigh_scatter = exp(-light_angle / ATMOSPHERE_DENSITY) * RAYLEIGH_COEFFICIENT;
    let mie_scatter = exp(-light_angle * MIE_SCATTERING) * MIE_COEFFICIENT;

    // Combine the scattering
    let scattering = rayleigh_scatter;// + mie_scatter;

    // Final color blending with Earth's texture
    var final_color = in.color.rgb * atmosphere_intensity * scattering;

    // Compute logarithmic depth
    let far_plane = 1e12;
    let view_depth = length(uniforms.camera_pos.xyz - in.world_pos);
    let log_depth = log2(view_depth + 1.0) / log2(far_plane + 1.0);

    var out: FragmentOutput;
    out.color = vec4<f32>(final_color, in.color.a);    
    out.depth = log_depth;

    return out;
}
