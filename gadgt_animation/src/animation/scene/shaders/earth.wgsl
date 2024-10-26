struct Uniforms {
    projection: mat4x4<f32>,
    camera_pos: vec4<f32>,    
    light_color: vec4<f32>,
    light_pos: vec3<f32>,
}

@group(0) @binding(0) var<uniform> uniforms: Uniforms;
@group(1) @binding(0) var earth_sampler: sampler;
@group(1) @binding(1) var earth_texture: texture_2d<f32>; 
@group(1) @binding(2) var earth_night: texture_2d<f32>; 
@group(1) @binding(3) var earth_spec: texture_2d<f32>; 
@group(1) @binding(4) var earth_clouds: texture_2d<f32>; 

struct Vertex {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,    
    @location(2) tangent: vec3<f32>,    
    @location(3) uv: vec2<f32>,
}

struct Earth {
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

struct Output {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) uv: vec2<f32>,
    @location(1) world_pos: vec3<f32>,
    @location(2) normal: vec3<f32>,    
    @location(3) tangent: vec3<f32>,    
}

@vertex
fn vs_main(vertex: Vertex, earth: Earth) -> Output {
    let transformation_matrix = mat4x4<f32>(
        earth.matrix_0,
        earth.matrix_1,
        earth.matrix_2,
        earth.matrix_3,
    );

    let normal_matrix = mat3x3<f32>(
        earth.normal_matrix_0,
        earth.normal_matrix_1,
        earth.normal_matrix_2,
    );

    let world_pos = transformation_matrix * vec4<f32>(vertex.position, 1.0);
    let normal = normalize(normal_matrix * vertex.normal);
    let tangent = normalize(normal_matrix * vertex.tangent);

    var out: Output;
    out.clip_pos = uniforms.projection * world_pos;
    out.uv = vertex.uv;
    out.world_pos = world_pos.xyz;
    out.normal = normal;
    out.tangent = tangent;

    return out;
}

struct FragOutput {
    @location(0) color: vec4<f32>,
    @builtin(frag_depth) depth: f32,
}

@fragment
fn fs_main(in: Output) -> FragOutput {
    let light_dir = normalize(uniforms.light_pos);
    let view_dir = normalize(uniforms.camera_pos.xyz - in.world_pos);

    // i cant figure out why turning on msaa darkens the image, this is a bandaid
    let msaa_scale = 5.0;
    let cloud_scale = 5.0;

    let day_color_raw = msaa_scale * textureSample(earth_texture, earth_sampler, in.uv).rgb;
    let cloud_texture  = textureSample(earth_clouds, earth_sampler, in.uv);
    let cloud_color = cloud_scale * cloud_texture.rgb;
    let cloud_alpha = cloud_texture.a;
    let night_texture = textureSample(earth_night, earth_sampler, in.uv).rgb;
    let night_color = msaa_scale * vec3<f32>(1.0,0.8745,0.7843) * night_texture;

    let cloud_intensity = 0.8;
    let day_color = mix(day_color_raw, cloud_color, cloud_intensity);

    // Ambient lighting
    let ambient = day_color * 0.1; //dont need ambient now that we're using a night map

    // Diffuse lighting (Lambertian reflectance)
    let diff = dot(in.normal, light_dir);
    let night_diff = dot(in.normal, -light_dir);    

    let color = diff * day_color + 0.5 * night_diff * night_color;        
    let diffuse = color * uniforms.light_color.rgb;    

    // Specular lighting (Phong reflection model)
    let light_reflect = normalize(reflect(-light_dir, in.normal));
    var specular_factor = dot(view_dir, light_reflect);
    var specular = vec3<f32>(0.0);

    if specular_factor > 0.0 {
        specular_factor = pow(specular_factor, 100.0);
        let spec_map_value = textureSample(earth_spec, earth_sampler, in.uv).rgb;        
        specular = uniforms.light_color.rgb * specular_factor * spec_map_value;
    } 

    // Adjust specular intensity based on the cloud alpha (less specular on clouds)
    let cloud_specular_multiplier = mix(1.0, 0.1, cloud_alpha); // Reduce specular on clouds
    let final_specular = specular * cloud_specular_multiplier;

    // Combine results
    let result = ambient + diffuse + final_specular;
    
    // Compute the logarithmic depth
    let far_plane = 1.1e13; // Adjust this value according to your far plane distance
    let view_depth = length(uniforms.camera_pos.xyz - in.world_pos);
    let log_depth = log2(view_depth + 1.0) / log2(far_plane + 1.0);

    var out: FragOutput;
    out.color = vec4<f32>(result, 1.0);    
    out.depth = log_depth;

    return out;
}
