struct Uniforms {
    projection: mat4x4<f32>,
    camera_pos: vec4<f32>,    
    light_color: vec4<f32>,
    light_pos: vec3<f32>,
}

@group(0) @binding(0) var<uniform> uniforms: Uniforms;
@group(0) @binding(1) var earth_texture: texture_2d<f32>; // Your texture
@group(0) @binding(2) var earth_sampler: sampler; // Your sampler
@group(0) @binding(3) var earth_night: texture_2d<f32>; // Your texture

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
    @location(8) color: vec4<f32>, // RGBA color for the ellipsoid
    @location(9) normal_matrix_0: vec3<f32>,    
    @location(10) normal_matrix_1: vec3<f32>,    
    @location(11) normal_matrix_2: vec3<f32>,    
    
}

struct Output {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) uv: vec2<f32>,
    @location(1) world_pos: vec3<f32>,
    @location(2) normal: vec3<f32>,
    @location(3) color: vec4<f32>,
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

    let world_pos = ellipsoid_matrix * vec4<f32>(vertex.position, 1.0);
    let normal = normalize(normal_matrix * vertex.normal);

    var out: Output;
    out.clip_pos = uniforms.projection * world_pos;
    out.uv = vertex.uv;
    out.world_pos = world_pos.xyz;
    out.normal = normal;
    out.color = ellipsoid.color;

    return out;
}

@fragment
fn fs_main(in: Output) -> @location(0) vec4<f32> {
    let light_dir = normalize(uniforms.light_pos);// - in.world_pos); not - world pos to be directional
    let view_dir = normalize(uniforms.camera_pos.xyz - in.world_pos);


    let diff = max(dot(in.normal, light_dir), 0.0);    
    
    let day_color = textureSample(earth_texture, earth_sampler, in.uv);
    let night_color = textureSample(earth_night, earth_sampler, in.uv);

    let color = diff * day_color + 0.1 * (1.0 - diff) * night_color;// 0.5 because it was just way too bright    

    //let ambient = 0.05 * in.color.rgb;
    let ambient = 0.0 * color.rgb;

    // Diffuse lighting (Lambertian reflectance)
    let diffuse = color.rgb * uniforms.light_color.rgb;

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

    return vec4<f32>(result, in.color.a);
}
