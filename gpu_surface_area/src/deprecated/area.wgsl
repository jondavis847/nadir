struct SimpleVertex {
    pos: vec3<f32>,
}

struct Uniforms {
    direction: vec3<f32>,
    origin: vec3<f32>,
    grid_x_size: f32,
    grid_y_size: f32,
    grid_z_distance: f32,
    grid_nrays_x: u32,
    grid_nrays_y: u32,
    _padding: u32,
}

struct GeometryMetadata {
    index_offset: u32,
    index_count: u32,
    triangle_count: u32,
    _padding: u32,
}

struct RayIntersection {
    hit: bool,
    t: f32,
    u: f32,
    v: f32,
}

struct BVHNode {
    min_bounds: vec3<f32>,
    left_first: u32,
    max_bounds: vec3<f32>,
    triangle_count: u32,
}

struct GeometryBVHInfo {
    root_node_index: u32,
    node_count: u32,
}

struct RayHit {
    closest_t: f32,
    geometry_id: u32,
    projected_area: f32,
    _padding: u32,
}

@group(0) @binding(0) var<storage, read> vertices: array<SimpleVertex>;
@group(0) @binding(1) var<storage, read> indices: array<u32>;
@group(0) @binding(2) var<storage, read> transforms: array<mat4x4<f32>>;
@group(0) @binding(3) var<uniform> uniforms: Uniforms;
@group(0) @binding(4) var<storage, read_write> results: array<f32>;
@group(0) @binding(5) var<storage, read> metadata: array<GeometryMetadata>;
@group(0) @binding(6) var<storage, read> bvh_nodes: array<BVHNode>;
@group(0) @binding(7) var<storage, read> geometry_bvhs: array<GeometryBVHInfo>;
@group(0) @binding(8) var<storage, read> inverse_transforms: array<mat4x4<f32>>;
@group(0) @binding(9) var<storage, read_write> ray_hits: array<RayHit>;

// Ray-BVH traversal function
fn find_closest_hit_bvh(ray_origin: vec3<f32>, ray_direction: vec3<f32>) -> RayHit {
    var closest_hit: RayHit;
    closest_hit.closest_t = 1e30;
    closest_hit.geometry_id = 0xFFFFFFFFu;
    closest_hit.projected_area = 0.0;
    
    // Test against each geometry's BVH
    for (var geom_id = 0u; geom_id < arrayLength(&geometry_bvhs); geom_id++) {
        let hit = traverse_geometry_bvh(ray_origin, ray_direction, geom_id);

        if hit.closest_t < closest_hit.closest_t {
            closest_hit = hit;
        }
    }

    return closest_hit;
}

@compute @workgroup_size(64)
fn find_closest_hits(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let ray_id = global_id.x;

    if ray_id >= uniforms.grid_nrays_x * uniforms.grid_nrays_y {
        return;
    }
    
    // Generate ray
    let ray_origin = generate_ray_origin(ray_id);
    let ray_direction = normalize(uniforms.direction);
    
    // Use BVH to find closest hit
    let hit = find_closest_hit_bvh(ray_origin, ray_direction);
    
    // Store result
    ray_hits[ray_id] = hit;
}