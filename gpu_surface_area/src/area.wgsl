struct SimpleVertex {
    pos: vec3<f32>,
}


struct Ray {
    origin: vec3<f32>,
    _pad1: f32,
    direction: vec3<f32>,
    _pad2: f32,
};

struct GeometryMetadata {
    index_offset: u32,
    index_count: u32,
    triangle_count: u32,
    _padding: u32,
}

@group(0) @binding(0) var<storage, read> vertices: array<SimpleVertex>;
@group(0) @binding(1) var<storage, read> indices: array<u32>;
@group(0) @binding(2) var<storage, read> transforms: array<mat4x4<f32>>;
@group(0) @binding(3) var<uniform> direction: vec4<f32>; // really just 3 element but need 4 for alignment, first 3 are direction
@group(0) @binding(4) var<storage, read_write> results: array<f32>; // One per triangle
@group(0) @binding(5) var<storage, read> metadata: array<GeometryMetadata>;
@group(0) @binding(6) var<storage, read> rays: array<Ray>;

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let triangle_id = global_id.x;
    let ray_id = global_id.y;

    let total_triangles = arrayLength(&indices) / 3u;
    if triangle_id >= total_triangles {
        return;
    }    
    if ray_id >= arrayLength(&rays) {
        return;
    }

    // Initialize to 0.0
    results[triangle_id] = 0.0;
    
    // Find which geometry this triangle belongs to
    var geometry_id: u32 = 0u;
    for (var g = 0u; g < arrayLength(&metadata); g++) {
        let geom = metadata[g];
        let triangle_start = geom.index_offset / 3u;
        let triangle_end = triangle_start + geom.triangle_count;

        if triangle_id >= triangle_start && triangle_id < triangle_end {
            geometry_id = g;
            break;
        }
    }
    let transform = transforms[geometry_id];

    // Calculate surface area for this triangle
    let i0 = indices[triangle_id * 3u];
    let i1 = indices[triangle_id * 3u + 1u];
    let i2 = indices[triangle_id * 3u + 2u];

    // Transform each vertex
    let v0 = (transform * vec4<f32>(vertices[i0].pos, 1.0)).xyz;
    let v1 = (transform * vec4<f32>(vertices[i1].pos, 1.0)).xyz;
    let v2 = (transform * vec4<f32>(vertices[i2].pos, 1.0)).xyz;

    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let normal = cross(edge1, edge2);
    let triangle_area = length(normal) * 0.5;
    let direction = direction.xyz;
    let normal_normalized = normalize(normal);
    let projection_factor = dot(normal_normalized, normalize(direction));

    let projected_area = triangle_area * max(projection_factor, 0.0);
    results[triangle_id] = projected_area;    
}
