struct SimpleVertex {
    pos: vec3<f32>,
}

struct GeometryMetadata {
    index_offset: u32,
    index_count: u32,
    triangle_count: u32,
    _padding: u32,
}

@group(0) @binding(0) var<storage, read> vertices: array<SimpleVertex>;
@group(0) @binding(1) var<storage, read> indices: array<u32>;
@group(0) @binding(2) var<uniform> direction: vec3<f32>;
@group(0) @binding(3) var<storage, read_write> results: array<f32>; // One per geometry
@group(0) @binding(4) var<storage, read> metadata: array<GeometryMetadata>; // Add this!

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let triangle_id = global_id.x;
    let total_triangles = arrayLength(&indices) / 3u;

    if triangle_id >= total_triangles {
        return;
    }
    
    // Clear results on first thread
    if triangle_id == 0u {
        for (var i = 0u; i < arrayLength(&results); i++) {
            results[i] = 0.0;
        }
    }
    workgroupBarrier();
    
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
    
    // Calculate surface area for this triangle
    let i0 = indices[triangle_id * 3u];
    let i1 = indices[triangle_id * 3u + 1u];
    let i2 = indices[triangle_id * 3u + 2u];

    let v0 = vertices[i0].pos;
    let v1 = vertices[i1].pos;
    let v2 = vertices[i2].pos;

    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let normal = cross(edge1, edge2);
    let triangle_area = length(normal) * 0.5;

    let normal_normalized = normalize(normal);
    let projection_factor = dot(normal_normalized, normalize(direction));

    if projection_factor > 0.0 {
        let projected_area = triangle_area * projection_factor;
        
        // This is problematic - need atomic operations for race conditions!
        results[geometry_id] += projected_area; // ← Race condition!
    }
}