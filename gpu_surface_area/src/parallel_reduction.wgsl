@group(0) @binding(0)
var object_id_tex: texture_2d<u32>;

@group(0) @binding(1)
var<storage, read_write> object_area: array<u32>;

@compute @workgroup_size(8, 8)
fn main(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let dims = textureDimensions(object_id_tex);
    if (global_id.x >= dims.x || global_id.y >= dims.y) {
        return;
    }

    let id = textureLoad(object_id_tex, vec2<i32>(global_id.xy), 0).r;

    if (id > 0u) {
        // Subtract 1 to get 0-based index
        object_area[id - 1u] += 1u; // NOTE: this requires atomics for correctness!
    }
}