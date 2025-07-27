@group(0) @binding(0)
var object_id_tex: texture_2d<u32>;

@group(0) @binding(1)
var<storage, read_write> per_object_sum: array<atomic<u32>>;

@compute @workgroup_size(8, 8)
fn main(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let dims = textureDimensions(object_id_tex);
    if (global_id.x >= dims.x || global_id.y >= dims.y) {
        return;
    }

    let coords = vec2<i32>(global_id.xy);
    let object_id = textureLoad(object_id_tex, coords, 0).r;

    // Clamp or guard object_id (optional)
    if (object_id > 0u && object_id - 1u < arrayLength(&per_object_sum)) {
        atomicAdd(&per_object_sum[object_id - 1u], 1u);
    }
}


@compute @workgroup_size(64)
fn zero_buffer(@builtin(global_invocation_id) id: vec3<u32>) {
    if (id.x < arrayLength(&per_object_sum)) {
        atomicStore(&per_object_sum[id.x], 0u);
    }
}