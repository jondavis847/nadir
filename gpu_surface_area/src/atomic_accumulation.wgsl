@group(0) @binding(0)
var object_id_tex: texture_2d<u32>;

@group(0) @binding(1)
var<storage, read_write> per_object_id_sum: array<atomic<u32>>;
@group(0) @binding(2)
var<storage, read_write> per_object_pos_sum_x: array<atomic<f32>>;
@group(0) @binding(3)
var<storage, read_write> per_object_pos_sum_y: array<atomic<f32>>;
@group(0) @binding(4)
var<storage, read_write> per_object_pos_sum_z: array<atomic<f32>>;

@group(0) @binding(5)
var position_tex: texture_2d<f32>; // RGBA32Float format, but we only need xyz

@compute @workgroup_size(8, 8)
fn main(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let dims = textureDimensions(object_id_tex);
    if (global_id.x >= dims.x || global_id.y >= dims.y) {
        return;
    }

    let coords = vec2<i32>(global_id.xy);
    let object_id = textureLoad(object_id_tex, coords, 0).r;

    if (object_id > 0u && object_id - 1u < arrayLength(&per_object_id_sum)) {
        atomicAdd(&per_object_id_sum[object_id - 1u], 1u);

        let pos = textureLoad(position_tex, coords, 0).xyz;

        atomicAdd(&per_object_pos_sum_x[object_id - 1u], pos.x);
        atomicAdd(&per_object_pos_sum_y[object_id - 1u], pos.y);
        atomicAdd(&per_object_pos_sum_z[object_id - 1u], pos.z);
    }
}


@compute @workgroup_size(64)
fn zero_buffer(@builtin(global_invocation_id) id: vec3<u32>) {
    if (id.x < arrayLength(&per_object_id_sum)) {
        atomicStore(&per_object_id_sum[id.x], 0u);
        atomicStore(&per_object_pos_sum_x[id.x], 0.0);
        atomicStore(&per_object_pos_sum_y[id.x], 0.0);
        atomicStore(&per_object_pos_sum_z[id.x], 0.0);
    }
}