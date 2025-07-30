/// Compute shader with shared memory optimization for better performance.
/// This replaces the count pass. Each workgroup does local accumulation
/// into workgroup-shared arrays, then writes to global `group_*` buffers.
@group(0) @binding(0) var object_id_tex: texture_2d<u32>;
@group(0) @binding(1) var position_tex: texture_2d<f32>;
@group(0) @binding(2) var<storage, read_write> group_counts: array<u32>;
@group(0) @binding(3) var<storage, read_write> group_sum_x: array<f32>;
@group(0) @binding(4) var<storage, read_write> group_sum_y: array<f32>;
@group(0) @binding(5) var<storage, read_write> group_sum_z: array<f32>;
@group(0) @binding(6) var<storage, read_write> total_counts: array<u32>;
@group(0) @binding(7) var<storage, read_write> total_sum_x: array<f32>;
@group(0) @binding(8) var<storage, read_write> total_sum_y: array<f32>;
@group(0) @binding(9) var<storage, read_write> total_sum_z: array<f32>;

var<workgroup> local_counts: array<u32, 256>;
var<workgroup> local_sum_x: array<f32, 256>;
var<workgroup> local_sum_y: array<f32, 256>;
var<workgroup> local_sum_z: array<f32, 256>;

@compute @workgroup_size(8, 8)
fn count_pixels_and_positions(@builtin(global_invocation_id) id: vec3<u32>,
                              @builtin(local_invocation_id) lid: vec3<u32>,
                              @builtin(workgroup_id) wid: vec3<u32>) {
    let tex_dims = textureDimensions(object_id_tex);
    if (id.x >= tex_dims.x || id.y >= tex_dims.y) {
        return;
    }

    let index = lid.y * 8u + lid.x;
    local_counts[index] = 0u;
    local_sum_x[index] = 0.0;
    local_sum_y[index] = 0.0;
    local_sum_z[index] = 0.0;
    workgroupBarrier();

    let obj_id = textureLoad(object_id_tex, vec2<i32>(id.xy), 0).r;
    if (obj_id == 0u) {
        return;
    }
    let pos = textureLoad(position_tex, vec2<i32>(id.xy), 0).xyz;
    local_counts[index] = 1u;
    local_sum_x[index] = pos.x;
    local_sum_y[index] = pos.y;
    local_sum_z[index] = pos.z;
    workgroupBarrier();

    // Intra-workgroup reduction
    var stride = 1u;
    while (stride < 64u) {
        if (index % (stride * 2u) == 0u) {
            local_counts[index] += local_counts[index + stride];
            local_sum_x[index] += local_sum_x[index + stride];
            local_sum_y[index] += local_sum_y[index + stride];
            local_sum_z[index] += local_sum_z[index + stride];
        }
        stride = stride * 2u;
        workgroupBarrier();
    }

    if (index == 0u) {
        let group_base = wid.y * gridDim.x + wid.x;
        let offset = group_base * 256u + (obj_id - 1u);
        group_counts[offset] = local_counts[0];
        group_sum_x[offset] = local_sum_x[0];
        group_sum_y[offset] = local_sum_y[0];
        group_sum_z[offset] = local_sum_z[0];
    }
}

@compute @workgroup_size(64)
fn clear_buffers(@builtin(global_invocation_id) id: vec3<u32>) {
    let i = id.x;
    if (i < arrayLength(&group_counts)) {
        group_counts[i] = 0u;
        group_sum_x[i] = 0.0;
        group_sum_y[i] = 0.0;
        group_sum_z[i] = 0.0;
    }
    if (i < arrayLength(&total_counts)) {
        total_counts[i] = 0u;
        total_sum_x[i] = 0.0;
        total_sum_y[i] = 0.0;
        total_sum_z[i] = 0.0;
    }
}
