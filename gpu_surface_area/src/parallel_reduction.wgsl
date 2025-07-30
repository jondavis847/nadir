// ======= SHARED STRUCTURES =======
struct Config {
    num_objects: u32,
}

struct GridInfo {
    num_workgroups: u32,
}

// ======= STAGE 1: PER-TILE REDUCTION WITHOUT ATOMICS =======
@group(0) @binding(0) var object_id_tex: texture_2d<u32>;
@group(0) @binding(1) var position_tex: texture_2d<f32>;
@group(0) @binding(2) var<storage, read_write> group_counts: array<u32>;
@group(0) @binding(3) var<storage, read_write> group_sum_x: array<f32>;
@group(0) @binding(4) var<storage, read_write> group_sum_y: array<f32>;
@group(0) @binding(5) var<storage, read_write> group_sum_z: array<f32>;
@group(0) @binding(6) var<uniform> config: Config;


var<workgroup> shared_tmp: array<u32, 4096>; // Treated as pool of memory

@compute @workgroup_size(16, 16)
fn stage1_per_tile_reduction_shared(@builtin(global_invocation_id) id: vec3<u32>,
    @builtin(local_invocation_id) lid: vec3<u32>,
    @builtin(workgroup_id) wid: vec3<u32>) {
    let tex_dims = textureDimensions(object_id_tex);
    let grid_width = (tex_dims.x + 15u) / 16u;
    let group_idx = wid.y * grid_width + wid.x;
    let thread_idx = lid.y * 16u + lid.x;
    
    // Define memory layout in the shared pool
    // Each thread gets space for all objects
    let thread_data_size = config.num_objects * 4u; // 4 values per object (count, x, y, z)
    let thread_offset = thread_idx * thread_data_size;
    
    // Clear our section of shared memory
    for (var i = 0u; i < config.num_objects; i++) {
        let base = thread_offset + i * 4u;
        shared_tmp[base] = 0u;     // count
        shared_tmp[base + 1u] = 0u; // x (as bitcast u32)
        shared_tmp[base + 2u] = 0u; // y (as bitcast u32)
        shared_tmp[base + 3u] = 0u; // z (as bitcast u32)
    }
    
    // Process our pixel
    if id.x < tex_dims.x && id.y < tex_dims.y {
        let obj_id = textureLoad(object_id_tex, vec2<i32>(id.xy), 0).r;
        if obj_id > 0u && obj_id <= config.num_objects {
            let pos = textureLoad(position_tex, vec2<i32>(id.xy), 0).xyz;
            let obj_index = obj_id - 1u;
            
            // Each thread only updates its own memory
            let base = thread_offset + obj_index * 4u;
            shared_tmp[base] = 1u;
            shared_tmp[base + 1u] = bitcast<u32>(pos.x);
            shared_tmp[base + 2u] = bitcast<u32>(pos.y);
            shared_tmp[base + 3u] = bitcast<u32>(pos.z);
        }
    }
    workgroupBarrier();
    
    // Now reduce across threads for each object
    // Each thread handles reduction for some objects
    for (var obj = 0u; obj < config.num_objects; obj += 1u) {
        if obj % 256u == thread_idx { // Distribute objects across threads
            var count = 0u;
            var x_sum = 0.0;
            var y_sum = 0.0;
            var z_sum = 0.0;
            
            // Sum this object across all threads
            for (var t = 0u; t < 256u; t++) {
                let base = t * thread_data_size + obj * 4u;
                count += shared_tmp[base];
                x_sum += bitcast<f32>(shared_tmp[base + 1u]);
                y_sum += bitcast<f32>(shared_tmp[base + 2u]);
                z_sum += bitcast<f32>(shared_tmp[base + 3u]);
            }
            
            // Write result to global memory if non-zero
            if count > 0u {
                let offset = group_idx * config.num_objects + obj;
                group_counts[offset] = count;
                group_sum_x[offset] = x_sum;
                group_sum_y[offset] = y_sum;
                group_sum_z[offset] = z_sum;
            }
        }
    }
}

// ======= STAGE 2: REDUCE WORKGROUP RESULTS =======
@group(0) @binding(0) var<storage, read> workgroup_counts: array<u32>;
@group(0) @binding(1) var<storage, read> workgroup_sum_x: array<f32>;
@group(0) @binding(2) var<storage, read> workgroup_sum_y: array<f32>;
@group(0) @binding(3) var<storage, read> workgroup_sum_z: array<f32>;
@group(0) @binding(4) var<storage, read_write> total_counts: array<u32>;
@group(0) @binding(5) var<storage, read_write> total_sum_x: array<f32>;
@group(0) @binding(6) var<storage, read_write> total_sum_y: array<f32>;
@group(0) @binding(7) var<storage, read_write> total_sum_z: array<f32>;
@group(0) @binding(8) var<uniform> stage2_config: Config;
@group(0) @binding(9) var<uniform> grid_info: GridInfo;

var<workgroup> shared_count: array<u32, 256>;
var<workgroup> shared_sum_x: array<f32, 256>;
var<workgroup> shared_sum_y: array<f32, 256>;
var<workgroup> shared_sum_z: array<f32, 256>;

@compute @workgroup_size(256)
fn stage2_reduce_workgroups(@builtin(global_invocation_id) id: vec3<u32>,
    @builtin(local_invocation_id) local_id: vec3<u32>) {
    // One workgroup per object
    let object_id = id.z; // Dispatched with (1, 1, num_objects)
    let thread_id = local_id.x;
    
    // Initialize local accumulators
    var local_count = 0u;
    var local_sum_x = 0.0;
    var local_sum_y = 0.0;
    var local_sum_z = 0.0;
    
    // Each thread processes a portion of the workgroups for this object
    for (var i = thread_id; i < grid_info.num_workgroups; i += 256u) {
        let offset = i * stage2_config.num_objects + object_id;
        local_count += workgroup_counts[offset];
        local_sum_x += workgroup_sum_x[offset];
        local_sum_y += workgroup_sum_y[offset];
        local_sum_z += workgroup_sum_z[offset];
    }
    
    // Store in shared memory for reduction
    shared_count[thread_id] = local_count;
    shared_sum_x[thread_id] = local_sum_x;
    shared_sum_y[thread_id] = local_sum_y;
    shared_sum_z[thread_id] = local_sum_z;
    workgroupBarrier();
    
    // Parallel reduction within workgroup
    for (var stride = 128u; stride > 0u; stride >>= 1u) {
        if thread_id < stride {
            shared_count[thread_id] += shared_count[thread_id + stride];
            shared_sum_x[thread_id] += shared_sum_x[thread_id + stride];
            shared_sum_y[thread_id] += shared_sum_y[thread_id + stride];
            shared_sum_z[thread_id] += shared_sum_z[thread_id + stride];
        }
        workgroupBarrier();
    }
    
    // Thread 0 writes final result for this object
    if thread_id == 0u {
        total_counts[object_id] = shared_count[0];
        total_sum_x[object_id] = shared_sum_x[0];
        total_sum_y[object_id] = shared_sum_y[0];
        total_sum_z[object_id] = shared_sum_z[0];
    }
}

// ======= STAGE 3: CALCULATE FINAL RESULTS =======
@group(0) @binding(0) var<storage, read> input_counts: array<u32>;
@group(0) @binding(1) var<storage, read> input_sum_x: array<f32>;
@group(0) @binding(2) var<storage, read> input_sum_y: array<f32>;
@group(0) @binding(3) var<storage, read> input_sum_z: array<f32>;
@group(0) @binding(4) var<storage, read_write> object_counts: array<u32>;
@group(0) @binding(5) var<storage, read_write> object_centers: array<vec3<f32>>;
@group(0) @binding(6) var<uniform> stage3_config: Config;

@compute @workgroup_size(64)
fn stage3_calculate_centers(@builtin(global_invocation_id) id: vec3<u32>) {
    let obj_idx = id.x;
    if obj_idx >= stage3_config.num_objects {
        return;
    }

    let count = input_counts[obj_idx];
    object_counts[obj_idx] = count;

    if count > 0u {
        // Calculate center of mass
        let center = vec3<f32>(
            input_sum_x[obj_idx] / f32(count),
            input_sum_y[obj_idx] / f32(count),
            input_sum_z[obj_idx] / f32(count)
        );
        object_centers[obj_idx] = center;
    } else {
        object_centers[obj_idx] = vec3<f32>(0.0, 0.0, 0.0);
    }
}

// ======= CLEAR BUFFERS =======
@group(0) @binding(0) var<storage, read_write> buffer_to_clear1: array<u32>;
@group(0) @binding(1) var<storage, read_write> buffer_to_clear2: array<f32>;
@group(0) @binding(2) var<storage, read_write> buffer_to_clear3: array<f32>;
@group(0) @binding(3) var<storage, read_write> buffer_to_clear4: array<f32>;
@group(0) @binding(4) var<uniform> clear_config: ClearConfig;

struct ClearConfig {
    count1: u32,
    count2: u32,
}

@compute @workgroup_size(256)
fn clear_buffers(@builtin(global_invocation_id) id: vec3<u32>) {
    let i = id.x;

    if i < clear_config.count1 {
        buffer_to_clear1[i] = 0u;
    }

    if i < clear_config.count2 {
        buffer_to_clear2[i] = 0.0;
        buffer_to_clear3[i] = 0.0;
        buffer_to_clear4[i] = 0.0;
    }
}