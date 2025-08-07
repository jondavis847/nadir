struct Uniforms {
    num_objects: u32,
    num_workgroups: u32,
};

struct WorkgroupResult {
  id: u32,
  count: u32,
  pos_x: f32,
  pos_y: f32,
  pos_z: f32,
  pad0: u32,
  pad1: u32,
  pad2: u32,
};

@group(0) @binding(0) var<uniform> uniforms: Uniforms;
@group(1) @binding(0) var<storage, read> input_result_buffer: array<WorkgroupResult>;
@group(1) @binding(1) var<storage, read_write> output_result_buffer: array<WorkgroupResult>;

var<workgroup> shared_count: array<u32, 256>;
var<workgroup> shared_sum_x: array<f32, 256>;
var<workgroup> shared_sum_y: array<f32, 256>;
var<workgroup> shared_sum_z: array<f32, 256>;

const WORKGROUP_SIZE: u32 = 256;

@compute @workgroup_size(256)
fn main(
    @builtin(global_invocation_id) gid: vec3<u32>,
    @builtin(local_invocation_id)  lid: vec3<u32>,
    @builtin(workgroup_id)         wid: vec3<u32>
) {
    let input_length = arrayLength(&input_result_buffer);
    let input_index = gid.x;
    let lane_id = lid.x;
    
    // Process each object separately
    for (var i = 1u; i <= uniforms.num_objects; i = i + 1u) {
        let output_index = wid.x * uniforms.num_objects + (i - 1u);        
        //let input_index = wid.x * WORKGROUP_SIZE + lane_id;
        
        // Load data for this object
        if (input_index >= input_length) {
            shared_count[lane_id] = 0u;
            shared_sum_x[lane_id] = 0.0;
            shared_sum_y[lane_id] = 0.0;
            shared_sum_z[lane_id] = 0.0;
        } else {
            let input_result = input_result_buffer[input_index];
            if (input_result.id == i) {
                shared_count[lane_id] = input_result.count;
                shared_sum_x[lane_id] = input_result.pos_x;
                shared_sum_y[lane_id] = input_result.pos_y;
                shared_sum_z[lane_id] = input_result.pos_z;
            } else {
                shared_count[lane_id] = 0u;
                shared_sum_x[lane_id] = 0.0;
                shared_sum_y[lane_id] = 0.0;
                shared_sum_z[lane_id] = 0.0;
            }
        }
        
        workgroupBarrier();
        
        // Perform reduction for this object
        var stride = WORKGROUP_SIZE / 2u;
        loop {
            if (stride == 0u) { break; }
            
            if (lane_id < stride) {
                let other = lane_id + stride;
                shared_count[lane_id] += shared_count[other];
                shared_sum_x[lane_id] += shared_sum_x[other];
                shared_sum_y[lane_id] += shared_sum_y[other];
                shared_sum_z[lane_id] += shared_sum_z[other];
            }
            
            workgroupBarrier();
            stride = stride >> 1u;
        }
        
        // Write result for this object
        if (lane_id == 0u) {
            output_result_buffer[output_index] = WorkgroupResult(
                i,
                shared_count[0],
                shared_sum_x[0], 
                shared_sum_y[0], 
                shared_sum_z[0],
                0u, 0u, 0u
            );
        }
        
        workgroupBarrier(); // Ensure write is complete before next object
    }
}
