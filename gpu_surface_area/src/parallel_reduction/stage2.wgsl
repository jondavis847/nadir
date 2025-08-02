struct Uniforms {
  num_objects: u32,
  num_workgroups: u32,
};

struct WorkgroupResult {
  id: u32,
  count: u32,
  pos: [f32;3],  
  _padding: [u32; 3],
}

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
    let input_length = input_result_buffer.length;
    let start = gid.x * WORKGROUP_SIZE;
    let lane_id = lid.x; // lane ID in [0..255] for 16×16 workgroup
    let input_index = start + lane_id;    
    let output_index = wid.x;   

    for (var i = 1; i <= uniforms.num_objects; i = i + 1) {     
        // populate inputs to shared memory in prep for binary tree reduction
        let input_result = input_result_buffer[input_index];
        if (input_result.id == i) {
            shared_count[lane_id] = input_result.count;
            shared_sum_x[lane_id] = input_result.pos[0];
            shared_sum_y[lane_id] = input_result.pos[1];
            shared_sum_z[lane_id] = input_result.pos[2];
        } else {
            shared_count[lane_id] = 0u;
            shared_sum_x[lane_id] = 0.0;
            shared_sum_y[lane_id] = 0.0;
            shared_sum_z[lane_id] = 0.0;
        }        
        workgroupBarrier();
        
        // binary tree reduction within the workgroup
        var stride = 128u; //initialize to half the workgroup size        
        loop {
            if (stride == 0u) { break; }

            if (lane_id < stride) {                
                let other = lane_id + stride;
                shared_count[lane_id] += shared_count[other];
                shared_sum_x[lane_id] += shared_sum_x[other];
                shared_sum_y[lane_id] += shared_sum_y[other];
                shared_sum_z[lane_id] += shared_sum_z[other];
            }

            workgroupBarrier();  // synchronize threads before next stride
            stride = stride >> 1u; //right shift operator, divides stride by 2
        }
        if (lane_id == 0u && shared_count[0] > 0u) {
            output_result_buffer[output_index * uniforms.num_objects + (i - 1u)] = WorkgroupResult {
                id: i,
                count: shared_count[0],
                pos: vec3<f32>(shared_sum_x[0], shared_sum_y[0], shared_sum_z[0]),
                _padding: vec3<u32>(0u, 0u, 0u),
            };
        }
        workgroupBarrier(); // ensure all lanes are done before next object
    }    
}
    