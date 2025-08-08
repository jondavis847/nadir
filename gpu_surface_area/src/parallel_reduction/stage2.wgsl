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
        
    // Process each object separately
    for (var i = 1u; i <= uniforms.num_objects; i = i + 1u) {
        let input_index = wid.x * WORKGROUP_SIZE * uniforms.num_objects + lid.x * uniforms.num_objects + (i - 1u);
        let output_index = wid.x * uniforms.num_objects + (i - 1u);        
        
        // Load data for this object
        if (input_index >= input_length) {
            shared_count[lid.x] = 0u;
            shared_sum_x[lid.x] = 0.0;
            shared_sum_y[lid.x] = 0.0;
            shared_sum_z[lid.x] = 0.0;
        } else {
            let input_result = input_result_buffer[input_index];
            if (input_result.id == i) {
                shared_count[lid.x] = input_result.count;
                shared_sum_x[lid.x] = input_result.pos_x;
                shared_sum_y[lid.x] = input_result.pos_y;
                shared_sum_z[lid.x] = input_result.pos_z;
            } else {
                shared_count[lid.x] = 0u;
                shared_sum_x[lid.x] = 0.0;
                shared_sum_y[lid.x] = 0.0;
                shared_sum_z[lid.x] = 0.0;
            }

        }
        
        workgroupBarrier();

   
        // Perform reduction for this object
        var stride = WORKGROUP_SIZE / 2u;
        loop {
            if (stride == 0u) { break; }
            
            if (lid.x < stride) {
                let other = lid.x + stride;
                shared_count[lid.x] += shared_count[other];
                shared_sum_x[lid.x] += shared_sum_x[other];
                shared_sum_y[lid.x] += shared_sum_y[other];
                shared_sum_z[lid.x] += shared_sum_z[other];
            }
            
            workgroupBarrier();
            stride = stride/2u;
        }
        
        // Write result for this object
        if (output_index < arrayLength(&output_result_buffer)) {
            if (lid.x == 0u) {
                output_result_buffer[output_index] = WorkgroupResult(
                    i,
                    shared_count[0],
                    shared_sum_x[0], 
                    shared_sum_y[0], 
                    shared_sum_z[0],
                    0u, 0u, 0u
                );
            }           
        }

        
        workgroupBarrier(); // Ensure write is complete before next object
    }
}



