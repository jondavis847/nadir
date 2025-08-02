struct Uniforms {
  num_objects: u32,
  num_workgroups: u32,
};

struct WorkgroupResult {
  id: u32,
  count: u32,
  pos: vec3<f32>,  
  _padding: vec3<u32>,
}

@group(0) @binding(0) var<uniform> uniforms: Uniforms;
@group(1) @binding(0) var object_id_tex: texture_2d<u32>;
@group(1) @binding(1) var position_tex:  texture_2d<f32>;
@group(1) @binding(2) var<storage, read_write> result_buffer: array<WorkgroupResult>;

const WORKGROUP_WIDTH: u32 = 16u;
const WORKGROUP_HEIGHT: u32 = 16u;

var<workgroup> shared_count: array<u32, 256>;
var<workgroup> shared_sum_x: array<f32, 256>;
var<workgroup> shared_sum_y: array<f32, 256>;
var<workgroup> shared_sum_z: array<f32, 256>;

@compute @workgroup_size(WORKGROUP_WIDTH, WORKGROUP_HEIGHT, 1)
fn main(
  @builtin(global_invocation_id) gid: vec3<u32>,  // absolute pixel coords this lane covers
  @builtin(local_invocation_id)  lid: vec3<u32>,  // lane coords within the tile
  @builtin(workgroup_id)         wid: vec3<u32>   // tile coords (x,y); z=0 in this stage
) {
  
  let tex_dims = textureDimensions(object_id_tex);
  let in_bounds = (gid.x < tex_dims.x && gid.y < tex_dims.y);  // guard for partially full tiles
  let lane_id = lid.y * WORKGROUP_WIDTH + lid.x; // lane ID in [0..255] for 16×16 workgroup
    
  let grid_width = (tex_dims.x + WORKGROUP_WIDTH - 1u) / WORKGROUP_WIDTH;
  let tile_idx = wid.y * grid_width + wid.x;  // flattened tile index in raster order

  // Load this lane's pixel once and reuse it for all objects
  // object_id==0 means "background / no object".
  var obj_id: u32 = 0u;
  var pos: vec3<f32> = vec3<f32>(0.0, 0.0, 0.0);
  if (in_bounds) {
    obj_id = textureLoad(object_id_tex, vec2<i32>(gid.xy), 0).r;
    pos    = textureLoad(position_tex,  vec2<i32>(gid.xy), 0).xyz;
  }

  // For each object, sum the contributions from all lanes in this tile.  
  for (var i=1u; i <= uniforms.num_objects; i = i + 1u) {
    if obj_id == i {
      shared_count[lane_id] = 1u; // this lane contributes 1 to the count
      shared_sum_x[lane_id] = pos.x; // this lane contributes its position
      shared_sum_y[lane_id] = pos.y;
      shared_sum_z[lane_id] = pos.z;
    } else {
      shared_count[lane_id] = 0u; // this lane does not contribute
      shared_sum_x[lane_id] = 0.0;
      shared_sum_y[lane_id] = 0.0;
      shared_sum_z[lane_id] = 0.0;
    }
    workgroupBarrier(); // ensure all lanes have reset before proceeding

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
      result_buffer[tile_idx * uniforms.num_objects + (i - 1u)] = WorkgroupResult (
        i,
        shared_count[0],
        vec3<f32>(shared_sum_x[0], shared_sum_y[0], shared_sum_z[0]),
        vec3<u32>(0u, 0u, 0u),
      );
    }
    workgroupBarrier(); // ensure all lanes are done before next object
  }
}

