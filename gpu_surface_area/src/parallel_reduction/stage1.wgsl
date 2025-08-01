// ====== UNIFORMS & BINDINGS ======

// Per-dispatch constants provided by the host (CPU).
// - num_objects: total number of labeled objects in the object_id texture
// - num_workgroups: total number of *tiles* (i.e., workgroups in X×Y) = ceil(W/ WG_X) * ceil(H / WG_Y)
//   (Used only for consistency/validation; Stage 1 doesn't need it directly except for clarity.)
struct Uniforms {
  num_objects: u32,
  num_workgroups: u32,
};

// Group(0) holds just the uniforms (separate bind group from the rest).
@group(0) @binding(0) var<uniform> uniforms: Uniforms;

// Group(1) holds the inputs and outputs for Stage 1.
// Inputs: per-pixel object IDs (u32) and per-pixel positions (f32 xyz).
// Outputs: per-(tile, object) partial reductions (count and sums).
@group(1) @binding(0) var object_id_tex: texture_2d<u32>;
@group(1) @binding(1) var position_tex:  texture_2d<f32>;

// Output buffers store one element per (tile, object), laid out as:
//   offset = tile_idx * uniforms.num_objects + object_index
// where tile_idx is the flattened workgroup (tile) index in raster order.
@group(1) @binding(2) var<storage, read_write> workgroup_id:     array<u32>;  // count per (tile, object)
@group(1) @binding(3) var<storage, read_write> workgroup_pos_x:  array<f32>;  // sum_x per (tile, object)
@group(1) @binding(4) var<storage, read_write> workgroup_pos_y:  array<f32>;  // sum_y per (tile, object)
@group(1) @binding(5) var<storage, read_write> workgroup_pos_z:  array<f32>;  // sum_z per (tile, object)


// ====== TUNABLE CONSTANTS (set via pipeline specialization constants if desired) ======

// Workgroup (tile) dimensions in threads/pixels.
override WG_X: u32 = 16u;
override WG_Y: u32 = 16u;

// Optional batching of objects processed per inner loop iteration. Larger CHUNK reduces loop overhead,
// smaller CHUNK may reduce register pressure. 32..128 is a good range to profile.
override CHUNK: u32 = 64u;


// ====== WORKGROUP-SHARED REDUCTION BUFFERS ======
//
// Fixed-size shared arrays for a 256-lane reduction (16×16 workgroup).
// This footprint is constant (~4 KB total), independent of num_objects.
// If you change WG_X/WG_Y, update these to WG_X*WG_Y (or introduce an override LANES).
var<workgroup> shared_count: array<u32, 256>;
var<workgroup> shared_sum_x: array<f32, 256>;
var<workgroup> shared_sum_y: array<f32, 256>;
var<workgroup> shared_sum_z: array<f32, 256>;


// ====== HELPERS ======

// Flatten local-invocation 2D coordinates (lid.x, lid.y) into a lane id [0..WG_X*WG_Y-1].
fn lane_id(lid: vec3<u32>) -> u32 {
  return lid.y * WG_X + lid.x; // for 16×16 -> 0..255
}

// Compute the number of tiles along X (ceil division). Must match CPU-side computation.
fn grid_width_for(tex_w: u32) -> u32 {
  return (tex_w + WG_X - 1u) / WG_X;
}

// Sum-reduce the 256 per-lane contributions in shared_* down to lane 0.
// Standard binary-tree reduction: 128, 64, 32, ..., 1 with a barrier between steps.
// (If you change WG_X/WG_Y, also change the initial stride to (WG_X*WG_Y)>>1.)
fn reduce256(tid: u32) {
  var stride = 128u;
  loop {
    if (stride == 0u) { break; }
    if (tid < stride) {
      let j = tid + stride;
      shared_count[tid] += shared_count[j];
      shared_sum_x[tid] += shared_sum_x[j];
      shared_sum_y[tid] += shared_sum_y[j];
      shared_sum_z[tid] += shared_sum_z[j];
    }
    workgroupBarrier();  // synchronize lanes before the next stride
    stride >>= 1u;
  }
}

// Write this tile's partial result for a specific object to global buffers.
// Layout: offset = tile_idx * num_objects + obj
// NOTE: As written, only writes when count>0 (see main). If you skip writing zeros here,
//       make sure buffers are cleared each frame or Stage 2 treats missing entries as zero.
fn write_partial(tile_idx: u32, obj: u32) {
  let cnt = shared_count[0];
  if (cnt > 0u) {
    let offset = tile_idx * uniforms.num_objects + obj; // (tile, object)
    workgroup_id[offset]    = cnt;
    workgroup_pos_x[offset] = shared_sum_x[0];
    workgroup_pos_y[offset] = shared_sum_y[0];
    workgroup_pos_z[offset] = shared_sum_z[0];
  }
}


// ====== KERNEL ======

@compute @workgroup_size(WG_X, WG_Y, 1)
fn main(
  @builtin(global_invocation_id) gid: vec3<u32>,  // absolute pixel coords this lane covers
  @builtin(local_invocation_id)  lid: vec3<u32>,  // lane coords within the tile
  @builtin(workgroup_id)         wid: vec3<u32>   // tile coords (x,y); z=0 in this stage
) {
  // --- Map this workgroup to a tile and this lane to a pixel in the image ---
  let tex_dims = textureDimensions(object_id_tex);
  let in_bounds = (gid.x < tex_dims.x && gid.y < tex_dims.y);  // guard for partially full tiles
  let tid = lane_id(lid);                                      // [0..255] for 16×16
  let grid_w = grid_width_for(tex_dims.x);
  let tile_idx = wid.y * grid_w + wid.x;                       // flattened tile index in raster order

  // --- Load this lane's pixel once and reuse it for all objects ---
  // Convention: object_id==0 means "background / no object".
  var obj_id: u32 = 0u;
  var pos: vec3<f32> = vec3<f32>(0.0, 0.0, 0.0);
  if (in_bounds) {
    obj_id = textureLoad(object_id_tex, vec2<i32>(gid.xy), 0).r;
    pos    = textureLoad(position_tex,  vec2<i32>(gid.xy), 0).xyz;
  }

  // --- Iterate over the object-ID space in batches of CHUNK (optional) ---
  // For each object in the batch:
  //   1) Each lane writes its local contribution (1/pos or 0) into shared arrays.
  //   2) All lanes reduce those arrays down to lane 0.
  //   3) Lane 0 writes one (tile, object) partial to global memory.
  var base: u32 = 0u;
  loop {
    if (base >= uniforms.num_objects) { break; }
    let end = min(base + CHUNK, uniforms.num_objects);

    var obj: u32 = base;
    loop {
      if (obj >= end) { break; }

      // --- Per-lane contribution for this specific object ---
      // Assumes object IDs in the texture are 1..num_objects (0 = background).
      let is_match = (obj_id == (obj + 1u));
      shared_count[tid] = select(0u,   1u,    is_match); // count contribution
      shared_sum_x[tid] = select(0.0,  pos.x, is_match); // x sum contribution
      shared_sum_y[tid] = select(0.0,  pos.y, is_match); // y sum contribution
      shared_sum_z[tid] = select(0.0,  pos.z, is_match); // z sum contribution
      workgroupBarrier();   // ensure all lanes have written before reducing

      // --- Reduce the 256-lane contributions to lane 0 ---
      reduce256(tid);

      // --- Lane 0 writes the (tile, obj) partial (only if count > 0) ---
      if (tid == 0u) {
        write_partial(tile_idx, obj);
      }
      workgroupBarrier();   // clean separation between objects inside the tile

      obj = obj + 1u;
    }

    base = base + CHUNK;
  }
}

