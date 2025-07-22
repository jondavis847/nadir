#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GridParameters {
    grid_x_size: f32,
    grid_y_size: f32,
    grid_z_distance: f32,
    grid_nrays_x: u32,
    grid_nrays_y: u32,
}

impl Default for GridParameters {
    fn default() -> Self {
        Self {
            grid_x_size: 1.0,
            grid_y_size: 1.0,
            grid_z_distance: 1.0,
            grid_nrays_x: 100,
            grid_nrays_y: 100,
        }
    }
}
