use nadir_3d::geometry::Geometry;

pub struct GpuSurfaceArea;

impl GpuSurfaceArea {
    pub fn new() -> Self {}

    pub fn init(&mut self) {}

    pub fn calculate_surface_area(&self, geometry: &Geometry, direction: [f64; 3]) -> f64 {
        0.0
    }
}
