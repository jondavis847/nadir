use super::{GeometryState, GeometryTrait, GeometryTransform};
use crate::vertex::Vertex;
use glam::{Mat3, Mat4, Quat, vec2, vec3};
use serde::{Deserialize, Serialize};

use thiserror::Error;

#[derive(Debug, Error)]
pub enum EllipsoidErrors {
    #[error("ellipsoid dimensions must be greater than 0")]
    Dimension,
}

// I wanted to make this a generic Ellipsoid<LAT,LON> but struggled to find a way to make it dynamic and generic for the gpu
// maybe 1 day

#[derive(Copy, Clone, Debug, Deserialize, Serialize)]
struct Ellipsoid {
    pub radius_x: f64, // Radius along the x-axis
    pub radius_y: f64, // Radius along the y-axis
    pub radius_z: f64, // Radius along the z-axis
}

impl Ellipsoid {
    pub fn new(radius_x: f64, radius_y: f64, radius_z: f64) -> Result<Self, EllipsoidErrors> {
        if radius_x <= 0.0 || radius_y <= 0.0 || radius_z <= 0.0 {
            return Err(EllipsoidErrors::Dimension);
        }
        Ok(Self { radius_x, radius_y, radius_z })
    }
    fn get_transform(&self, state: &GeometryState) -> GeometryTransform {
        let transformation = Mat4::from_scale_rotation_translation(
            vec3(
                self.radius_x as f32,
                self.radius_y as f32,
                self.radius_z as f32,
            ),
            Quat::from_xyzw(
                state
                    .rotation
                    .x as f32,
                state
                    .rotation
                    .y as f32,
                state
                    .rotation
                    .z as f32,
                state
                    .rotation
                    .w as f32,
            ),
            vec3(
                state
                    .position
                    .x as f32,
                state
                    .position
                    .y as f32,
                state
                    .position
                    .z as f32,
            ),
        );
        let normal = transformation
            .inverse()
            .transpose();
        let normal = Mat3::from_mat4(normal);
        GeometryTransform::new(transformation, normal)
    }
}

#[derive(Copy, Clone, Debug, Deserialize, Serialize)]
pub struct Ellipsoid16(Ellipsoid);
impl Ellipsoid16 {
    pub fn new(radius_x: f64, radius_y: f64, radius_z: f64) -> Result<Self, EllipsoidErrors> {
        Ok(Self(Ellipsoid::new(
            radius_x, radius_y, radius_z,
        )?))
    }
    pub fn vertices() -> Vec<Vertex> {
        ellipsoid_vertices(16)
    }
}

impl GeometryTrait for Ellipsoid16 {
    fn get_transform(&self, state: &GeometryState) -> GeometryTransform {
        self.0
            .get_transform(state)
    }

    fn get_vertices(&self) -> Vec<Vertex> {
        Self::vertices()
    }
}

#[derive(Copy, Clone, Debug, Deserialize, Serialize)]
pub struct Ellipsoid32(Ellipsoid);
impl Ellipsoid32 {
    pub fn new(radius_x: f64, radius_y: f64, radius_z: f64) -> Result<Self, EllipsoidErrors> {
        Ok(Self(Ellipsoid::new(
            radius_x, radius_y, radius_z,
        )?))
    }

    pub fn vertices() -> Vec<Vertex> {
        ellipsoid_vertices(32)
    }
}

impl GeometryTrait for Ellipsoid32 {
    fn get_transform(&self, state: &GeometryState) -> GeometryTransform {
        self.0
            .get_transform(state)
    }

    fn get_vertices(&self) -> Vec<Vertex> {
        Self::vertices()
    }
}

#[derive(Copy, Clone, Debug, Deserialize, Serialize)]
pub struct Ellipsoid64(Ellipsoid);
impl Ellipsoid64 {
    pub fn new(radius_x: f64, radius_y: f64, radius_z: f64) -> Result<Self, EllipsoidErrors> {
        Ok(Self(Ellipsoid::new(
            radius_x, radius_y, radius_z,
        )?))
    }
    pub fn vertices() -> Vec<Vertex> {
        ellipsoid_vertices(64)
    }
}

impl GeometryTrait for Ellipsoid64 {
    fn get_transform(&self, state: &GeometryState) -> GeometryTransform {
        self.0
            .get_transform(state)
    }

    fn get_vertices(&self) -> Vec<Vertex> {
        Self::vertices()
    }
}

fn ellipsoid_vertices(n_lat: u32) -> Vec<Vertex> {
    let n_lon = 2 * n_lat; // twice as long as lat since 2 hemispheres

    let mut grid_points = Vec::with_capacity(n_lat as usize + 1);
    let mut vertices = Vec::new();

    let up = vec3(0.0, 0.0, 1.0);

    // using wikipedia spherical coordinate frame.
    // z up.
    // theta is angle from z, 0 to pi. 0 at top (latitude)
    // phi is angle from phi, 0 to 2pi (longitude)

    // Generate grid points
    for lat in 0..=n_lat {
        let mut row = Vec::with_capacity(n_lon as usize + 1);

        let theta = lat as f32 * std::f32::consts::PI / n_lat as f32;
        let sin_theta = theta.sin();
        let cos_theta = theta.cos();

        for lon in 0..=n_lon {
            let phi = lon as f32 * 2.0 * std::f32::consts::PI / n_lon as f32;
            let sin_phi = phi.sin();
            let cos_phi = phi.cos();

            let x = cos_phi * sin_theta;
            let y = sin_phi * sin_theta;
            let z = cos_theta;

            let normal = vec3(x, y, z).normalize();
            let tangent = up
                .cross(normal)
                .normalize();
            let uv = vec2(
                lon as f32 / n_lon as f32,
                lat as f32 / n_lat as f32,
            );

            row.push(Vertex { pos: vec3(x, y, z), normal, tangent, uv });
        }

        grid_points.push(row);
    }
    let north_pole = grid_points[0][0];
    let south_pole = grid_points[n_lat as usize][0];
    // Create triangles from grid points
    for lat in 0..n_lat as usize {
        for lon in 0..n_lon as usize {
            let next_lon = lon + 1;
            let next_lat = lat + 1;

            // Vertices for the current quad
            let top_left = grid_points[lat][lon];
            let bottom_left = grid_points[next_lat][lon];
            let bottom_right = grid_points[next_lat][next_lon];
            let top_right = grid_points[lat][next_lon];

            // Handle poles
            if lat == 0 {
                // North pole triangle
                vertices.push(north_pole);
                vertices.push(bottom_left);
                vertices.push(bottom_right);
            } else if lat == n_lat as usize - 1 {
                // South pole triangle
                vertices.push(top_left);
                vertices.push(south_pole);
                vertices.push(top_right);
            } else {
                // Two triangles for non-pole regions

                //Triangle 1
                vertices.push(top_left);
                vertices.push(bottom_left);
                vertices.push(bottom_right);

                //Triangle 2
                vertices.push(top_left);
                vertices.push(bottom_right);
                vertices.push(top_right);
            }
        }
    }

    vertices
}
