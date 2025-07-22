use glam::Vec3;
use nadir_3d::vertex::SimpleVertex;

#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct BVHNode {
    pub min_bounds: [f32; 3],
    pub left_first: u32, // For internal nodes: first child index
    // For leaf nodes: first triangle index
    pub max_bounds: [f32; 3],
    pub triangle_count: u32, // 0 for internal nodes, triangle count for leaf nodes
}

#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GeometryBVHInfo {
    pub root_node_index: u32,
    pub node_count: u32,
}

pub struct BVH {
    pub nodes: Vec<BVHNode>,
    pub max_leaf_size: u32,
}

impl BVH {
    pub fn new(vertices: &[SimpleVertex], indices: &[u32], max_leaf_size: u32) -> Self {
        // Create a reference list of triangles
        let mut triangles: Vec<TriangleRef> = Vec::with_capacity(indices.len() / 3);

        for i in 0..(indices.len() / 3) {
            let idx = i * 3;
            let v0 = &vertices[indices[idx] as usize];
            let v1 = &vertices[indices[idx + 1] as usize];
            let v2 = &vertices[indices[idx + 2] as usize];

            let min_bounds = Vec3::min(
                Vec3::min(
                    Vec3::from(v0.pos),
                    Vec3::from(v1.pos),
                ),
                Vec3::from(v2.pos),
            );

            let max_bounds = Vec3::max(
                Vec3::max(
                    Vec3::from(v0.pos),
                    Vec3::from(v1.pos),
                ),
                Vec3::from(v2.pos),
            );

            let centroid = (min_bounds + max_bounds) * 0.5;

            triangles.push(TriangleRef { index: i as u32, min_bounds, max_bounds, centroid });
        }

        // Allocate space for BVH nodes (worst case: 2N-1 nodes for N triangles)
        let max_nodes = 2 * triangles.len() - 1;
        let mut nodes = Vec::with_capacity(max_nodes);

        // Create root node
        nodes.push(BVHNode {
            min_bounds: [0.0, 0.0, 0.0],
            max_bounds: [0.0, 0.0, 0.0],
            left_first: 0,
            triangle_count: 0,
        });

        // Recursively build the BVH
        let mut builder = BVHBuilder {
            triangles,
            nodes,
            next_node_index: 1, // Start at 1, as 0 is the root
            max_leaf_size,
        };

        builder.build_node(
            0,
            0,
            builder
                .triangles
                .len(),
        );

        Self { nodes: builder.nodes, max_leaf_size }
    }
}

struct TriangleRef {
    index: u32,       // Index of the triangle in the original mesh
    min_bounds: Vec3, // Minimum bounds of the triangle
    max_bounds: Vec3, // Maximum bounds of the triangle
    centroid: Vec3,   // Centroid of the triangle
}

struct BVHBuilder {
    triangles: Vec<TriangleRef>,
    nodes: Vec<BVHNode>,
    next_node_index: usize,
    max_leaf_size: u32,
}

impl BVHBuilder {
    fn build_node(&mut self, node_index: usize, start: usize, end: usize) {
        let triangle_count = end - start;

        // Calculate bounds for this node
        let mut node_min = Vec3::new(f32::MAX, f32::MAX, f32::MAX);
        let mut node_max = Vec3::new(f32::MIN, f32::MIN, f32::MIN);

        for i in start..end {
            node_min = Vec3::min(
                node_min,
                self.triangles[i].min_bounds,
            );
            node_max = Vec3::max(
                node_max,
                self.triangles[i].max_bounds,
            );
        }

        // Update node bounds
        self.nodes[node_index].min_bounds = [node_min.x, node_min.y, node_min.z];
        self.nodes[node_index].max_bounds = [node_max.x, node_max.y, node_max.z];

        // If few enough triangles, create a leaf node
        if triangle_count <= self.max_leaf_size as usize {
            self.nodes[node_index].left_first = start as u32;
            self.nodes[node_index].triangle_count = triangle_count as u32;
            return;
        }

        // Otherwise, split along longest axis
        let extent = node_max - node_min;
        let axis = if extent.x >= extent.y && extent.x >= extent.z {
            0 // X axis
        } else if extent.y >= extent.z {
            1 // Y axis
        } else {
            2 // Z axis
        };

        // Sort triangles by centroid along chosen axis
        let mid = start + triangle_count / 2;

        // Partition triangles - this is a simplified approach using nth_element
        // In a real implementation, you might use a more optimized approach
        let triangles = &mut self.triangles[start..end];
        match axis {
            0 => triangles.select_nth_unstable_by(triangle_count / 2, |a, b| {
                a.centroid
                    .x
                    .partial_cmp(
                        &b.centroid
                            .x,
                    )
                    .unwrap()
            }),
            1 => triangles.select_nth_unstable_by(triangle_count / 2, |a, b| {
                a.centroid
                    .y
                    .partial_cmp(
                        &b.centroid
                            .y,
                    )
                    .unwrap()
            }),
            _ => triangles.select_nth_unstable_by(triangle_count / 2, |a, b| {
                a.centroid
                    .z
                    .partial_cmp(
                        &b.centroid
                            .z,
                    )
                    .unwrap()
            }),
        };

        // Create child nodes
        let left_child = self.next_node_index;
        self.next_node_index += 1;
        let right_child = self.next_node_index;
        self.next_node_index += 1;

        // Add two empty nodes to the array
        self.nodes
            .push(BVHNode {
                min_bounds: [0.0, 0.0, 0.0],
                max_bounds: [0.0, 0.0, 0.0],
                left_first: 0,
                triangle_count: 0,
            });

        self.nodes
            .push(BVHNode {
                min_bounds: [0.0, 0.0, 0.0],
                max_bounds: [0.0, 0.0, 0.0],
                left_first: 0,
                triangle_count: 0,
            });

        // Update internal node
        self.nodes[node_index].left_first = left_child as u32;
        self.nodes[node_index].triangle_count = 0; // Internal node

        // Recursively build children
        self.build_node(left_child, start, mid);
        self.build_node(right_child, mid, end);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{DQuat, DVec3};
    use nadir_3d::{
        geometry::{Geometry, GeometryState, GeometryTrait, cuboid::Cuboid},
        vertex::simple_vertices,
    };
    use std::f32;

    #[test]
    fn test_bvh_creation_for_cube() {
        // Create a simple cube
        let cube = Cuboid::new(2.0, 2.0, 2.0).unwrap();
        let geometry: Geometry = cube.into();

        // Get vertices and indices
        let vertices = simple_vertices(&geometry.get_vertices());
        let indices: Vec<u32> = (0..vertices.len() as u32).collect();

        // Create BVH
        let bvh = BVH::new(&vertices, &indices, 4); // 4 triangles per leaf

        // A cube should have 12 triangles and should create multiple nodes
        assert!(
            bvh.nodes
                .len()
                > 1,
            "BVH should have multiple nodes for a cube"
        );

        // Check bounds of the root node (should encompass the entire cube)
        let root = &bvh.nodes[0];

        // For a 2x2x2 cube centered at origin, bounds should be approximately [-1,-1,-1] to [1,1,1]
        // with some floating point tolerance
        const EPSILON: f32 = 0.001;

        // Check min bounds
        assert!(
            (root.min_bounds[0] + 1.0).abs() < EPSILON,
            "Min X bound incorrect: {}",
            root.min_bounds[0]
        );
        assert!(
            (root.min_bounds[1] + 1.0).abs() < EPSILON,
            "Min Y bound incorrect: {}",
            root.min_bounds[1]
        );
        assert!(
            (root.min_bounds[2] + 1.0).abs() < EPSILON,
            "Min Z bound incorrect: {}",
            root.min_bounds[2]
        );

        // Check max bounds
        assert!(
            (root.max_bounds[0] - 1.0).abs() < EPSILON,
            "Max X bound incorrect: {}",
            root.max_bounds[0]
        );
        assert!(
            (root.max_bounds[1] - 1.0).abs() < EPSILON,
            "Max Y bound incorrect: {}",
            root.max_bounds[1]
        );
        assert!(
            (root.max_bounds[2] - 1.0).abs() < EPSILON,
            "Max Z bound incorrect: {}",
            root.max_bounds[2]
        );

        // Verify that internal nodes have triangle_count = 0
        let internal_nodes: Vec<&BVHNode> = bvh
            .nodes
            .iter()
            .filter(|node| node.triangle_count == 0)
            .collect();

        assert!(
            !internal_nodes.is_empty(),
            "BVH should have internal nodes"
        );

        // Verify that leaf nodes have triangle_count > 0 and <= max_leaf_size
        let leaf_nodes: Vec<&BVHNode> = bvh
            .nodes
            .iter()
            .filter(|node| node.triangle_count > 0)
            .collect();

        assert!(
            !leaf_nodes.is_empty(),
            "BVH should have leaf nodes"
        );

        for leaf in &leaf_nodes {
            assert!(
                leaf.triangle_count <= bvh.max_leaf_size,
                "Leaf node should have at most {} triangles, but has {}",
                bvh.max_leaf_size,
                leaf.triangle_count
            );
        }

        // Verify that the sum of triangles in all leaf nodes equals the total triangle count
        let total_triangles_in_leaves: u32 = leaf_nodes
            .iter()
            .map(|node| node.triangle_count)
            .sum();

        assert_eq!(
            total_triangles_in_leaves,
            (indices.len() / 3) as u32,
            "Sum of triangles in leaves should equal total triangle count"
        );
    }

    #[test]
    fn test_bvh_structure_validity() {
        // Create a cube
        let cube = Cuboid::new(2.0, 2.0, 2.0).unwrap();
        let geometry: Geometry = cube.into();

        let vertices = simple_vertices(&geometry.get_vertices());
        let indices: Vec<u32> = (0..vertices.len() as u32).collect();

        // Create BVH with small leaf size to force deeper tree
        let bvh = BVH::new(&vertices, &indices, 2); // 2 triangles per leaf

        // Verify BVH structure consistency
        let node_count = bvh
            .nodes
            .len();

        for i in 0..node_count {
            let node = &bvh.nodes[i];

            // Internal nodes
            if node.triangle_count == 0 {
                // Check that child indices are valid
                assert!(
                    node.left_first < node_count as u32,
                    "Node {} has invalid left child index: {}",
                    i,
                    node.left_first
                );
                assert!(
                    node.left_first + 1 < node_count as u32,
                    "Node {} has invalid right child index: {}",
                    i,
                    node.left_first + 1
                );

                // Check that child nodes are different from parent
                assert_ne!(
                    node.left_first as usize, i,
                    "Node {} has itself as a child",
                    i
                );
                assert_ne!(
                    (node.left_first + 1) as usize,
                    i,
                    "Node {} has itself as a child",
                    i
                );

                // Check that bounds are valid (min <= max)
                for axis in 0..3 {
                    assert!(
                        node.min_bounds[axis] <= node.max_bounds[axis],
                        "Node {} has invalid bounds: min[{}]={} > max[{}]={}",
                        i,
                        axis,
                        node.min_bounds[axis],
                        axis,
                        node.max_bounds[axis]
                    );
                }
            }
            // Leaf nodes
            else {
                // Check that triangle indices are valid
                assert!(
                    node.left_first + node.triangle_count <= (indices.len() / 3) as u32,
                    "Node {} references invalid triangles",
                    i
                );
            }
        }
    }

    #[test]
    fn test_bvh_with_transformed_geometry() {
        // Create a cube and transform it
        let cube = Cuboid::new(1.0, 1.0, 1.0).unwrap();
        let state = GeometryState {
            position: DVec3::new(2.0, 3.0, 4.0), // Translate
            rotation: DQuat::from_rotation_x(std::f64::consts::FRAC_PI_4), // 45 degree X rotation
        };

        let geometry: Geometry = cube.into();
        let transform = geometry
            .get_transform(&state)
            .transformation_matrix;

        // Get vertices and create indices
        let original_vertices = simple_vertices(&geometry.get_vertices());
        let indices: Vec<u32> = (0..original_vertices.len() as u32).collect();

        // Transform vertices (simulating what would happen when rendering)
        let transformed_vertices: Vec<SimpleVertex> = original_vertices
            .iter()
            .map(|v| {
                let transformed = transform
                    * glam::Vec4::new(
                        v.pos[0], v.pos[1], v.pos[2], 1.0,
                    );
                SimpleVertex {
                    pos: [transformed.x, transformed.y, transformed.z].into(),
                    _padding: 0.0,
                }
            })
            .collect();

        // Create BVHs for both original and transformed vertices
        let original_bvh = BVH::new(
            &original_vertices,
            &indices,
            4,
        );
        let transformed_bvh = BVH::new(
            &transformed_vertices,
            &indices,
            4,
        );

        // Get root nodes
        let original_root = &original_bvh.nodes[0];
        let transformed_root = &transformed_bvh.nodes[0];

        // Debug output to see actual bounds
        println!(
            "Original bounds: [{:.3}, {:.3}, {:.3}] to [{:.3}, {:.3}, {:.3}]",
            original_root.min_bounds[0],
            original_root.min_bounds[1],
            original_root.min_bounds[2],
            original_root.max_bounds[0],
            original_root.max_bounds[1],
            original_root.max_bounds[2]
        );

        println!(
            "Transformed bounds: [{:.3}, {:.3}, {:.3}] to [{:.3}, {:.3}, {:.3}]",
            transformed_root.min_bounds[0],
            transformed_root.min_bounds[1],
            transformed_root.min_bounds[2],
            transformed_root.max_bounds[0],
            transformed_root.max_bounds[1],
            transformed_root.max_bounds[2]
        );

        // Test 1: Original should be centered around origin
        assert!(
            original_root.min_bounds[0] < 0.0 && original_root.max_bounds[0] > 0.0,
            "Original X bounds should straddle zero"
        );
        assert!(
            original_root.min_bounds[1] < 0.0 && original_root.max_bounds[1] > 0.0,
            "Original Y bounds should straddle zero"
        );
        assert!(
            original_root.min_bounds[2] < 0.0 && original_root.max_bounds[2] > 0.0,
            "Original Z bounds should straddle zero"
        );

        // Test 2: Transformed bounds should be valid (min <= max)
        for axis in 0..3 {
            assert!(
                transformed_root.min_bounds[axis] <= transformed_root.max_bounds[axis],
                "Transformed bounds invalid on axis {}: min={:.3} > max={:.3}",
                axis,
                transformed_root.min_bounds[axis],
                transformed_root.max_bounds[axis]
            );
        }

        // Test 3: Transformed bounds should be different from original (it was moved)
        let original_center = [
            (original_root.min_bounds[0] + original_root.max_bounds[0]) * 0.5,
            (original_root.min_bounds[1] + original_root.max_bounds[1]) * 0.5,
            (original_root.min_bounds[2] + original_root.max_bounds[2]) * 0.5,
        ];

        let transformed_center = [
            (transformed_root.min_bounds[0] + transformed_root.max_bounds[0]) * 0.5,
            (transformed_root.min_bounds[1] + transformed_root.max_bounds[1]) * 0.5,
            (transformed_root.min_bounds[2] + transformed_root.max_bounds[2]) * 0.5,
        ];

        println!(
            "Original center: [{:.3}, {:.3}, {:.3}]",
            original_center[0], original_center[1], original_center[2]
        );
        println!(
            "Transformed center: [{:.3}, {:.3}, {:.3}]",
            transformed_center[0], transformed_center[1], transformed_center[2]
        );

        // Test 4: The transformed center should be roughly at the expected position (2, 3, 4)
        // But allow for rotation effects with larger epsilon
        const EPSILON: f32 = 1.0; // Much larger epsilon to account for rotation

        assert!(
            (transformed_center[0] - 2.0).abs() < EPSILON,
            "Transformed X center should be near 2.0, got {:.3}",
            transformed_center[0]
        );
        assert!(
            (transformed_center[1] - 3.0).abs() < EPSILON,
            "Transformed Y center should be near 3.0, got {:.3}",
            transformed_center[1]
        );
        assert!(
            (transformed_center[2] - 4.0).abs() < EPSILON,
            "Transformed Z center should be near 4.0, got {:.3}",
            transformed_center[2]
        );

        // Test 5: The transformed bounds should be larger than a point (have volume)
        let transformed_size = [
            transformed_root.max_bounds[0] - transformed_root.min_bounds[0],
            transformed_root.max_bounds[1] - transformed_root.min_bounds[1],
            transformed_root.max_bounds[2] - transformed_root.min_bounds[2],
        ];

        for axis in 0..3 {
            assert!(
                transformed_size[axis] > 0.0,
                "Transformed bounds should have positive size on axis {}, got {:.3}",
                axis,
                transformed_size[axis]
            );
        }
    }
}
