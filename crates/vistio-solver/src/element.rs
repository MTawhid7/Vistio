//! FEM element computation for Projective Dynamics.
//!
//! Precomputes per-triangle rest-state data and provides the SVK (St. Venant-Kirchhoff)
//! local projection used in the PD local step. Each triangle element stores its
//! rest-state edge matrix inverse, area, and stiffness weight.
//!
//! ## PD Local Step
//!
//! For each triangle, the local step finds the closest valid configuration:
//! 1. Compute deformation gradient F = Ds · Dm⁻¹
//! 2. Polar decomposition F = R · S
//! 3. Project: target = R · rest_edges (rotation-aware, resists stretch)

use vistio_math::Vec2;
use vistio_math::Vec3;
use vistio_math::decomposition::{deformation_gradient, polar_decomposition_3x2};
use vistio_mesh::TriangleMesh;

/// Precomputed rest-state data for a single triangle element.
#[derive(Debug, Clone, Copy)]
pub struct RestTriangle {
    /// Triangle vertex indices (into the global vertex buffer).
    pub indices: [usize; 3],
    /// Inverse of the rest-state edge matrix Dm (2×2, column-major).
    /// Dm = [e1, e2] where e1 = p1-p0, e2 = p2-p0 (projected to 2D).
    pub dm_inv: [f32; 4],
    /// Rest-state area (half the cross product magnitude of the edge vectors).
    pub rest_area: f32,
    /// Stiffness weight for this element in the PD system matrix.
    /// weight = stiffness * area (energy contribution per element).
    pub weight: f32,
}

/// Collection of all triangle elements with precomputed rest-state data.
pub struct ElementData {
    /// Per-triangle element data.
    pub elements: Vec<RestTriangle>,
    /// Global stiffness parameter (Young's modulus proxy).
    pub stiffness: f32,
}

impl ElementData {
    /// Compute rest-state data for all triangles in a mesh.
    ///
    /// # Arguments
    /// * `mesh` — Triangle mesh (vertex positions + triangle indices)
    /// * `stiffness` — Global stiffness parameter (higher = stiffer fabric)
    pub fn from_mesh(mesh: &TriangleMesh, stiffness: f32) -> Self {
        let tri_count = mesh.triangle_count();
        let mut elements = Vec::with_capacity(tri_count);

        for t in 0..tri_count {
            let idx_base = t * 3;
            let i0 = mesh.indices[idx_base] as usize;
            let i1 = mesh.indices[idx_base + 1] as usize;
            let i2 = mesh.indices[idx_base + 2] as usize;

            let p0 = Vec3::new(mesh.pos_x[i0], mesh.pos_y[i0], mesh.pos_z[i0]);
            let p1 = Vec3::new(mesh.pos_x[i1], mesh.pos_y[i1], mesh.pos_z[i1]);
            let p2 = Vec3::new(mesh.pos_x[i2], mesh.pos_y[i2], mesh.pos_z[i2]);

            let e1 = p1 - p0;
            let e2 = p2 - p0;

            // Rest-state area = 0.5 * ||e1 × e2||
            let cross = e1.cross(e2);
            let area = 0.5 * cross.length();

            // Compute Dm (rest-state edge matrix) in 2D
            // Project edges into the triangle's local frame to get a 2×2 matrix.
            // Use the triangle's own plane as the reference.
            let dm_inv = compute_dm_inv(e1, e2);

            let weight = stiffness * area;

            elements.push(RestTriangle {
                indices: [i0, i1, i2],
                dm_inv,
                rest_area: area,
                weight,
            });
        }

        Self {
            elements,
            stiffness,
        }
    }

    /// Returns the number of elements.
    pub fn len(&self) -> usize {
        self.elements.len()
    }

    /// Returns true if there are no elements.
    pub fn is_empty(&self) -> bool {
        self.elements.is_empty()
    }

    /// Compute the local projection for one element.
    ///
    /// Given current vertex positions, computes the deformation gradient F,
    /// performs polar decomposition F = R·S, and returns the projected
    /// target positions for the three vertices.
    ///
    /// The projection finds the closest rotation to the current deformation,
    /// which is the core of the ARAP/co-rotational energy projection.
    ///
    /// Returns `(target_p0, target_p1, target_p2)` — the positions the element
    /// "wants" the vertices to be at.
    pub fn project(
        &self,
        elem: &RestTriangle,
        pos_x: &[f32],
        pos_y: &[f32],
        pos_z: &[f32],
    ) -> (Vec3, Vec3, Vec3) {
        let [i0, i1, i2] = elem.indices;

        let p0 = Vec3::new(pos_x[i0], pos_y[i0], pos_z[i0]);
        let p1 = Vec3::new(pos_x[i1], pos_y[i1], pos_z[i1]);
        let p2 = Vec3::new(pos_x[i2], pos_y[i2], pos_z[i2]);

        // 1. Compute deformation gradient: F = Ds · Dm⁻¹
        let f = deformation_gradient(p0, p1, p2, elem.dm_inv);

        // 2. Polar decomposition: F = R · S
        let pd = polar_decomposition_3x2(&f);

        // 3. Project: target edges = R · rest_edges
        // Rest edges in 2D local frame:
        //   dm = Dm_inv⁻¹ (we need the forward Dm, but we stored Dm_inv)
        // Instead, we use: target = R · Dm  (Dm = Ds_rest)
        // But we don't store Dm directly. We can reconstruct:
        //   Dm = [e1_rest, e2_rest] as 3×2 in the rest frame
        //
        // Simpler approach: The projected positions are:
        //   p0_proj = centroid - R * (avg offset in rest frame)
        //   p1_proj = p0_proj + R * e1_rest_2d
        //   p2_proj = p0_proj + R * e2_rest_2d
        //
        // However, for PD we project each vertex toward where the rotation R
        // would place it relative to the element's centroid.
        //
        // The standard ARAP PD projection:
        //   For each triangle with rotation R_t, the projected positions are:
        //   p_i_proj = centroid + R_t * (rest_pos_i - rest_centroid)
        //
        // We compute this using the centroid of current positions.

        // Current centroid
        let centroid = (p0 + p1 + p2) / 3.0;

        // We need rest-state relative positions. Reconstruct from Dm_inv:
        // Dm = Dm_inv^{-1}, where Dm is the 2x2 rest edge matrix
        // Dm_inv = [a, b, c, d] (column-major)
        let det = elem.dm_inv[0] * elem.dm_inv[3] - elem.dm_inv[1] * elem.dm_inv[2];
        let (rest_e1_2d, rest_e2_2d) = if det.abs() > 1e-12 {
            let inv_det = 1.0 / det;
            // Dm = inverse of Dm_inv
            let dm00 = elem.dm_inv[3] * inv_det;
            let dm01 = -elem.dm_inv[2] * inv_det;
            let dm10 = -elem.dm_inv[1] * inv_det;
            let dm11 = elem.dm_inv[0] * inv_det;
            (Vec2::new(dm00, dm10), Vec2::new(dm01, dm11))
        } else {
            // Degenerate — use identity as fallback
            return (p0, p1, p2);
        };

        // Rest-state relative positions (from vertex 0)
        // In 3D, we apply R (3×2) to the 2D rest offsets
        let rot = &pd.rotation;

        // Rotated rest edges: R * e_rest_2d
        let re1 = rot.col0 * rest_e1_2d.x + rot.col1 * rest_e1_2d.y;
        let re2 = rot.col0 * rest_e2_2d.x + rot.col1 * rest_e2_2d.y;

        // Rest centroid offset from p0
        let rest_c = (re1 + re2) / 3.0;

        // Projected positions: anchor at current centroid
        let p0_proj = centroid - rest_c;
        let p1_proj = p0_proj + re1;
        let p2_proj = p0_proj + re2;

        (p0_proj, p1_proj, p2_proj)
    }
}

/// Compute the 2×2 inverse of the rest-state edge matrix.
///
/// Projects 3D edge vectors into the triangle's local 2D frame to build
/// a 2×2 matrix Dm, then inverts it.
///
/// Local frame construction:
/// - u-axis: normalized e1
/// - v-axis: component of e2 perpendicular to e1 (in the triangle plane)
fn compute_dm_inv(e1: Vec3, e2: Vec3) -> [f32; 4] {
    let len_e1 = e1.length();
    if len_e1 < 1e-10 {
        return [1.0, 0.0, 0.0, 1.0]; // Degenerate fallback
    }

    let u = e1 / len_e1;
    let n = e1.cross(e2);
    let n_len = n.length();
    if n_len < 1e-10 {
        return [1.0, 0.0, 0.0, 1.0]; // Degenerate fallback
    }

    let v = n.cross(u).normalize();

    // Project edges into local 2D frame:
    // Dm = [[e1·u, e2·u], [e1·v, e2·v]]
    let dm00 = e1.dot(u); // = len_e1
    let dm10 = e1.dot(v); // = 0 by construction
    let dm01 = e2.dot(u);
    let dm11 = e2.dot(v);

    // Invert the 2×2 matrix (column-major storage)
    let det = dm00 * dm11 - dm01 * dm10;
    if det.abs() < 1e-10 {
        return [1.0, 0.0, 0.0, 1.0]; // Degenerate fallback
    }

    let inv_det = 1.0 / det;
    [
        dm11 * inv_det,   // [0,0]
        -dm10 * inv_det,  // [1,0]
        -dm01 * inv_det,  // [0,1]
        dm00 * inv_det,   // [1,1]
    ]
}
