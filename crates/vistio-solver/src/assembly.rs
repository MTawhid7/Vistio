//! PD system matrix assembly for Projective Dynamics.
//!
//! Builds the constant system matrix `A = (M/h²) + Σ wᵢ Sᵢᵀ Sᵢ`
//! where:
//! - `M/h²` is the mass matrix scaled by 1/dt²
//! - `Sᵢ` is the selection-gradient matrix for triangle i
//! - `wᵢ` is the stiffness weight for triangle i
//!
//! The matrix is N×N (one per coordinate axis), solved three times
//! per global step (for X, Y, Z) using the same cached factorization.
//!
//! Also assembles the RHS vector for the PD global step.

use vistio_math::sparse::CsrMatrix;
use crate::element::ElementData;

/// Assemble the PD system matrix A = (M/h²) + Σ wᵢ Sᵢᵀ Sᵢ.
///
/// This matrix is constant as long as:
/// - Topology doesn't change (no remeshing)
/// - Masses don't change
/// - Timestep dt doesn't change
///
/// Returns an N×N CSR matrix.
///
/// # Arguments
/// * `n` — Number of vertices
/// * `mass` — Per-vertex mass (length N)
/// * `dt` — Timestep
/// * `elements` — Precomputed element data
pub fn assemble_system_matrix(
    n: usize,
    mass: &[f32],
    dt: f32,
    elements: &ElementData,
) -> CsrMatrix {
    let inv_dt2 = 1.0 / (dt * dt);

    // Accumulate via triplets (row, col, value)
    // Capacity estimate: N diagonal + ~6 entries per triangle × #triangles
    let mut triplets: Vec<(usize, usize, f32)> =
        Vec::with_capacity(n + elements.len() * 9);

    // Mass term: M/h² → diagonal entries
    for (i, &m) in mass.iter().enumerate().take(n) {
        triplets.push((i, i, m * inv_dt2));
    }

    // Stiffness term: Σ wᵢ Sᵢᵀ Sᵢ
    // For each triangle (i0, i1, i2) with Dm_inv = [a, b, c, d]:
    //
    // The selection-gradient matrix Sᵢ maps global positions to local
    // deformation gradient contributions. For a triangle element:
    //   Ds = [p1-p0, p2-p0]
    //   F = Ds · Dm_inv
    //
    // The PD energy contribution is:
    //   E_i = w_i/2 ||F - R||²
    //
    // The SᵢᵀSᵢ contribution to the system matrix scatters into
    // a 3×3 block indexed by (i0, i1, i2).
    //
    // The gradient operator G_i for vertex positions is:
    //   G_i = [-g1-g2, g1, g2]
    // where g1 = column 0 of Dm_inv, g2 = column 1 of Dm_inv.
    //
    // The stiffness contribution is: w_i * G_iᵀ G_i

    for elem in &elements.elements {
        let [i0, i1, i2] = elem.indices;
        let w = elem.weight;

        // Dm_inv columns (2D gradient operators)
        // Dm_inv stored column-major: [a, b, c, d]
        // Column 0: (a, b) = g1
        // Column 1: (c, d) = g2
        let g1x = elem.dm_inv[0];
        let g1y = elem.dm_inv[1];
        let g2x = elem.dm_inv[2];
        let g2y = elem.dm_inv[3];

        // Gradient operator rows for each vertex:
        //   vertex 0: -(g1 + g2) = [-(g1x+g2x), -(g1y+g2y)]
        //   vertex 1: g1 = [g1x, g1y]
        //   vertex 2: g2 = [g2x, g2y]
        let g = [
            [-(g1x + g2x), -(g1y + g2y)], // vertex 0
            [g1x, g1y],                     // vertex 1
            [g2x, g2y],                     // vertex 2
        ];
        let idx = [i0, i1, i2];

        // SᵀS contribution: for each pair (a, b) in {0,1,2}:
        //   A[idx[a], idx[b]] += w * dot(g[a], g[b])
        for a in 0..3 {
            for b in 0..3 {
                let dot = g[a][0] * g[b][0] + g[a][1] * g[b][1];
                if dot.abs() > 1e-12 {
                    triplets.push((idx[a], idx[b], w * dot));
                }
            }
        }
    }

    CsrMatrix::from_triplets(n, n, &triplets)
}

/// Assemble the RHS vector for the PD global step.
///
/// RHS = (M/h²) · q_pred + Σ wᵢ Sᵢᵀ pᵢ
///
/// where:
/// - `q_pred` is the predicted position (from inertia)
/// - `pᵢ` is the local projection target for element i
/// - `Sᵢᵀ` scatters the triangle-local projection back to global
///
/// # Arguments
/// * `n` — Number of vertices
/// * `mass` — Per-vertex mass
/// * `dt` — Timestep
/// * `pred` — Predicted positions (one coordinate axis)
/// * `proj_targets` — Per-element projection target for each vertex
///   `proj_targets[e] = (p0, p1, p2)` target positions
///   for element `e`, extracting the relevant coordinate
///   component.
/// * `elements` — Precomputed element data
/// * `coord` — Which coordinate axis (0=X, 1=Y, 2=Z)
pub fn assemble_rhs(
    n: usize,
    mass: &[f32],
    dt: f32,
    pred: &[f32],
    proj_targets: &[(f32, f32, f32)],
    elements: &ElementData,
    coord: usize,
) -> Vec<f32> {
    let inv_dt2 = 1.0 / (dt * dt);
    let mut rhs = vec![0.0_f32; n];

    // Mass/inertia term: (M/h²) · q_pred
    for i in 0..n {
        rhs[i] += mass[i] * inv_dt2 * pred[i];
    }

    // Stiffness term: Σ wᵢ Sᵢᵀ pᵢ
    // Same gradient operator as the system matrix.
    // Sᵢᵀpᵢ scatters the projection target back to global vertices.
    for (e, elem) in elements.elements.iter().enumerate() {
        let [i0, i1, i2] = elem.indices;
        let w = elem.weight;

        let g1x = elem.dm_inv[0];
        let g1y = elem.dm_inv[1];
        let g2x = elem.dm_inv[2];
        let g2y = elem.dm_inv[3];

        let g = [
            [-(g1x + g2x), -(g1y + g2y)],
            [g1x, g1y],
            [g2x, g2y],
        ];
        let idx = [i0, i1, i2];

        // Get target positions for this element (the relevant coordinate)
        let target = [proj_targets[e].0, proj_targets[e].1, proj_targets[e].2];

        // For the RHS, we need: Sᵢᵀpᵢ
        // The projection target pᵢ represents the target edge displacements.
        // We apply: Sᵢᵀ * [target edges expressed through gradient operator]
        //
        // Specifically, the target edge contributions:
        //   target_e1 = target[1] - target[0]  (edge 1)
        //   target_e2 = target[2] - target[0]  (edge 2)
        let te1 = target[1] - target[0];
        let te2 = target[2] - target[0];

        // RHS contribution from Sᵢᵀ:
        // For each vertex a: rhs[idx[a]] += w * (g[a] · [te1, te2])
        // But more precisely, applying the transpose of the gradient:
        let _ = coord; // coord already selected in the projected targets
        for a in 0..3 {
            let contrib = g[a][0] * te1 + g[a][1] * te2;
            rhs[idx[a]] += w * contrib;
        }
    }

    rhs
}
