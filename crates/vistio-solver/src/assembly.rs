//! PD system matrix assembly for Projective Dynamics.
//!
//! Builds the constant system matrix `A = (M/h²) + Σ wᵢ Sᵢᵀ Sᵢ + Σ wᵇⱼ Lⱼᵀ Lⱼ`
//! where:
//! - `M/h²` is the mass matrix scaled by 1/dt²
//! - `Sᵢ` is the selection-gradient matrix for triangle i (membrane)
//! - `wᵢ` is the stiffness weight for triangle i
//! - `Lⱼ` is the discrete Laplacian operator for bending edge j
//! - `wᵇⱼ` is the bending stiffness weight for edge j
//!
//! The matrix is N×N (one per coordinate axis), solved three times
//! per global step (for X, Y, Z) using the same cached factorization.
//!
//! Also assembles the RHS vector for the PD global step.

use vistio_math::sparse::CsrMatrix;
use crate::bending::BendingData;
use crate::element::ElementData;

/// Assemble the PD system matrix A = (M/h²) + Σ wᵢ Sᵢᵀ Sᵢ + Σ wᵇⱼ Lⱼᵀ Lⱼ.
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
/// * `elements` — Precomputed element data (membrane stiffness)
/// * `bending` — Optional bending data (bending stiffness)
pub fn assemble_system_matrix(
    n: usize,
    mass: &[f32],
    dt: f32,
    elements: &ElementData,
    bending: Option<&BendingData>,
) -> CsrMatrix {
    let inv_dt2 = 1.0 / (dt * dt);

    // Accumulate via triplets (row, col, value)
    // Capacity estimate: N diagonal + ~6 entries per triangle × #triangles + 16 per bending edge
    let bending_count = bending.map_or(0, |b| b.len());
    let mut triplets: Vec<(usize, usize, f32)> =
        Vec::with_capacity(n + elements.len() * 9 + bending_count * 16);

    // Mass term: M/h² → diagonal entries
    for (i, &m) in mass.iter().enumerate().take(n) {
        triplets.push((i, i, m * inv_dt2));
    }

    // Membrane stiffness term: Σ wᵢ Sᵢᵀ Sᵢ
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
        let a = elem.dm_inv[0]; // dm00
        let b = elem.dm_inv[1]; // dm10
        let c = elem.dm_inv[2]; // dm01
        let d = elem.dm_inv[3]; // dm11

        let g = [
            [-(a + b), -(c + d)],
            [a, c],
            [b, d],
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

    // Bending stiffness term: Σ wᵇⱼ Lⱼᵀ Lⱼ
    //
    // For each bending element (v0, v1, wa, wb), we use a uniform
    // discrete Laplacian stencil: L = [-1, -1, 1, 1] for the 4 vertices.
    // This represents the "difference of differences" across the hinge.
    //
    // LᵀL produces a 4×4 matrix:
    //   [ 1  1 -1 -1]   (self-products on diagonal, cross-products off-diagonal)
    //   [ 1  1 -1 -1]
    //   [-1 -1  1  1]
    //   [-1 -1  1  1]
    //
    // Each entry is scaled by wᵇⱼ = bending_weight.
    if let Some(bend) = bending {
        for elem in &bend.elements {
            let idx = [elem.v0, elem.v1, elem.wing_a, elem.wing_b];
            let w = elem.weight;

            // Laplacian stencil coefficients: [-1, -1, 1, 1]
            let stencil = [-1.0_f32, -1.0, 1.0, 1.0];

            for a in 0..4 {
                for b in 0..4 {
                    let val = w * stencil[a] * stencil[b];
                    if val.abs() > 1e-12 {
                        triplets.push((idx[a], idx[b], val));
                    }
                }
            }
        }
    }

    CsrMatrix::from_triplets(n, n, &triplets)
}

/// Assemble the RHS vector for the PD global step.
///
/// RHS = (M/h²) · q_pred + Σ wᵢ Sᵢᵀ pᵢ + Σ wᵇⱼ Lⱼᵀ pᵇⱼ
///
/// where:
/// - `q_pred` is the predicted position (from inertia)
/// - `pᵢ` is the local projection target for element i (membrane)
/// - `pᵇⱼ` is the local projection target for bending element j
/// - `Sᵢᵀ` scatters the triangle-local projection back to global
/// - `Lⱼᵀ` scatters the bending projection back to global
///
/// # Arguments
/// * `n` — Number of vertices
/// * `mass` — Per-vertex mass
/// * `dt` — Timestep
/// * `pred` — Predicted positions (one coordinate axis)
/// * `proj_targets` — Per-element projection target for each vertex
/// * `elements` — Precomputed element data
/// * `coord` — Which coordinate axis (0=X, 1=Y, 2=Z)
/// * `bending` — Optional bending data
/// * `bending_targets` — Optional per-bending-element projection targets (v0, v1, wa, wb)
pub fn assemble_rhs(
    n: usize,
    mass: &[f32],
    dt: f32,
    pred: &[f32],
    proj_targets: &[(f32, f32, f32)],
    elements: &ElementData,
    coord: usize,
    bending: Option<&BendingData>,
    bending_targets: Option<&[(f32, f32, f32, f32)]>,
) -> Vec<f32> {
    let inv_dt2 = 1.0 / (dt * dt);
    let mut rhs = vec![0.0_f32; n];

    // Mass/inertia term: (M/h²) · q_pred
    for i in 0..n {
        rhs[i] += mass[i] * inv_dt2 * pred[i];
    }

    // Membrane stiffness term: Σ wᵢ Sᵢᵀ pᵢ
    // Same gradient operator as the system matrix.
    // Sᵢᵀpᵢ scatters the projection target back to global vertices.
    for (e, elem) in elements.elements.iter().enumerate() {
        let [i0, i1, i2] = elem.indices;
        let w = elem.weight;

        let a = elem.dm_inv[0]; // dm00
        let b = elem.dm_inv[1]; // dm10
        let c = elem.dm_inv[2]; // dm01
        let d = elem.dm_inv[3]; // dm11

        let g = [
            [-(a + b), -(c + d)],
            [a, c],
            [b, d],
        ];
        let idx = [i0, i1, i2];

        // Get target positions for this element (the relevant coordinate)
        let target = [proj_targets[e].0, proj_targets[e].1, proj_targets[e].2];

        // The projection target pᵢ represents the 3D target positions.
        // The target edge vectors (for this coordinate) are:
        let te1 = target[1] - target[0];
        let te2 = target[2] - target[0];

        // Compute the target deformation gradient F_target (for this coordinate):
        // F_target = [te1, te2] * D_m^{-1}
        let f_target_0 = te1 * a + te2 * b;
        let f_target_1 = te1 * c + te2 * d;

        // RHS contribution from Sᵢᵀ:
        // We apply the transpose of the gradient operator to F_target
        let _ = coord; // coord already selected in the projected targets
        for a_idx in 0..3 {
            let contrib = g[a_idx][0] * f_target_0 + g[a_idx][1] * f_target_1;
            rhs[idx[a_idx]] += w * contrib;
        }
    }

    // Bending stiffness term: Σ wᵇⱼ Lⱼᵀ pᵇⱼ
    // Uses the same Laplacian stencil as the system matrix.
    if let (Some(bend), Some(targets)) = (bending, bending_targets) {
        let stencil = [-1.0_f32, -1.0, 1.0, 1.0];

        for (e, elem) in bend.elements.iter().enumerate() {
            let idx = [elem.v0, elem.v1, elem.wing_a, elem.wing_b];
            let w = elem.weight;

            let target = [targets[e].0, targets[e].1, targets[e].2, targets[e].3];

            // Lᵀ pᵇ: for each vertex, accumulate w * L[i] * dot(L, target)
            let l_dot_target: f32 = (0..4)
                .map(|k| stencil[k] * target[k])
                .sum();

            for i in 0..4 {
                rhs[idx[i]] += w * stencil[i] * l_dot_target;
            }
        }
    }

    rhs
}
