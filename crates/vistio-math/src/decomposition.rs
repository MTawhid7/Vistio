//! Matrix decompositions for constitutive models.
//!
//! Provides polar decomposition (F = R·S) needed by co-rotational FEM,
//! and SVD for robust handling of degenerate/inverted elements.

use crate::mat3x2::Mat3x2;
use glam::Vec3;

/// Result of a 3×2 polar decomposition: F = R · S
///
/// - `R` is a 3×2 rotation matrix (orthonormal columns)
/// - `S` is a 2×2 symmetric positive semi-definite stretch tensor
#[derive(Debug, Clone, Copy)]
pub struct PolarDecomposition {
    /// Rotation part (3×2, orthonormal columns).
    pub rotation: Mat3x2,
    /// Stretch part (2×2 symmetric, stored as [s00, s01, s01, s11]).
    pub stretch: [f32; 4],
}

/// Compute the polar decomposition of a 3×2 deformation gradient.
///
/// Uses the SVD-based approach for robustness:
/// 1. Compute C = F^T F (2×2 symmetric)
/// 2. Eigendecompose C to get S = sqrt(C)
/// 3. R = F · S^{-1}
///
/// For degenerate triangles (det(S) ≈ 0), falls back to a safe default.
pub fn polar_decomposition_3x2(f: &Mat3x2) -> PolarDecomposition {
    let c = f.ftf(); // C = F^T F is a 2x2 symmetric matrix [a, b, b, d]
    let a = c[0];
    let b = c[1];
    let d = c[3];

    // Eigenvalues of 2x2 symmetric matrix: λ = (a+d)/2 ± sqrt(((a-d)/2)^2 + b^2)
    let half_trace = 0.5 * (a + d);
    let half_diff = 0.5 * (a - d);
    let disc = (half_diff * half_diff + b * b).sqrt();

    let lambda0 = (half_trace + disc).max(0.0);
    let lambda1 = (half_trace - disc).max(0.0);

    let s0 = lambda0.sqrt();
    let s1 = lambda1.sqrt();

    let eps = vistio_types::constants::DEGENERATE_AREA_THRESHOLD;

    if s0 < eps {
        // Fully degenerate — return identity-like decomposition
        return PolarDecomposition {
            rotation: Mat3x2::IDENTITY,
            stretch: [0.0, 0.0, 0.0, 0.0],
        };
    }

    // Eigenvectors of the 2x2 symmetric matrix
    let (v0, v1) = if b.abs() > eps {
        let v0 = glam::Vec2::new(lambda0 - d, b).normalize();
        let v1 = glam::Vec2::new(-v0.y, v0.x);
        (v0, v1)
    } else if a >= d {
        (glam::Vec2::X, glam::Vec2::Y)
    } else {
        (glam::Vec2::Y, glam::Vec2::X)
    };

    // S = V * diag(s0, s1) * V^T (2x2)
    let s00 = v0.x * v0.x * s0 + v1.x * v1.x * s1;
    let s01 = v0.x * v0.y * s0 + v1.x * v1.y * s1;
    let s11 = v0.y * v0.y * s0 + v1.y * v1.y * s1;

    // S_inv = V * diag(1/s0, 1/s1) * V^T
    let inv_s0 = 1.0 / s0;
    let inv_s1 = if s1 > eps { 1.0 / s1 } else { 0.0 };

    let si00 = v0.x * v0.x * inv_s0 + v1.x * v1.x * inv_s1;
    let si01 = v0.x * v0.y * inv_s0 + v1.x * v1.y * inv_s1;
    let si11 = v0.y * v0.y * inv_s0 + v1.y * v1.y * inv_s1;

    // R = F * S^{-1}
    let rotation = f.mul_mat2([si00, si01, si01, si11]);

    PolarDecomposition {
        rotation,
        stretch: [s00, s01, s01, s11],
    }
}

/// Compute the deformation gradient F for a triangle.
///
/// Given three 3D vertex positions (p0, p1, p2) and the precomputed
/// inverse of the rest-state edge matrix (Dm_inv, 2×2), returns
/// the 3×2 deformation gradient.
///
/// F = Ds * Dm_inv, where Ds = [p1-p0, p2-p0] (3×2).
pub fn deformation_gradient(
    p0: Vec3,
    p1: Vec3,
    p2: Vec3,
    dm_inv: [f32; 4], // 2x2 column-major [a, b, c, d]
) -> Mat3x2 {
    let ds = Mat3x2::from_cols(p1 - p0, p2 - p0);
    ds.mul_mat2(dm_inv)
}
