//! Integration tests for vistio-math.

use vistio_math::mat3x2::Mat3x2;
use vistio_math::decomposition::{deformation_gradient, polar_decomposition_3x2};
use vistio_math::sparse::CsrMatrix;
use vistio_math::Vec3;

// ─── Mat3x2 Tests ─────────────────────────────────────────────

#[test]
fn identity_ftf() {
    let f = Mat3x2::IDENTITY;
    let c = f.ftf();
    assert!((c[0] - 1.0).abs() < 1e-6);
    assert!((c[1]).abs() < 1e-6);
    assert!((c[2]).abs() < 1e-6);
    assert!((c[3] - 1.0).abs() < 1e-6);
}

#[test]
fn frobenius_norm_identity() {
    let f = Mat3x2::IDENTITY;
    assert!((f.frobenius_norm_sq() - 2.0).abs() < 1e-6);
}

#[test]
fn zero_matrix() {
    let f = Mat3x2::ZERO;
    assert_eq!(f.frobenius_norm_sq(), 0.0);
}

#[test]
fn mul_mat2_identity() {
    let f = Mat3x2::from_cols(Vec3::new(1.0, 2.0, 3.0), Vec3::new(4.0, 5.0, 6.0));
    let result = f.mul_mat2([1.0, 0.0, 0.0, 1.0]);
    assert_eq!(result, f);
}

// ─── Polar Decomposition Tests ────────────────────────────────

#[test]
fn polar_identity() {
    let f = Mat3x2::IDENTITY;
    let pd = polar_decomposition_3x2(&f);
    let r = pd.rotation;
    assert!((r.col0.x - 1.0).abs() < 1e-4);
    assert!((r.col0.y).abs() < 1e-4);
    assert!((r.col1.y - 1.0).abs() < 1e-4);
    assert!((pd.stretch[0] - 1.0).abs() < 1e-4);
    assert!((pd.stretch[3] - 1.0).abs() < 1e-4);
}

#[test]
fn polar_scaled() {
    let f = Mat3x2::from_cols(Vec3::new(2.0, 0.0, 0.0), Vec3::new(0.0, 2.0, 0.0));
    let pd = polar_decomposition_3x2(&f);
    assert!((pd.stretch[0] - 2.0).abs() < 1e-4);
    assert!((pd.stretch[3] - 2.0).abs() < 1e-4);
}

#[test]
fn polar_degenerate_does_not_panic() {
    let f = Mat3x2::ZERO;
    let pd = polar_decomposition_3x2(&f);
    assert!(!pd.rotation.col0.x.is_nan());
    assert!(!pd.stretch[0].is_nan());
}

#[test]
fn deformation_gradient_identity() {
    let p0 = Vec3::ZERO;
    let p1 = Vec3::X;
    let p2 = Vec3::Y;
    let dm_inv = [1.0, 0.0, 0.0, 1.0];
    let f = deformation_gradient(p0, p1, p2, dm_inv);
    assert!((f.col0.x - 1.0).abs() < 1e-6);
    assert!((f.col1.y - 1.0).abs() < 1e-6);
}

// ─── Sparse Matrix Tests ─────────────────────────────────────

#[test]
fn empty_csr() {
    let m = CsrMatrix::new(3, 3);
    assert_eq!(m.nnz(), 0);
    assert_eq!(m.rows, 3);
    assert_eq!(m.cols, 3);
    assert_eq!(m.row_ptr.len(), 4);
}

#[test]
fn csr_from_triplets() {
    let triplets = vec![(0, 0, 1.0), (1, 1, 1.0), (2, 2, 1.0)];
    let m = CsrMatrix::from_triplets(3, 3, &triplets);
    assert_eq!(m.nnz(), 3);
    assert_eq!(m.row_ptr, vec![0, 1, 2, 3]);
    assert_eq!(m.col_idx, vec![0, 1, 2]);
    assert_eq!(m.values, vec![1.0, 1.0, 1.0]);
}

#[test]
fn csr_from_triplets_unordered() {
    let triplets = vec![(0, 2, 3.0), (0, 0, 1.0), (0, 1, 2.0)];
    let m = CsrMatrix::from_triplets(1, 3, &triplets);
    assert_eq!(m.col_idx, vec![0, 1, 2]);
    assert_eq!(m.values, vec![1.0, 2.0, 3.0]);
}
