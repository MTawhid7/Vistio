//! Co-rotational constitutive model for Projective Dynamics.
//!
//! Performs polar decomposition F = R·S on the deformation gradient,
//! then projects to the rotation R as the target. This handles large
//! rotations (folds, drapes) correctly — every fold is a large rotation,
//! and co-rotational projection applies linear elasticity in the rotated
//! frame rather than the world frame.
//!
//! Energy: E = w/2 · ||F - R||²_F  (ARAP / As-Rigid-As-Possible)

use vistio_math::decomposition::polar_decomposition_3x2;
use vistio_math::mat3x2::Mat3x2;

use crate::traits::{ConstitutiveModel, ProjectedGradient};

/// Co-rotational FEM constitutive model.
///
/// The production default for cloth simulation. Extracts the rotation
/// from the deformation gradient via polar decomposition, then returns
/// it as the projection target. This is equivalent to ARAP (As-Rigid-As-Possible)
/// energy minimization — the element "wants" to stay rigid.
///
/// # Why co-rotational?
///
/// Linear elasticity (StVK) produces energy artifacts when triangles
/// rotate significantly (e.g., a folded cloth edge). Co-rotational
/// projection eliminates these artifacts by measuring strain in the
/// element's rotated frame.
pub struct CoRotationalModel;

impl CoRotationalModel {
    /// Creates a new co-rotational model.
    pub fn new() -> Self {
        Self
    }
}

impl Default for CoRotationalModel {
    fn default() -> Self {
        Self::new()
    }
}

impl ConstitutiveModel for CoRotationalModel {
    fn project(
        &self,
        deformation_gradient: &Mat3x2,
        rest_area: f32,
        stiffness: f32,
    ) -> ProjectedGradient {
        let polar = polar_decomposition_3x2(deformation_gradient);

        // ARAP energy: E = w/2 · ||F - R||²_F
        let diff = *deformation_gradient - polar.rotation;
        let energy = 0.5 * stiffness * rest_area * diff.frobenius_norm_sq();

        // The target deformation gradient is the pure rotation R.
        // The solver will drive the element toward this rigid configuration.
        ProjectedGradient {
            target_f: polar.rotation,
            energy,
        }
    }

    fn name(&self) -> &str {
        "co_rotational"
    }
}
