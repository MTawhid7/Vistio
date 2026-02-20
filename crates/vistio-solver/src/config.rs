//! Solver configuration.
//!
//! Parameters that control solver behavior: iteration counts,
//! convergence tolerance, material stiffness weights.

use serde::{Deserialize, Serialize};

/// Configuration for the simulation solver.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SolverConfig {
    /// Maximum solver iterations per timestep.
    pub max_iterations: u32,

    /// Convergence tolerance (residual norm threshold).
    /// Solver stops early if residual < tolerance.
    pub tolerance: f64,

    /// Gravity vector [gx, gy, gz] in m/s².
    pub gravity: [f32; 3],

    /// Velocity damping factor (0.0 = no damping, 1.0 = full damping).
    pub damping: f32,

    /// Stretch stiffness weight (scales constitutive model energy).
    pub stretch_weight: f32,

    /// Bending stiffness weight.
    pub bending_weight: f32,

    /// Contact stiffness weight.
    pub contact_weight: f32,

    /// Whether to use Chebyshev acceleration.
    pub chebyshev_acceleration: bool,

    /// Chebyshev spectral radius estimate (0.0–1.0).
    pub spectral_radius: f32,
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            max_iterations: vistio_types::constants::DEFAULT_PD_ITERATIONS,
            tolerance: 1e-6,
            gravity: [0.0, -vistio_types::constants::GRAVITY, 0.0],
            damping: 0.01,
            stretch_weight: 1.0,
            bending_weight: 0.5,
            contact_weight: 1.0,
            chebyshev_acceleration: false,
            spectral_radius: 0.5,
        }
    }
}

impl SolverConfig {
    /// Creates a config for debugging (fewer iterations, looser tolerance).
    pub fn debug() -> Self {
        Self {
            max_iterations: 3,
            tolerance: 1e-3,
            ..Default::default()
        }
    }

    /// Creates a high-quality config (more iterations, tighter tolerance).
    pub fn high_quality() -> Self {
        Self {
            max_iterations: 30,
            tolerance: 1e-8,
            chebyshev_acceleration: true,
            ..Default::default()
        }
    }
}
