//! Ground plane collision.
//!
//! A simple collision object that prevents vertices from penetrating
//! a horizontal ground plane at `y = height`.

use vistio_solver::state::SimulationState;

use crate::response::ContactResult;

/// Ground plane collision at a fixed Y height.
///
/// Generates contacts for any vertex below `y = height` and applies
/// direct position correction to push them above the plane.
pub struct GroundPlane {
    /// Height of the ground plane (Y coordinate).
    pub height: f32,
}

impl GroundPlane {
    /// Creates a new ground plane at the given height.
    pub fn new(height: f32) -> Self {
        Self { height }
    }

    /// Resolve ground plane contacts by projecting vertices above the plane.
    pub fn resolve(&self, state: &mut SimulationState) -> ContactResult {
        let mut resolved = 0u32;
        let mut max_penetration = 0.0_f32;
        let mut total_force = 0.0_f32;

        for i in 0..state.vertex_count {
            if state.inv_mass[i] == 0.0 {
                continue;
            }
            let depth = self.height - state.pos_y[i];
            if depth > 0.0 {
                state.pos_y[i] = self.height;

                // Also update prev_y so the velocity computation
                // (pos - prev) / dt doesn't produce spurious downward
                // velocity next step — this prevents jelly oscillation.
                state.prev_y[i] = self.height;

                // Apply simple kinetic friction to tangential velocities
                state.vel_x[i] *= 0.5;
                state.vel_z[i] *= 0.5;

                // Perfectly inelastic impulse against the floor:
                // If the vertex is moving downwards (into the floor), stop it.
                if state.vel_y[i] < 0.0 {
                    state.vel_y[i] = 0.0;
                }

                resolved += 1;
                max_penetration = max_penetration.max(depth);
                total_force += depth;
            }
        }

        ContactResult {
            resolved_count: resolved,
            max_residual_penetration: max_penetration,
            total_force_magnitude: total_force,
        }
    }
}
