//! Analytical vertical cylinder collision.
//!
//! A simple collision object that prevents vertices from falling down through
//! the top face or passing inside the cylinder's walls.
//! Suitable for the Cusick Drape pedestal.

use vistio_solver::state::SimulationState;

use crate::response::ContactResult;

/// Analytical vertical cylinder collider.
pub struct CylinderCollider {
    /// X coordinate of the center.
    pub center_x: f32,
    /// Z coordinate of the center.
    pub center_z: f32,
    /// The top surface height (Y axis).
    pub top_y: f32,
    /// Radius of the cylinder.
    pub radius: f32,
}

impl CylinderCollider {
    /// Creates a new cylinder collider.
    pub fn new(center_x: f32, center_z: f32, top_y: f32, radius: f32) -> Self {
        Self { center_x, center_z, top_y, radius }
    }

    /// Resolve cylinder contacts by projecting penetrating vertices to the top or side surface.
    pub fn resolve(&self, state: &mut SimulationState) -> ContactResult {
        let mut resolved = 0u32;
        let mut max_penetration = 0.0_f32;
        let mut total_force = 0.0_f32;

        let r2 = self.radius * self.radius;
        let friction = 0.7; // Moderate friction for pedestal

        for i in 0..state.vertex_count {
            if state.inv_mass[i] == 0.0 {
                continue;
            }

            // Only consider vertices strictly below the top face.
            if state.pos_y[i] > self.top_y {
                continue;
            }

            let dx = state.pos_x[i] - self.center_x;
            let dz = state.pos_z[i] - self.center_z;
            let dist2 = dx * dx + dz * dz;

            // Note: Since this cylinder extends to y = -infinity, anything inside r2 is a hit.
            if dist2 < r2 {
                let dist = dist2.sqrt();
                let depth_side = self.radius - dist;
                let depth_top = self.top_y - state.pos_y[i];

                // CCD heuristic: if it was above the top surface in the previous frame,
                // it MUST have tunnelled vertically. Push it UP.
                let mut push_up = depth_top < depth_side;

                if state.prev_y[i] >= self.top_y {
                    push_up = true;
                }

                // If it hits dead center, we also must push up to avoid NaN normal
                if dist < 1e-12 {
                    push_up = true;
                }

                if push_up {
                    // It penetrated the top face. Push UP.
                    state.pos_y[i] = self.top_y;

                    if state.vel_y[i] < 0.0 {
                        state.vel_y[i] = 0.0;
                    }

                    // Friction resists horizontal motion. Very high friction for pedestal
                    // to prevent it sliding off like ice.
                    state.vel_x[i] *= 0.1;
                    state.vel_z[i] *= 0.1;

                    resolved += 1;
                    max_penetration = max_penetration.max(depth_top);
                    total_force += depth_top;
                } else {
                    // It penetrated the side more than the top, push OUT.
                    if dist > 1e-12 {
                        let nx = dx / dist;
                        let nz = dz / dist;
                        state.pos_x[i] += nx * depth_side;
                        state.pos_z[i] += nz * depth_side;

                        // Inelastic collision for normal velocity
                        let v_dot_n = state.vel_x[i] * nx + state.vel_z[i] * nz;
                        if v_dot_n < 0.0 {
                            state.vel_x[i] -= v_dot_n * nx;
                            state.vel_z[i] -= v_dot_n * nz;
                        }

                        // Friction resists vertical sliding
                        state.vel_y[i] *= friction;
                    }

                    resolved += 1;
                    max_penetration = max_penetration.max(depth_side);
                    total_force += depth_side;
                }
            }
        }

        ContactResult {
            resolved_count: resolved,
            max_residual_penetration: max_penetration,
            total_force_magnitude: total_force,
        }
    }
}
