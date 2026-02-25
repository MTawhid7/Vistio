//! Analytical sphere collision.
//!
//! A simple collision object that prevents vertices from penetrating
//! a sphere of a given radius and center.

use vistio_math::Vec3;
use vistio_solver::state::SimulationState;

use crate::response::ContactResult;

/// Analytical sphere collision.
///
/// Generates contacts for any vertex inside the sphere and applies
/// direct position correction to push them to the surface.
pub struct SphereCollider {
    /// Center of the sphere.
    pub center: Vec3,
    /// Radius of the sphere.
    pub radius: f32,
}

impl SphereCollider {
    /// Creates a new sphere collider.
    pub fn new(center: Vec3, radius: f32) -> Self {
        Self { center, radius }
    }

    /// Resolve sphere contacts by projecting penetrating vertices to the surface.
    pub fn resolve(&self, state: &mut SimulationState) -> ContactResult {
        let mut resolved = 0u32;
        let mut max_penetration = 0.0_f32;
        let mut total_force = 0.0_f32;

        let r2 = self.radius * self.radius;

        for i in 0..state.vertex_count {
            if state.inv_mass[i] == 0.0 {
                continue;
            }

            let dx = state.pos_x[i] - self.center.x;
            let dy = state.pos_y[i] - self.center.y;
            let dz = state.pos_z[i] - self.center.z;

            let dist2 = dx * dx + dy * dy + dz * dz;

            if dist2 < r2 && dist2 > 1e-12 {
                let dist = dist2.sqrt();
                let depth = self.radius - dist;

                let nx = dx / dist;
                let ny = dy / dist;
                let nz = dz / dist;

                let dx_corr = nx * depth;
                let dy_corr = ny * depth;
                let dz_corr = nz * depth;

                // Push to surface
                state.pos_x[i] += dx_corr;
                state.pos_y[i] += dy_corr;
                state.pos_z[i] += dz_corr;

                // Remove velocity component along the normal and apply kinetic friction
                let v_dot_n = state.vel_x[i] * nx + state.vel_y[i] * ny + state.vel_z[i] * nz;
                if v_dot_n < 0.0 {
                    let v_tangent_x = state.vel_x[i] - nx * v_dot_n;
                    let v_tangent_y = state.vel_y[i] - ny * v_dot_n;
                    let v_tangent_z = state.vel_z[i] - nz * v_dot_n;

                    let friction = 0.5;
                    state.vel_x[i] = v_tangent_x * friction;
                    state.vel_y[i] = v_tangent_y * friction;
                    state.vel_z[i] = v_tangent_z * friction;
                }

                resolved += 1;
                max_penetration = max_penetration.max(depth);
                total_force += depth;
            } else if dist2 <= 1e-12 {
                // Exactly at center, push up
                state.pos_y[i] += self.radius;
                // Purely inelastic velocity damping
                state.vel_x[i] = 0.0;
                state.vel_y[i] = 0.0;
                state.vel_z[i] = 0.0;

                resolved += 1;
                max_penetration = max_penetration.max(self.radius);
                total_force += self.radius;
            }
        }

        ContactResult {
            resolved_count: resolved,
            max_residual_penetration: max_penetration,
            total_force_magnitude: total_force,
        }
    }
}
