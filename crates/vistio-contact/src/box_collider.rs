//! Analytical Axis-Aligned Bounding Box (AABB) collision.
//!
//! A semi-infinite or finite box collider that prevents vertices from
//! moving inside its volume. Suitable for Cantilever Bending ledges.

use vistio_solver::state::SimulationState;

use crate::response::ContactResult;

/// Analytical AABB collider.
pub struct BoxCollider {
    pub min_x: f32,
    pub max_x: f32,
    pub min_y: f32,
    pub max_y: f32,
    pub min_z: f32,
    pub max_z: f32,
}

impl BoxCollider {
    /// Creates a new box collider with definite bounds.
    pub fn new(min_x: f32, max_x: f32, min_y: f32, max_y: f32, min_z: f32, max_z: f32) -> Self {
        Self { min_x, max_x, min_y, max_y, min_z, max_z }
    }

    /// Resolve box contacts by projecting penetrating vertices to the nearest face.
    pub fn resolve(&self, state: &mut SimulationState) -> ContactResult {
        let mut resolved = 0u32;
        let mut max_pen = 0.0_f32;
        let mut tf = 0.0_f32;

        for i in 0..state.vertex_count {
            if state.inv_mass[i] == 0.0 { continue; }

            let x = state.pos_x[i];
            let y = state.pos_y[i];
            let z = state.pos_z[i];

            if x >= self.min_x && x <= self.max_x &&
               y >= self.min_y && y <= self.max_y &&
               z >= self.min_z && z <= self.max_z {

                // Inside the box. Find shortest penetration depth to a face.
                let d_min_x = x - self.min_x;
                let d_max_x = self.max_x - x;
                let d_min_y = y - self.min_y;
                let d_max_y = self.max_y - y;
                let d_min_z = z - self.min_z;
                let d_max_z = self.max_z - z;

                let mut min_d = d_min_x.min(d_max_x).min(d_min_y).min(d_max_y).min(d_min_z).min(d_max_z);

                let friction = 0.7; // Friction factor

                // CCD heuristic: if it was above the top surface in the previous frame,
                // it MUST have tunnelled vertically. Push it UP.
                if state.prev_y[i] >= self.max_y {
                    min_d = d_max_y;
                }

                if min_d == d_min_x {
                    // Push out left (X-)
                    state.pos_x[i] = self.min_x;
                    if state.vel_x[i] > 0.0 { state.vel_x[i] = 0.0; }
                    state.vel_y[i] *= friction; state.vel_z[i] *= friction;
                } else if min_d == d_max_x {
                    // Push out right (X+)
                    state.pos_x[i] = self.max_x;
                    if state.vel_x[i] < 0.0 { state.vel_x[i] = 0.0; }
                    state.vel_y[i] *= friction; state.vel_z[i] *= friction;
                } else if min_d == d_max_y {
                    // Push out top (Y+)
                    state.pos_y[i] = self.max_y;
                    if state.vel_y[i] < 0.0 { state.vel_y[i] = 0.0; }
                    state.vel_x[i] *= friction; state.vel_z[i] *= friction;
                } else if min_d == d_min_y {
                    // Push out bottom (Y-)
                    state.pos_y[i] = self.min_y;
                    if state.vel_y[i] > 0.0 { state.vel_y[i] = 0.0; }
                    state.vel_x[i] *= friction; state.vel_z[i] *= friction;
                } else if min_d == d_max_z {
                    // Push out front (Z+)
                    state.pos_z[i] = self.max_z;
                    if state.vel_z[i] < 0.0 { state.vel_z[i] = 0.0; }
                    state.vel_x[i] *= friction; state.vel_y[i] *= friction;
                } else if min_d == d_min_z {
                    // Push out back (Z-)
                    state.pos_z[i] = self.min_z;
                    if state.vel_z[i] > 0.0 { state.vel_z[i] = 0.0; }
                    state.vel_x[i] *= friction; state.vel_y[i] *= friction;
                }

                resolved += 1;
                max_pen = max_pen.max(min_d);
                tf += min_d;
            }
        }

        ContactResult {
            resolved_count: resolved,
            max_residual_penetration: max_pen,
            total_force_magnitude: tf,
        }
    }
}
