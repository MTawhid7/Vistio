//! Self-collision detection and resolution.
//!
//! Implements the 3-phase self-collision pipeline:
//! 1. **Detection**: Spatial hash → candidate pairs, filtered by topology exclusion
//! 2. **Coloring**: Graph coloring for parallel-safe batching
//! 3. **Resolution**: Mass-weighted position corrections per batch

use vistio_mesh::TriangleMesh;
use vistio_mesh::topology::Topology;
use vistio_solver::state::SimulationState;

use crate::broad::BroadPhase;
use crate::coloring::CollisionColoring;
use crate::exclusion::TopologyExclusion;

/// Self-collision system: detect → color → resolve.
///
/// Uses topology exclusion to filter adjacent vertices,
/// graph coloring for parallel safety, and mass-weighted
/// position corrections for resolution.
pub struct SelfCollisionSystem {
    /// Topology-based exclusion (n-ring neighborhood).
    exclusion: TopologyExclusion,
    /// Collision thickness / proximity threshold.
    thickness: f32,
    /// Response stiffness (0.0–1.0).
    stiffness: f32,
}

/// Result of self-collision resolution.
#[derive(Debug, Clone)]
pub struct SelfCollisionResult {
    /// Number of candidate pairs from broad phase.
    pub candidate_pairs: u32,
    /// Number of pairs after topology filtering.
    pub filtered_pairs: u32,
    /// Number of pairs where vertices are within thickness.
    pub proximity_pairs: u32,
    /// Number of color batches used.
    pub batch_count: u32,
    /// Number of position corrections applied.
    pub corrections_applied: u32,
}

impl SelfCollisionSystem {
    /// Create a new self-collision system.
    ///
    /// # Arguments
    /// - `mesh`: Reference mesh for topology computation
    /// - `topology`: Pre-computed mesh topology
    /// - `exclusion_depth`: N-ring exclusion depth (typically 2)
    /// - `thickness`: Proximity threshold for collision detection
    /// - `stiffness`: Response stiffness (0.0 = no response, 1.0 = full correction)
    pub fn new(
        mesh: &TriangleMesh,
        topology: &Topology,
        exclusion_depth: usize,
        thickness: f32,
        stiffness: f32,
    ) -> Self {
        let exclusion = TopologyExclusion::new(mesh, topology, exclusion_depth);
        Self {
            exclusion,
            thickness,
            stiffness,
        }
    }

    /// Run the full self-collision pipeline: detect → color → resolve.
    pub fn solve(
        &mut self,
        state: &mut SimulationState,
        broad: &mut dyn BroadPhase,
    ) -> SelfCollisionResult {
        // Phase 1: Detection — query broad phase and filter with topology
        broad.update(state, self.thickness).ok();
        let candidates = broad.query_pairs();

        let mut proximity_pairs: Vec<(u32, u32)> = Vec::new();

        for candidate in &candidates {
            let i = candidate.a as usize;
            let j = candidate.b as usize;

            // Skip if topologically adjacent
            if self.exclusion.should_exclude(i, j) {
                continue;
            }

            // Distance test
            let dx = state.pos_x[i] - state.pos_x[j];
            let dy = state.pos_y[i] - state.pos_y[j];
            let dz = state.pos_z[i] - state.pos_z[j];
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();

            if dist < self.thickness {
                proximity_pairs.push((candidate.a, candidate.b));
            }
        }

        let candidate_count = candidates.len() as u32;
        let proximity_count = proximity_pairs.len() as u32;

        if proximity_pairs.is_empty() {
            return SelfCollisionResult {
                candidate_pairs: candidate_count,
                filtered_pairs: 0,
                proximity_pairs: 0,
                batch_count: 0,
                corrections_applied: 0,
            };
        }

        // Phase 2: Coloring — organize into parallel-safe batches
        let (sorted_pairs, batch_offsets) =
            CollisionColoring::color_pairs(&proximity_pairs, state.vertex_count);

        let batch_count = (batch_offsets.len().saturating_sub(1)) as u32;

        // Phase 3: Resolution — mass-weighted position corrections
        let mut corrections = 0u32;

        for batch_idx in 0..batch_offsets.len().saturating_sub(1) {
            let start = batch_offsets[batch_idx];
            let end = batch_offsets[batch_idx + 1];

            for &(a, b) in &sorted_pairs[start..end] {
                let i = a as usize;
                let j = b as usize;

                // Recompute distance (positions may have changed from earlier batches)
                let dx = state.pos_x[i] - state.pos_x[j];
                let dy = state.pos_y[i] - state.pos_y[j];
                let dz = state.pos_z[i] - state.pos_z[j];
                let dist = (dx * dx + dy * dy + dz * dz).sqrt();

                if dist >= self.thickness || dist < 1e-10 {
                    continue;
                }

                let overlap = self.thickness - dist;
                let nx = dx / dist;
                let ny = dy / dist;
                let nz = dz / dist;

                // Mass-weighted correction
                let w_i = state.inv_mass[i];
                let w_j = state.inv_mass[j];
                let w_sum = w_i + w_j;

                if w_sum < 1e-10 {
                    continue; // Both pinned
                }

                let correction = overlap * self.stiffness;
                let ratio_i = w_i / w_sum;
                let ratio_j = w_j / w_sum;

                let dx_i = nx * correction * ratio_i;
                let dy_i = ny * correction * ratio_i;
                let dz_i = nz * correction * ratio_i;

                let dx_j = nx * correction * ratio_j;
                let dy_j = ny * correction * ratio_j;
                let dz_j = nz * correction * ratio_j;

                // Push apart along the normal
                state.pos_x[i] += dx_i;
                state.pos_y[i] += dy_i;
                state.pos_z[i] += dz_i;

                state.pos_x[j] -= dx_j;
                state.pos_y[j] -= dy_j;
                state.pos_z[j] -= dz_j;

                // Calculate relative velocity
                let v_rel_x = state.vel_x[i] - state.vel_x[j];
                let v_rel_y = state.vel_y[i] - state.vel_y[j];
                let v_rel_z = state.vel_z[i] - state.vel_z[j];

                // Approach velocity along the normal
                let vn = v_rel_x * nx + v_rel_y * ny + v_rel_z * nz;

                // Stop further approach (perfectly inelastic impulse)
                // Only act if they are moving towards each other
                if vn < 0.0 {
                    let impulse_mag = -vn;

                    let dv_x = impulse_mag * nx;
                    let dv_y = impulse_mag * ny;
                    let dv_z = impulse_mag * nz;

                    state.vel_x[i] += dv_x * ratio_i;
                    state.vel_y[i] += dv_y * ratio_i;
                    state.vel_z[i] += dv_z * ratio_i;

                    state.vel_x[j] -= dv_x * ratio_j;
                    state.vel_y[j] -= dv_y * ratio_j;
                    state.vel_z[j] -= dv_z * ratio_j;
                }

                corrections += 1;
            }
        }

        SelfCollisionResult {
            candidate_pairs: candidate_count,
            filtered_pairs: proximity_count,
            proximity_pairs: proximity_count,
            batch_count,
            corrections_applied: corrections,
        }
    }
}
