//! Unified collision pipeline that orchestrates broad → narrow → response.
//!
//! The pipeline is handed to the PD solver and called each timestep
//! (or each PD iteration) to detect and resolve contacts.

use vistio_mesh::TriangleMesh;
use vistio_solver::state::SimulationState;
use vistio_types::VistioResult;

use crate::broad::BroadPhase;
use crate::ground_plane::GroundPlane;
use crate::narrow::NarrowPhase;
use crate::response::{ContactResponse, ContactResult};

/// Unified collision pipeline: broad → narrow → response.
///
/// Orchestrates the three collision phases and optional ground plane.
/// Handed to the PD solver via `set_collision_pipeline()`.
pub struct CollisionPipeline {
    /// Broad phase acceleration structure.
    pub broad: Box<dyn BroadPhase>,
    /// Narrow phase exact testing.
    pub narrow: Box<dyn NarrowPhase>,
    /// Contact response (position correction or barrier).
    pub response: Box<dyn ContactResponse>,
    /// Collision thickness / separation margin.
    pub thickness: f32,
    /// Response stiffness.
    pub stiffness: f32,
    /// Optional ground plane.
    pub ground: Option<GroundPlane>,
    /// Reference mesh (for triangle queries in narrow phase).
    mesh: TriangleMesh,
}

impl CollisionPipeline {
    /// Create a new collision pipeline.
    pub fn new(
        broad: Box<dyn BroadPhase>,
        narrow: Box<dyn NarrowPhase>,
        response: Box<dyn ContactResponse>,
        mesh: TriangleMesh,
        thickness: f32,
        stiffness: f32,
    ) -> Self {
        Self {
            broad,
            narrow,
            response,
            thickness,
            stiffness,
            ground: None,
            mesh,
        }
    }

    /// Add an optional ground plane.
    pub fn with_ground(mut self, height: f32) -> Self {
        self.ground = Some(GroundPlane::new(height));
        self
    }

    /// Run the full collision pipeline: broad → narrow → response + ground.
    pub fn step(&mut self, state: &mut SimulationState) -> VistioResult<CollisionStepResult> {
        // 1. Broad phase: update acceleration structure and query pairs
        self.broad.update(state, self.thickness)?;
        let candidates = self.broad.query_pairs();

        // 2. Narrow phase: exact proximity tests
        let contacts = self.narrow.detect(&candidates, state, &self.mesh, self.thickness)?;

        // 3. Contact response: resolve penetrations
        let mesh_result = self.response.resolve(&contacts, state, self.stiffness)?;

        // 4. Ground plane (if present)
        let ground_result = if let Some(ref ground) = self.ground {
            ground.resolve(state)
        } else {
            ContactResult {
                resolved_count: 0,
                max_residual_penetration: 0.0,
                total_force_magnitude: 0.0,
            }
        };

        let candidate_count = candidates.len() as u32;

        Ok(CollisionStepResult {
            candidate_pairs: candidate_count,
            contacts_detected: contacts.len() as u32,
            mesh_result,
            ground_result,
        })
    }
}

/// Result of a full collision pipeline step.
#[derive(Debug, Clone)]
pub struct CollisionStepResult {
    /// Number of broad-phase candidate pairs.
    pub candidate_pairs: u32,
    /// Number of narrow-phase contacts detected.
    pub contacts_detected: u32,
    /// Mesh collision resolution result.
    pub mesh_result: ContactResult,
    /// Ground plane resolution result.
    pub ground_result: ContactResult,
}
