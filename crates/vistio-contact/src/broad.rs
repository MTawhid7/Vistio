//! Broad phase collision detection trait.
//!
//! Quickly identifies *candidate* collision pairs using spatial
//! acceleration structures. The narrow phase then refines these
//! candidates into actual contacts.

use vistio_solver::SimulationState;
use vistio_types::VistioResult;

/// Candidate pair from broad phase (indices into vertex/triangle arrays).
#[derive(Debug, Clone, Copy)]
pub struct CandidatePair {
    /// First primitive index.
    pub a: u32,
    /// Second primitive index.
    pub b: u32,
    /// Whether both primitives belong to the same mesh (self-collision).
    pub is_self: bool,
}

/// Trait for broad phase collision detection.
///
/// Implementations use spatial acceleration to quickly cull pairs
/// that are too far apart to collide.
///
/// # Implementations
/// - `SpatialHash` — Uniform grid (Tier 1, good for self-collision)
/// - `BvhBroadPhase` — Bounding volume hierarchy (Tier 2, good for cloth-body)
pub trait BroadPhase: Send {
    /// Build or update the acceleration structure from current positions.
    fn update(&mut self, state: &SimulationState, thickness: f32) -> VistioResult<()>;

    /// Query candidate collision pairs.
    ///
    /// Returns all pairs where the bounding volumes overlap.
    fn query_pairs(&self) -> Vec<CandidatePair>;

    /// Returns the broad phase strategy name.
    fn name(&self) -> &str;
}

/// No-op broad phase for benchmarks that don't need collision.
pub struct NullBroadPhase;

impl BroadPhase for NullBroadPhase {
    fn update(&mut self, _state: &SimulationState, _thickness: f32) -> VistioResult<()> {
        Ok(())
    }

    fn query_pairs(&self) -> Vec<CandidatePair> {
        Vec::new()
    }

    fn name(&self) -> &str {
        "null_broad_phase"
    }
}
