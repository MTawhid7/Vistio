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
use crate::sphere::SphereCollider;
use crate::cylinder::CylinderCollider;
use crate::box_collider::BoxCollider;
use crate::self_collision::{SelfCollisionSystem, SelfCollisionResult};

/// Unified collision pipeline: broad → narrow → response.
///
/// Orchestrates the three collision phases and optional ground plane.
/// Handed to the PD solver via `set_collision_pipeline()`.
pub struct CollisionPipeline {
    /// Broad phase acceleration structure.
    pub broad: Box<dyn BroadPhase + Send + Sync>,
    /// Narrow phase exact testing.
    pub narrow: Box<dyn NarrowPhase + Send + Sync>,
    /// Contact response (position correction or barrier).
    pub response: Box<dyn ContactResponse + Send + Sync>,
    /// Collision thickness / separation margin.
    pub thickness: f32,
    /// Response stiffness.
    pub stiffness: f32,
    /// Optional ground plane.
    pub ground: Option<GroundPlane>,
    /// Optional analytical sphere collider.
    pub sphere: Option<SphereCollider>,
    pub cylinder: Option<CylinderCollider>,
    pub box_collider: Option<BoxCollider>,
    /// Optional self-collision system.
    pub self_collision: Option<SelfCollisionSystem>,
    /// Reference mesh (for triangle queries in narrow phase).
    mesh: TriangleMesh,
}

impl CollisionPipeline {
    /// Create a new collision pipeline.
    pub fn new(
        broad: Box<dyn BroadPhase + Send + Sync>,
        narrow: Box<dyn NarrowPhase + Send + Sync>,
        response: Box<dyn ContactResponse + Send + Sync>,
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
            sphere: None,
            cylinder: None,
            box_collider: None,
            self_collision: None,
            mesh,
        }
    }

    /// Add an optional ground plane.
    pub fn with_ground(mut self, height: f32) -> Self {
        self.ground = Some(GroundPlane::new(height));
        self
    }

    /// Add an optional analytical sphere collider.
        pub fn with_cylinder(mut self, center_x: f32, center_z: f32, top_y: f32, radius: f32) -> Self {
        self.cylinder = Some(CylinderCollider::new(center_x, center_z, top_y, radius));
        self
    }

    pub fn with_box(mut self, min_x: f32, max_x: f32, min_y: f32, max_y: f32, min_z: f32, max_z: f32) -> Self {
        self.box_collider = Some(BoxCollider::new(min_x, max_x, min_y, max_y, min_z, max_z));
        self
    }

    pub fn with_sphere(mut self, center: vistio_math::Vec3, radius: f32) -> Self {
        self.sphere = Some(SphereCollider::new(center, radius));
        self
    }

    /// Add the self-collision module.
    pub fn with_self_collision(
        mut self,
        topology: &vistio_mesh::topology::Topology,
        exclusion_depth: usize,
    ) -> Self {
        self.self_collision = Some(SelfCollisionSystem::new(
            &self.mesh,
            topology,
            exclusion_depth,
            self.thickness,
            self.stiffness,
        ));
        self
    }

    /// Run the full collision pipeline: broad → narrow → response + ground.
    pub fn step(&mut self, state: &mut SimulationState) -> VistioResult<CollisionStepResult> {
        let mut candidates = Vec::new();
        let mut contacts = Vec::new();
        let mut mesh_result = ContactResult { resolved_count: 0, max_residual_penetration: 0.0, total_force_magnitude: 0.0 };

        if self.thickness > 0.0 && self.stiffness > 0.0 {
            // 1. Broad phase: update acceleration structure and query pairs
            self.broad.update(state, self.thickness)?;
            candidates = self.broad.query_pairs();

            // 2. Narrow phase: exact proximity tests
            contacts = self.narrow.detect(&candidates, state, &self.mesh, self.thickness)?;

            // 3. Contact response: resolve penetrations
            mesh_result = self.response.resolve(&contacts, state, self.stiffness)?;
        }

        // 4. Ground plane (if present) — hard constraint, must run first
        let ground_result = if let Some(ref ground) = self.ground {
            ground.resolve(state)
        } else {
            ContactResult {
                resolved_count: 0,
                max_residual_penetration: 0.0,
                total_force_magnitude: 0.0,
            }
        };

        // 5. Self collision resolution (if enabled)
        let self_collision_result = if let Some(ref mut self_col) = self.self_collision {
            Some(self_col.solve(state, self.broad.as_mut()))
        } else {
            None
        };

        // 6. Ground plane again (post self-collision enforcement)
        if let Some(ref ground) = self.ground {
            ground.resolve(state);
        }

        // 7. Sphere collider (if present)
        let sphere_result = if let Some(ref sphere) = self.sphere {
            sphere.resolve(state)
        } else {
            ContactResult {
                resolved_count: 0,
                max_residual_penetration: 0.0,
                total_force_magnitude: 0.0,
            }
        };

                let cylinder_result = if let Some(ref cylinder) = self.cylinder {
            cylinder.resolve(state)
        } else {
            ContactResult { resolved_count: 0, max_residual_penetration: 0.0, total_force_magnitude: 0.0 }
        };

        let box_result = if let Some(ref box_col) = self.box_collider {
            box_col.resolve(state)
        } else {
            ContactResult { resolved_count: 0, max_residual_penetration: 0.0, total_force_magnitude: 0.0 }
        };

        let candidate_count = candidates.len() as u32;

        Ok(CollisionStepResult {
            candidate_pairs: candidate_count,
            contacts_detected: contacts.len() as u32,
            mesh_result,
            ground_result,
            sphere_result,
            cylinder_result,
            box_result,
            self_collision_result,
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
    /// Sphere collision resolution result.
    pub sphere_result: ContactResult,
    pub cylinder_result: ContactResult,
    pub box_result: ContactResult,
    /// Result from self collision resolution.
    pub self_collision_result: Option<SelfCollisionResult>,
}
