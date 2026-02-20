//! Integration tests for vistio-contact.

use vistio_contact::broad::{BroadPhase, NullBroadPhase};
use vistio_contact::contact::{ContactPair, ContactType};
use vistio_contact::narrow::{NarrowPhase, NullNarrowPhase};
use vistio_contact::response::{ContactResponse, NullContactResponse};
use vistio_mesh::generators::quad_grid;
use vistio_solver::state::SimulationState;

// ─── Contact Pair Tests ───────────────────────────────────────

#[test]
fn contact_pair_penetrating() {
    let pair = ContactPair {
        contact_type: ContactType::VertexTriangle,
        indices: [0, 1, 2, 3],
        distance: -0.005,
        normal: [0.0, 1.0, 0.0],
        barycentric: [0.33, 0.33, 0.34],
        is_self: false,
    };
    assert!(pair.is_penetrating());
    assert!((pair.penetration_depth() - 0.005).abs() < 1e-6);
}

#[test]
fn contact_pair_not_penetrating() {
    let pair = ContactPair {
        contact_type: ContactType::EdgeEdge,
        indices: [0, 1, 2, 3],
        distance: 0.01,
        normal: [1.0, 0.0, 0.0],
        barycentric: [0.5, 0.5, 0.0],
        is_self: true,
    };
    assert!(!pair.is_penetrating());
    assert_eq!(pair.penetration_depth(), 0.0);
}

#[test]
fn contact_pair_serialization() {
    let pair = ContactPair {
        contact_type: ContactType::VertexTriangle,
        indices: [10, 20, 30, 40],
        distance: -0.001,
        normal: [0.0, 0.0, 1.0],
        barycentric: [0.25, 0.25, 0.5],
        is_self: false,
    };
    let json = serde_json::to_string(&pair).unwrap();
    let recovered: ContactPair = serde_json::from_str(&json).unwrap();
    assert_eq!(recovered.indices, [10, 20, 30, 40]);
}

// ─── Null Pipeline Tests ──────────────────────────────────────

#[test]
fn null_broad_phase_returns_empty() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    let mut bp = NullBroadPhase;
    bp.update(&state, 0.001).unwrap();
    assert!(bp.query_pairs().is_empty());
    assert_eq!(bp.name(), "null_broad_phase");
}

#[test]
fn null_narrow_phase_returns_empty() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    let np = NullNarrowPhase;
    let contacts = np.detect(&[], &state, &mesh, 0.001).unwrap();
    assert!(contacts.is_empty());
    assert_eq!(np.name(), "null_narrow_phase");
}

#[test]
fn null_contact_response_returns_default() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    let cr = NullContactResponse;
    let result = cr.resolve(&[], &mut state, 1.0).unwrap();
    assert_eq!(result.resolved_count, 0);
    assert_eq!(result.max_residual_penetration, 0.0);
    assert_eq!(cr.name(), "null_contact_response");
}
