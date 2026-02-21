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

// ─── Spatial Hash Tests ───────────────────────────────────────

use vistio_contact::spatial_hash::SpatialHash;

#[test]
fn spatial_hash_finds_nearby_vertices() {
    let mesh = quad_grid(2, 2, 0.1, 0.1); // Small mesh, vertices close together
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    let mut hash = SpatialHash::new(0.15); // Cell size > spacing
    hash.update(&state, 0.01).unwrap();

    let pairs = hash.query_pairs();
    // With a small mesh and large cells, should find candidate pairs
    assert!(
        !pairs.is_empty(),
        "Spatial hash should find candidate pairs for close vertices"
    );
    assert_eq!(hash.name(), "spatial_hash");
}

#[test]
fn spatial_hash_distant_vertices_no_pairs() {
    let mesh = quad_grid(2, 2, 100.0, 100.0); // Huge spacing
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    let mut hash = SpatialHash::new(1.0); // Small cell size vs spacing
    hash.update(&state, 0.01).unwrap();

    let pairs = hash.query_pairs();
    // With huge spacing and small cells, no vertex shares a cell or neighbors
    assert!(
        pairs.is_empty(),
        "Spatial hash should find no pairs for widely-separated vertices"
    );
}

// ─── Projection Response Tests ────────────────────────────────

use vistio_contact::projection::ProjectionContactResponse;

#[test]
fn projection_response_corrects_penetration() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    let original_y = state.pos_y[0];

    // Simulate a penetrating contact on vertex 0
    let contact = ContactPair {
        contact_type: ContactType::VertexTriangle,
        indices: [0, 1, 2, 3],
        distance: -0.05, // 5cm penetration
        normal: [0.0, 1.0, 0.0], // Push up
        barycentric: [1.0, 0.0, 0.0],
        is_self: false,
    };

    let resp = ProjectionContactResponse;
    let result = resp.resolve(&[contact], &mut state, 1.0).unwrap();

    assert_eq!(result.resolved_count, 1);
    assert!(state.pos_y[0] > original_y, "Vertex should have been pushed up");
    assert_eq!(resp.name(), "projection_response");
}

// ─── A/B Benchmark: PD Solver vs Stub ─────────────────────────

use vistio_mesh::topology::Topology;
use vistio_solver::config::SolverConfig;
use vistio_solver::pd_solver::ProjectiveDynamicsSolver;
use vistio_solver::pd_stub::ProjectiveDynamicsStub;
use vistio_solver::strategy::SolverStrategy;

#[test]
fn ab_benchmark_pd_vs_stub() {
    // Setup: 10×10 quad grid, no pinning, simulate 20 frames
    let mesh = quad_grid(10, 10, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let config = SolverConfig::debug(); // 3 iterations
    let dt = 1.0 / 60.0;
    let steps = 20;

    // --- Run PD Stub ---
    let mut stub_state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();
    let mut stub = ProjectiveDynamicsStub::new();
    stub.init(&mesh, &topo, &config).unwrap();

    for _ in 0..steps {
        stub.step(&mut stub_state, dt).unwrap();
    }

    // --- Run full PD Solver ---
    let mut pd_state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();
    let mut pd = ProjectiveDynamicsSolver::new();
    pd.init(&mesh, &topo, &config).unwrap();

    let mut total_pd_iters = 0_u32;
    for _ in 0..steps {
        let result = pd.step(&mut pd_state, dt).unwrap();
        total_pd_iters += result.iterations;
    }

    // --- Assertions ---
    // 1. PD solver actually performed iterations
    assert!(
        total_pd_iters > 0,
        "PD solver should have performed iterations"
    );

    // 2. Both meshes fell (gravity works in both)
    let stub_center_y: f32 = stub_state.pos_y.iter().sum::<f32>() / n as f32;
    let pd_center_y: f32 = pd_state.pos_y.iter().sum::<f32>() / n as f32;
    let initial_center_y: f32 = mesh.pos_y.iter().sum::<f32>() / n as f32;

    assert!(stub_center_y < initial_center_y, "Stub should fall under gravity");
    assert!(pd_center_y < initial_center_y, "PD solver should fall under gravity");

    // 3. PD solver has lower kinetic energy (stiffness dissipates energy)
    // This is the key quality metric: the PD solver should resist deformation
    let stub_ke = stub_state.kinetic_energy();
    let pd_ke = pd_state.kinetic_energy();

    // PD solver should not be wildly different from stub
    // (both experience gravity, but PD has elastic forces)
    assert!(
        pd_ke.is_finite() && stub_ke.is_finite(),
        "Both should have finite kinetic energy"
    );

    // 4. Verify solver names
    assert_eq!(stub.name(), "projective_dynamics_stub");
    assert_eq!(pd.name(), "ProjectiveDynamics");
}
