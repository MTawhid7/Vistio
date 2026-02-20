//! Integration tests for vistio-solver.

use vistio_mesh::generators::quad_grid;
use vistio_mesh::topology::Topology;
use vistio_solver::config::SolverConfig;
use vistio_solver::pd_stub::ProjectiveDynamicsStub;
use vistio_solver::state::SimulationState;
use vistio_solver::strategy::SolverStrategy;

// ─── SimulationState Tests ────────────────────────────────────

#[test]
fn state_from_mesh() {
    let mesh = quad_grid(4, 4, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    assert_eq!(state.vertex_count, n);
    assert_eq!(state.pos_x.len(), n);
    assert_eq!(state.vel_x.len(), n);
    assert!(state.vel_x.iter().all(|&v| v == 0.0)); // Starts at rest
}

#[test]
fn state_pinning() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let n = mesh.vertex_count();
    let mut pinned = vec![false; n];
    pinned[0] = true; // Pin top-left
    pinned[2] = true; // Pin top-right

    let state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();
    assert_eq!(state.inv_mass[0], 0.0);
    assert_eq!(state.inv_mass[2], 0.0);
    assert!(state.inv_mass[4] > 0.0); // Center is free
}

#[test]
fn state_predict() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    let initial_y = state.pos_y.clone();
    let dt = 1.0 / 60.0;
    let gravity = [0.0, -9.81, 0.0];

    state.predict(dt, gravity);

    // After prediction with zero initial velocity:
    // pred_y = pos_y + dt*0 + dt²*(-9.81)
    let dt2g = dt * dt * (-9.81);
    for i in 0..n {
        let expected = initial_y[i] + dt2g;
        assert!(
            (state.pred_y[i] - expected).abs() < 1e-5,
            "Vertex {}: pred_y={}, expected={}",
            i, state.pred_y[i], expected
        );
    }
}

#[test]
fn state_pinned_predict() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let n = mesh.vertex_count();
    let mut pinned = vec![false; n];
    pinned[0] = true;

    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();
    let original_pos = state.pos_y[0];

    state.predict(1.0 / 60.0, [0.0, -9.81, 0.0]);

    // Pinned vertex: predicted position = current position
    assert_eq!(state.pred_y[0], original_pos);
}

#[test]
fn state_velocity_update() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    // Save previous, move one vertex, update velocities
    state.save_previous();
    state.pos_x[0] += 0.5;
    let dt = 1.0 / 60.0;
    state.update_velocities(dt);

    let expected_vx = 0.5 / dt;
    assert!((state.vel_x[0] - expected_vx).abs() < 1e-3);
    assert_eq!(state.vel_y[0], 0.0); // No movement in Y
}

#[test]
fn state_damping() {
    let mesh = quad_grid(1, 1, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    state.vel_x[0] = 10.0;
    state.damp_velocities(0.1);
    assert!((state.vel_x[0] - 9.0).abs() < 1e-5);
}

#[test]
fn state_kinetic_energy() {
    let mesh = quad_grid(1, 1, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 1.0, &pinned).unwrap();

    // Give vertex 0 velocity [1, 0, 0]
    state.vel_x[0] = 1.0;
    let ke = state.kinetic_energy();
    // KE = 0.5 * 1.0 * 1.0² = 0.5
    assert!((ke - 0.5).abs() < 1e-6);
}

// ─── SolverConfig Tests ───────────────────────────────────────

#[test]
fn config_default() {
    let config = SolverConfig::default();
    assert_eq!(config.max_iterations, 15);
    assert!(config.tolerance < 1e-4);
    assert!((config.gravity[1] + 9.81).abs() < 1e-3);
}

#[test]
fn config_debug() {
    let config = SolverConfig::debug();
    assert_eq!(config.max_iterations, 3);
}

#[test]
fn config_high_quality() {
    let config = SolverConfig::high_quality();
    assert_eq!(config.max_iterations, 30);
    assert!(config.chebyshev_acceleration);
}

#[test]
fn config_serialization() {
    let config = SolverConfig::default();
    let toml = toml::to_string(&config).unwrap();
    let recovered: SolverConfig = toml::from_str(&toml).unwrap();
    assert_eq!(recovered.max_iterations, config.max_iterations);
}

// ─── PD Stub Solver Tests ─────────────────────────────────────

#[test]
fn pd_stub_init_and_step() {
    let mesh = quad_grid(4, 4, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let n = mesh.vertex_count();
    let config = SolverConfig::default();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    let mut solver = ProjectiveDynamicsStub::new();
    solver.init(&mesh, &topo, &config).unwrap();

    let result = solver.step(&mut state, 1.0 / 60.0).unwrap();
    assert!(result.converged);
    assert_eq!(result.iterations, 0); // Stub does zero iterations
}

#[test]
fn pd_stub_gravity_motion() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();
    let config = SolverConfig::default();

    let mut solver = ProjectiveDynamicsStub::new();
    solver.init(&mesh, &topo, &config).unwrap();

    let initial_y: Vec<f32> = state.pos_y.clone();

    // Run 10 steps
    let dt = 1.0 / 60.0;
    for _ in 0..10 {
        solver.step(&mut state, dt).unwrap();
    }

    // All unpinned vertices should have moved down (gravity -Y)
    for i in 0..n {
        assert!(
            state.pos_y[i] < initial_y[i],
            "Vertex {} should have fallen: y={} vs initial={}",
            i, state.pos_y[i], initial_y[i]
        );
    }
}

#[test]
fn pd_stub_pinned_vertices_stay() {
    let mesh = quad_grid(4, 4, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let n = mesh.vertex_count();
    let mut pinned = vec![false; n];
    pinned[0] = true;
    pinned[4] = true;

    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();
    let config = SolverConfig::default();

    let pin0_y = state.pos_y[0];
    let pin4_y = state.pos_y[4];

    let mut solver = ProjectiveDynamicsStub::new();
    solver.init(&mesh, &topo, &config).unwrap();

    for _ in 0..20 {
        solver.step(&mut state, 1.0 / 60.0).unwrap();
    }

    assert_eq!(state.pos_y[0], pin0_y, "Pinned vertex 0 should not move");
    assert_eq!(state.pos_y[4], pin4_y, "Pinned vertex 4 should not move");
}

#[test]
fn pd_stub_not_initialized_error() {
    let mesh = quad_grid(1, 1, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    let mut solver = ProjectiveDynamicsStub::new();
    // Don't call init()
    assert!(solver.step(&mut state, 1.0 / 60.0).is_err());
}
