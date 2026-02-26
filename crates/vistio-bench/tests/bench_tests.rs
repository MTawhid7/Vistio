//! Integration tests for vistio-bench.

use vistio_bench::metrics::BenchmarkMetrics;
use vistio_bench::runner::BenchmarkRunner;
use vistio_bench::scenarios::{Scenario, ScenarioKind};
use vistio_solver::pd_solver::ProjectiveDynamicsSolver;

// ─── Scenario Tests ───────────────────────────────────────────

#[test]
fn hanging_sheet_setup() {
    let s = Scenario::hanging_sheet();
    assert_eq!(s.kind, ScenarioKind::HangingSheet);
    assert_eq!(s.garment.vertex_count(), 441);    // 21×21
    assert_eq!(s.garment.triangle_count(), 800);   // 20×20×2
    // Top row should be pinned (21 vertices)
    let pinned_count = s.pinned.iter().filter(|&&p| p).count();
    assert_eq!(pinned_count, 21);
}

#[test]
fn sphere_drape_setup() {
    let s = Scenario::sphere_drape();
    assert_eq!(s.kind, ScenarioKind::SphereDrape);
    assert!(s.body.is_some());
    let pinned_count = s.pinned.iter().filter(|&&p| p).count();
    assert_eq!(pinned_count, 0); // Free fall
}

#[test]
fn all_scenarios() {
    assert_eq!(ScenarioKind::all().len(), 2);
}

// ─── Runner Tests ─────────────────────────────────────────────

#[test]
fn run_hanging_sheet() {
    let mut scenario = Scenario::hanging_sheet();
    scenario.timesteps = 5; // Very short for testing
    let mut solver = ProjectiveDynamicsSolver::new();
    let metrics = BenchmarkRunner::run(&scenario, &mut solver).unwrap();

    assert_eq!(metrics.scenario, "hanging_sheet");
    assert_eq!(metrics.timesteps, 5);
    assert!(metrics.total_wall_time > 0.0);
    assert!(metrics.max_displacement > 0.0); // Gravity should cause displacement
}

#[test]
fn run_all_scenarios() {
    // Use minimal timesteps for speed
    let mut solver = ProjectiveDynamicsSolver::new();
    let kinds = ScenarioKind::all();
    for &kind in kinds {
        let mut scenario = Scenario::from_kind(kind);
        scenario.timesteps = 3;
        let metrics = BenchmarkRunner::run(&scenario, &mut solver).unwrap();
        assert_eq!(metrics.scenario, kind.name());
        assert!(metrics.total_wall_time >= 0.0);
    }
}

// ─── Metrics Tests ────────────────────────────────────────────

#[test]
fn metrics_csv_output() {
    let metrics = BenchmarkMetrics {
        scenario: "test".into(),
        total_wall_time: 1.5,
        timesteps: 100,
        avg_step_time: 0.015,
        min_step_time: 0.01,
        max_step_time: 0.02,
        final_kinetic_energy: 1e-5,
        max_displacement: 0.5,
        avg_iterations: 10.0,
        vertex_count: 441,
        triangle_count: 800,
    };

    let csv_row = metrics.to_csv_row();
    assert!(csv_row.contains("test"));
    assert!(csv_row.contains("441"));
    assert!(csv_row.contains("800"));
}

#[test]
fn metrics_csv_multi() {
    let m1 = BenchmarkMetrics {
        scenario: "a".into(),
        total_wall_time: 1.0,
        timesteps: 10,
        avg_step_time: 0.1,
        min_step_time: 0.05,
        max_step_time: 0.15,
        final_kinetic_energy: 0.0,
        max_displacement: 0.0,
        avg_iterations: 0.0,
        vertex_count: 4,
        triangle_count: 2,
    };
    let csv = BenchmarkMetrics::to_csv(&[m1]);
    let lines: Vec<&str> = csv.lines().collect();
    assert_eq!(lines.len(), 2); // Header + 1 data row
    assert!(lines[0].contains("scenario"));
}

#[test]
fn metrics_json_round_trip() {
    let metrics = BenchmarkMetrics {
        scenario: "test".into(),
        total_wall_time: 1.0,
        timesteps: 10,
        avg_step_time: 0.1,
        min_step_time: 0.05,
        max_step_time: 0.15,
        final_kinetic_energy: 1e-3,
        max_displacement: 0.25,
        avg_iterations: 5.0,
        vertex_count: 100,
        triangle_count: 180,
    };
    let json = serde_json::to_string(&metrics).unwrap();
    let recovered: BenchmarkMetrics = serde_json::from_str(&json).unwrap();
    assert_eq!(recovered.timesteps, 10);
}
