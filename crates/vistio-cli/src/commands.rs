//! CLI command implementations.

use vistio_bench::metrics::BenchmarkMetrics;
use vistio_bench::runner::BenchmarkRunner;
use vistio_bench::scenarios::{Scenario, ScenarioKind};
use vistio_debug::snapshot::StateSnapshot;
use vistio_solver::pd_solver::ProjectiveDynamicsSolver;

/// Run a simulation from config file.
pub fn simulate(config_path: &str) -> Result<(), Box<dyn std::error::Error>> {
    println!("Vistio Simulation");
    println!("─────────────────");
    println!("Config: {config_path}");
    println!();
    println!("Note: TOML config loading is a Tier-1 feature.");
    println!("Use `vistio benchmark` to run procedural scenarios now.");
    Ok(())
}

/// Run benchmark suite.
pub fn benchmark(
    scenario_name: &str,
    output_path: Option<&str>,
) -> Result<(), Box<dyn std::error::Error>> {
    println!("Vistio Benchmark Suite");
    println!("══════════════════════");
    println!();

    let scenarios: Vec<ScenarioKind> = if scenario_name == "all" {
        ScenarioKind::all().to_vec()
    } else {
        let kind = match scenario_name {
            "hanging_sheet" => ScenarioKind::HangingSheet,
            "sphere_drape" => ScenarioKind::SphereDrape,
            "self_fold" => ScenarioKind::SelfFold,
            other => {
                eprintln!("Unknown scenario: {other}");
                eprintln!("Available: hanging_sheet, sphere_drape, self_fold, all");
                return Err("Unknown scenario".into());
            }
        };
        vec![kind]
    };

    let mut all_metrics = Vec::new();
    let mut solver = ProjectiveDynamicsSolver::new();

    for &kind in &scenarios {
        let scenario = Scenario::from_kind(kind);

        println!("Running: {} ({} verts, {} tris, {} steps)",
            kind.name(),
            scenario.garment.vertex_count(),
            scenario.garment.triangle_count(),
            scenario.timesteps,
        );

        let metrics = BenchmarkRunner::run(&scenario, &mut solver)
            .map_err(|e| format!("Benchmark failed: {e}"))?;

        println!("  Wall time:     {:.3}s", metrics.total_wall_time);
        println!("  Avg step:      {:.3}ms", metrics.avg_step_time * 1000.0);
        println!("  Final KE:      {:.6e}", metrics.final_kinetic_energy);
        println!("  Max displace:  {:.4}m", metrics.max_displacement);
        println!();

        all_metrics.push(metrics);
    }

    // Output CSV
    if let Some(path) = output_path {
        let csv = BenchmarkMetrics::to_csv(&all_metrics);
        std::fs::write(path, &csv)?;
        println!("Results written to: {path}");
    } else {
        // Print to stdout
        println!("CSV Output:");
        println!("{}", BenchmarkMetrics::to_csv(&all_metrics));
    }

    Ok(())
}

/// Inspect a state snapshot.
pub fn inspect(path: &str) -> Result<(), Box<dyn std::error::Error>> {
    println!("Vistio Snapshot Inspector");
    println!("────────────────────────");
    println!();

    let data = std::fs::read(path)?;
    let snapshot = StateSnapshot::from_bytes(&data)
        .map_err(|e| format!("Failed to read snapshot: {e}"))?;

    println!("Timestep:     {}", snapshot.timestep);
    println!("Sim time:     {:.4}s", snapshot.sim_time);
    println!("Vertices:     {}", snapshot.vertex_count);
    println!("Pos entries:  {}", snapshot.positions.len());
    println!("Vel entries:  {}", snapshot.velocities.len());

    // Quick stats
    if !snapshot.positions.is_empty() {
        let min_y = snapshot.positions.iter()
            .enumerate()
            .filter(|(i, _)| i % 3 == 1) // Y components
            .map(|(_, v)| *v)
            .fold(f32::INFINITY, f32::min);
        let max_y = snapshot.positions.iter()
            .enumerate()
            .filter(|(i, _)| i % 3 == 1)
            .map(|(_, v)| *v)
            .fold(f32::NEG_INFINITY, f32::max);
        println!("Y range:      [{:.4}, {:.4}]", min_y, max_y);
    }

    Ok(())
}

/// Validate a mesh or config.
pub fn validate(path: &str) -> Result<(), Box<dyn std::error::Error>> {
    println!("Vistio Validator");
    println!("────────────────");
    println!();

    if path.ends_with(".toml") {
        println!("Validating config: {path}");
        let content = std::fs::read_to_string(path)?;
        let _config: vistio_solver::SolverConfig = toml::from_str(&content)?;
        println!("✅ Config is valid.");
    } else if path.ends_with(".json") {
        println!("Validating mesh: {path}");
        let content = std::fs::read_to_string(path)?;
        let mesh: vistio_mesh::TriangleMesh = serde_json::from_str(&content)?;
        match mesh.validate() {
            Ok(()) => println!("✅ Mesh is valid ({} verts, {} tris).", mesh.vertex_count(), mesh.triangle_count()),
            Err(e) => println!("❌ Mesh validation failed: {e}"),
        }
    } else {
        println!("Unsupported file format. Use .toml (config) or .json (mesh).");
    }

    Ok(())
}
