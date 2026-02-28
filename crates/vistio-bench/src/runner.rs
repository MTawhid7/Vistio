//! Benchmark runner — executes scenarios with a solver and collects metrics.

use std::time::Instant;

use vistio_contact::CollisionPipeline;
use vistio_material::CoRotationalModel;
use vistio_mesh::topology::Topology;
use vistio_solver::pd_solver::ProjectiveDynamicsSolver;
use vistio_solver::state::SimulationState;
use vistio_solver::strategy::{SolverStrategy, StepResult};
use vistio_types::VistioResult;
use vistio_contact::{SpatialHash, VertexTriangleTest, ProjectionContactResponse};

use crate::metrics::BenchmarkMetrics;
use crate::scenarios::Scenario;

/// Runs benchmark scenarios and collects metrics.
pub struct BenchmarkRunner;

impl BenchmarkRunner {
    /// Run a single scenario with the given solver.
    ///
    /// If the scenario has material properties set, uses `init_with_material()`
    /// for material-aware simulation. Otherwise falls back to standard `init()`.
    pub fn run(
        scenario: &Scenario,
        solver: &mut ProjectiveDynamicsSolver,
    ) -> VistioResult<BenchmarkMetrics> {
        use crate::scenarios::ScenarioKind;

        let mut pipeline = CollisionPipeline::new(
            Box::new(SpatialHash::new(0.05)),
            Box::new(VertexTriangleTest),
            Box::new(ProjectionContactResponse),
            scenario.garment.clone(),
            0.01, // thickness
            1.0,  // stiffness
        );

        match scenario.kind {
            ScenarioKind::SphereDrape => {
                pipeline = pipeline
                    .with_ground(-0.3)
                    .with_sphere(vistio_math::Vec3::new(0.0, 0.0, 0.0), 0.3);
            },
            _ => {
                pipeline = pipeline.with_ground(-0.3);
            }
        }

        Self::run_with_collision(scenario, solver, Some(pipeline))
    }

    /// Run a scenario with an optional collision pipeline.
    ///
    /// When a `CollisionPipeline` is provided, it is called after each solver
    /// step to detect and resolve contacts before the next timestep.
    pub fn run_with_collision(
        scenario: &Scenario,
        solver: &mut ProjectiveDynamicsSolver,
        mut collision: Option<CollisionPipeline>,
    ) -> VistioResult<BenchmarkMetrics> {
        let topology = Topology::build(&scenario.garment);

        // Choose init path based on whether a material is specified
        let vertex_mass = if let Some(ref properties) = scenario.material {
            // Tier 3 path: when material is anisotropic, use Discrete Shells + anisotropic model
            // Tier 2 path: when material is isotropic, use dihedral + co-rotational model
            if properties.is_anisotropic() {
                let model = Box::new(vistio_material::AnisotropicCoRotationalModel::from_properties(properties));
                solver.init_with_material_tier3(
                    &scenario.garment,
                    &topology,
                    &scenario.config,
                    properties,
                    model,
                    &scenario.pinned,
                )?;
            } else {
                let model = Box::new(CoRotationalModel::new());
                solver.init_with_material(
                    &scenario.garment,
                    &topology,
                    &scenario.config,
                    properties,
                    model,
                    &scenario.pinned,
                )?;
            }
            // Derive mass from material
            let total_area = compute_mesh_area(&scenario.garment);
            properties.mass_per_vertex(scenario.garment.vertex_count(), total_area)
        } else {
            // Tier 1 path: standard initialization
            solver.init(&scenario.garment, &topology, &scenario.config, &scenario.pinned)?;
            scenario.vertex_mass
        };

        // Initialize simulation state
        let mut state = SimulationState::from_mesh(
            &scenario.garment,
            vertex_mass,
            &scenario.pinned,
        )?;

        // Wire ground height into the solver's internal constraint enforcement
        if let Some(ref pipeline) = collision {
            if let Some(ref ground) = pipeline.ground {
                state.ground_height = Some(ground.height);
            }
        }

        // Save initial positions for displacement tracking
        let initial_y: Vec<f32> = state.pos_y.clone();

        let mut step_times: Vec<f64> = Vec::with_capacity(scenario.timesteps as usize);
        let mut total_iterations: u32 = 0;

        let total_start = Instant::now();

        // Run timesteps
        for _ in 0..scenario.timesteps {
            // Solver step (PD local-global iterations)
            let result: StepResult = solver.step(&mut state, scenario.dt)?;
            step_times.push(result.wall_time);
            total_iterations += result.iterations;

            // Collision step (after solver resolves elastic forces)
            if let Some(ref mut pipeline) = collision {
                let _ = pipeline.step(&mut state)?;
            }
        }

        let total_wall_time = total_start.elapsed().as_secs_f64();

        // Compute final metrics
        let final_ke = state.kinetic_energy();

        let max_displacement = (0..state.vertex_count)
            .map(|i| {
                let dx = state.pos_x[i] - scenario.garment.pos_x[i];
                let dy = state.pos_y[i] - initial_y[i];
                let dz = state.pos_z[i] - scenario.garment.pos_z[i];
                (dx * dx + dy * dy + dz * dz).sqrt()
            })
            .fold(0.0f32, f32::max);

        // --- DEBUG DUMP: Write final mesh out! ---
        if scenario.kind == crate::scenarios::ScenarioKind::HangingSheet {
            use std::io::Write;
            if let Ok(mut f) = std::fs::File::create("/tmp/end_mesh.obj") {
                for i in 0..state.vertex_count {
                    let _ = writeln!(f, "v {} {} {}", state.pos_x[i], state.pos_y[i], state.pos_z[i]);
                }
                for t in 0..scenario.garment.triangle_count() {
                    let idx = t * 3;
                    let i0 = scenario.garment.indices[idx] + 1;
                    let i1 = scenario.garment.indices[idx + 1] + 1;
                    let i2 = scenario.garment.indices[idx + 2] + 1;
                    let _ = writeln!(f, "f {} {} {}", i0, i1, i2);
                }
            }
        }
        // -----------------------------------------

        let avg_step = if step_times.is_empty() {
            0.0
        } else {
            step_times.iter().sum::<f64>() / step_times.len() as f64
        };
        let min_step = step_times.iter().copied().fold(f64::MAX, f64::min);
        let max_step = step_times.iter().copied().fold(0.0, f64::max);
        let avg_iter = if scenario.timesteps > 0 {
            total_iterations as f32 / scenario.timesteps as f32
        } else {
            0.0
        };

        Ok(BenchmarkMetrics {
            scenario: scenario.kind.name().to_string(),
            total_wall_time,
            timesteps: scenario.timesteps,
            avg_step_time: avg_step,
            min_step_time: min_step,
            max_step_time: max_step,
            final_kinetic_energy: final_ke,
            max_displacement,
            avg_iterations: avg_iter,
            vertex_count: scenario.garment.vertex_count(),
            triangle_count: scenario.garment.triangle_count(),
        })
    }

    /// Run all scenarios and return metrics for each.
    pub fn run_all(
        solver: &mut ProjectiveDynamicsSolver,
    ) -> VistioResult<Vec<BenchmarkMetrics>> {
        use crate::scenarios::ScenarioKind;
        let mut results = Vec::new();
        for &kind in ScenarioKind::all() {
            let scenario = Scenario::from_kind(kind);
            let metrics = Self::run(&scenario, solver)?;
            results.push(metrics);
        }
        Ok(results)
    }
}

/// Compute the total surface area of a triangle mesh.
fn compute_mesh_area(mesh: &vistio_mesh::TriangleMesh) -> f32 {
    use vistio_math::Vec3;
    let tri_count = mesh.triangle_count();
    let mut total = 0.0_f32;
    for t in 0..tri_count {
        let idx_base = t * 3;
        let i0 = mesh.indices[idx_base] as usize;
        let i1 = mesh.indices[idx_base + 1] as usize;
        let i2 = mesh.indices[idx_base + 2] as usize;
        let p0 = Vec3::new(mesh.pos_x[i0], mesh.pos_y[i0], mesh.pos_z[i0]);
        let p1 = Vec3::new(mesh.pos_x[i1], mesh.pos_y[i1], mesh.pos_z[i1]);
        let p2 = Vec3::new(mesh.pos_x[i2], mesh.pos_y[i2], mesh.pos_z[i2]);
        let e1 = p1 - p0;
        let e2 = p2 - p0;
        total += 0.5 * e1.cross(e2).length();
    }
    total
}
