//! Projective Dynamics solver — the core Tier 1 solver.
//!
//! Implements the local-global iteration loop:
//! 1. **Predict** — inertial position from velocity + gravity
//! 2. **Local step** — project each element toward its closest valid configuration
//! 3. **Global step** — solve the constant SPD system A * q = rhs
//! 4. **Repeat** steps 2–3 until convergence or max iterations
//! 5. **Finalize** — update velocities from position change

use std::time::Instant;

use vistio_math::faer_solver::FaerSolver;
use vistio_math::sparse::SparseSolver;
use vistio_mesh::TriangleMesh;
use vistio_mesh::topology::Topology;
use vistio_types::VistioResult;

use crate::assembly::{assemble_rhs, assemble_system_matrix};
use crate::bending::BendingData;
use crate::config::SolverConfig;
use crate::element::ElementData;
use crate::state::SimulationState;
use crate::strategy::{SolverStrategy, StepResult};

/// Projective Dynamics solver with co-rotational (ARAP) constitutive model.
///
/// Uses `faer` for sparse Cholesky factorization. The system matrix is
/// prefactored once during `init()` and reused for all timesteps.
pub struct ProjectiveDynamicsSolver {
    /// Precomputed FEM element data.
    elements: Option<ElementData>,
    /// Dihedral bending data.
    bending: Option<BendingData>,
    /// Sparse Cholesky solver with cached factorization.
    solver: FaerSolver,
    /// Configuration snapshot from init().
    config: SolverConfig,
    /// Whether init() has been called successfully.
    initialized: bool,
    /// Cached per-vertex mass.
    mass: Vec<f32>,
    /// Number of vertices.
    n: usize,
}

impl ProjectiveDynamicsSolver {
    /// Creates a new solver (uninitialized).
    pub fn new() -> Self {
        Self {
            elements: None,
            bending: None,
            solver: FaerSolver::new(),
            config: SolverConfig::default(),
            initialized: false,
            mass: Vec::new(),
            n: 0,
        }
    }
}

impl Default for ProjectiveDynamicsSolver {
    fn default() -> Self {
        Self::new()
    }
}

impl SolverStrategy for ProjectiveDynamicsSolver {
    fn init(
        &mut self,
        mesh: &TriangleMesh,
        topology: &Topology,
        config: &SolverConfig,
    ) -> VistioResult<()> {
        self.n = mesh.vertex_count();
        self.config = config.clone();

        // Compute per-vertex mass (uniform for now)
        // TODO: Area-weighted mass in Tier 2
        let total_mass = 1.0_f32; // 1 kg total
        let per_vertex = total_mass / self.n as f32;
        self.mass = vec![per_vertex; self.n];

        // Build FEM elements with stiffness from config
        let elements = ElementData::from_mesh(mesh, config.stretch_weight * 1000.0);

        // Assemble the constant system matrix
        let dt = 1.0 / 60.0; // Default timestep for prefactoring
        let system_matrix = assemble_system_matrix(self.n, &self.mass, dt, &elements);

        // Prefactor via sparse Cholesky
        self.solver.factorize(&system_matrix).map_err(|e| {
            vistio_types::VistioError::InvalidConfig(format!("Cholesky factorization failed: {e}"))
        })?;

        self.elements = Some(elements);

        // Build bending elements from topology
        let bending = BendingData::from_topology(mesh, topology, config.bending_weight * 100.0);
        self.bending = Some(bending);

        self.initialized = true;
        Ok(())
    }

    fn step(
        &mut self,
        state: &mut SimulationState,
        dt: f32,
    ) -> VistioResult<StepResult> {
        if !self.initialized {
            return Err(vistio_types::VistioError::InvalidConfig(
                "Solver not initialized. Call init() first.".into(),
            ));
        }

        let start = Instant::now();
        let elements = self.elements.as_ref().unwrap();
        let n = self.n;

        // 1. Save previous positions
        state.save_previous();

        // 2. Predict: q_pred = pos + dt*vel + dt²*gravity
        state.predict(dt, self.config.gravity);

        // 3. Initialize current positions from predictions (initial guess)
        state.pos_x.copy_from_slice(&state.pred_x);
        state.pos_y.copy_from_slice(&state.pred_y);
        state.pos_z.copy_from_slice(&state.pred_z);

        // Buffers for the global solve
        let mut sol_x = vec![0.0_f32; n];
        let mut sol_y = vec![0.0_f32; n];
        let mut sol_z = vec![0.0_f32; n];

        let mut iterations = 0_u32;
        let mut final_residual = f64::MAX;

        // 4. Local-Global iteration loop
        for iter in 0..self.config.max_iterations {
            // === LOCAL STEP ===
            // For each element, compute the projection (target positions)
            let mut proj_x = Vec::with_capacity(elements.len());
            let mut proj_y = Vec::with_capacity(elements.len());
            let mut proj_z = Vec::with_capacity(elements.len());

            for elem in &elements.elements {
                let (p0, p1, p2) = elements.project(
                    elem,
                    &state.pos_x,
                    &state.pos_y,
                    &state.pos_z,
                );
                proj_x.push((p0.x, p1.x, p2.x));
                proj_y.push((p0.y, p1.y, p2.y));
                proj_z.push((p0.z, p1.z, p2.z));
            }

            // === GLOBAL STEP ===
            // Assemble RHS for each coordinate
            let rhs_x = assemble_rhs(
                n, &self.mass, dt, &state.pred_x, &proj_x, elements, 0,
            );
            let rhs_y = assemble_rhs(
                n, &self.mass, dt, &state.pred_y, &proj_y, elements, 1,
            );
            let rhs_z = assemble_rhs(
                n, &self.mass, dt, &state.pred_z, &proj_z, elements, 2,
            );

            // Solve A * q = rhs (three backsubstitutions)
            self.solver.solve(&rhs_x, &mut sol_x).map_err(|e| {
                vistio_types::VistioError::InvalidConfig(format!("X solve failed: {e}"))
            })?;
            self.solver.solve(&rhs_y, &mut sol_y).map_err(|e| {
                vistio_types::VistioError::InvalidConfig(format!("Y solve failed: {e}"))
            })?;
            self.solver.solve(&rhs_z, &mut sol_z).map_err(|e| {
                vistio_types::VistioError::InvalidConfig(format!("Z solve failed: {e}"))
            })?;

            // Compute convergence: ||q_new - q_old||² / ||q_old||²
            let mut diff_sq = 0.0_f64;
            let mut norm_sq = 0.0_f64;
            for i in 0..n {
                let dx = (sol_x[i] - state.pos_x[i]) as f64;
                let dy = (sol_y[i] - state.pos_y[i]) as f64;
                let dz = (sol_z[i] - state.pos_z[i]) as f64;
                diff_sq += dx * dx + dy * dy + dz * dz;

                let ox = state.pos_x[i] as f64;
                let oy = state.pos_y[i] as f64;
                let oz = state.pos_z[i] as f64;
                norm_sq += ox * ox + oy * oy + oz * oz;
            }

            final_residual = if norm_sq > 1e-12 {
                (diff_sq / norm_sq).sqrt()
            } else {
                diff_sq.sqrt()
            };

            // Update positions
            state.pos_x.copy_from_slice(&sol_x);
            state.pos_y.copy_from_slice(&sol_y);
            state.pos_z.copy_from_slice(&sol_z);

            // Enforce pinning constraints
            for i in 0..n {
                if state.inv_mass[i] == 0.0 {
                    state.pos_x[i] = state.prev_x[i];
                    state.pos_y[i] = state.prev_y[i];
                    state.pos_z[i] = state.prev_z[i];
                }
            }

            // Apply bending corrections (post-solve position adjustment)
            if let Some(ref bending) = self.bending {
                let bend_scale = self.config.bending_weight * dt * dt;
                for elem in &bending.elements {
                    let (_v0, _v1, wa_proj, wb_proj) = bending.project(
                        elem,
                        &state.pos_x,
                        &state.pos_y,
                        &state.pos_z,
                    );

                    // Blend toward bending target
                    let wa = elem.wing_a;
                    let wb = elem.wing_b;
                    if state.inv_mass[wa] > 0.0 {
                        state.pos_x[wa] += bend_scale * (wa_proj.x - state.pos_x[wa]);
                        state.pos_y[wa] += bend_scale * (wa_proj.y - state.pos_y[wa]);
                        state.pos_z[wa] += bend_scale * (wa_proj.z - state.pos_z[wa]);
                    }
                    if state.inv_mass[wb] > 0.0 {
                        state.pos_x[wb] += bend_scale * (wb_proj.x - state.pos_x[wb]);
                        state.pos_y[wb] += bend_scale * (wb_proj.y - state.pos_y[wb]);
                        state.pos_z[wb] += bend_scale * (wb_proj.z - state.pos_z[wb]);
                    }
                }
            }

            iterations = iter + 1;

            if final_residual < self.config.tolerance {
                break;
            }
        }

        // 5. Update velocities from position difference
        state.update_velocities(dt);

        // 6. Apply damping
        state.damp_velocities(self.config.damping);

        let wall_time = start.elapsed().as_secs_f64();

        Ok(StepResult {
            iterations,
            final_residual,
            converged: final_residual < self.config.tolerance,
            wall_time,
        })
    }

    fn name(&self) -> &str {
        "ProjectiveDynamics"
    }
}
