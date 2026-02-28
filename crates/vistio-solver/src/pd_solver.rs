//! Projective Dynamics solver — the core Tier 1/2/3 solver.
//!
//! Implements the local-global iteration loop:
//! 1. **Predict** — inertial position from velocity + gravity
//! 2. **Local step** — project each element toward its closest valid configuration
//!    (membrane + bending projections)
//! 3. **Global step** — solve the constant SPD system A * q = rhs
//! 4. **Repeat** steps 2–3 until convergence or max iterations
//! 5. **Finalize** — update velocities from position change
//!
//! ## Material-Aware Mode (Tier 2+)
//!
//! When initialized via `init_with_material()`, the solver uses a pluggable
//! `ConstitutiveModel` for the local step and derives stiffness/mass from
//! `FabricProperties`. This produces material-specific drape behavior.
//!
//! ## Discrete Shells Bending (Tier 3)
//!
//! Uses the cotangent-weighted Discrete Shells model (Grinspun 2003) for
//! curvature-based bending energy. Bending projections are fully integrated
//! into the local-global loop (not applied as a post-solve correction).

use std::time::Instant;

use vistio_material::ConstitutiveModel;
use vistio_material::FabricProperties;
use vistio_math::faer_solver::FaerSolver;
use vistio_math::sparse::SparseSolver;
use vistio_mesh::TriangleMesh;
use vistio_mesh::topology::Topology;
use vistio_types::VistioResult;

use crate::assembly::{assemble_rhs, assemble_system_matrix, BendingModel, BendingRhs};
use crate::bending::BendingData;
use crate::config::SolverConfig;
use crate::discrete_shells::DiscreteShellsBendingData;
use crate::element::ElementData;
use crate::state::SimulationState;
use crate::strategy::{SolverStrategy, StepResult};

/// Projective Dynamics solver with pluggable constitutive model.
///
/// Uses `faer` for sparse Cholesky factorization. The system matrix is
/// prefactored once during `init()` and reused for all timesteps.
///
/// ## Three initialization modes
///
/// - `init()` — Tier 1 mode: hardcoded ARAP projection, uniform mass, dihedral bending
/// - `init_with_material()` — Tier 2 mode: uses `ConstitutiveModel` + `FabricProperties`
///   with dihedral bending
/// - `init_with_material_tier3()` — Tier 3 mode: uses `ConstitutiveModel` + `FabricProperties`
///   with Discrete Shells bending (cotangent-weighted, fully integrated into local-global loop)
pub struct ProjectiveDynamicsSolver {
    /// Precomputed FEM element data.
    elements: Option<ElementData>,
    /// Legacy dihedral bending data (Tier 1-2).
    bending: Option<BendingData>,
    /// Discrete Shells bending data (Tier 3+).
    ds_bending: Option<DiscreteShellsBendingData>,
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
    /// Optional pluggable constitutive model (Tier 2+).
    /// When `Some`, the local step delegates to this model.
    /// When `None`, falls back to hardcoded ARAP.
    material_model: Option<Box<dyn ConstitutiveModel>>,
}

impl ProjectiveDynamicsSolver {
    /// Creates a new solver (uninitialized).
    pub fn new() -> Self {
        Self {
            elements: None,
            bending: None,
            ds_bending: None,
            solver: FaerSolver::new(),
            config: SolverConfig::default(),
            initialized: false,
            mass: Vec::new(),
            n: 0,
            material_model: None,
        }
    }

    /// Initialize the solver with material properties and a constitutive model.
    ///
    /// This is the Tier 2 initialization path. It:
    /// - Derives per-element stiffness from `FabricProperties`
    /// - Derives per-vertex mass from the material's areal density
    /// - Derives bending stiffness from the material's bending properties
    /// - Uses dihedral bending (legacy) with the provided `ConstitutiveModel` in the local step
    pub fn init_with_material(
        &mut self,
        mesh: &TriangleMesh,
        topology: &Topology,
        config: &SolverConfig,
        properties: &FabricProperties,
        model: Box<dyn ConstitutiveModel>,
        pinned: &[bool],
    ) -> VistioResult<()> {
        self.n = mesh.vertex_count();
        self.config = config.clone();

        // Build FEM elements with material-derived stiffness
        let elements = ElementData::from_mesh_with_material(mesh, properties);

        // Compute area-weighted lumped mass matrix
        self.mass = compute_lumped_masses(self.n, &elements, properties.density, pinned);

        // Build dihedral bending elements with material-derived stiffness
        let bending = BendingData::from_topology_with_material(mesh, topology, properties);
        self.bending = Some(bending);
        self.ds_bending = None;

        // Assemble the constant system matrix (includes bending stiffness)
        let dt = 1.0 / 60.0;
        let bending_model = self.bending.as_ref().map(BendingModel::Dihedral);
        let system_matrix = assemble_system_matrix(self.n, &self.mass, dt, &elements, bending_model);

        // Prefactor via sparse Cholesky
        self.solver.factorize(&system_matrix).map_err(|e| {
            vistio_types::VistioError::InvalidConfig(format!("Cholesky factorization failed: {e}"))
        })?;

        self.elements = Some(elements);
        self.material_model = Some(model);
        self.initialized = true;
        Ok(())
    }

    /// Initialize the solver with Tier 3 Discrete Shells bending.
    ///
    /// This is the Tier 3 initialization path. It:
    /// - Derives per-element stiffness from `FabricProperties`
    /// - Derives per-vertex mass from the material's areal density
    /// - Uses **Discrete Shells** (cotangent-weighted) bending model
    /// - Bending projections are fully integrated into the PD local-global loop
    pub fn init_with_material_tier3(
        &mut self,
        mesh: &TriangleMesh,
        topology: &Topology,
        config: &SolverConfig,
        properties: &FabricProperties,
        model: Box<dyn ConstitutiveModel>,
        pinned: &[bool],
    ) -> VistioResult<()> {
        self.n = mesh.vertex_count();
        self.config = config.clone();

        // Build FEM elements with material-derived stiffness
        let elements = ElementData::from_mesh_with_material(mesh, properties);

        // Compute area-weighted lumped mass matrix
        self.mass = compute_lumped_masses(self.n, &elements, properties.density, pinned);

        // Build Discrete Shells bending elements
        let ds_bending = if properties.is_anisotropic() {
            DiscreteShellsBendingData::from_topology_with_anisotropic_material(mesh, topology, properties)
        } else {
            DiscreteShellsBendingData::from_topology_with_material(mesh, topology, properties)
        };
        self.ds_bending = Some(ds_bending);
        self.bending = None;

        // Assemble the constant system matrix with Discrete Shells stencils
        let dt = 1.0 / 60.0;
        let bending_model = self.ds_bending.as_ref().map(BendingModel::DiscreteShells);
        let system_matrix = assemble_system_matrix(self.n, &self.mass, dt, &elements, bending_model);

        // Prefactor via sparse Cholesky
        self.solver.factorize(&system_matrix).map_err(|e| {
            vistio_types::VistioError::InvalidConfig(format!("Cholesky factorization failed: {e}"))
        })?;

        self.elements = Some(elements);
        self.material_model = Some(model);
        self.initialized = true;
        Ok(())
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
        pinned: &[bool],
    ) -> VistioResult<()> {
        self.n = mesh.vertex_count();
        self.config = config.clone();

        // Build FEM elements with stiffness from config
        let elements = ElementData::from_mesh(mesh, config.stretch_weight * 1000.0);

        // Default density 200 g/m² for the base config
        let density = 200.0;
        self.mass = compute_lumped_masses(self.n, &elements, density, pinned);

        // Build bending elements from topology (legacy dihedral)
        let bending = BendingData::from_topology(mesh, topology, config.bending_weight * 100.0);
        self.bending = Some(bending);
        self.ds_bending = None;

        // Assemble the constant system matrix (includes bending stiffness)
        let dt = 1.0 / 60.0;
        let bending_model = self.bending.as_ref().map(BendingModel::Dihedral);
        let system_matrix = assemble_system_matrix(self.n, &self.mass, dt, &elements, bending_model);

        // Prefactor via sparse Cholesky
        self.solver.factorize(&system_matrix).map_err(|e| {
            vistio_types::VistioError::InvalidConfig(format!("Cholesky factorization failed: {e}"))
        })?;

        self.elements = Some(elements);
        self.material_model = None;
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
            // === LOCAL STEP: MEMBRANE ===
            let mut proj_x = Vec::with_capacity(elements.len());
            let mut proj_y = Vec::with_capacity(elements.len());
            let mut proj_z = Vec::with_capacity(elements.len());

            for elem in &elements.elements {
                let (p0, p1, p2) = if let Some(ref model) = self.material_model {
                    elements.project_with_model(
                        elem,
                        &state.pos_x,
                        &state.pos_y,
                        &state.pos_z,
                        model.as_ref(),
                    )
                } else {
                    elements.project(
                        elem,
                        &state.pos_x,
                        &state.pos_y,
                        &state.pos_z,
                    )
                };
                proj_x.push((p0.x, p1.x, p2.x));
                proj_y.push((p0.y, p1.y, p2.y));
                proj_z.push((p0.z, p1.z, p2.z));
            }

            // === LOCAL STEP: BENDING ===
            // Compute bending projection targets for the RHS assembly.
            // Both dihedral (Tier 1-2) and Discrete Shells (Tier 3) bending are
            // fully integrated into the local-global loop via the RHS.
            let (bend_targets_x, bend_targets_y, bend_targets_z) =
                if let Some(ref ds_bend) = self.ds_bending {
                    // Discrete Shells path
                    let mut btx = Vec::with_capacity(ds_bend.len());
                    let mut bty = Vec::with_capacity(ds_bend.len());
                    let mut btz = Vec::with_capacity(ds_bend.len());

                    for elem in &ds_bend.elements {
                        let (p_v0, p_v1, p_wa, p_wb) = ds_bend.project(
                            elem,
                            &state.pos_x,
                            &state.pos_y,
                            &state.pos_z,
                        );
                        btx.push((p_v0.x, p_v1.x, p_wa.x, p_wb.x));
                        bty.push((p_v0.y, p_v1.y, p_wa.y, p_wb.y));
                        btz.push((p_v0.z, p_v1.z, p_wa.z, p_wb.z));
                    }

                    (Some(btx), Some(bty), Some(btz))
                } else if let Some(ref bending) = self.bending {
                    // Dihedral path — compute projections for RHS integration
                    let mut btx = Vec::with_capacity(bending.len());
                    let mut bty = Vec::with_capacity(bending.len());
                    let mut btz = Vec::with_capacity(bending.len());

                    for elem in &bending.elements {
                        let (p_v0, p_v1, p_wa, p_wb) = bending.project(
                            elem,
                            &state.pos_x,
                            &state.pos_y,
                            &state.pos_z,
                        );
                        btx.push((p_v0.x, p_v1.x, p_wa.x, p_wb.x));
                        bty.push((p_v0.y, p_v1.y, p_wa.y, p_wb.y));
                        btz.push((p_v0.z, p_v1.z, p_wa.z, p_wb.z));
                    }

                    (Some(btx), Some(bty), Some(btz))
                } else {
                    (None, None, None)
                };

            // === GLOBAL STEP ===
            // Build bending RHS references
            let bending_rhs_x = self.build_bending_rhs(
                bend_targets_x.as_deref(),
            );
            let bending_rhs_y = self.build_bending_rhs(
                bend_targets_y.as_deref(),
            );
            let bending_rhs_z = self.build_bending_rhs(
                bend_targets_z.as_deref(),
            );

            let rhs_x = assemble_rhs(
                n, &self.mass, dt, &state.pred_x, &proj_x, elements, 0, bending_rhs_x,
            );
            let rhs_y = assemble_rhs(
                n, &self.mass, dt, &state.pred_y, &proj_y, elements, 1, bending_rhs_y,
            );
            let rhs_z = assemble_rhs(
                n, &self.mass, dt, &state.pred_z, &proj_z, elements, 2, bending_rhs_z,
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

            // Enforce ground plane constraint
            state.enforce_ground();

            iterations = iter + 1;

            if final_residual < self.config.tolerance {
                break;
            }
        }

        // 5. Update velocities from position difference
        state.update_velocities(dt);

        // 6. Enforce ground velocity constraints
        state.enforce_ground_velocities();

        // 7. Apply basic damping
        state.damp_velocities(self.config.damping);

        // 8. Rayleigh mass-proportional damping: v *= 1 / (1 + α_M * dt)
        if self.config.rayleigh_mass_damping > 0.0 {
            let factor = 1.0 / (1.0 + self.config.rayleigh_mass_damping * dt);
            for i in 0..n {
                if state.inv_mass[i] > 0.0 {
                    state.vel_x[i] *= factor;
                    state.vel_y[i] *= factor;
                    state.vel_z[i] *= factor;
                }
            }
        }

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

impl ProjectiveDynamicsSolver {
    /// Build the bending RHS enum from optional targets, referencing internal bending data.
    fn build_bending_rhs<'a>(
        &'a self,
        targets: Option<&'a [(f32, f32, f32, f32)]>,
    ) -> Option<BendingRhs<'a>> {
        let targets = targets?;
        if let Some(ref ds_bend) = self.ds_bending {
            Some(BendingRhs::DiscreteShells {
                data: ds_bend,
                targets,
            })
        } else if let Some(ref bending) = self.bending {
            Some(BendingRhs::Dihedral {
                data: bending,
                targets,
            })
        } else {
            None
        }
    }
}

/// Compute an area-weighted lumped mass matrix (stored as a vector).
/// Distributes 1/3 of each triangle's mass to its three vertices.
/// Pinned vertices receive an extremely large mass (1e8) so they are
/// mathematically anchored in the implicit system matrix.
fn compute_lumped_masses(n: usize, elements: &ElementData, density_gsm: f32, pinned: &[bool]) -> Vec<f32> {
    let mut mass = vec![0.0; n];
    let density_kgm2 = density_gsm / 1000.0;

    for elem in &elements.elements {
        let tri_mass = elem.rest_area * density_kgm2;
        let third_mass = tri_mass / 3.0;

        for &idx in &elem.indices {
            mass[idx] += third_mass;
        }
    }

    // Ensure no zero masses for floating vertices, and infinite mass for pinned
    for (i, m) in mass.iter_mut().enumerate() {
        if pinned[i] {
            *m = 1e8; // Infinite mass for the implicit solver
        } else if *m < 1e-8 {
            *m = 1e-8;
        }
    }

    mass
}
