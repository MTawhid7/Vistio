//! Simulation state — SoA buffers for all per-vertex data.
//!
//! This is the primary mutable data structure during simulation.
//! The solver reads and writes these buffers each timestep.

use vistio_mesh::TriangleMesh;
use vistio_types::VistioResult;

/// SoA simulation state buffers.
///
/// Holds all per-vertex mutable data used during the simulation loop.
/// Separate from the mesh topology (which is immutable after initialization).
///
/// # Layout
///
/// All arrays have length `vertex_count`. Channels are stored contiguously
/// for GPU coalescing:
/// ```text
/// pos_x: [x0, x1, x2, ...]
/// pos_y: [y0, y1, y2, ...]
/// ...
/// ```
pub struct SimulationState {
    /// Number of vertices.
    pub vertex_count: usize,

    // ─── Position (current) ───
    pub pos_x: Vec<f32>,
    pub pos_y: Vec<f32>,
    pub pos_z: Vec<f32>,

    // ─── Velocity ───
    pub vel_x: Vec<f32>,
    pub vel_y: Vec<f32>,
    pub vel_z: Vec<f32>,

    // ─── Previous position (for Verlet-style updates) ───
    pub prev_x: Vec<f32>,
    pub prev_y: Vec<f32>,
    pub prev_z: Vec<f32>,

    // ─── Predicted position (inertial target for PD) ───
    pub pred_x: Vec<f32>,
    pub pred_y: Vec<f32>,
    pub pred_z: Vec<f32>,

    // ─── Per-vertex mass (inverse mass for pinning) ───
    pub mass: Vec<f32>,
    pub inv_mass: Vec<f32>,
}

impl SimulationState {
    /// Initialize simulation state from a mesh and per-vertex mass.
    ///
    /// Copies positions from the mesh. Velocities initialized to zero.
    /// Pinned vertices get `inv_mass = 0.0` (infinite mass).
    pub fn from_mesh(
        mesh: &TriangleMesh,
        vertex_mass: f32,
        pinned: &[bool],
    ) -> VistioResult<Self> {
        let n = mesh.vertex_count();

        if pinned.len() != n {
            return Err(vistio_types::VistioError::InvalidMesh(format!(
                "Pinned array length ({}) != vertex count ({})",
                pinned.len(),
                n
            )));
        }

        let mut mass = vec![vertex_mass; n];
        let mut inv_mass = vec![1.0 / vertex_mass; n];

        for i in 0..n {
            if pinned[i] {
                mass[i] = f32::MAX;
                inv_mass[i] = 0.0;
            }
        }

        Ok(Self {
            vertex_count: n,
            pos_x: mesh.pos_x.clone(),
            pos_y: mesh.pos_y.clone(),
            pos_z: mesh.pos_z.clone(),
            vel_x: vec![0.0; n],
            vel_y: vec![0.0; n],
            vel_z: vec![0.0; n],
            prev_x: mesh.pos_x.clone(),
            prev_y: mesh.pos_y.clone(),
            prev_z: mesh.pos_z.clone(),
            pred_x: vec![0.0; n],
            pred_y: vec![0.0; n],
            pred_z: vec![0.0; n],
            mass,
            inv_mass,
        })
    }

    /// Compute predicted positions: p_pred = p + dt * v + dt² * gravity.
    ///
    /// This is the inertial target for the Projective Dynamics global step.
    pub fn predict(&mut self, dt: f32, gravity: [f32; 3]) {
        let dt2 = dt * dt;
        for i in 0..self.vertex_count {
            if self.inv_mass[i] == 0.0 {
                // Pinned: predicted = current
                self.pred_x[i] = self.pos_x[i];
                self.pred_y[i] = self.pos_y[i];
                self.pred_z[i] = self.pos_z[i];
                continue;
            }

            self.pred_x[i] = self.pos_x[i] + dt * self.vel_x[i] + dt2 * gravity[0];
            self.pred_y[i] = self.pos_y[i] + dt * self.vel_y[i] + dt2 * gravity[1];
            self.pred_z[i] = self.pos_z[i] + dt * self.vel_z[i] + dt2 * gravity[2];
        }
    }

    /// Update velocities from position change: v = (p_new - p_old) / dt.
    pub fn update_velocities(&mut self, dt: f32) {
        let inv_dt = 1.0 / dt;
        for i in 0..self.vertex_count {
            self.vel_x[i] = (self.pos_x[i] - self.prev_x[i]) * inv_dt;
            self.vel_y[i] = (self.pos_y[i] - self.prev_y[i]) * inv_dt;
            self.vel_z[i] = (self.pos_z[i] - self.prev_z[i]) * inv_dt;
        }
    }

    /// Save current positions as "previous" for the next timestep.
    pub fn save_previous(&mut self) {
        self.prev_x.copy_from_slice(&self.pos_x);
        self.prev_y.copy_from_slice(&self.pos_y);
        self.prev_z.copy_from_slice(&self.pos_z);
    }

    /// Apply velocity damping: v *= (1 - damping).
    pub fn damp_velocities(&mut self, damping: f32) {
        let factor = 1.0 - damping;
        for i in 0..self.vertex_count {
            self.vel_x[i] *= factor;
            self.vel_y[i] *= factor;
            self.vel_z[i] *= factor;
        }
    }

    /// Compute total kinetic energy: 0.5 * Σ m_i * ||v_i||².
    pub fn kinetic_energy(&self) -> f64 {
        let mut energy = 0.0f64;
        for i in 0..self.vertex_count {
            let vx = self.vel_x[i] as f64;
            let vy = self.vel_y[i] as f64;
            let vz = self.vel_z[i] as f64;
            let m = self.mass[i] as f64;
            if m < f32::MAX as f64 {
                energy += 0.5 * m * (vx * vx + vy * vy + vz * vz);
            }
        }
        energy
    }
}
