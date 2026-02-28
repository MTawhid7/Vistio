//! # vistio-solver
//!
//! Time integration, solver strategies, and simulation state management.
//!
//! ## Key Types
//!
//! - [`SimulationState`] — SoA buffers for positions, velocities, masses
//! - [`SolverStrategy`] — Pluggable solver trait (PD, implicit, XPBD)
//! - [`SolverConfig`] — Solver-specific configuration
//! - [`ProjectiveDynamicsStub`] — Tier-0 stub solver (no-op local step)
//! - [`element::ElementData`] — FEM element precomputation (Tier 1)

pub mod assembly;
pub mod bending;
pub mod config;
pub mod discrete_shells;
pub mod element;
pub mod pd_solver;
pub mod pd_stub;
pub mod state;
pub mod strategy;

pub use config::SolverConfig;
pub use pd_solver::ProjectiveDynamicsSolver;
pub use pd_stub::ProjectiveDynamicsStub;
pub use state::SimulationState;
pub use strategy::SolverStrategy;
