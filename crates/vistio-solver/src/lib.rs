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

pub mod config;
pub mod pd_stub;
pub mod state;
pub mod strategy;

pub use config::SolverConfig;
pub use pd_stub::ProjectiveDynamicsStub;
pub use state::SimulationState;
pub use strategy::SolverStrategy;
