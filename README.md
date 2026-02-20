# Vistio

**GPU-accelerated garment physics simulation engine.**

Built in Rust with a modular crate architecture, Vistio targets production-grade drape quality using Projective Dynamics, pluggable constitutive models, IPC-class contact handling, and (planned) `wgpu` compute shaders.

> *Vistio* — from Italian *"vestito"* (dress/garment).

---

## Status

✅ **Tier 0 — Foundation & Interface Contract** — Complete

| Gate | Result |
| --- | --- |
| Build | ✅ 13 crates |
| Tests | ✅ 111 pass |
| Clippy | ✅ 0 warnings |
| CLI | ✅ 4 subcommands |

### What's Implemented

- **Core types** — Strongly-typed IDs, error handling, physical constants
- **Linear algebra** — 3×2 deformation gradients, polar decomposition, CSR sparse matrices
- **Mesh system** — SoA TriangleMesh, procedural generators (quad grid, UV sphere), topology queries, vertex normals
- **Materials** — ConstitutiveModel trait, FabricProperties (KES-mapped), 5 built-in presets
- **Solver** — SolverStrategy trait, SimulationState (SoA buffers), PD stub solver, SolverConfig
- **Contact** — BroadPhase / NarrowPhase / ContactResponse trait pipeline
- **GPU abstraction** — GpuBackend trait, CpuFallback (axpy, dot, fill), ComputeBuffer
- **Telemetry** — EventBus (mpsc), 7 event types, pluggable sinks
- **Debug** — InspectionHook trait, state snapshots with bincode serialization
- **Benchmarks** — 3 procedural scenarios, metrics collection, CSV export
- **Rendering** — Renderer trait, HeadlessRenderer (visual rendering planned for Tier 2)
- **CLI** — `simulate`, `benchmark`, `inspect`, `validate` subcommands

### What's Not Yet Implemented

- **Real PD solver** — local-global iteration loop (Tier 1)
- **Collision detection** — spatial hash, BVH, vertex-triangle tests (Tier 1-2)
- **GPU compute** — wgpu backend with WGSL shaders (Tier 1-2)
- **Live visual simulation** — real-time wgpu renderer (Tier 2)
- **IPC barriers** — Incremental Potential Contact (Tier 3)
- **Adaptive remeshing** — dynamic mesh refinement (Tier 4)

---

## Architecture

```
vistio/
├── Cargo.toml           # Workspace root
└── crates/
    ├── vistio-types/     # IDs, errors, constants
    ├── vistio-math/      # Mat3x2, polar decomp, CSR sparse
    ├── vistio-mesh/      # SoA TriangleMesh, generators, topology
    ├── vistio-material/  # ConstitutiveModel, 5 fabric presets
    ├── vistio-solver/    # SolverStrategy, SimulationState, PD stub
    ├── vistio-contact/   # BroadPhase/NarrowPhase/ContactResponse
    ├── vistio-gpu/       # GpuBackend trait, CpuFallback
    ├── vistio-telemetry/ # EventBus, event types, sinks
    ├── vistio-debug/     # InspectionHook, StateSnapshot
    ├── vistio-bench/     # 3 benchmark scenarios, metrics
    ├── vistio-render/    # Renderer trait, HeadlessRenderer
    ├── vistio-io/        # SimulationInput/Output, validator
    └── vistio-cli/       # CLI binary (clap)
```

| Layer | Crates | Purpose |
| --- | --- | --- |
| **Foundation** | `types`, `math` | Shared types, linear algebra |
| **Core** | `mesh`, `material`, `solver`, `contact`, `gpu` | Simulation engine |
| **Infrastructure** | `bench`, `debug`, `telemetry` | Testing & monitoring |
| **Interface** | `io`, `render`, `cli` | I/O, rendering, CLI |

---

## Getting Started

### Prerequisites

- Rust 1.83+ (stable)
- Cargo

### Build

```bash
cargo build --workspace
```

### Test

```bash
# Run all 111 tests
cargo test --workspace

# Run a single crate's tests
cargo test -p vistio-solver

# Run a single test with output
cargo test -- pd_stub_gravity_motion --nocapture
```

### Lint

```bash
cargo clippy --workspace -- -D warnings
```

### CLI

```bash
# Run all 3 benchmark scenarios
cargo run --bin vistio -- benchmark --scenario all

# Run a single scenario with CSV output
cargo run --bin vistio -- benchmark --scenario hanging_sheet --output results.csv

# Validate a solver config
cargo run --bin vistio -- validate simulation.toml

# Inspect a state snapshot
cargo run --bin vistio -- inspect snapshot.bin
```

---

## Benchmark Scenarios

| Scenario | Description | Mesh | Timesteps |
| --- | --- | --- | --- |
| `hanging_sheet` | 1m² cloth pinned at top edge | 20×20 (441 verts) | 120 (2s) |
| `sphere_drape` | 1.5m² cloth falling onto sphere | 20×20 (441 verts) | 180 (3s) |
| `self_fold` | Corner-pinned cloth folding | 20×10 (231 verts) | 120 (2s) |

---

## Material Presets

| Preset | Density (g/m²) | Use Case |
| --- | --- | --- |
| `chiffon` | 40 | Sheer, lightweight drape |
| `silk_charmeuse` | 80 | Flowing formal wear |
| `jersey_knit` | 160 | Stretchy casual wear |
| `cotton_twill` | 200 | Structured garments |
| `denim_14oz` | 400 | Heavy, stiff jeans |

---

## Testing Guidelines

All tests live in `crates/<name>/tests/<name>_tests.rs`. No inline `#[cfg(test)]` blocks.

| Category | Example |
| --- | --- |
| Construction | `state_from_mesh`, `buffer_zeros` |
| Invariants | `validate_catches_oob_index` |
| Behavior | `pd_stub_gravity_motion` |
| Edge cases | `polar_degenerate_does_not_panic` |
| Serialization | `config_serialization` |

---

## Technology Stack

| Component | Technology |
| --- | --- |
| Language | Rust (stable 1.83+) |
| Math | `glam` (SIMD-accelerated) |
| GPU (planned) | `wgpu` + WGSL |
| Sparse LA (planned) | `faer`, CHOLMOD FFI |
| Debug (planned) | Rerun, Tracy |
| Config | TOML + serde |
| CLI | clap 4.x |

---

## Roadmap

| Tier | Focus | Status |
| --- | --- | --- |
| **Tier 0** | Foundation, traits, pipeline | ✅ Complete |
| **Tier 1** | Real PD solver, spatial hash, `faer` | 🔲 Planned |
| **Tier 2** | wgpu shaders, real-time renderer, BVH | 🔲 Planned |
| **Tier 3** | IPC barriers, CCD, anisotropic models | 🔲 Planned |
| **Tier 4** | Adaptive remeshing, implicit solver | 🔲 Planned |

## License

MIT
