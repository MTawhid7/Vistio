//! # vistio-contact
//!
//! Collision detection and contact response for garment simulation.
//!
//! The collision pipeline is split into three phases:
//! 1. **Broad phase** — Spatial acceleration (spatial hash / BVH)
//! 2. **Narrow phase** — Exact proximity/penetration tests
//! 3. **Contact response** — Position correction / barrier forces
//!
//! Each phase is a pluggable trait, enabling different strategies
//! (e.g., spatial hash for self-collision, BVH for cloth-body).

pub mod broad;
pub mod contact;
pub mod narrow;
pub mod projection;
pub mod response;
pub mod spatial_hash;

pub use broad::BroadPhase;
pub use contact::{ContactPair, ContactType};
pub use narrow::NarrowPhase;
pub use projection::ProjectionContactResponse;
pub use response::ContactResponse;
pub use spatial_hash::SpatialHash;
