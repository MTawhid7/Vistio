//! # vistio-render
//!
//! Pluggable rendering abstraction for Vistio.
//!
//! Provides a `Renderer` trait with a `HeadlessRenderer` stub.
//! Future implementations: wgpu visualizer, Rerun sink, glTF exporter.

pub mod renderer;

pub use renderer::{HeadlessRenderer, RenderFrame, Renderer};
