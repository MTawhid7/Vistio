//! Procedural mesh generators for benchmarks and testing.
//!
//! These generators produce deterministic, resolution-configurable meshes
//! with correct winding order and UV coordinates.

use vistio_types::MaterialId;

use crate::mesh::TriangleMesh;

/// Generates a flat rectangular quad grid in the XY plane.
///
/// The grid spans `[-width/2, width/2]` in X and `[-height/2, height/2]` in Y,
/// centered at the origin at Z=0.
///
/// # Arguments
/// - `cols` — Number of quads along X (vertex count = cols + 1).
/// - `rows` — Number of quads along Y (vertex count = rows + 1).
/// - `width` — Total width in meters.
/// - `height` — Total height in meters.
///
/// # Example
/// ```
/// use vistio_mesh::generators::quad_grid;
/// let mesh = quad_grid(2, 2, 1.0, 1.0);
/// assert_eq!(mesh.vertex_count(), 9);  // 3×3 vertices
/// assert_eq!(mesh.triangle_count(), 8); // 2×2 quads × 2 tris each
/// ```
pub fn quad_grid(cols: usize, rows: usize, width: f32, height: f32) -> TriangleMesh {
    let verts_x = cols + 1;
    let verts_y = rows + 1;
    let vertex_count = verts_x * verts_y;
    let tri_count = cols * rows * 2;

    let mut mesh = TriangleMesh::with_capacity(vertex_count, tri_count);

    let half_w = width / 2.0;
    let half_h = height / 2.0;

    // Generate vertices
    for j in 0..verts_y {
        for i in 0..verts_x {
            let u = i as f32 / cols as f32;
            let v = j as f32 / rows as f32;

            mesh.pos_x.push(-half_w + u * width);
            mesh.pos_y.push(half_h - v * height); // Top to bottom
            mesh.pos_z.push(0.0);

            mesh.normal_x.push(0.0);
            mesh.normal_y.push(0.0);
            mesh.normal_z.push(1.0); // Facing +Z

            mesh.uv_u.push(u);
            mesh.uv_v.push(v);
        }
    }

    // Generate triangles (two per quad)
    // We use a checkerboard pattern for diagonals to prevent structural anisotropy.
    // Every quad has two possible diagonals:
    // - Option A (even): (top_left, bot_right) splitting the quad into [TL, BL, TR] and [TR, BL, BR]
    // - Option B (odd) : (bot_left, top_right) splitting the quad into [TL, BL, BR] and [TL, BR, TR]
    for j in 0..rows {
        for i in 0..cols {
            let top_left = (j * verts_x + i) as u32;
            let top_right = top_left + 1;
            let bot_left = top_left + verts_x as u32;
            let bot_right = bot_left + 1;

            if (i + j) % 2 == 0 {
                // Diagonal: top-left to bottom-right
                // Upper-left triangle
                mesh.indices.push(top_left);
                mesh.indices.push(bot_left);
                mesh.indices.push(top_right);

                // Lower-right triangle
                mesh.indices.push(top_right);
                mesh.indices.push(bot_left);
                mesh.indices.push(bot_right);
            } else {
                // Diagonal: bottom-left to top-right
                // Lower-left triangle
                mesh.indices.push(top_left);
                mesh.indices.push(bot_left);
                mesh.indices.push(bot_right);

                // Upper-right triangle
                mesh.indices.push(top_left);
                mesh.indices.push(bot_right);
                mesh.indices.push(top_right);
            }

            mesh.material_ids.push(MaterialId(0));
            mesh.material_ids.push(MaterialId(0));
        }
    }

    mesh
}

/// Generates a UV sphere centered at the origin.
///
/// # Arguments
/// - `radius` — Sphere radius in meters.
/// - `stacks` — Number of horizontal slices (latitude divisions).
/// - `slices` — Number of vertical slices (longitude divisions).
pub fn uv_sphere(radius: f32, stacks: usize, slices: usize) -> TriangleMesh {
    let vertex_count = (stacks + 1) * (slices + 1);
    let tri_count = stacks * slices * 2;
    let mut mesh = TriangleMesh::with_capacity(vertex_count, tri_count);

    // Generate vertices
    for i in 0..=stacks {
        let phi = std::f32::consts::PI * i as f32 / stacks as f32; // 0 to PI
        let sin_phi = phi.sin();
        let cos_phi = phi.cos();

        for j in 0..=slices {
            let theta = 2.0 * std::f32::consts::PI * j as f32 / slices as f32;
            let sin_theta = theta.sin();
            let cos_theta = theta.cos();

            let x = sin_phi * cos_theta;
            let y = cos_phi;
            let z = sin_phi * sin_theta;

            mesh.pos_x.push(radius * x);
            mesh.pos_y.push(radius * y);
            mesh.pos_z.push(radius * z);

            // Normal = position normalized (for a unit sphere, equals position/radius)
            mesh.normal_x.push(x);
            mesh.normal_y.push(y);
            mesh.normal_z.push(z);

            mesh.uv_u.push(j as f32 / slices as f32);
            mesh.uv_v.push(i as f32 / stacks as f32);
        }
    }

    // Generate triangles
    for i in 0..stacks {
        for j in 0..slices {
            let a = (i * (slices + 1) + j) as u32;
            let b = a + (slices + 1) as u32;

            // Skip degenerate triangles at poles
            if i != 0 {
                mesh.indices.push(a);
                mesh.indices.push(b);
                mesh.indices.push(a + 1);
                mesh.material_ids.push(MaterialId(0));
            }

            if i != stacks - 1 {
                mesh.indices.push(a + 1);
                mesh.indices.push(b);
                mesh.indices.push(b + 1);
                mesh.material_ids.push(MaterialId(0));
            }
        }
    }

    mesh
}

/// Generates a flat circular mesh in the XY plane.
///
/// # Arguments
/// - `radius` — Radius of the circle.
/// - `segments` — Number of angular subdivisions.
/// - `rings` — Number of concentric radial subdivisions.
pub fn circular_grid(radius: f32, segments: usize, rings: usize) -> TriangleMesh {
    let vertex_count = 1 + rings * segments;
    let tri_count = segments + (rings - 1) * segments * 2;
    let mut mesh = TriangleMesh::with_capacity(vertex_count, tri_count);

    mesh.pos_x.push(0.0);
    mesh.pos_y.push(0.0);
    mesh.pos_z.push(0.0);
    mesh.normal_x.push(0.0);
    mesh.normal_y.push(0.0);
    mesh.normal_z.push(1.0);
    mesh.uv_u.push(0.5);
    mesh.uv_v.push(0.5);

    for r in 1..=rings {
        let current_radius = radius * (r as f32 / rings as f32);
        for s in 0..segments {
            let theta = 2.0 * std::f32::consts::PI * (s as f32 / segments as f32);
            let x = current_radius * theta.cos();
            let y = current_radius * theta.sin();

            mesh.pos_x.push(x);
            mesh.pos_y.push(y);
            mesh.pos_z.push(0.0);
            mesh.normal_x.push(0.0);
            mesh.normal_y.push(0.0);
            mesh.normal_z.push(1.0);
            mesh.uv_u.push(0.5 + 0.5 * (x / radius));
            mesh.uv_v.push(0.5 + 0.5 * (y / radius));
        }
    }

    for s in 0..segments {
        let v0 = 0;
        let v1 = 1 + s;
        let v2 = 1 + (s + 1) % segments;
        mesh.indices.push(v0 as u32);
        mesh.indices.push(v1 as u32);
        mesh.indices.push(v2 as u32);
        mesh.material_ids.push(MaterialId(0));
    }

    for r in 1..rings {
        let ring_start = 1 + (r - 1) * segments;
        let next_ring_start = 1 + r * segments;
        for s in 0..segments {
            let next_s = (s + 1) % segments;
            let v0 = ring_start + s;
            let v1 = ring_start + next_s;
            let v2 = next_ring_start + s;
            let v3 = next_ring_start + next_s;

            mesh.indices.push(v0 as u32);
            mesh.indices.push(v1 as u32);
            mesh.indices.push(v3 as u32);
            mesh.material_ids.push(MaterialId(0));

            mesh.indices.push(v0 as u32);
            mesh.indices.push(v3 as u32);
            mesh.indices.push(v2 as u32);
            mesh.material_ids.push(MaterialId(0));
        }
    }

    mesh
}
