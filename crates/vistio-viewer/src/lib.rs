//! Vistio High-Fidelity Viewer using Bevy.
//!
//! Provides a physically based rendering environment for Vistio cloth simulations.

use std::time::Instant;

use bevy::prelude::*;
use bevy::render::mesh::Indices;
use bevy::render::render_resource::PrimitiveTopology;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};

use vistio_bench::scenarios::Scenario;
use vistio_solver::pd_solver::ProjectiveDynamicsSolver;
use vistio_solver::state::SimulationState;
use vistio_solver::strategy::SolverStrategy;
use vistio_mesh::topology::Topology;
use vistio_material::CoRotationalModel;
use vistio_contact::{CollisionPipeline, SpatialHash, VertexTriangleTest, ProjectionContactResponse};

/// System resource holding the Vistio simulation state.
#[derive(Resource)]
struct SimRunner {
    solver: ProjectiveDynamicsSolver,
    state: SimulationState,
    dt: f32,
    current_step: u32,
    indices: Vec<u32>, // Flat array of [i0, i1, i2, ...]
    collision: Option<CollisionPipeline>,
}

#[derive(Resource)]
struct SceneData {
    body: Option<vistio_mesh::TriangleMesh>,
}

/// Component to tag the Bevy cloth entity.
#[derive(Component)]
struct ClothMesh;

/// Launch the Bevy viewer for a given scenario.
pub fn launch_viewer(scenario: Scenario) -> Result<(), Box<dyn std::error::Error>> {
    println!("Initializing Bevy Viewer (PBR)...");

    // Initialize solver
    let topology = Topology::build(&scenario.garment);
    let mut solver = ProjectiveDynamicsSolver::new();

    let vertex_mass: f32 = if let Some(ref properties) = scenario.material {
        let model = Box::new(CoRotationalModel::new());
        solver.init_with_material(
            &scenario.garment,
            &topology,
            &scenario.config,
            properties,
            model,
        ).map_err(|e| format!("Solver init failed: {e}"))?;

        let total_area: f32 = {
            use bevy::prelude::Vec3;
            let mesh = &scenario.garment;
            (0..mesh.triangle_count()).map(|t| {
                let idx_base = t * 3;
                let i0 = mesh.indices[idx_base] as usize;
                let i1 = mesh.indices[idx_base + 1] as usize;
                let i2 = mesh.indices[idx_base + 2] as usize;
                let p0 = Vec3::new(mesh.pos_x[i0], mesh.pos_y[i0], mesh.pos_z[i0]);
                let p1 = Vec3::new(mesh.pos_x[i1], mesh.pos_y[i1], mesh.pos_z[i1]);
                let p2 = Vec3::new(mesh.pos_x[i2], mesh.pos_y[i2], mesh.pos_z[i2]);
                0.5 * (p1 - p0).cross(p2 - p0).length()
            }).sum()
        };
        properties.mass_per_vertex(scenario.garment.vertex_count(), total_area)
    } else {
        solver.init(&scenario.garment, &topology, &scenario.config)
            .map_err(|e| format!("Solver init failed: {e}"))?;
        scenario.vertex_mass
    };

    let state = SimulationState::from_mesh(
        &scenario.garment,
        vertex_mass,
        &scenario.pinned,
    ).map_err(|e| format!("State init failed: {e}"))?;

    let mut pipeline = CollisionPipeline::new(
        Box::new(SpatialHash::new(0.05)),
        Box::new(VertexTriangleTest),
        Box::new(ProjectionContactResponse),
        scenario.garment.clone(),
        0.01,
        1.0,
    ).with_ground(-0.3);

    match scenario.kind {
        vistio_bench::scenarios::ScenarioKind::SphereDrape => {
            pipeline = pipeline.with_sphere(vistio_math::Vec3::new(0.0, 0.0, 0.0), 0.3);
        },
        vistio_bench::scenarios::ScenarioKind::SelfFold => {
            pipeline = pipeline.with_self_collision(&topology, 2);
        },
        _ => {}
    }

    let runner = SimRunner {
        solver,
        state,
        dt: scenario.dt,
        current_step: 0,
        indices: scenario.garment.indices.clone(),
        collision: Some(pipeline),
    };

    let scene_data = SceneData {
        body: scenario.body.clone(),
    };

    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: format!("Vistio Viewer - {}", scenario.kind.name()),
            resolution: (1280., 720.).into(),
            ..default()
        }),
        ..default()
    }));
    app.add_plugins(PanOrbitCameraPlugin);

    app.insert_resource(runner);
    app.insert_resource(scene_data);
    app.insert_resource(ClearColor(Color::srgb(0.05, 0.05, 0.08))); // Dark background

    // Setup scene
    app.add_systems(Startup, setup_scene);

    // Update simulation
    app.add_systems(Update, simulate_cloth);

    app.run();

    Ok(())
}

fn compute_smooth_normals(
    pos_x: &[f32],
    pos_y: &[f32],
    pos_z: &[f32],
    indices: &[u32],
) -> Vec<[f32; 3]> {
    let n = pos_x.len();
    let mut normals = vec![[0.0_f32; 3]; n];

    for chunk in indices.chunks_exact(3) {
        let i0 = chunk[0] as usize;
        let i1 = chunk[1] as usize;
        let i2 = chunk[2] as usize;

        let p0 = Vec3::new(pos_x[i0], pos_y[i0], pos_z[i0]);
        let p1 = Vec3::new(pos_x[i1], pos_y[i1], pos_z[i1]);
        let p2 = Vec3::new(pos_x[i2], pos_y[i2], pos_z[i2]);

        // Triangle normal (area weighted)
        let cross = (p1 - p0).cross(p2 - p0);

        normals[i0][0] += cross.x;
        normals[i0][1] += cross.y;
        normals[i0][2] += cross.z;
        normals[i1][0] += cross.x;
        normals[i1][1] += cross.y;
        normals[i1][2] += cross.z;
        normals[i2][0] += cross.x;
        normals[i2][1] += cross.y;
        normals[i2][2] += cross.z;
    }

    // Normalize
    for normal in &mut normals {
        let len_sq = normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2];
        if len_sq > 1e-12 {
            let inv_len = 1.0 / len_sq.sqrt();
            normal[0] *= inv_len;
            normal[1] *= inv_len;
            normal[2] *= inv_len;
        } else {
            // Fallback: point up
            normal[0] = 0.0;
            normal[1] = 1.0;
            normal[2] = 0.0;
        }
    }

    normals
}

fn simulate_cloth(
    mut runner: ResMut<SimRunner>,
    mut meshes: ResMut<Assets<Mesh>>,
    query: Query<(Entity, &Handle<Mesh>), With<ClothMesh>>,
) {
    let _start = Instant::now();
    let dt = runner.dt;

    // Borrow parts independently
    let SimRunner { ref mut solver, ref mut state, ref mut collision, .. } = *runner;

    let _result = match solver.step(state, dt) {
        Ok(res) => res,
        Err(e) => {
            eprintln!("Simulation error: {}", e);
            return;
        }
    };

    if let Some(ref mut pipeline) = collision {
        let _ = pipeline.step(state);
    }

    runner.current_step += 1;

    let n = runner.state.vertex_count;
    let mut positions = Vec::with_capacity(n);
    let mut uvs = Vec::with_capacity(n); // We just pass dummy UVs for now, or we can use the mesh UVs if we stored them

    for i in 0..n {
        positions.push([runner.state.pos_x[i], runner.state.pos_y[i], runner.state.pos_z[i]]);
        uvs.push([0.0_f32, 0.0_f32]); // Dummy UVs for StandardMaterial
    }

    let normals = compute_smooth_normals(
        &runner.state.pos_x,
        &runner.state.pos_y,
        &runner.state.pos_z,
        &runner.indices,
    );

    // Update Bevy mesh
    if let Ok((_entity, mesh_handle)) = query.get_single() {
        if let Some(mesh) = meshes.get_mut(mesh_handle) {
            mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
            mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
            // Updating UVs is technically not needed every frame if they are static,
            // but for simplicity we keep it here.
        }
    }
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    runner: Res<SimRunner>,
    scene_data: Res<SceneData>,
) {
    // 1. Setup Initial Mesh
    let n = runner.state.vertex_count;
    let mut positions = Vec::with_capacity(n);
    let mut uvs = Vec::with_capacity(n);

    for i in 0..n {
        positions.push([runner.state.pos_x[i], runner.state.pos_y[i], runner.state.pos_z[i]]);
        // Approximate UVs based on index since we didn't store original UVs in SimulationState
        // (For the real checkerboard we should pass the actual UVs, but StandardMaterial doesn't strictly need them unless mapped)
        uvs.push([(i % 20) as f32 / 20.0, (i / 20) as f32 / 20.0]);
    }

    let normals = compute_smooth_normals(
        &runner.state.pos_x,
        &runner.state.pos_y,
        &runner.state.pos_z,
        &runner.indices,
    );

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        Default::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    mesh.insert_indices(Indices::U32(runner.indices.clone()));

    // Fabric Material - Double-sided PBR
    let cloth_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.8, 0.2, 0.2), // Deep red cloth
        perceptual_roughness: 0.9, // Fabric is rough
        metallic: 0.05,
        double_sided: true, // Crucial for cloth!
        cull_mode: None,    // Render both sides
        ..default()
    });

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(mesh),
            material: cloth_material,
            ..default()
        },
        bevy::render::view::NoFrustumCulling,
        ClothMesh,
    ));

    // 2. Setup Ground Plane
    let ground_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.2, 0.2, 0.25),
        perceptual_roughness: 0.8,
        ..default()
    });

    commands.spawn(PbrBundle {
        mesh: meshes.add(Cuboid::new(4.0, 0.05, 4.0)),
        material: ground_material,
        transform: Transform::from_xyz(0.0, -0.325, 0.0), // top face at Y = -0.3
        ..default()
    });

    // 2.5. Setup Body (e.g. Sphere) if present
    if let Some(ref body) = scene_data.body {
        let n_body = body.vertex_count();
        let mut body_positions = Vec::with_capacity(n_body);
        let mut body_uvs = Vec::with_capacity(n_body);

        for i in 0..n_body {
            body_positions.push([body.pos_x[i], body.pos_y[i], body.pos_z[i]]);
            body_uvs.push([0.0_f32, 0.0_f32]);
        }

        let body_normals = compute_smooth_normals(&body.pos_x, &body.pos_y, &body.pos_z, &body.indices);

        let mut body_mesh = Mesh::new(
            PrimitiveTopology::TriangleList,
            Default::default(),
        );
        body_mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, body_positions);
        body_mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, body_normals);
        body_mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, body_uvs);
        body_mesh.insert_indices(Indices::U32(body.indices.clone()));

        let body_material = materials.add(StandardMaterial {
            base_color: Color::srgb(0.7, 0.7, 0.7),
            perceptual_roughness: 0.6,
            ..default()
        });

        commands.spawn(PbrBundle {
            mesh: meshes.add(body_mesh),
            material: body_material,
            ..default()
        });
    }

    // 3. Setup Directional Light (Sun/Key Light) with Shadows
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(4.0, 8.0, 4.0)
            .looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    // 4. Setup Ambient Light (Fill Light)
    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 100.0,
    });

    // 5. Setup Camera
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0.0, 1.5, 3.5).looking_at(Vec3::new(0.0, 0.5, 0.0), Vec3::Y),
            ..default()
        },
        PanOrbitCamera {
            focus: Vec3::new(0.0, 0.5, 0.0),
            radius: Some(3.5),
            ..default()
        },
    ));
}
