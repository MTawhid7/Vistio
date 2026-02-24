//! Integration tests for vistio-material.

use vistio_material::{FabricProperties, MaterialDatabase};

// ─── FabricProperties Tests ───────────────────────────────────

#[test]
fn mass_calculation() {
    let props = FabricProperties {
        name: "Test".into(),
        density: 200.0,
        stretch_stiffness_warp: 0.9,
        stretch_stiffness_weft: 0.9,
        shear_stiffness: 0.5,
        bending_stiffness_warp: 0.5,
        bending_stiffness_weft: 0.5,
        thickness: 0.001,
        friction: 0.5,
        damping: 0.01,
    };
    let mass = props.mass_per_vertex(100, 1.0);
    assert!((mass - 0.002).abs() < 1e-6);
}

#[test]
fn avg_stiffness() {
    let props = FabricProperties {
        name: "Test".into(),
        density: 100.0,
        stretch_stiffness_warp: 0.8,
        stretch_stiffness_weft: 0.6,
        shear_stiffness: 0.5,
        bending_stiffness_warp: 0.4,
        bending_stiffness_weft: 0.2,
        thickness: 0.001,
        friction: 0.5,
        damping: 0.01,
    };
    assert!((props.avg_stretch_stiffness() - 0.7).abs() < 1e-6);
    assert!((props.avg_bending_stiffness() - 0.3).abs() < 1e-6);
}

// ─── MaterialDatabase Tests ──────────────────────────────────

#[test]
fn default_database_has_five_materials() {
    let db = MaterialDatabase::with_defaults();
    assert_eq!(db.len(), 5);
}

#[test]
fn lookup_by_name() {
    let db = MaterialDatabase::with_defaults();
    let cotton = db.get("cotton_twill").unwrap();
    assert_eq!(cotton.density, 200.0);
}

#[test]
fn lookup_all_presets() {
    let db = MaterialDatabase::with_defaults();
    assert!(db.get("cotton_twill").is_some());
    assert!(db.get("silk_charmeuse").is_some());
    assert!(db.get("denim_14oz").is_some());
    assert!(db.get("jersey_knit").is_some());
    assert!(db.get("chiffon").is_some());
}

#[test]
fn missing_material_returns_none() {
    let db = MaterialDatabase::with_defaults();
    assert!(db.get("nonexistent").is_none());
}

#[test]
fn custom_material() {
    let mut db = MaterialDatabase::empty();
    assert!(db.is_empty());
    db.register(FabricProperties {
        name: "custom_fabric".into(),
        density: 150.0,
        stretch_stiffness_warp: 0.80,
        stretch_stiffness_weft: 0.75,
        shear_stiffness: 0.40,
        bending_stiffness_warp: 0.30,
        bending_stiffness_weft: 0.28,
        thickness: 0.0005,
        friction: 0.35,
        damping: 0.015,
    });
    assert_eq!(db.len(), 1);
    assert!(db.get("custom_fabric").is_some());
}

#[test]
fn density_ordering() {
    let db = MaterialDatabase::with_defaults();
    let chiffon = db.get("chiffon").unwrap().density;
    let silk = db.get("silk_charmeuse").unwrap().density;
    let jersey = db.get("jersey_knit").unwrap().density;
    let cotton = db.get("cotton_twill").unwrap().density;
    let denim = db.get("denim_14oz").unwrap().density;
    assert!(chiffon < silk);
    assert!(silk < jersey);
    assert!(jersey < cotton);
    assert!(cotton < denim);
}

#[test]
fn stiffness_ordering() {
    let db = MaterialDatabase::with_defaults();
    let denim = db.get("denim_14oz").unwrap();
    let chiffon = db.get("chiffon").unwrap();
    assert!(denim.stretch_stiffness_warp > chiffon.stretch_stiffness_warp);
    assert!(denim.bending_stiffness_warp > chiffon.bending_stiffness_warp);
}

// ─── ConstitutiveModel Tests ─────────────────────────────────

use vistio_material::{CoRotationalModel, ConstitutiveModel, IsotropicLinearModel};
use vistio_math::mat3x2::Mat3x2;
use vistio_math::Vec3;

#[test]
fn corotational_identity_produces_identity_target() {
    let model = CoRotationalModel::new();
    let f = Mat3x2::IDENTITY;
    let result = model.project(&f, 1.0, 1.0);
    // Undeformed → target should be identity (rotation = identity)
    assert!((result.target_f.col0 - Vec3::X).length() < 1e-4);
    assert!((result.target_f.col1 - Vec3::Y).length() < 1e-4);
}

#[test]
fn corotational_energy_zero_at_rest() {
    let model = CoRotationalModel::new();
    let f = Mat3x2::IDENTITY;
    let result = model.project(&f, 1.0, 1.0);
    // No deformation → zero energy
    assert!(result.energy < 1e-8, "Energy at rest should be ~0, got {}", result.energy);
}

#[test]
fn corotational_energy_positive_under_strain() {
    let model = CoRotationalModel::new();
    // Uniform 2x stretch
    let f = Mat3x2::from_cols(Vec3::X * 2.0, Vec3::Y * 2.0);
    let result = model.project(&f, 1.0, 1.0);
    assert!(result.energy > 0.0, "Stretched element should have positive energy");
}

#[test]
fn corotational_rotated_element_zero_energy() {
    let model = CoRotationalModel::new();
    // Pure 90° rotation around Z axis: (X→Y, Y→-X)
    let f = Mat3x2::from_cols(Vec3::Y, -Vec3::X);
    let result = model.project(&f, 1.0, 1.0);
    // Pure rotation → F = R, energy = ||F - R||² = 0
    assert!(result.energy < 1e-4, "Pure rotation should have ~0 energy, got {}", result.energy);
}

#[test]
fn corotational_target_is_rotation() {
    let model = CoRotationalModel::new();
    // Stretch + rotation: scale X by 1.5 then rotate 45°
    let angle = std::f32::consts::FRAC_PI_4;
    let c = angle.cos();
    let s = angle.sin();
    let f = Mat3x2::from_cols(
        Vec3::new(c * 1.5, s * 1.5, 0.0),
        Vec3::new(-s, c, 0.0),
    );
    let result = model.project(&f, 1.0, 1.0);
    // Target should be orthonormal (columns should be unit length)
    let len0 = result.target_f.col0.length();
    let len1 = result.target_f.col1.length();
    assert!((len0 - 1.0).abs() < 0.1, "Target col0 should be ~unit, got {}", len0);
    assert!((len1 - 1.0).abs() < 0.1, "Target col1 should be ~unit, got {}", len1);
}

#[test]
fn isotropic_identity_produces_identity_target() {
    let model = IsotropicLinearModel::new();
    let f = Mat3x2::IDENTITY;
    let result = model.project(&f, 1.0, 1.0);
    // Isotropic always targets identity
    assert!((result.target_f.col0 - Vec3::X).length() < 1e-6);
    assert!((result.target_f.col1 - Vec3::Y).length() < 1e-6);
}

#[test]
fn isotropic_energy_zero_at_rest() {
    let model = IsotropicLinearModel::new();
    let f = Mat3x2::IDENTITY;
    let result = model.project(&f, 1.0, 1.0);
    assert!(result.energy < 1e-8, "Energy at rest should be ~0, got {}", result.energy);
}

#[test]
fn isotropic_energy_positive_under_strain() {
    let model = IsotropicLinearModel::new();
    let f = Mat3x2::from_cols(Vec3::X * 1.5, Vec3::Y * 1.5);
    let result = model.project(&f, 1.0, 1.0);
    assert!(result.energy > 0.0, "Stretched should have positive energy");
}

#[test]
fn isotropic_rotation_produces_nonzero_energy() {
    // This is the key difference from co-rotational:
    // A pure rotation is NOT at rest for the isotropic model.
    let model = IsotropicLinearModel::new();
    let f = Mat3x2::from_cols(Vec3::Y, -Vec3::X); // 90° rotation
    let result = model.project(&f, 1.0, 1.0);
    // ||R - I||² > 0 because R ≠ I
    assert!(result.energy > 0.1, "Rotation should produce energy for isotropic model, got {}", result.energy);
}

#[test]
fn corotational_model_name() {
    let model = CoRotationalModel::new();
    assert_eq!(model.name(), "co_rotational");
}

#[test]
fn isotropic_model_name() {
    let model = IsotropicLinearModel::new();
    assert_eq!(model.name(), "isotropic_linear");
}
