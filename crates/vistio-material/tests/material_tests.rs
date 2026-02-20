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
