#[cfg(feature = "vendored-mujoco")]
fn main() {
    use std::path::{Path, PathBuf};

    let mut cfg = cc::Build::new();

    let dst = PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
    let include = dst.join("include");
    std::fs::create_dir_all(&include).expect("Failed to create include dir");
    let ccd_include = include.join("ccd");
    std::fs::create_dir_all(&ccd_include).expect("Failed to create ccd include dir");

    let template = std::fs::read_to_string("libccd/src/ccd/config.h.cmake.in")
        .expect("Failed to read config.h.cmake.in");

    let config_content = template
        .replace("#cmakedefine CCD_SINGLE", "/* #undef CCD_SINGLE */")
        .replace("#cmakedefine CCD_DOUBLE", "#define CCD_DOUBLE");

    let dest_path = Path::new(&ccd_include).join("config.h");
    std::fs::write(&dest_path, config_content).expect("Failed to write config.h");

    println!("cargo:include={}", include.display());
    println!("cargo:root={}", dst.display());

    cfg.define("MC_IMPLEM_ENABLE", "1");
    cfg.include(&include);
    cfg.include("TriangleMeshDistance");
    cfg.include("lodepng");
    cfg.include("qhull/src/libqhull_r");
    cfg.include("MarchingCubeCpp");
    cfg.include("tinyxml2");
    cfg.include("tinyobjloader");
    cfg.include("libccd/src");
    cfg.include("mujoco/include");
    cfg.include("mujoco/src");
    cfg.files(&[
        // tinyxml2
        "tinyxml2/tinyxml2.cpp",
        // lodepng
        "lodepng/lodepng.cpp",
        // qhull
        "qhull/src/libqhull_r/global_r.c",
        "qhull/src/libqhull_r/stat_r.c",
        "qhull/src/libqhull_r/geom2_r.c",
        "qhull/src/libqhull_r/poly2_r.c",
        "qhull/src/libqhull_r/merge_r.c",
        "qhull/src/libqhull_r/libqhull_r.c",
        "qhull/src/libqhull_r/geom_r.c",
        "qhull/src/libqhull_r/poly_r.c",
        "qhull/src/libqhull_r/qset_r.c",
        "qhull/src/libqhull_r/mem_r.c",
        "qhull/src/libqhull_r/random_r.c",
        "qhull/src/libqhull_r/usermem_r.c",
        "qhull/src/libqhull_r/userprintf_r.c",
        "qhull/src/libqhull_r/io_r.c",
        "qhull/src/libqhull_r/user_r.c",
        "qhull/src/libqhull_r/accessors_r.c",
        "qhull/src/libqhull_r/rboxlib_r.c",
        "qhull/src/libqhull_r/userprintf_rbox_r.c",
        // tinyobjloader
        "tinyobjloader/tiny_obj_loader.cc",
        // MuJoCo
        "mujoco/src/xml/xml_api.cc",
        "mujoco/src/xml/xml_base.cc",
        "mujoco/src/xml/xml.cc",
        "mujoco/src/xml/xml_native_reader.cc",
        "mujoco/src/xml/xml_numeric_format.cc",
        "mujoco/src/xml/xml_native_writer.cc",
        "mujoco/src/xml/xml_urdf.cc",
        "mujoco/src/xml/xml_util.cc",
        "mujoco/src/user/user_api.cc",
        "mujoco/src/user/user_cache.cc",
        "mujoco/src/user/user_composite.cc",
        "mujoco/src/user/user_flexcomp.cc",
        "mujoco/src/user/user_init.c",
        "mujoco/src/user/user_mesh.cc",
        "mujoco/src/user/user_model.cc",
        "mujoco/src/user/user_objects.cc",
        "mujoco/src/user/user_resource.cc",
        "mujoco/src/user/user_util.cc",
        "mujoco/src/user/user_vfs.cc",
        "mujoco/src/thread/thread_pool.cc",
        "mujoco/src/thread/thread_task.cc",
    ]);
    cfg.cpp(true);
    cfg.warnings(false);
    cfg.compile("mujoco_cpp");

    // Add C source files separately
    let mut c_cfg = cc::Build::new();
    c_cfg.include(&include);
    c_cfg.include("mujoco/include");
    c_cfg.include("mujoco/src");
    c_cfg.include("libccd/src");
    c_cfg.include("qhull/src/libqhull_r");
    c_cfg.files(&[
        // ccd
        "libccd/src/ccd.c",
        "libccd/src/mpr.c",
        "libccd/src/polytope.c",
        "libccd/src/support.c",
        "libccd/src/vec3.c",
        // MuJoCo
        "mujoco/src/engine/engine_callback.c",
        "mujoco/src/engine/engine_collision_box.c",
        "mujoco/src/engine/engine_collision_convex.c",
        "mujoco/src/engine/engine_collision_driver.c",
        "mujoco/src/engine/engine_collision_gjk.c",
        "mujoco/src/engine/engine_collision_primitive.c",
        "mujoco/src/engine/engine_collision_sdf.c",
        "mujoco/src/engine/engine_core_constraint.c",
        "mujoco/src/engine/engine_core_smooth.c",
        "mujoco/src/engine/engine_crossplatform.cc",
        "mujoco/src/engine/engine_derivative.c",
        "mujoco/src/engine/engine_derivative_fd.c",
        "mujoco/src/engine/engine_forward.c",
        "mujoco/src/engine/engine_inverse.c",
        "mujoco/src/engine/engine_island.c",
        "mujoco/src/engine/engine_io.c",
        "mujoco/src/engine/engine_name.c",
        "mujoco/src/engine/engine_passive.c",
        "mujoco/src/engine/engine_plugin.cc",
        "mujoco/src/engine/engine_print.c",
        "mujoco/src/engine/engine_ray.c",
        "mujoco/src/engine/engine_sensor.c",
        "mujoco/src/engine/engine_setconst.c",
        "mujoco/src/engine/engine_solver.c",
        "mujoco/src/engine/engine_support.c",
        "mujoco/src/engine/engine_util_blas.c",
        "mujoco/src/engine/engine_util_errmem.c",
        "mujoco/src/engine/engine_util_misc.c",
        "mujoco/src/engine/engine_util_solve.c",
        "mujoco/src/engine/engine_util_sparse.c",
        "mujoco/src/engine/engine_util_spatial.c",
        "mujoco/src/engine/engine_vis_init.c",
        "mujoco/src/engine/engine_vis_interact.c",
        "mujoco/src/engine/engine_vis_visualize.c",
    ]);

    c_cfg.warnings(false);
    c_cfg.compile("mujoco_c");
}

#[cfg(not(feature = "vendored-mujoco"))]
fn main() {
    todo!("Link to system-installed MuJoCo");
}
