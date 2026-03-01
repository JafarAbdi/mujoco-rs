"""A script that uses MuJoCo's introspection module to generate MuJoCo Rust bindings."""

import sys
import os
import re
import pathlib

if len(sys.argv) != 2:
    print("Usage: python generate_from_introspect.py <path_to_mujoco>")
    sys.exit(1)

mujoco_path = sys.argv[1]
sys.path.append(os.path.abspath(f"{mujoco_path}/python/mujoco"))

from introspect import structs
from introspect import functions
from introspect import enums

FILE_DIR = pathlib.Path(__file__).parent

RUST_TYPES = {
    "int": "usize",
    "mjtSize": "usize",
    "float": "f32",
    "uint64_t": "u64",
    "mjtByte": "u8",
    "size_t": "usize",
    "uintptr_t": "usize",
}
RUST_POINTER_TYPES = {
    "int": "i32",
    "char": "i8",
    "mjtSize": "i64",
}
RUST_ARRAY_TYPES = {
    "int": "i32",
    "mjtSize": "i64",
}

# Fields that should have nalgebra accessors generated
# Maps field name -> (element_stride, nalgebra_type, count_field, has_setter)
# element_stride: number of elements per item (3 for Vec3, 4 for Quat, 9 for Mat3)
# nalgebra_type: the nalgebra type to return
# count_field: field name for the count (prefixed with model. or self.)
# has_setter: whether to generate a set_ method (only for user-controllable fields)
NALGEBRA_DATA_FIELDS = {
    # Body positions and orientations (computed by simulation, no setters)
    "xpos": (3, "Vec3", "nbody", False),
    "xquat": (4, "Quat", "nbody", False),
    "xmat": (9, "Mat3", "nbody", False),
    # Body inertial frame (computed, no setters)
    "xipos": (3, "Vec3", "nbody", False),
    "ximat": (9, "Mat3", "nbody", False),
    # Subtree center of mass (computed, no setter)
    "subtree_com": (3, "Vec3", "nbody", False),
    # Geom positions and orientations (computed, no setters)
    "geom_xpos": (3, "Vec3", "ngeom", False),
    "geom_xmat": (9, "Mat3", "ngeom", False),
    # Site positions and orientations (computed, no setters)
    "site_xpos": (3, "Vec3", "nsite", False),
    "site_xmat": (9, "Mat3", "nsite", False),
    # Mocap body positions and orientations (user input, has setters)
    "mocap_pos": (3, "Vec3", "nmocap", True),
    "mocap_quat": (4, "Quat", "nmocap", True),
}

SKIP_FUNCTIONS = {
    # Functions that are manually bound in Rust wrapper types
    # model.rs
    "mj_loadXML",
    "mj_deleteModel",
    "mj_copyModel",
    # data.rs
    "mj_makeData",
    "mj_deleteData",
    "mj_copyData",
    # spec.rs
    "mj_parseXML",
    "mj_parseXMLString",
    "mj_compile",
    "mj_deleteSpec",
    # Complex derivative functions
    "mjd_transitionFD",
    "mjd_inverseFD",
    # Internal/low-level mj_ functions
    "mj_freeLastXML",
    "mj_defaultSolRefImp",
    "mj_resetCallbacks",
    "mj_makeSpec",
    "mj_loadPluginLibrary",
    "mj_loadAllPluginLibraries",
    # VFS functions
    "mj_defaultVFS",
    "mj_addFileVFS",
    "mj_addBufferVFS",
    "mj_deleteFileVFS",
    "mj_deleteVFS",
    # Spec functions
    "mj_copyBack",
    "mj_recompile",
    "mj_copySpec",
    # XML/Save/Load with error buffers
    "mj_saveLastXML",
    "mj_saveXMLString",
    "mj_saveXML",
    "mj_saveModel",
    "mj_loadModel",
    "mj_printSchema",
    # Default/Init structs
    "mj_defaultLROpt",
    "mj_defaultOption",
    "mj_defaultVisual",
    # Stack allocation
    "mj_markStack",
    "mj_freeStack",
    "mj_stackAllocByte",
    "mj_stackAllocNum",
    "mj_stackAllocInt",
    # Model query
    "mj_isPyramidal",
    "mj_isSparse",
    "mj_isDual",
    "mj_getTotalmass",
    "mj_sizeModel",
    "mj_stateSize",
    # Simulation with extra params
    "mj_forwardSkip",
    "mj_inverseSkip",
    "mj_RungeKutta",
    "mj_resetDataDebug",
    # Solver/matrix functions
    "mj_solveM",
    "mj_solveM2",
    "mj_rne",
    "mj_mulJacVec",
    "mj_mulJacTVec",
    "mj_mulM",
    "mj_mulM2",
    "mj_fullM",
    "mj_getState",
    "mj_setState",
    "mj_addM",
    "mj_constraintUpdate",
    # Print functions
    "mj_printModel",
    "mj_printData",
    "mj_printFormattedModel",
    "mj_printFormattedData",
    # Misc
    "mj_setTotalmass",
    "mj_setConst",
    "mj_setKeyframe",
    "mj_local2Global",
    "mj_applyFT",
    "mj_warning",
    "mj_addContact",
    "mj_multiRay",
    "mj_ray",
    "mj_rayHfield",
    "mj_rayMesh",
    "mj_getPluginConfig",
    # VFS mount/unmount
    "mj_mountVFS",
    "mj_unmountVFS",
    # Cache functions
    "mj_getCacheSize",
    "mj_getCacheCapacity",
    "mj_setCacheCapacity",
    "mj_getCache",
    "mj_clearCache",
    # Alternative model loading
    "mj_parse",
    "mj_loadModelBuffer",
    # Length range
    "mj_setLengthRange",
    # Scene printing
    "mj_printScene",
    "mj_printFormattedScene",
    # State management
    "mj_extractState",
    "mj_copyState",
    # History functions
    "mj_readCtrl",
    "mj_readSensor",
    "mj_initCtrlHistory",
    "mj_initSensorHistory",
    # Flex raycast
    "mj_rayFlex",
}

# Prefixes to skip
SKIP_PREFIXES = ("mjs_", "mjc_", "mjp_", "mjui_", "mjr_", "mjv_", "mju_")


def camel_to_snake(camel_str):
    """Convert camelCase to snake_case"""
    # Insert underscore before uppercase letters (except at start of word)
    # But not after digits (e.g., euler2Quat -> euler2quat, not euler2_quat)
    snake_str = re.sub("([a-z])([A-Z])", r"\1_\2", camel_str)
    # Insert underscore between consecutive uppercase letters
    snake_str = re.sub("([A-Z])([A-Z])", r"\1_\2", snake_str)
    return snake_str.lower()


def save_file(filename, header, accessors, footer):
    output_path = FILE_DIR / ".." / "mujoco" / "src" / filename
    with open(output_path, "w") as f:
        f.write(header)
        for accessor in accessors:
            f.write(f"    {accessor}\n")
        f.write(footer)


def generate_nalgebra_accessor(field_name, stride, nalgebra_type, count_field, has_setter):
    """Generate nalgebra accessor methods for a spatial data field.

    Returns a tuple of (getter, setter) where setter is None if has_setter is False.
    Uses get_/set_ naming convention for clean API.
    """
    # Determine count expression
    if count_field in model_fields:
        count_expr = f"self.model.{count_field}()"
    else:
        count_expr = f"self.{count_field}()"

    if nalgebra_type == "Vec3":
        getter = f"""
/// Get {field_name} at index as Vec3
pub fn get_{field_name}(&self, idx: usize) -> crate::Vec3 {{
    debug_assert!(idx < {count_expr}, "{field_name} index out of bounds");
    let data = self.{field_name}();
    let i = idx * {stride};
    crate::Vec3::new(data[i], data[i + 1], data[i + 2])
}}"""
        setter = f"""
/// Set {field_name} at index from Vec3
pub fn set_{field_name}(&mut self, idx: usize, value: &crate::Vec3) {{
    debug_assert!(idx < {count_expr}, "{field_name} index out of bounds");
    let data = self.{field_name}_mut();
    let i = idx * {stride};
    data[i] = value[0];
    data[i + 1] = value[1];
    data[i + 2] = value[2];
}}""" if has_setter else None

    elif nalgebra_type == "Quat":
        getter = f"""
/// Get {field_name} at index as UnitQuaternion
pub fn get_{field_name}(&self, idx: usize) -> crate::Quat {{
    debug_assert!(idx < {count_expr}, "{field_name} index out of bounds");
    let data = self.{field_name}();
    let i = idx * {stride};
    // MuJoCo uses (w, x, y, z), nalgebra Quaternion::new takes (w, x, y, z)
    crate::Quat::from_quaternion(nalgebra::Quaternion::new(
        data[i], data[i + 1], data[i + 2], data[i + 3]
    ))
}}"""
        setter = f"""
/// Set {field_name} at index from UnitQuaternion
pub fn set_{field_name}(&mut self, idx: usize, value: &crate::Quat) {{
    debug_assert!(idx < {count_expr}, "{field_name} index out of bounds");
    let data = self.{field_name}_mut();
    let i = idx * {stride};
    data[i] = value.w;
    data[i + 1] = value.i;
    data[i + 2] = value.j;
    data[i + 3] = value.k;
}}""" if has_setter else None

    elif nalgebra_type == "Mat3":
        getter = f"""
/// Get {field_name} at index as Mat3
pub fn get_{field_name}(&self, idx: usize) -> crate::Mat3 {{
    debug_assert!(idx < {count_expr}, "{field_name} index out of bounds");
    let data = self.{field_name}();
    let i = idx * {stride};
    // MuJoCo stores row-major, nalgebra is column-major
    crate::Mat3::from_row_slice(&data[i..i + 9])
}}"""
        setter = f"""
/// Set {field_name} at index from Mat3
pub fn set_{field_name}(&mut self, idx: usize, value: &crate::Mat3) {{
    debug_assert!(idx < {count_expr}, "{field_name} index out of bounds");
    let data = self.{field_name}_mut();
    let i = idx * {stride};
    // Convert from column-major nalgebra to row-major MuJoCo
    for row in 0..3 {{
        for col in 0..3 {{
            data[i + row * 3 + col] = value[(row, col)];
        }}
    }}
}}""" if has_setter else None

    else:
        raise ValueError(f"Unknown nalgebra type: {nalgebra_type}")

    return (getter, setter)


def generate_math_rs():
    """Generate math.rs with nalgebra type aliases."""
    content = '''//! Math type aliases using nalgebra
//! Generated by generate_from_introspect.py - DO NOT EDIT MANUALLY

pub use nalgebra::{
    DMatrix, DVector, Isometry3, Matrix3, Matrix3xX, Matrix4, Matrix6xX, MatrixXx3, MatrixXx6,
    Quaternion, Rotation3, UnitQuaternion, Vector3, Vector4, Vector6,
};

/// 3D vector (position, velocity, etc.)
pub type Vec3 = Vector3<f64>;

/// 4D vector
pub type Vec4 = Vector4<f64>;

/// 6D vector (spatial velocity, wrench, etc.)
pub type Vec6 = Vector6<f64>;

/// 3x3 matrix (rotation, inertia, etc.)
pub type Mat3 = Matrix3<f64>;

/// 4x4 matrix (homogeneous transform)
pub type Mat4 = Matrix4<f64>;

/// Unit quaternion (orientation)
pub type Quat = UnitQuaternion<f64>;

/// 6D pose (position + orientation)
pub type Pose = Isometry3<f64>;

/// 3×nv Jacobian matrix (position or rotation Jacobian)
pub type Jacobian3xN = Matrix3xX<f64>;

/// 6×nv Jacobian matrix (combined position + rotation Jacobian)
pub type Jacobian6xN = Matrix6xX<f64>;

/// nv×6 transposed Jacobian (internal, for row-major FFI conversion)
pub(crate) type JacobianNx6 = MatrixXx6<f64>;

/// nv×3 transposed Jacobian (internal, for row-major FFI conversion)
pub(crate) type JacobianNx3 = MatrixXx3<f64>;
'''
    output_path = FILE_DIR / ".." / "mujoco" / "src" / "math.rs"
    with open(output_path, "w") as f:
        f.write(content)
    print("Generated math.rs")


def generate_value_accessor(field):
    rust_type = RUST_TYPES.get(field.type.name, field.type.name)
    cast_type = f"as {rust_type}"
    includes = []
    if rust_type == field.type.name:
        cast_type = ""
        includes.append(rust_type)
    return (
        f"pub fn {field.name}(&self) -> {rust_type} {{ self.raw().{field.name} {cast_type} }}",
        includes,
    )


def generate_array_accessor(field):
    array_type_name = field.type.inner_type.name
    rust_type = RUST_ARRAY_TYPES.get(
        array_type_name, RUST_TYPES.get(array_type_name, array_type_name)
    )
    includes = []
    if rust_type == array_type_name:
        includes.append(rust_type)
    return (
        f"""pub fn {field.name}(&self) -> &[{rust_type};{field.type.extents[0]}] {{
            &self.raw().{field.name}
    }}""",
        includes,
    )


# mjModel accessors
model_struct = structs.STRUCTS["mjModel"]
model_accessors = []
model_includes = set(["mjtNum"])
model_fields = set()

for field in model_struct.fields:
    model_fields.add(field.name)
    match type(field.type):
        case structs.ValueType:
            accessor, includes = generate_value_accessor(field)
            model_includes.update(includes)
            model_accessors.append(accessor)

        case structs.PointerType:
            pointee_type = field.type.inner_type.name
            rust_type = RUST_POINTER_TYPES.get(
                pointee_type, RUST_TYPES.get(pointee_type, pointee_type)
            )
            if rust_type == pointee_type:
                model_includes.add(rust_type)
            if field.array_extent is None and field.type.inner_type.name == "void":
                print(f"mjModel: Skipping void pointer field: {field.name}")
                continue
            array_extents = []
            for extent in field.array_extent:
                if isinstance(extent, int):
                    array_extents.append(str(extent))
                elif extent in model_fields:
                    array_extents.append(f"self.{extent}()")
                elif extent.startswith("mjN"):
                    NAME_OVERRIDE = {"mjNTEXROLE": "mjtTextureRole::mjNTEXROLE"}
                    array_extents.append(
                        f"(mujoco_sys::{NAME_OVERRIDE.get(extent, extent)} as usize)"
                    )
                elif "*" in extent:
                    left_part, right_part = extent.split("*")
                    array_extents.extend([f"self.{left_part}()", str(right_part)])
                else:
                    raise NotImplementedError(
                        f"Unsupported array extent: {field.array_extent}"
                    )
            array_extent = " * ".join(array_extents)
            model_accessors.append(
                f"""
/// {field.doc}
pub fn {field.name}(&self) -> &[{rust_type}] {{
    unsafe {{
        std::slice::from_raw_parts(self.raw().{field.name}, {array_extent})
    }}
}}"""
            )

        case structs.ArrayType:
            accessor, includes = generate_array_accessor(field)
            model_includes.update(includes)
            model_accessors.append(accessor)
        case _:
            raise NotImplementedError(f"Unsupported field type: {field}")

model_includes.discard("void")
model_header = f"""//! Auto-generated Model accessor functions
//! Generated by generate_from_introspect.py - DO NOT EDIT MANUALLY

use crate::Model;
use mujoco_sys::{{{', '.join(sorted(model_includes))}}};

#[allow(non_snake_case)]
impl Model {{
"""
footer = "}\n"

# mjData accessors
data_struct = structs.STRUCTS["mjData"]
data_accessors = []
data_includes = set(["mjtNum"])

for field in data_struct.fields:
    match type(field.type):
        case structs.ValueType:
            accessor, includes = generate_value_accessor(field)
            data_includes.update(includes)
            data_accessors.append(accessor)

        case structs.PointerType:
            pointee_type = field.type.inner_type.name
            rust_type = RUST_POINTER_TYPES.get(
                pointee_type, RUST_TYPES.get(pointee_type, pointee_type)
            )
            if rust_type == pointee_type:
                data_includes.add(rust_type)
            if field.array_extent is None and field.type.inner_type.name == "void":
                print(f"mjData: Skipping void pointer field: {field.name}")
                continue
            array_extent = (
                f"self.model.{field.array_extent[0]}()"
                if field.array_extent[0] in model_fields
                else f"self.{field.array_extent[0]}()"
            )
            if len(field.array_extent) > 1:
                assert len(field.array_extent) == 2
                if isinstance(field.array_extent[1], int):
                    array_extent = f"{array_extent} * {field.array_extent[1]}"
                elif field.array_extent[1] in model_fields:
                    array_extent = (
                        f"{array_extent} * self.model.{field.array_extent[1]}()"
                    )
                else:
                    raise NotImplementedError(
                        f"Unsupported array extent: {field.array_extent[1]}"
                    )
            # TODO: Should we use .as_chunks_unchecked::<3>() for 2D arrays with extent 3?
            # The return type would be &[[T; 3]]
            data_accessors.append(
                f"""
/// {field.doc}
pub fn {field.name}(&self) -> &[{rust_type}] {{
    unsafe {{
        std::slice::from_raw_parts(self.raw().{field.name}, {array_extent})
    }}
}}"""
            )
            data_accessors.append(
                f"""
/// {field.doc}
pub fn {field.name}_mut(&mut self) -> &mut [{rust_type}] {{
    unsafe {{
        std::slice::from_raw_parts_mut(self.raw_mut().{field.name}, {array_extent})
        }}
}}"""
            )
            # Generate nalgebra accessors for spatial data fields
            if field.name in NALGEBRA_DATA_FIELDS:
                stride, nalgebra_type, count_field, has_setter = NALGEBRA_DATA_FIELDS[field.name]
                getter, setter = generate_nalgebra_accessor(
                    field.name, stride, nalgebra_type, count_field, has_setter
                )
                data_accessors.append(getter)
                if setter:
                    data_accessors.append(setter)

        case structs.ArrayType:
            accessor, includes = generate_array_accessor(field)
            data_includes.update(includes)
            data_accessors.append(accessor)

        case _:
            raise NotImplementedError(f"Unsupported field type: {field}")

data_includes.discard("void")

data_header = f"""//! Auto-generated Data accessor functions
//! Generated by generate_from_introspect.py - DO NOT EDIT MANUALLY

use crate::Data;
use mujoco_sys::{{{', '.join(sorted(data_includes))}}};

#[allow(non_snake_case)]
impl<'a> Data<'a> {{
"""

save_file("model_struct.rs", model_header, model_accessors, footer)
save_file("data_struct.rs", data_header, data_accessors, footer)

# =============================================================================
# Manually specified mj_* functions with special signatures
# =============================================================================

MANUAL_MJ_FUNCTIONS = {
    "mj_version": lambda f: f"""
/// {f.doc}
pub fn version() -> i32 {{
    unsafe {{ mujoco_sys::mj_version() }}
}}""",
    "mj_versionString": lambda f: f"""
/// {f.doc}
pub fn version_string() -> &'static str {{
    unsafe {{
        std::ffi::CStr::from_ptr(mujoco_sys::mj_versionString())
            .to_str()
            .unwrap_or("")
    }}
}}""",
    "mj_name2id": lambda f: f"""
/// {f.doc}
pub fn name2id(model: &crate::Model, obj_type: mujoco_sys::mjtObj, name: &str) -> Option<i32> {{
    let name_cstr = std::ffi::CString::new(name).unwrap();
    let id = unsafe {{ mujoco_sys::mj_name2id(model.as_ptr(), obj_type as i32, name_cstr.as_ptr()) }};
    if id >= 0 {{ Some(id) }} else {{ None }}
}}""",
    "mj_id2name": lambda f: f"""
/// {f.doc}
pub fn id2name<'a>(model: &'a crate::Model, obj_type: mujoco_sys::mjtObj, id: i32) -> Option<&'a str> {{
    unsafe {{
        let ptr = mujoco_sys::mj_id2name(model.as_ptr(), obj_type as i32, id);
        if ptr.is_null() {{
            None
        }} else {{
            Some(std::ffi::CStr::from_ptr(ptr).to_str().unwrap_or(""))
        }}
    }}
}}""",
    "mj_resetDataKeyframe": lambda f: f"""
/// {f.doc}
pub fn reset_data_keyframe(data: &mut crate::Data, key: i32) {{
    unsafe {{ mujoco_sys::mj_resetDataKeyframe(data.model.as_ptr(), data.as_mut_ptr(), key) }}
}}""",
    "mj_differentiatePos": lambda f: f"""
/// {f.doc}
pub fn differentiate_pos(model: &crate::Model, qvel: &mut [f64], dt: f64, qpos1: &[f64], qpos2: &[f64]) {{
    unsafe {{ mujoco_sys::mj_differentiatePos(model.as_ptr(), qvel.as_mut_ptr(), dt, qpos1.as_ptr(), qpos2.as_ptr()) }}
}}""",
    "mj_integratePos": lambda f: f"""
/// {f.doc}
pub fn integrate_pos(model: &crate::Model, qpos: &mut [f64], qvel: &[f64], dt: f64) {{
    unsafe {{ mujoco_sys::mj_integratePos(model.as_ptr(), qpos.as_mut_ptr(), qvel.as_ptr(), dt) }}
}}""",
    "mj_normalizeQuat": lambda f: f"""
/// {f.doc}
pub fn normalize_quat(model: &crate::Model, qpos: &mut [f64]) {{
    unsafe {{ mujoco_sys::mj_normalizeQuat(model.as_ptr(), qpos.as_mut_ptr()) }}
}}""",
    "mj_objectVelocity": lambda f: f"""
/// {f.doc}
pub fn object_velocity(data: &crate::Data, objtype: mujoco_sys::mjtObj, objid: i32, flg_local: bool) -> crate::Vec6 {{
    let mut res = crate::Vec6::zeros();
    unsafe {{ mujoco_sys::mj_objectVelocity(data.model.as_ptr(), data.as_ptr(), objtype as i32, objid, res.as_mut_ptr(), flg_local as i32) }}
    res
}}""",
    "mj_objectAcceleration": lambda f: f"""
/// {f.doc}
pub fn object_acceleration(data: &crate::Data, objtype: mujoco_sys::mjtObj, objid: i32, flg_local: bool) -> crate::Vec6 {{
    let mut res = crate::Vec6::zeros();
    unsafe {{ mujoco_sys::mj_objectAcceleration(data.model.as_ptr(), data.as_ptr(), objtype as i32, objid, res.as_mut_ptr(), flg_local as i32) }}
    res
}}""",
    "mj_contactForce": lambda f: f"""
/// {f.doc}
pub fn contact_force(data: &crate::Data, id: i32) -> crate::Vec6 {{
    let mut res = crate::Vec6::zeros();
    unsafe {{ mujoco_sys::mj_contactForce(data.model.as_ptr(), data.as_ptr(), id, res.as_mut_ptr()) }}
    res
}}""",
    "mj_geomDistance": lambda f: f"""
/// {f.doc}
pub fn geom_distance(data: &crate::Data, geom1: i32, geom2: i32, distmax: f64) -> (f64, crate::Vec6) {{
    let mut fromto = crate::Vec6::zeros();
    let dist = unsafe {{ mujoco_sys::mj_geomDistance(data.model.as_ptr(), data.as_ptr(), geom1, geom2, distmax, fromto.as_mut_ptr()) }};
    (dist, fromto)
}}""",
    "mj_jac": lambda f: f"""
/// {f.doc}
/// Returns a 6×nv Jacobian matrix (top 3 rows: position, bottom 3 rows: rotation).
pub fn jac(data: &crate::Data, point: &crate::Vec3, body: i32) -> crate::Jacobian6xN {{
    debug_assert!(
        (body as usize) < data.model.nbody(),
        "body index {{}} out of bounds (nbody = {{}})", body, data.model.nbody()
    );
    let nv = data.model.nv();
    let mut jac_t = crate::JacobianNx6::zeros(nv);
    unsafe {{
        let ptr = jac_t.as_mut_ptr();
        mujoco_sys::mj_jac(data.model.as_ptr(), data.as_ptr(), ptr, ptr.add(3 * nv), point.as_ptr(), body);
    }}
    jac_t.transpose()
}}""",
    "mj_jacBody": lambda f: f"""
/// {f.doc}
/// Returns a 6×nv Jacobian matrix (top 3 rows: position, bottom 3 rows: rotation).
pub fn jac_body(data: &crate::Data, body: i32) -> crate::Jacobian6xN {{
    debug_assert!(
        (body as usize) < data.model.nbody(),
        "body index {{}} out of bounds (nbody = {{}})", body, data.model.nbody()
    );
    let nv = data.model.nv();
    let mut jac_t = crate::JacobianNx6::zeros(nv);
    unsafe {{
        let ptr = jac_t.as_mut_ptr();
        mujoco_sys::mj_jacBody(data.model.as_ptr(), data.as_ptr(), ptr, ptr.add(3 * nv), body);
    }}
    jac_t.transpose()
}}""",
    "mj_jacBodyCom": lambda f: f"""
/// {f.doc}
/// Returns a 6×nv Jacobian matrix (top 3 rows: position, bottom 3 rows: rotation).
pub fn jac_body_com(data: &crate::Data, body: i32) -> crate::Jacobian6xN {{
    debug_assert!(
        (body as usize) < data.model.nbody(),
        "body index {{}} out of bounds (nbody = {{}})", body, data.model.nbody()
    );
    let nv = data.model.nv();
    let mut jac_t = crate::JacobianNx6::zeros(nv);
    unsafe {{
        let ptr = jac_t.as_mut_ptr();
        mujoco_sys::mj_jacBodyCom(data.model.as_ptr(), data.as_ptr(), ptr, ptr.add(3 * nv), body);
    }}
    jac_t.transpose()
}}""",
    "mj_jacSubtreeCom": lambda f: f"""
/// {f.doc}
/// Returns a 3×nv position Jacobian matrix.
pub fn jac_subtree_com(data: &mut crate::Data, body: i32) -> crate::Jacobian3xN {{
    debug_assert!(
        (body as usize) < data.model.nbody(),
        "body index {{}} out of bounds (nbody = {{}})", body, data.model.nbody()
    );
    let nv = data.model.nv();
    let mut jac_t = crate::JacobianNx3::zeros(nv);
    unsafe {{
        mujoco_sys::mj_jacSubtreeCom(data.model.as_ptr(), data.as_mut_ptr(), jac_t.as_mut_ptr(), body);
    }}
    jac_t.transpose()
}}""",
    "mj_jacGeom": lambda f: f"""
/// {f.doc}
/// Returns a 6×nv Jacobian matrix (top 3 rows: position, bottom 3 rows: rotation).
pub fn jac_geom(data: &crate::Data, geom: i32) -> crate::Jacobian6xN {{
    debug_assert!(
        (geom as usize) < data.model.ngeom(),
        "geom index {{}} out of bounds (ngeom = {{}})", geom, data.model.ngeom()
    );
    let nv = data.model.nv();
    let mut jac_t = crate::JacobianNx6::zeros(nv);
    unsafe {{
        let ptr = jac_t.as_mut_ptr();
        mujoco_sys::mj_jacGeom(data.model.as_ptr(), data.as_ptr(), ptr, ptr.add(3 * nv), geom);
    }}
    jac_t.transpose()
}}""",
    "mj_jacSite": lambda f: f"""
/// {f.doc}
/// Returns a 6×nv Jacobian matrix (top 3 rows: position, bottom 3 rows: rotation).
pub fn jac_site(data: &crate::Data, site: i32) -> crate::Jacobian6xN {{
    debug_assert!(
        (site as usize) < data.model.nsite(),
        "site index {{}} out of bounds (nsite = {{}})", site, data.model.nsite()
    );
    let nv = data.model.nv();
    let mut jac_t = crate::JacobianNx6::zeros(nv);
    unsafe {{
        let ptr = jac_t.as_mut_ptr();
        mujoco_sys::mj_jacSite(data.model.as_ptr(), data.as_ptr(), ptr, ptr.add(3 * nv), site);
    }}
    jac_t.transpose()
}}""",
    "mj_jacPointAxis": lambda f: f"""
/// {f.doc}
/// Returns a 6×nv Jacobian matrix (top 3 rows: point translation, bottom 3 rows: axis rotation).
pub fn jac_point_axis(data: &mut crate::Data, point: &crate::Vec3, axis: &crate::Vec3, body: i32) -> crate::Jacobian6xN {{
    debug_assert!(
        (body as usize) < data.model.nbody(),
        "body index {{}} out of bounds (nbody = {{}})", body, data.model.nbody()
    );
    let nv = data.model.nv();
    let mut jac_t = crate::JacobianNx6::zeros(nv);
    unsafe {{
        let ptr = jac_t.as_mut_ptr();
        mujoco_sys::mj_jacPointAxis(data.model.as_ptr(), data.as_mut_ptr(), ptr, ptr.add(3 * nv), point.as_ptr(), axis.as_ptr(), body);
    }}
    jac_t.transpose()
}}""",
    "mj_jacDot": lambda f: f"""
/// {f.doc}
/// Returns a 6×nv Jacobian time derivative matrix (top 3 rows: position, bottom 3 rows: rotation).
pub fn jac_dot(data: &crate::Data, point: &crate::Vec3, body: i32) -> crate::Jacobian6xN {{
    debug_assert!(
        (body as usize) < data.model.nbody(),
        "body index {{}} out of bounds (nbody = {{}})", body, data.model.nbody()
    );
    let nv = data.model.nv();
    let mut jac_t = crate::JacobianNx6::zeros(nv);
    unsafe {{
        let ptr = jac_t.as_mut_ptr();
        mujoco_sys::mj_jacDot(data.model.as_ptr(), data.as_ptr(), ptr, ptr.add(3 * nv), point.as_ptr(), body);
    }}
    jac_t.transpose()
}}""",
    "mj_angmomMat": lambda f: f"""
/// {f.doc}
/// Returns a 3×nv angular momentum matrix.
pub fn angmom_mat(data: &mut crate::Data, body: i32) -> crate::Jacobian3xN {{
    debug_assert!(
        (body as usize) < data.model.nbody(),
        "body index {{}} out of bounds (nbody = {{}})", body, data.model.nbody()
    );
    let nv = data.model.nv();
    let mut mat_t = crate::JacobianNx3::zeros(nv);
    unsafe {{
        mujoco_sys::mj_angmomMat(data.model.as_ptr(), data.as_mut_ptr(), mat_t.as_mut_ptr(), body);
    }}
    mat_t.transpose()
}}""",
}

data_functions = []

for function_name, function in functions.FUNCTIONS.items():
    if function_name in SKIP_FUNCTIONS:
        continue
    if function_name.startswith(SKIP_PREFIXES):
        continue
    if not function_name.startswith("mj_"):
        continue
    if function_name in MANUAL_MJ_FUNCTIONS:
        continue
    if len(function.parameters) != 2:
        print(f"Skipping mj_ function {function}")
        continue
    first_param, second_param = function.parameters
    if (
        not isinstance(first_param.type, structs.PointerType)
        or first_param.type.inner_type.name != "mjModel"
        or not first_param.type.inner_type.is_const
    ):
        print(f"Skipping mj_ function {function}")
        continue
    if (
        not isinstance(second_param.type, structs.PointerType)
        or second_param.type.inner_type.name != "mjData"
    ):
        print(f"Skipping mj_ function {function}")
        continue
    assert (
        not second_param.type.inner_type.is_const
    ), f"Second parameter to {function.name} should be non-const"
    assert (
        function.return_type.name == "void"
    ), f"Function {function.name} should return void"

    function_name = camel_to_snake(function.name).removeprefix("mj_")
    data_functions.append(
        f"""
/// {function.doc}
pub fn {function_name}(data: &mut Data) {{
        unsafe {{
            mujoco_sys::{function.name}(data.model.as_ptr(), data.as_mut_ptr());
        }}
    }}"""
    )
data_functions_header = f"""//! Auto-generated Data functions
//! Generated by generate_from_introspect.py - DO NOT EDIT MANUALLY

use crate::Data;
"""

save_file("data_functions.rs", data_functions_header, data_functions, "")

# Generate manual functions using docs from introspect
output_path = FILE_DIR / ".." / "mujoco" / "src" / "data_functions.rs"
with open(output_path, "a") as f:
    for func_name, generator in MANUAL_MJ_FUNCTIONS.items():
        if func_name in functions.FUNCTIONS:
            f.write(generator(functions.FUNCTIONS[func_name]))
            f.write("\n")


# Enums
def generate_from_trait(enum_name, prefix):
    enum = enums.ENUMS[enum_name]
    lines = [
        f"""
impl From<usize> for {enum_name} {{
    fn from(value: usize) -> Self {{
        match value {{
"""
    ]
    for key, value in enum.values.items():
        variant_name = key.removeprefix(prefix)
        lines.append(f"            {value} => {enum_name}::{variant_name},")
    lines.append(
        f"""
            _ => panic!("Invalid value for {enum_name}: {{}}", value),
        }}
    }}
}}"""
    )
    return "\n".join(lines)


LIB_RS_MARKER = "// === GENERATED BY generate_from_introspect.py - DO NOT EDIT BELOW THIS LINE ==="

lib_rs_path = FILE_DIR / "src" / "lib.rs"
lib_rs_content = lib_rs_path.read_text()
if LIB_RS_MARKER in lib_rs_content:
    lib_rs_content = lib_rs_content[: lib_rs_content.index(LIB_RS_MARKER)]
with lib_rs_path.open("w") as f:
    f.write(lib_rs_content)
    f.write(LIB_RS_MARKER + "\n")
    f.write(generate_from_trait("mjtGeom", "mjGEOM_"))
    f.write(generate_from_trait("mjtJoint", "mjJNT_"))

# Generate math.rs with nalgebra type aliases
generate_math_rs()
