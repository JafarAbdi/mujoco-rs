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
    "float": "f32",
    "uint64_t": "u64",
    "mjtByte": "u8",
    "size_t": "usize",
    "uintptr_t": "usize",
}
RUST_POINTER_TYPES = {
    "int": "i32",
    "char": "i8",
}
RUST_ARRAY_TYPES = {
    "int": "i32",
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
    # Functions that are not needed or have complex signatures
    # Complex/unsafe functions
    "mju_malloc",
    "mju_free",
    "mju_error",
    "mju_warning",
    "mju_clearHandlers",
    "mju_threadPoolCreate",
    "mju_threadPoolDestroy",
    "mju_defaultTask",
    "mju_taskJoin",
    "mju_boxQPmalloc",
    # Sparse matrix ops
    "mju_dense2sparse",
    "mju_sparse2dense",
    "mju_printMatSparse",
    # Complex derivative functions
    "mjd_transitionFD",
    "mjd_inverseFD",
    # Threading
    "mju_bindThreadPool",
    "mju_threadPoolEnqueue",
    # String/type conversion
    "mju_type2Str",
    "mju_str2Type",
    "mju_writeNumBytes",
    "mju_strncpy",
    # Error/warning helpers
    "mju_error_i",
    "mju_error_s",
    "mju_warning_i",
    "mju_warning_s",
    "mju_writeLog",
    # Physics utilities (complex signatures)
    "mju_encodePyramid",
    "mju_decodePyramid",
    "mju_springDamper",
    "mju_clip",
    "mju_muscleGain",
    "mju_muscleBias",
    "mju_muscleDynamics",
    # Type conversion (float/double)
    "mju_f2n",
    "mju_n2f",
    "mju_d2n",
    "mju_n2d",
    # Complex/other
    "mju_boxQP",
    "mju_standardNormal",
    "mju_printMat",
    "mju_rayFlex",
    "mju_raySkin",
    # Sorting
    "mju_insertionSort",
    "mju_insertionSortInt",
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
}

# Prefixes to skip for now
SKIP_PREFIXES = ("mjs_", "mjc_", "mjp_", "mjui_", "mjr_", "mjv_")


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
use mujoco_sys::{{{', '.join(model_includes)}}};

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
use mujoco_sys::{{{', '.join(data_includes)}}};

#[allow(non_snake_case)]
impl<'a> Data<'a> {{
"""

save_file("model_struct.rs", model_header, model_accessors, footer)
save_file("data_struct.rs", data_header, data_accessors, footer)

data_functions = []

for function_name, function in functions.FUNCTIONS.items():
    if function_name in SKIP_FUNCTIONS:
        continue
    if function_name.startswith(SKIP_PREFIXES):
        continue
    if not function_name.startswith("mj_"):
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

    function_name = camel_to_snake(function.name)
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

# =============================================================================
# Manually specified mj_* functions with special signatures
# =============================================================================

MANUAL_MJ_FUNCTIONS = {
    "mj_version": lambda f: f"""
/// {f.doc}
pub fn mj_version() -> i32 {{
    unsafe {{ mujoco_sys::mj_version() }}
}}""",
    "mj_versionString": lambda f: f"""
/// {f.doc}
pub fn mj_version_string() -> &'static str {{
    unsafe {{
        std::ffi::CStr::from_ptr(mujoco_sys::mj_versionString())
            .to_str()
            .unwrap_or("")
    }}
}}""",
    "mj_name2id": lambda f: f"""
/// {f.doc}
pub fn mj_name2id(model: &crate::Model, obj_type: i32, name: &str) -> i32 {{
    let name_cstr = std::ffi::CString::new(name).unwrap();
    unsafe {{ mujoco_sys::mj_name2id(model.as_ptr(), obj_type, name_cstr.as_ptr()) }}
}}""",
    "mj_id2name": lambda f: f"""
/// {f.doc}
pub fn mj_id2name(model: &crate::Model, obj_type: i32, id: i32) -> &'static str {{
    unsafe {{
        let ptr = mujoco_sys::mj_id2name(model.as_ptr(), obj_type, id);
        if ptr.is_null() {{
            ""
        }} else {{
            std::ffi::CStr::from_ptr(ptr).to_str().unwrap_or("")
        }}
    }}
}}""",
    "mj_resetDataKeyframe": lambda f: f"""
/// {f.doc}
pub fn mj_reset_data_keyframe(data: &mut crate::Data, key: i32) {{
    unsafe {{ mujoco_sys::mj_resetDataKeyframe(data.model.as_ptr(), data.as_mut_ptr(), key) }}
}}""",
    "mj_differentiatePos": lambda f: f"""
/// {f.doc}
pub fn mj_differentiate_pos(model: &crate::Model, qvel: &mut [f64], dt: f64, qpos1: &[f64], qpos2: &[f64]) {{
    unsafe {{ mujoco_sys::mj_differentiatePos(model.as_ptr(), qvel.as_mut_ptr(), dt, qpos1.as_ptr(), qpos2.as_ptr()) }}
}}""",
    "mj_integratePos": lambda f: f"""
/// {f.doc}
pub fn mj_integrate_pos(model: &crate::Model, qpos: &mut [f64], qvel: &[f64], dt: f64) {{
    unsafe {{ mujoco_sys::mj_integratePos(model.as_ptr(), qpos.as_mut_ptr(), qvel.as_ptr(), dt) }}
}}""",
    "mj_normalizeQuat": lambda f: f"""
/// {f.doc}
pub fn mj_normalize_quat(model: &crate::Model, qpos: &mut [f64]) {{
    unsafe {{ mujoco_sys::mj_normalizeQuat(model.as_ptr(), qpos.as_mut_ptr()) }}
}}""",
    "mj_objectVelocity": lambda f: f"""
/// {f.doc}
pub fn mj_object_velocity(data: &crate::Data, objtype: i32, objid: i32, flg_local: i32) -> [f64; 6] {{
    let mut res = [0.0; 6];
    unsafe {{ mujoco_sys::mj_objectVelocity(data.model.as_ptr(), data.as_ptr(), objtype, objid, res.as_mut_ptr(), flg_local) }}
    res
}}""",
    "mj_objectAcceleration": lambda f: f"""
/// {f.doc}
pub fn mj_object_acceleration(data: &crate::Data, objtype: i32, objid: i32, flg_local: i32) -> [f64; 6] {{
    let mut res = [0.0; 6];
    unsafe {{ mujoco_sys::mj_objectAcceleration(data.model.as_ptr(), data.as_ptr(), objtype, objid, res.as_mut_ptr(), flg_local) }}
    res
}}""",
    "mj_contactForce": lambda f: f"""
/// {f.doc}
pub fn mj_contact_force(data: &crate::Data, id: i32) -> [f64; 6] {{
    let mut res = [0.0; 6];
    unsafe {{ mujoco_sys::mj_contactForce(data.model.as_ptr(), data.as_ptr(), id, res.as_mut_ptr()) }}
    res
}}""",
    "mj_geomDistance": lambda f: f"""
/// {f.doc}
pub fn mj_geom_distance(data: &crate::Data, geom1: i32, geom2: i32, distmax: f64) -> (f64, [f64; 6]) {{
    let mut fromto = [0.0; 6];
    let dist = unsafe {{ mujoco_sys::mj_geomDistance(data.model.as_ptr(), data.as_ptr(), geom1, geom2, distmax, fromto.as_mut_ptr()) }};
    (dist, fromto)
}}""",
    "mj_jac": lambda f: f"""
/// {f.doc}
pub fn mj_jac(data: &crate::Data, jacp: &mut [f64], jacr: &mut [f64], point: &[f64; 3], body: i32) {{
    unsafe {{ mujoco_sys::mj_jac(data.model.as_ptr(), data.as_ptr(), jacp.as_mut_ptr(), jacr.as_mut_ptr(), point.as_ptr(), body) }}
}}""",
    "mj_jacBody": lambda f: f"""
/// {f.doc}
pub fn mj_jac_body(data: &crate::Data, jacp: &mut [f64], jacr: &mut [f64], body: i32) {{
    unsafe {{ mujoco_sys::mj_jacBody(data.model.as_ptr(), data.as_ptr(), jacp.as_mut_ptr(), jacr.as_mut_ptr(), body) }}
}}""",
    "mj_jacBodyCom": lambda f: f"""
/// {f.doc}
pub fn mj_jac_body_com(data: &crate::Data, jacp: &mut [f64], jacr: &mut [f64], body: i32) {{
    unsafe {{ mujoco_sys::mj_jacBodyCom(data.model.as_ptr(), data.as_ptr(), jacp.as_mut_ptr(), jacr.as_mut_ptr(), body) }}
}}""",
    "mj_jacSubtreeCom": lambda f: f"""
/// {f.doc}
pub fn mj_jac_subtree_com(data: &mut crate::Data, jacp: &mut [f64], body: i32) {{
    unsafe {{ mujoco_sys::mj_jacSubtreeCom(data.model.as_ptr(), data.as_mut_ptr(), jacp.as_mut_ptr(), body) }}
}}""",
    "mj_jacGeom": lambda f: f"""
/// {f.doc}
pub fn mj_jac_geom(data: &crate::Data, jacp: &mut [f64], jacr: &mut [f64], geom: i32) {{
    unsafe {{ mujoco_sys::mj_jacGeom(data.model.as_ptr(), data.as_ptr(), jacp.as_mut_ptr(), jacr.as_mut_ptr(), geom) }}
}}""",
    "mj_jacSite": lambda f: f"""
/// {f.doc}
pub fn mj_jac_site(data: &crate::Data, jacp: &mut [f64], jacr: &mut [f64], site: i32) {{
    unsafe {{ mujoco_sys::mj_jacSite(data.model.as_ptr(), data.as_ptr(), jacp.as_mut_ptr(), jacr.as_mut_ptr(), site) }}
}}""",
    "mj_jacPointAxis": lambda f: f"""
/// {f.doc}
pub fn mj_jac_point_axis(data: &mut crate::Data, jac_point: &mut [f64], jac_axis: &mut [f64], point: &[f64; 3], axis: &[f64; 3], body: i32) {{
    unsafe {{ mujoco_sys::mj_jacPointAxis(data.model.as_ptr(), data.as_mut_ptr(), jac_point.as_mut_ptr(), jac_axis.as_mut_ptr(), point.as_ptr(), axis.as_ptr(), body) }}
}}""",
    "mj_jacDot": lambda f: f"""
/// {f.doc}
pub fn mj_jac_dot(data: &crate::Data, jacp: &mut [f64], jacr: &mut [f64], point: &[f64; 3], body: i32) {{
    unsafe {{ mujoco_sys::mj_jacDot(data.model.as_ptr(), data.as_ptr(), jacp.as_mut_ptr(), jacr.as_mut_ptr(), point.as_ptr(), body) }}
}}""",
    "mj_angmomMat": lambda f: f"""
/// {f.doc}
pub fn mj_angmom_mat(data: &mut crate::Data, mat: &mut [f64], body: i32) {{
    unsafe {{ mujoco_sys::mj_angmomMat(data.model.as_ptr(), data.as_mut_ptr(), mat.as_mut_ptr(), body) }}
}}""",
}

# Generate manual functions using docs from introspect
output_path = FILE_DIR / ".." / "mujoco" / "src" / "data_functions.rs"
with open(output_path, "a") as f:
    for func_name, generator in MANUAL_MJ_FUNCTIONS.items():
        if func_name in functions.FUNCTIONS:
            f.write(generator(functions.FUNCTIONS[func_name]))
            f.write("\n")


# =============================================================================
# Math utility functions (mju_* and mjd_*)
# =============================================================================

# In-place modify functions (use &mut for first param, don't return)
IN_PLACE_FUNCTIONS = {
    "mju_normalize3",
    "mju_normalize4",
    "mju_quatIntegrate",
    "mju_zero3",
    "mju_zero4",
    "mju_unit4",
}

MATH_TYPE_MAP = {
    "mjtNum": "f64",
    "int": "i32",
    "float": "f32",
    "double": "f64",
    "mjtByte": "u8",
    "size_t": "usize",
}

# Rust reserved keywords that need escaping
RUST_KEYWORDS = {
    "type",
}


def escape_keyword(name):
    """Escape Rust keywords with r# prefix."""
    if name in RUST_KEYWORDS:
        return f"r#{name}"
    return name


def is_array_type(param_type):
    return isinstance(param_type, structs.ArrayType)


def is_value_type(param_type):
    return isinstance(param_type, structs.ValueType)


def is_pointer_type(param_type):
    return isinstance(param_type, structs.PointerType)


def get_array_info(param_type):
    """Returns (inner_type, size) for array types."""
    if is_array_type(param_type):
        return param_type.inner_type.name, param_type.extents[0]
    return None, None


def get_pointer_info(param_type):
    """Returns inner type name for pointer types."""
    if is_pointer_type(param_type):
        return param_type.inner_type.name, param_type.inner_type.is_const
    return None, None


def is_runtime_sized_function(params):
    """Check if function uses runtime-sized arrays (pointer + n pattern)."""
    has_pointer = False
    has_n_param = False
    for param in params:
        if is_pointer_type(param.type):
            inner = param.type.inner_type.name
            if inner in ("mjtNum", "int", "float", "double"):
                has_pointer = True
        if is_value_type(param.type) and param.name in ("n", "nr", "nc"):
            has_n_param = True
    return has_pointer and has_n_param


def generate_math_function(func_name, function):
    """Generate a math utility function binding."""
    params = function.parameters
    rust_name = camel_to_snake(func_name)
    doc = function.doc

    # Check if first param is output array (non-const)
    first_param = params[0] if params else None
    is_in_place = func_name in IN_PLACE_FUNCTIONS

    # Determine if this returns a value or has output param
    has_output_param = False
    output_type = None
    output_size = None

    if first_param and is_array_type(first_param.type):
        inner_type, size = get_array_info(first_param.type)
        if not first_param.type.inner_type.is_const:
            has_output_param = True
            output_type = MATH_TYPE_MAP.get(inner_type, inner_type)
            output_size = size

    # Build parameter list and call arguments
    rust_params = []
    call_args = []

    for i, param in enumerate(params):
        pname = escape_keyword(param.name)
        ptype = param.type

        if i == 0 and has_output_param:
            if is_in_place:
                # In-place: &mut [f64; N]
                rust_params.append(f"{pname}: &mut [{output_type}; {output_size}]")
                call_args.append(f"{pname}.as_mut_ptr()")
            else:
                # Output param handled separately (we create local array)
                call_args.append("res.as_mut_ptr()")
            continue

        if is_array_type(ptype):
            inner_type, size = get_array_info(ptype)
            rust_type = MATH_TYPE_MAP.get(inner_type, inner_type)
            is_const = ptype.inner_type.is_const
            if is_const:
                rust_params.append(f"{pname}: &[{rust_type}; {size}]")
                call_args.append(f"{pname}.as_ptr()")
            else:
                rust_params.append(f"{pname}: &mut [{rust_type}; {size}]")
                call_args.append(f"{pname}.as_mut_ptr()")
        elif is_value_type(ptype):
            rust_type = MATH_TYPE_MAP.get(ptype.name, ptype.name)
            rust_params.append(f"{pname}: {rust_type}")
            call_args.append(pname)
        elif is_pointer_type(ptype):
            inner_type, is_const = get_pointer_info(ptype)
            # Handle const char* as &str
            if inner_type == "char" and is_const:
                rust_params.append(f"{pname}: &str")
                call_args.append(f"std::ffi::CString::new({pname}).unwrap().as_ptr()")
            # Skip non-numeric pointer types (structs, etc.)
            elif inner_type not in ("mjtNum", "int", "float", "double"):
                return None
            else:
                rust_type = MATH_TYPE_MAP.get(inner_type, inner_type)
                if is_const:
                    rust_params.append(f"{pname}: &[{rust_type}]")
                    call_args.append(f"{pname}.as_ptr()")
                else:
                    rust_params.append(f"{pname}: &mut [{rust_type}]")
                    call_args.append(f"{pname}.as_mut_ptr()")
        else:
            return None

    params_str = ", ".join(rust_params)
    args_str = ", ".join(call_args)

    # Handle return type
    ret_type = function.return_type
    has_return = is_value_type(ret_type) and ret_type.name != "void"
    rust_ret_type = (
        MATH_TYPE_MAP.get(ret_type.name, ret_type.name) if has_return else None
    )

    # Generate function body
    if is_in_place:
        # In-place modification
        if has_return:
            return f"""
/// {doc}
pub fn {rust_name}({params_str}) -> {rust_ret_type} {{
    unsafe {{ mujoco_sys::{func_name}({args_str}) }}
}}"""
        else:
            return f"""
/// {doc}
pub fn {rust_name}({params_str}) {{
    unsafe {{ mujoco_sys::{func_name}({args_str}); }}
}}"""
    elif has_output_param:
        # Return array style
        if has_return:
            # Has both output array and return value - rare case
            return f"""
/// {doc}
pub fn {rust_name}({params_str}) -> ([{output_type}; {output_size}], {rust_ret_type}) {{
    let mut res = [0.0; {output_size}];
    let ret = unsafe {{ mujoco_sys::{func_name}({args_str}) }};
    (res, ret)
}}"""
        else:
            return f"""
/// {doc}
pub fn {rust_name}({params_str}) -> [{output_type}; {output_size}] {{
    let mut res = [0.0; {output_size}];
    unsafe {{ mujoco_sys::{func_name}({args_str}); }}
    res
}}"""
    else:
        # No output param, just return value or void
        if has_return:
            return f"""
/// {doc}
pub fn {rust_name}({params_str}) -> {rust_ret_type} {{
    unsafe {{ mujoco_sys::{func_name}({args_str}) }}
}}"""
        else:
            return f"""
/// {doc}
pub fn {rust_name}({params_str}) {{
    unsafe {{ mujoco_sys::{func_name}({args_str}); }}
}}"""


math_functions = []

for func_name, function in functions.FUNCTIONS.items():
    if func_name in SKIP_FUNCTIONS:
        continue

    # Only process mju_* and mjd_* functions (mj_* handled separately)
    if not (func_name.startswith("mju_") or func_name.startswith("mjd_")):
        continue

    result = generate_math_function(func_name, function)
    if result:
        math_functions.append(result)
    else:
        print(f"Skipping math function {function} - unsupported signature")

math_functions_header = """//! Auto-generated math utility functions
//! Generated by generate_from_introspect.py - DO NOT EDIT MANUALLY

#![allow(non_snake_case)]
"""

# Write math functions to file
output_path = FILE_DIR / ".." / "mujoco" / "src" / "math.rs"
with open(output_path, "w") as f:
    f.write(math_functions_header)
    for func in math_functions:
        f.write(f"{func}\n")


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


with (FILE_DIR / "src" / "lib.rs").open("a") as f:
    f.write(generate_from_trait("mjtGeom", "mjGEOM_"))
    f.write(generate_from_trait("mjtJoint", "mjJNT_"))
