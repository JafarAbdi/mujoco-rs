#!/bin/bash

SCRIPT_DIR=$(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)

bindgen $SCRIPT_DIR/binding.h -o $SCRIPT_DIR/src/lib.rs \
  --no-layout-tests \
  --no-doc-comments \
  --raw-line "#![allow(non_camel_case_types)]" \
  --raw-line "#![allow(non_snake_case)]" \
  --raw-line "#![allow(non_upper_case_globals)]" \
  --default-enum-style rust \
  --generate=functions,types,vars \
  --allowlist-function="(mj).*" \
  --allowlist-type="(mj).*" \
  --allowlist-var="mj.*" \
  -- -I$SCRIPT_DIR/mujoco/include

# Remove prefixes from enums
prefixes=(
    "mjDSBL_"
    "mjENBL_"
    "mjJNT_"
    "mjGEOM_"
    "mjCAMLIGHT_"
    "mjLIGHT_"
    "mjTEXTURE_"
    "mjTEXROLE_"
    "mjCOLORSPACE_"
    "mjINT_"
    "mjCONE_"
    "mjJAC_"
    "mjSOL_"
    "mjEQ_"
    "mjWRAP_"
    "mjTRN_"
    "mjDYN_"
    "mjGAIN_"
    "mjBIAS_"
    "mjOBJ_"
    "mjCNSTR_"
    "mjCNSTRSTATE_"
    "mjSENS_"
    "mjSTAGE_"
    "mjDATATYPE_"
    "mjCONDATA_"
    "mjSAMEFRAME_"
    "mjFLEXSELF_"
    "mjSDFTYPE_"
    "mjSTATE_"
    "mjWARN_"
    "mjTIMER_"
    "mjCAT_"
    "mjMOUSE_"
    "mjPERT_"
    "mjCAMERA_"
    "mjLABEL_"
    "mjFRAME_"
    "mjVIS_"
    "mjRND_"
    "mjSTEREO_"
    "mjPLUGIN_"
    "mjGRID_"
    "mjFB_"
    "mjDEPTH_"
    "mjFONTSCALE_"
    "mjFONT_"
    "mjINERTIA_"
    "mjMESH_INERTIA_"
    "mjMESH_BUILTIN_"
    "mjBUILTIN_"
    "mjMARK_"
    "mjLIMITED_"
    "mjINERTIAFROMGEOM_"
    "mjORIENTATION_"
    "mjBUTTON_"
    "mjEVENT_"
    "mjITEM_"
    "mjSECT_"
)

for prefix in "${prefixes[@]}"; do
    sed -i "s/${prefix}\([A-Z][A-Z_]*\)/\1/g" $SCRIPT_DIR/src/lib.rs
done

uv run $SCRIPT_DIR/generate_from_introspect.py $SCRIPT_DIR/mujoco
cargo fmt -- $SCRIPT_DIR/../mujoco/src/{data_struct,model_struct,data_functions}.rs
