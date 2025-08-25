# mujoco-sys

This crate is a simple and minimal Rust FFI binding for [MuJoCo](https://github.com/google-deepmind/mujoco).

## Versioning

This crate follows the same versioning scheme as MuJoCo itself. The current version corresponds to MuJoCo 3.3.5.

## Dependencies

The vendored MuJoCo build includes the following third-party libraries at the versions specified by the MuJoCo:

- MuJoCo
- libccd
- qhull
- tinyxml2
- tinyobjloader
- lodepng
- TriangleMeshDistance
- MarchingCubeCpp

All dependency versions are locked to those used by the corresponding MuJoCo release to ensure compatibility and reproducible builds. See [MujocoDependencies.cmake](https://github.com/google-deepmind/mujoco/blob/3.3.5/cmake/MujocoDependencies.cmake)

## Features

- vendored-mujoco (default) - Builds MuJoCo from source with all dependencies included
- System MuJoCo linking support (planned)

## Building

The crate will automatically build MuJoCo and its dependencies from source when using the vendored-mujoco feature. No additional system dependencies are required.

## Update FFI bindings

To update the FFI bindings, run the provided script:

```bash
./update-ffi.sh
```
