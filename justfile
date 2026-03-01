# mujoco-rs development tasks
# https://just.systems

# Run all checks (format, lint, test).
check: fmt-check lint test

# Build the workspace.
build:
    cargo build

# Run tests.
test:
    cargo test

# Run clippy with pedantic lints, treating warnings as errors.
lint:
    cargo clippy --all-targets -- -D warnings

# Format all code.
fmt:
    cargo fmt

# Check formatting without modifying files.
fmt-check:
    cargo fmt --check

# Build documentation.
doc:
    cargo doc --no-deps

# Run a specific example.
# Usage: just example demo
example name:
    cargo run -p mujoco --example {{name}}

# Regenerate FFI bindings (requires bindgen-cli: cargo install bindgen-cli).
update-ffi:
    ./mujoco-sys/update-ffi.sh

# Sync submodules to versions pinned in MuJoCo's MujocoDependencies.cmake.
# Run after updating the mujoco submodule.
update-submodules:
    ./mujoco-sys/update-submodules.sh
