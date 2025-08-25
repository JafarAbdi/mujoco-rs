# mujoco-rs

Bindings to [MuJoCo](https://github.com/google-deepmind/mujoco).

This crate provides `mujoco-sys`, a low-level FFI binding to MuJoCo, and `mujoco` crate which provides safe bindings based on `mujoco-sys`.

```rust
use mujoco as mj;
use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    // Load a model from an XML file
    let model = mj::Model::from_file("model.xml")?;
    let mut data = mj::Data::new(&model);
    data.qpos_mut()[0] = 0.5;
    mj::mj_forward(&mut data);
    Ok(())
}
```

## Development

Clone the repository and run:

```bash
git clone --recurse-submodules https://github.com/JafarAbdi/mujoco-rs.git
cd mujoco-rs
cargo build
```

## Acknowledgements

- A few parts of the `mujoco` crate are inspired by the excellent work done in the [mujoco-rs](https://github.com/howird/mujoco-rs). Main differences are:
  - This package rely on MuJoCo's [introspect python package](https://github.com/google-deepmind/mujoco/tree/main/python/mujoco/introspect) to generate bindings, which makes it easier to keep up with new MuJoCo releases.
  - This package try to stay as close as possible to the original MuJoCo C, while keeping safety in mind.
  - The Data struct uses lifetimes to ensure it cannot outlive the Model it references. This makes it possible to return slices for `qpos`, `qvel`, etc. instead of raw pointers.
- The `mujoco-sys` crate is heavily inspired by these two projects:
  - [mosquitto-rs](https://github.com/wez/mosquitto-rs/)
  - [libssh-rs](https://github.com/wez/libssh-rs/)
