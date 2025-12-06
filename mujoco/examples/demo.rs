use mujoco as mj;
use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    // Load a model from an XML file
    println!("MuJoCo version: {}", mj::mj_version_string());
    let spec = mj::Spec::from_str(include_str!("../src/tests/rrr.xml"))?;
    let model = spec.compile();
    let mut data = mj::Data::new(&model);
    mj::mj_forward(&mut data);
    dbg!(mj::mju_euler2quat(&[3.14, 0.0, 0.0], "xyz"));
    data.qpos_mut()[1] = 1.0;
    Ok(())
}
