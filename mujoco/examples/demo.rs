use mujoco as mj;
use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    // Load a model from an XML file
    let spec = mj::Spec::from_str(include_str!("../src/tests/rrr.xml"))?;
    let model = spec.compile();
    let mut data = mj::Data::new(&model);
    mj::mj_forward(&mut data);
    data.qpos_mut()[1] = 1.0;
    Ok(())
}
