use mujoco as mj;
use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    // Load a model from an XML file
    println!("MuJoCo version: {}", mj::version_string());
    let spec = mj::Spec::from_str(include_str!("../src/tests/rrr.xml"))?;
    let model = spec.compile();
    let mut data = mj::Data::new(&model);
    let id = mj::name2id(&model, mujoco_sys::mjtObj::BODY, "link1").expect("link1 should exist");
    let jac = mj::jac_body(&data, id);
    println!("qpos: {:?}", data.qpos());
    println!("Jacobian at body 0: {}", jac);
    data.qpos_mut()[1] = 1.0;
    mj::forward(&mut data);
    let jac = mj::jac_body(&data, id);
    println!("qpos: {:?}", data.qpos());
    println!("Jacobian at body 0: {}", jac);
    Ok(())
}
