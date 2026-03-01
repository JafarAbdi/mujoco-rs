use mujoco as mj;
use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    // Load a model from an XML file
    let v =
        nalgebra::UnitQuaternion::new_unchecked(nalgebra::Quaternion::new(0.707, 0.0, 0.707, 0.0));
    println!("MuJoCo version: {}", mj::mj_version_string());
    let spec = mj::Spec::from_str(include_str!("../src/tests/rrr.xml"))?;
    let model = spec.compile();
    let mut data = mj::Data::new(&model);
    let jac = mj::mj_jac_site(&data, 0);
    println!("nsite: {}", model.nsite());
    println!("Jacobian at site 0: {:?}", jac);
    mj::mj_forward(&mut data);
    data.qpos_mut()[1] = 1.0;
    Ok(())
}
