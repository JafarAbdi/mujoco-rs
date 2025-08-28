use mujoco as mj;
use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    // Load a model from an XML file
    // let spec = mj::Spec::from_str(include_str!("../src/tests/rrr.xml"))?;
    let spec = mj::Spec::from_str(
        r#"
<mujoco model="simple_robot">
  <worldbody>
    <geom name="ground" type="plane" size="5 5 0.1"/>

    <!-- FREE JOINT -->
    <body name="free_body" pos="0 0 1">
      <joint type="free" name="free_joint" />
      <geom type="box" size="0.1 0.1 0.1" rgba="1 0 0 1"/>
    </body>

    <!-- BALL JOINT -->
    <body name="ball_body" pos="1 0 0.5">
      <joint type="ball" name="ball_joint" />
      <geom type="box" size="0.1 0.1 0.3" rgba="0 1 0 1"/>
    </body>

    <!-- SLIDE JOINT -->
    <body name="slide_body" pos="0 1 0.5">
      <joint type="slide" axis="1 0 0" name="slide_joint" />
      <geom type="box" size="0.1 0.1 0.1" rgba="0 0 1 1"/>
    </body>

    <!-- HINGE JOINT -->
    <body name="hinge_body" pos="-1 0 0.5">
      <joint type="hinge" axis="0 0 1" name="hinge_joint" />
      <geom type="box" size="0.1 0.1 0.3" rgba="1 1 0 1"/>
    </body>
  </worldbody>
</mujoco>"#,
    )?;
    let model = spec.compile();
    let mut data = mj::Data::new(&model);
    for i in 0..model.njnt() {
        let joint = data.joint(i).expect("Joint should exist");
        println!("{joint:?}");
    }
    mj::mj_forward(&mut data);
    Ok(())
}
