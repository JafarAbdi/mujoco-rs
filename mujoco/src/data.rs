#[derive(Debug)]
pub struct Data<'a> {
    pub(crate) ptr: *mut mujoco_sys::mjData,
    pub(crate) model: &'a crate::Model,
}

unsafe impl Send for Data<'_> {}
unsafe impl Sync for Data<'_> {}

#[allow(non_snake_case)]
#[derive(Debug)]
pub struct JointData<'a, T>
where
    T: AsRef<[f64]>,
{
    pub cdof: T,
    pub cdof_dot: T,
    pub id: usize,
    pub name: &'a str,
    pub qLDiagInv: T,
    pub qacc: T,
    pub qacc_smooth: T,
    pub qacc_warmstart: T,
    pub qfrc_actuator: T,
    pub qfrc_applied: T,
    pub qfrc_bias: T,
    pub qfrc_constraint: T,
    pub qfrc_inverse: T,
    pub qfrc_passive: T,
    pub qfrc_smooth: T,
    pub qpos: T,
    pub qvel: T,
    pub xanchor: [f64; 3],
    pub xaxis: [f64; 3],
}

impl<'a> Data<'a> {
    pub fn new(model: &'a crate::Model) -> Self {
        let ptr = unsafe { mujoco_sys::mj_makeData(model.as_ptr()) };
        assert_ne!(ptr, std::ptr::null_mut());
        Self { ptr, model }
    }

    /// Get the raw pointer (for FFI calls)
    pub fn as_ptr(&self) -> *const mujoco_sys::mjData {
        self.ptr
    }

    /// Get the mutable raw pointer (for FFI calls)
    pub fn as_mut_ptr(&mut self) -> *mut mujoco_sys::mjData {
        self.ptr
    }

    /// Get immutable reference to the raw model
    pub fn raw(&self) -> &mujoco_sys::mjData {
        unsafe { &*self.ptr }
    }

    /// Get mutable reference to the raw model
    pub fn raw_mut(&mut self) -> &mut mujoco_sys::mjData {
        unsafe { &mut *self.ptr }
    }

    pub fn joint(&self, id: usize) -> Option<JointData<'a, &[f64]>> {
        if id >= self.model.njnt() {
            return None;
        }
        let name_ptr = self
            .model
            .names()
            .get(self.model.name_jntadr()[id] as usize)?;
        let name = unsafe { std::ffi::CStr::from_ptr(name_ptr).to_str().ok()? };
        let dof_start = self.model.jnt_dofadr()[id] as usize;
        let qpos_start = self.model.jnt_qposadr()[id] as usize;
        let joint_type = mujoco_sys::mjtJoint::from(self.model.jnt_type()[id] as usize);
        let nq = crate::joint_nq(joint_type);
        let nv = crate::joint_nv(joint_type);
        Some(JointData {
            cdof: &self.cdof()[dof_start..dof_start + nv],
            cdof_dot: &self.cdof_dot()[dof_start..dof_start + nv],
            id,
            name,
            qLDiagInv: &self.qLDiagInv()[dof_start..dof_start + nv],
            qacc: &self.qacc()[dof_start..dof_start + nv],
            qacc_smooth: &self.qacc_smooth()[dof_start..dof_start + nv],
            qacc_warmstart: &self.qacc_warmstart()[dof_start..dof_start + nv],
            qfrc_actuator: &self.qfrc_actuator()[dof_start..dof_start + nv],
            qfrc_applied: &self.qfrc_applied()[dof_start..dof_start + nv],
            qfrc_bias: &self.qfrc_bias()[dof_start..dof_start + nv],
            qfrc_constraint: &self.qfrc_constraint()[dof_start..dof_start + nv],
            qfrc_inverse: &self.qfrc_inverse()[dof_start..dof_start + nv],
            qfrc_passive: &self.qfrc_passive()[dof_start..dof_start + nv],
            qfrc_smooth: &self.qfrc_smooth()[dof_start..dof_start + nv],
            qpos: &self.qpos()[qpos_start..qpos_start + nq],
            qvel: &self.qvel()[dof_start..dof_start + nv],
            xanchor: [
                self.xanchor()[3 * id],
                self.xanchor()[3 * id + 1],
                self.xanchor()[3 * id + 2],
            ],
            xaxis: [
                self.xaxis()[3 * id],
                self.xaxis()[3 * id + 1],
                self.xaxis()[3 * id + 2],
            ],
        })
    }
}

impl Drop for Data<'_> {
    fn drop(&mut self) {
        unsafe { mujoco_sys::mj_deleteData(self.ptr) }
    }
}

impl Clone for Data<'_> {
    fn clone(&self) -> Self {
        let ptr =
            unsafe { mujoco_sys::mj_copyData(std::ptr::null_mut(), self.model.as_ptr(), self.ptr) };
        assert_ne!(ptr, std::ptr::null_mut());
        Self {
            ptr,
            model: self.model,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new() {
        let model = crate::Model::from_file(crate::tests::test_xml_path()).unwrap();
        let mut data = Data::new(&model);
        assert_ne!(data.ptr, std::ptr::null_mut());
        crate::mj_forward(&mut data);
        crate::mj_step(&mut data);
        data.qpos_mut()[0] += 0.1;
        crate::mj_forward(&mut data);
        crate::mj_step(&mut data);
        let mut data2 = data.clone();
        assert_eq!(data2.qpos(), data.qpos());
        data2.qpos_mut()[0] += 0.1;
        assert_ne!(data2.qpos(), data.qpos());
    }
}
