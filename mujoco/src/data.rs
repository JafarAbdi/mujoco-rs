#[derive(Debug)]
pub struct Data<'a> {
    pub(crate) ptr: *mut mujoco_sys::mjData,
    pub(crate) model: &'a crate::Model,
}

unsafe impl Send for Data<'_> {}
unsafe impl Sync for Data<'_> {}

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
        crate::forward(&mut data);
        crate::step(&mut data);
        data.qpos_mut()[0] += 0.1;
        crate::forward(&mut data);
        crate::step(&mut data);
        let mut data2 = data.clone();
        assert_eq!(data2.qpos(), data.qpos());
        data2.qpos_mut()[0] += 0.1;
        assert_ne!(data2.qpos(), data.qpos());
    }

    /// Verify the Jacobian wrapper produces correct element values by comparing
    /// against raw FFI output. MuJoCo writes row-major 3×nv matrices; nalgebra
    /// stores column-major. This test catches any layout mismatch.
    #[test]
    fn jacobian_layout_matches_raw_ffi() {
        let model = crate::Model::from_file(crate::tests::test_xml_path()).unwrap();
        let mut data = Data::new(&model);

        // Non-trivial configuration so the Jacobian has interesting values
        data.qpos_mut()[0] = 0.5;
        data.qpos_mut()[1] = 0.3;
        data.qpos_mut()[2] = -0.2;
        crate::forward(&mut data);

        let nv = model.nv();
        let body = 4; // end_effector (base_link=0, link1=1, link2=2, link3=3, end_effector=4)

        // Get Jacobian via our wrapper
        let jac = crate::jac_body(&data, body);

        // Get Jacobian via raw FFI into separate flat buffers
        let mut jacp_raw = vec![0.0f64; 3 * nv];
        let mut jacr_raw = vec![0.0f64; 3 * nv];
        unsafe {
            mujoco_sys::mj_jacBody(
                model.as_ptr(),
                data.as_ptr(),
                jacp_raw.as_mut_ptr(),
                jacr_raw.as_mut_ptr(),
                body,
            );
        }

        // jacp_raw is 3×nv row-major: element (row, col) = jacp_raw[row * nv + col]
        // jac rows 0-2 should be the position Jacobian (jacp)
        for row in 0..3 {
            for col in 0..nv {
                let wrapper_val = jac[(row, col)];
                let raw_val = jacp_raw[row * nv + col];
                assert!(
                    (wrapper_val - raw_val).abs() < 1e-10,
                    "jacp mismatch at ({row}, {col}): wrapper={wrapper_val}, raw={raw_val}"
                );
            }
        }
        // jac rows 3-5 should be the rotation Jacobian (jacr)
        for row in 0..3 {
            for col in 0..nv {
                let wrapper_val = jac[(row + 3, col)];
                let raw_val = jacr_raw[row * nv + col];
                assert!(
                    (wrapper_val - raw_val).abs() < 1e-10,
                    "jacr mismatch at ({}, {col}): wrapper={wrapper_val}, raw={raw_val}",
                    row + 3
                );
            }
        }
    }
}
