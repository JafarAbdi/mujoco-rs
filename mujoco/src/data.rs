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
