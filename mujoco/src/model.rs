use std::ffi::CStr;
use std::ffi::CString;
use std::path::Path;

/// Safe Rust wrapper around MuJoCo's mjModel
#[derive(Debug)]
pub struct Model {
    pub(crate) ptr: *mut mujoco_sys::mjModel,
}

// Safety: mjModel is thread-safe for read operations
unsafe impl Send for Model {}
unsafe impl Sync for Model {}

impl Model {
    /// Load a MuJoCo model from an XML file
    pub fn from_file(filename: impl AsRef<Path>) -> Result<Self, String> {
        let c_filename = CString::new(filename.as_ref().to_string_lossy().as_ref())
            .map_err(|e| format!("Failed to convert filename to CString: {}", e))?;

        const ERROR_SIZE: usize = 1000;
        let mut error_buf = [0u8; ERROR_SIZE];

        let ptr = unsafe {
            mujoco_sys::mj_loadXML(
                c_filename.as_ptr(),
                std::ptr::null(),
                error_buf.as_mut_ptr() as *mut std::os::raw::c_char,
                ERROR_SIZE as std::os::raw::c_int,
            )
        };

        if ptr.is_null() {
            // Extract error message
            let error_msg = unsafe {
                CStr::from_ptr(error_buf.as_ptr() as *const std::os::raw::c_char)
                    .to_string_lossy()
                    .into_owned()
            };
            return Err(error_msg);
        }
        Ok(Self { ptr })
    }

    /// Get the raw pointer (for FFI calls)
    pub fn as_ptr(&self) -> *const mujoco_sys::mjModel {
        self.ptr
    }

    /// Get the mutable raw pointer (for FFI calls)
    pub fn as_mut_ptr(&mut self) -> *mut mujoco_sys::mjModel {
        self.ptr
    }

    /// Get immutable reference to the raw model
    pub fn raw(&self) -> &mujoco_sys::mjModel {
        unsafe { &*self.ptr }
    }

    /// Get mutable reference to the raw model
    pub fn raw_mut(&mut self) -> &mut mujoco_sys::mjModel {
        unsafe { &mut *self.ptr }
    }
}

/// Get number of generalized coordinates for a given joint type
pub fn joint_nq(joint_type: mujoco_sys::mjtJoint) -> usize {
    match joint_type {
        mujoco_sys::mjtJoint::FREE => 7,
        mujoco_sys::mjtJoint::BALL => 4,
        mujoco_sys::mjtJoint::SLIDE => 1,
        mujoco_sys::mjtJoint::HINGE => 1,
    }
}

/// Get number of degrees of freedom for a given joint type
/// joint_nv(joint_type) == dim(qvel) for a joint of type joint_type
pub fn joint_nv(joint_type: mujoco_sys::mjtJoint) -> usize {
    match joint_type {
        mujoco_sys::mjtJoint::FREE => 6,
        mujoco_sys::mjtJoint::BALL => 3,
        mujoco_sys::mjtJoint::SLIDE => 1,
        mujoco_sys::mjtJoint::HINGE => 1,
    }
}

impl Drop for Model {
    fn drop(&mut self) {
        if !self.ptr.is_null() {
            unsafe {
                mujoco_sys::mj_deleteModel(self.ptr);
            }
        }
    }
}

impl Clone for Model {
    fn clone(&self) -> Self {
        let ptr = unsafe { mujoco_sys::mj_copyModel(std::ptr::null_mut(), self.ptr) };
        Self { ptr }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_model_from_xml_file() {
        let model = Model::from_file(crate::tests::test_xml_path());
        assert!(model.is_ok());
        let model = model.unwrap();
        assert_ne!(model.as_ptr(), std::ptr::null());
        assert!(model.nq() == 3);
        assert!(model.nu() == 3);
    }

    #[test]
    fn test_model_from_invalid_xml_file() {
        let model = Model::from_file(crate::tests::test_malformed_xml_path());
        assert!(model.is_err());
    }
}
