use std::ffi::CStr;
use std::fmt;
use std::str::FromStr;

/// Error returned when parsing or loading a MuJoCo spec fails.
#[derive(Debug, Clone)]
pub struct ParseError(String);

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for ParseError {}

// Safe Rust wrapper around MuJoCo's mjSpec
pub struct Spec {
    pub(crate) ptr: *mut mujoco_sys::mjSpec,
}

impl Spec {
    pub fn from_file(filename: impl AsRef<std::path::Path>) -> Result<Self, ParseError> {
        let c_filename = std::ffi::CString::new(
            filename
                .as_ref()
                .to_str()
                .ok_or_else(|| ParseError("Path contains invalid UTF-8".into()))?,
        )
        .map_err(|e| ParseError(format!("Failed to convert path to CString: {e}")))?;

        const ERROR_SIZE: usize = 1024;
        let mut error_buf = [0u8; ERROR_SIZE];

        let ptr = unsafe {
            mujoco_sys::mj_parseXML(
                c_filename.as_ptr(),
                std::ptr::null(),
                error_buf.as_mut_ptr() as *mut std::os::raw::c_char,
                ERROR_SIZE as std::os::raw::c_int,
            )
        };

        if ptr.is_null() {
            let error_msg = unsafe {
                CStr::from_ptr(error_buf.as_ptr() as *const std::os::raw::c_char)
                    .to_string_lossy()
                    .into_owned()
            };
            return Err(ParseError(error_msg));
        }

        Ok(Spec { ptr })
    }

    /// Get the raw pointer (for FFI calls)
    pub fn as_ptr(&self) -> *mut mujoco_sys::mjSpec {
        self.ptr
    }

    /// Get immutable reference to the raw model
    pub fn raw(&self) -> &mujoco_sys::mjSpec {
        unsafe { &*self.ptr }
    }

    pub fn compile(self) -> crate::Model {
        let ptr = unsafe { mujoco_sys::mj_compile(self.ptr, std::ptr::null()) };
        if ptr.is_null() {
            panic!("Failed to compile mjSpec into mjModel");
        }
        crate::Model { ptr }
    }
}

impl FromStr for Spec {
    type Err = ParseError;

    fn from_str(xml: &str) -> Result<Self, Self::Err> {
        let c_xml = std::ffi::CString::new(xml)
            .map_err(|e| ParseError(format!("Failed to convert XML to CString: '{e}'")))?;

        const ERROR_SIZE: usize = 1024;
        let mut error_buf = [0u8; ERROR_SIZE];

        let ptr = unsafe {
            mujoco_sys::mj_parseXMLString(
                c_xml.as_ptr(),
                std::ptr::null(),
                error_buf.as_mut_ptr() as *mut std::os::raw::c_char,
                ERROR_SIZE as std::os::raw::c_int,
            )
        };
        if ptr.is_null() {
            let error_msg = unsafe {
                CStr::from_ptr(error_buf.as_ptr() as *const std::os::raw::c_char)
                    .to_string_lossy()
                    .into_owned()
            };
            return Err(ParseError(error_msg));
        }
        Ok(Spec { ptr })
    }
}

impl Drop for Spec {
    fn drop(&mut self) {
        unsafe {
            mujoco_sys::mj_deleteSpec(self.ptr);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_spec_from_str() {
        let spec = Spec::from_str(crate::tests::test_xml_str());
        assert!(spec.is_ok());
        let model = spec.unwrap().compile();
        assert_ne!(model.as_ptr(), std::ptr::null());
        assert!(model.nq() == 3);
        assert!(model.nu() == 3);
    }

    #[test]
    fn test_spec_from_str_invalid() {
        let spec = Spec::from_str("<mujoco></mujoco");
        assert!(spec.is_err());
    }

    #[test]
    fn test_spec_from_file() {
        let spec = Spec::from_file(crate::tests::test_xml_path());
        assert!(spec.is_ok());
        let model = spec.unwrap().compile();
        assert_ne!(model.as_ptr(), std::ptr::null());
    }
}
