/// Re-exporting the underlying unsafe API, should you need it
pub use mujoco_sys as sys;

pub mod data;
pub mod data_functions;
pub mod data_struct;
pub mod model;
pub mod model_struct;
pub mod spec;

pub use data::Data;
pub use data_functions::*;
pub use model::Model;
pub use spec::Spec;

#[cfg(test)]
mod tests {
    pub(crate) fn test_xml_path() -> std::path::PathBuf {
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("src")
            .join("tests")
            .join("rrr.xml")
    }

    pub(crate) fn test_malformed_xml_path() -> std::path::PathBuf {
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("src")
            .join("tests")
            .join("malformed.xml")
    }

    pub(crate) fn test_xml_str() -> &'static str {
        include_str!("tests/rrr.xml")
    }
}
