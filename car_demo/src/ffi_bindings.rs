use once_cell::sync::Lazy;
use std::ffi::CStr;

#[allow(non_camel_case_types, non_snake_case, non_upper_case_globals, unused)]
pub mod ffi {
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}

// Re-export common FFI aliases for easier use in other modules
pub use self::ffi::{
    lcc_abs_mode_e_LCC_ABS_OFF as LCC_ABS_OFF, lcc_abs_mode_e_LCC_ABS_ON as LCC_ABS_ON,
    lcc_drivetrain_layout_e_LCC_LAYOUT_AWD as LCC_LAYOUT_AWD,
    lcc_drivetrain_layout_e_LCC_LAYOUT_FWD as LCC_LAYOUT_FWD,
    lcc_drivetrain_layout_e_LCC_LAYOUT_RWD as LCC_LAYOUT_RWD,
    lcc_esc_mode_e_LCC_ESC_OFF as LCC_ESC_OFF, lcc_esc_mode_e_LCC_ESC_ON as LCC_ESC_ON,
    lcc_tc_mode_e_LCC_TC_OFF as LCC_TC_OFF, lcc_tc_mode_e_LCC_TC_ON as LCC_TC_ON,
    lcc_transmission_type_e_LCC_TRANS_DCT as LCC_TRANS_DCT,
    lcc_transmission_type_e_LCC_TRANS_MANUAL as LCC_TRANS_MANUAL,
};

pub static VERSION: Lazy<String> = Lazy::new(|| unsafe {
    let cstr = ffi::lcc_version_string();
    if cstr.is_null() {
        "unknown".to_string()
    } else {
        CStr::from_ptr(cstr).to_string_lossy().to_string()
    }
});
