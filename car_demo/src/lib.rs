#![allow(
    clippy::missing_safety_doc,
    non_camel_case_types,
    non_snake_case,
    non_upper_case_globals
)]

mod app;
mod car;
mod config;
mod ffi_bindings;
mod input;
mod telemetry;
mod track;
mod ui;
mod util;

pub use app::App;
pub use config::ViewPreset;

#[cfg(target_os = "android")]
use android_activity::AndroidApp;

#[cfg(target_os = "android")]
#[no_mangle]
fn android_main(app: AndroidApp) {
    android_logger::init_once(
        android_logger::Config::default()
            .with_max_level(log::LevelFilter::Info)
            .with_tag("libccar_demo"),
    );

    log::info!("Starting libccar_demo on Android");

    let mut native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_title("libccar Frontend"),
        ..Default::default()
    };

    native_options.android_app = Some(app);

    eframe::run_native(
        "libccar Frontend",
        native_options,
        Box::new(|_cc| Ok(Box::new(unsafe { App::new(ViewPreset::Hypercar) }))),
    )
    .expect("eframe::run_native failed on Android");
}
