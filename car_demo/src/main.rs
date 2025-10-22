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
mod ui;
mod util;

use app::App;
use config::ViewPreset;

fn main() -> eframe::Result<()> {
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size(egui::Vec2::new(1300.0, 800.0))
            .with_title("libccar Frontend"),
        ..Default::default()
    };

    eframe::run_native(
        "libccar Frontend",
        native_options,
        Box::new(|_cc| Ok(Box::new(unsafe { App::new(ViewPreset::Hypercar) }))),
    )
}
