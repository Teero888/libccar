use crate::{config::*, ffi_bindings::*, input::*, telemetry::*, ui::*};
use eframe::{egui, egui::Color32};
use glam::Vec2;
use std::time::Instant;

/// Main application state
pub struct App {
    pub car: *mut ffi::lcc_car_t,
    pub desc: ffi::lcc_car_desc_t,
    pub env: ffi::lcc_environment_t,
    pub controls: ffi::lcc_controls_t,

    pub preset: ViewPreset,
    pub last_tick: Instant,
    pub accumulator: f32,
    pub fixed_dt: f32,
    pub paused: bool,
    pub sim_speed: f32,
    pub input: InputState,
    pub telemetry_rec: TelemetryRec,
    pub plots: Plots,
    pub zoom: f32,
    pub follow_camera: bool,
    pub camera_pos: Vec2,

    pub path_trace: Vec<Vec2>,
    pub max_trace_points: usize,
    pub trace_point_spacing_m: f32,
    pub show_trace: bool,

    pub skid_segments: Vec<SkidSegment>,
    pub skid_ttl: f32,
    pub show_skids: bool,
    pub slip_ratio_threshold: f32,
    pub slip_angle_threshold: f32,
    pub min_speed_for_skid_kmh: f32,

    pub last_wheel_world: [Vec2; 4],
    pub have_last_wheel_world: bool,

    // driver aids toggles
    pub abs_on: bool,
    pub tc_on: bool,
    pub esc_on: bool,

    // config editor
    pub config: CarConfig,

    // Steering wheel / Gamepad
    pub gilrs: Option<gilrs::Gilrs>,
    pub active_gamepad: Option<gilrs::GamepadId>,
}

impl Drop for App {
    fn drop(&mut self) {
        unsafe {
            if !self.car.is_null() {
                ffi::lcc_car_destroy(self.car);
            }
        }
    }
}

impl App {
    #[inline(always)]
    pub unsafe fn car_ref(&self) -> &ffi::lcc_car_t {
        &*self.car
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Poll gamepad events
        if let Some(gilrs) = self.gilrs.as_mut() {
            while let Some(gilrs::Event { id, .. }) = gilrs.next_event() {
                self.active_gamepad = Some(id);
            }
        }

        unsafe {
            self.step_sim(ctx);
        }

        egui::TopBottomPanel::bottom("bottom")
            .resizable(true)
            .show(ctx, |ui| {
                self.draw_bottom_panel(ui);
            });

        egui::SidePanel::left("left")
            .resizable(true)
            .default_width(360.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical()
                    .auto_shrink([false; 2])
                    .show(ui, |ui| {
                        self.draw_side_panel(ui);
                    });
            });

        egui::CentralPanel::default()
            .frame(egui::Frame::NONE.fill(Color32::from_gray(30)))
            .show(ctx, |ui| {
                self.draw_world(ui);
                self.draw_hud(ui);
            });

        ctx.request_repaint();
    }
}
