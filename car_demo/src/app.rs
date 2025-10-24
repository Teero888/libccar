use crate::{config::*, ffi_bindings::*, input::*, telemetry::*, track::*, ui::*};
use eframe::{egui, egui::Color32};
use glam::Vec2;
use std::time::Instant;

// Main application state
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

    pub tracks: Vec<Track>,
    pub selected_track: usize, // 0 = "None"
    pub lap_times: Vec<f32>,
    pub lap_start_time: Option<Instant>,
    pub current_checkpoint: usize,
    pub last_pos: Vec2,
    pub have_last_pos: bool,
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
    // Checks for checkpoint and start/finish line crossings.
    // This should be called every fixed_update.
    pub fn check_lap_timing(&mut self, p0: Vec2, p1: Vec2, track: &Track) {
        // 1. Check for starting a lap
        if self.lap_start_time.is_none() {
            if crate::util::line_segments_intersect(
                p0,
                p1,
                track.start_finish.0,
                track.start_finish.1,
            ) {
                // Crossed start line, begin timing
                self.lap_start_time = Some(Instant::now());
                self.current_checkpoint = 0;
            }
            return;
        }

        // 2. Lap is in progress. Check for next checkpoint.
        let next_checkpoint_idx = self.current_checkpoint;
        if next_checkpoint_idx < track.checkpoints.len() {
            let cp = track.checkpoints[next_checkpoint_idx];
            if crate::util::line_segments_intersect(p0, p1, cp.0, cp.1) {
                // Hit the next checkpoint
                self.current_checkpoint += 1;
            }
        } else {
            // 3. All checkpoints passed. Check for start/finish line to complete lap.
            if crate::util::line_segments_intersect(
                p0,
                p1,
                track.start_finish.0,
                track.start_finish.1,
            ) {
                // Lap complete!
                if let Some(start) = self.lap_start_time {
                    let duration = start.elapsed().as_secs_f32();
                    self.lap_times.push(duration);
                }
                // Start new lap
                self.lap_start_time = Some(Instant::now());
                self.current_checkpoint = 0;
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
