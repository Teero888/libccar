#![allow(clippy::missing_safety_doc)]
use eframe::{
    egui,
    egui::{Color32, Painter, RichText, Stroke},
};
use egui::{vec2, Align2, Pos2, Rect};
use egui_extras::{Size, StripBuilder};
use egui_plot::{HLine, Legend, Line, Plot, PlotPoints};
use glam::{Mat2, Vec2};
use once_cell::sync::Lazy;
use serde::Serialize;
use std::{f32::consts::PI, time::Instant};

mod ffi {
    #![allow(non_camel_case_types, non_snake_case, non_upper_case_globals)]
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}

static VERSION: Lazy<String> = Lazy::new(|| unsafe {
    let cstr = ffi::lcc_get_version();
    if cstr.is_null() {
        "unknown".to_string()
    } else {
        let s = std::ffi::CStr::from_ptr(cstr);
        s.to_string_lossy().to_string()
    }
});

#[derive(Clone, Copy, PartialEq)]
enum ViewPreset {
    Economy,
    Midsize,
    Sports,
    Supercar,
    Hypercar,
}

impl ViewPreset {
    fn all() -> &'static [ViewPreset] {
        &[
            ViewPreset::Economy,
            ViewPreset::Midsize,
            ViewPreset::Sports,
            ViewPreset::Supercar,
            ViewPreset::Hypercar,
        ]
    }
    fn as_label(self) -> &'static str {
        match self {
            ViewPreset::Economy => "Economy",
            ViewPreset::Midsize => "Midsize",
            ViewPreset::Sports => "Sports",
            ViewPreset::Supercar => "Supercar",
            ViewPreset::Hypercar => "Hypercar",
        }
    }
    fn to_c_enum(self) -> ffi::lcc_preset_t {
        match self {
            ViewPreset::Economy => ffi::lcc_preset_t_LCC_PRESET_ECONOMY,
            ViewPreset::Midsize => ffi::lcc_preset_t_LCC_PRESET_MIDSIZE,
            ViewPreset::Sports => ffi::lcc_preset_t_LCC_PRESET_SPORTS,
            ViewPreset::Supercar => ffi::lcc_preset_t_LCC_PRESET_SUPERCAR,
            ViewPreset::Hypercar => ffi::lcc_preset_t_LCC_PRESET_HYPERCAR,
        }
    }
}

#[derive(Serialize, Clone)]
struct WheelSample {
    slip_ratio: f32,
    slip_angle: f32,
    load: f32,
    temp: f32,
    surface_mu: f32,
}

#[derive(Serialize, Clone)]
struct TelemetrySample {
    t: f32,
    pos_x: f32,
    pos_y: f32,
    vel_x: f32,
    vel_y: f32,
    speed_kmh: f32,
    yaw: f32,
    yaw_rate: f32,
    engine_rpm: f32,
    gear: i32,
    throttle: f32,
    brake: f32,
    steering: f32,
    clutch: f32,
    wheels: [WheelSample; 4],
}

struct Telemetry {
    recording: bool,
    data: Vec<TelemetrySample>,
    max_points: usize,
}

impl Telemetry {
    fn new() -> Self {
        Self {
            recording: false,
            data: Vec::new(),
            max_points: 20000,
        }
    }
    fn clear(&mut self) {
        self.data.clear();
    }
    fn push(&mut self, s: TelemetrySample) {
        if self.recording {
            self.data.push(s);
            if self.data.len() > self.max_points {
                self.data.drain(0..(self.data.len() - self.max_points));
            }
        }
    }
    fn export_csv(&self, path: &std::path::Path) -> anyhow::Result<()> {
        let mut wtr = csv::Writer::from_path(path)?;
        wtr.write_record(&[
            "t",
            "pos_x",
            "pos_y",
            "vel_x",
            "vel_y",
            "speed_kmh",
            "yaw",
            "yaw_rate",
            "engine_rpm",
            "gear",
            "throttle",
            "brake",
            "steering",
            "clutch",
            "w0_slip_ratio",
            "w0_slip_angle",
            "w0_load",
            "w0_temp",
            "w0_mu",
            "w1_slip_ratio",
            "w1_slip_angle",
            "w1_load",
            "w1_temp",
            "w1_mu",
            "w2_slip_ratio",
            "w2_slip_angle",
            "w2_load",
            "w2_temp",
            "w2_mu",
            "w3_slip_ratio",
            "w3_slip_angle",
            "w3_load",
            "w3_temp",
            "w3_mu",
        ])?;
        for s in &self.data {
            let rec = vec![
                s.t,
                s.pos_x,
                s.pos_y,
                s.vel_x,
                s.vel_y,
                s.speed_kmh,
                s.yaw,
                s.yaw_rate,
                s.engine_rpm,
                s.gear as f32,
                s.throttle,
                s.brake,
                s.steering,
                s.clutch,
                s.wheels[0].slip_ratio,
                s.wheels[0].slip_angle,
                s.wheels[0].load,
                s.wheels[0].temp,
                s.wheels[0].surface_mu,
                s.wheels[1].slip_ratio,
                s.wheels[1].slip_angle,
                s.wheels[1].load,
                s.wheels[1].temp,
                s.wheels[1].surface_mu,
                s.wheels[2].slip_ratio,
                s.wheels[2].slip_angle,
                s.wheels[2].load,
                s.wheels[2].temp,
                s.wheels[2].surface_mu,
                s.wheels[3].slip_ratio,
                s.wheels[3].slip_angle,
                s.wheels[3].load,
                s.wheels[3].temp,
                s.wheels[3].surface_mu,
            ];
            wtr.serialize(rec)?;
        }
        wtr.flush()?;
        Ok(())
    }
    fn export_json(&self, path: &std::path::Path) -> anyhow::Result<()> {
        let s = serde_json::to_string_pretty(&self.data)?;
        std::fs::write(path, s)?;
        Ok(())
    }
}

struct InputState {
    throttle: f32,
    brake: f32,
    steer: f32,
    clutch: f32,
    last_w: bool,
    last_s: bool,
}

impl InputState {
    fn new() -> Self {
        Self {
            throttle: 0.0,
            brake: 0.0,
            steer: 0.0,
            clutch: 0.0,
            last_w: false,
            last_s: false,
        }
    }
}

struct App {
    car: ffi::lcc_car_t,
    preset: ViewPreset,
    last_tick: Instant,
    accumulator: f32,
    fixed_dt: f32,
    paused: bool,
    sim_speed: f32,
    input: InputState,
    telemetry: Telemetry,
    plots: Plots,
    zoom: f32,
    follow_camera: bool,
    camera_pos: Vec2,

    // visuals
    path_trace: Vec<Vec2>,
    max_trace_points: usize,
    trace_point_spacing_m: f32,
    show_trace: bool,

    skid_segments: Vec<SkidSegment>,
    skid_ttl: f32,
    show_skids: bool,
    slip_ratio_threshold: f32, // longitudinal slip threshold for skid mark
    slip_angle_threshold: f32, // lateral slip threshold (rad) for skid mark
    min_speed_for_skid_kmh: f32,

    last_wheel_world: [Vec2; 4],
    have_last_wheel_world: bool,
}

#[derive(Clone, Copy)]
struct SkidSegment {
    p0: Vec2,
    p1: Vec2,
    strength: f32, // 0..1, used as opacity/width
    ttl: f32,      // remaining lifetime in seconds
}

struct Plots {
    rpm: Vec<(f32, f32)>,
    speed: Vec<(f32, f32)>,
    inputs: [Vec<(f32, f32)>; 4], // throttle, brake, clutch, steer (0..1)
    slip_ratios: [Vec<(f32, f32)>; 4], // per wheel
    yaw_rate: Vec<(f32, f32)>,    // rad/s
    start_time: Instant,
    max_len: usize,
}

impl Plots {
    fn new() -> Self {
        Self {
            rpm: Vec::new(),
            speed: Vec::new(),
            inputs: [Vec::new(), Vec::new(), Vec::new(), Vec::new()],
            slip_ratios: [Vec::new(), Vec::new(), Vec::new(), Vec::new()],
            yaw_rate: Vec::new(),
            start_time: Instant::now(),
            max_len: 3000,
        }
    }
    fn t(&self) -> f32 {
        (Instant::now() - self.start_time).as_secs_f32()
    }
    fn push(
        &mut self,
        rpm: f32,
        speed: f32,
        inputs: [f32; 4],
        slip_ratios: [f32; 4],
        yaw_rate: f32,
    ) {
        let t = self.t();
        self.rpm.push((t, rpm));
        self.speed.push((t, speed));
        self.yaw_rate.push((t, yaw_rate));
        for i in 0..4 {
            self.inputs[i].push((t, inputs[i]));
            self.slip_ratios[i].push((t, slip_ratios[i]));
            if self.slip_ratios[i].len() > self.max_len {
                self.slip_ratios[i].remove(0);
            }
        }
        if self.rpm.len() > self.max_len {
            self.rpm.remove(0);
        }
        if self.speed.len() > self.max_len {
            self.speed.remove(0);
        }
        if self.yaw_rate.len() > self.max_len {
            self.yaw_rate.remove(0);
        }
        for i in 0..4 {
            if self.inputs[i].len() > self.max_len {
                self.inputs[i].remove(0);
            }
        }
    }
}

impl App {
    unsafe fn new(preset: ViewPreset) -> Self {
        let car = ffi::lcc_car_create(preset.to_c_enum());
        let mut this = Self {
            car,
            preset,
            last_tick: Instant::now(),
            accumulator: 0.0,
            fixed_dt: 1.0 / 120.0,
            paused: false,
            sim_speed: 1.0,
            input: InputState::new(),
            telemetry: Telemetry::new(),
            plots: Plots::new(),
            zoom: 32.0,
            follow_camera: true,
            camera_pos: Vec2::ZERO,

            skid_segments: Vec::new(),
            skid_ttl: 6.0,
            show_skids: true,
            slip_ratio_threshold: 0.15,
            slip_angle_threshold: 0.12,
            min_speed_for_skid_kmh: 10.0,

            last_wheel_world: [Vec2::ZERO; 4],
            have_last_wheel_world: false,

            // --- Add the missing fields below ---
            max_trace_points: 4000,
            path_trace: Vec::new(),
            show_trace: true,
            trace_point_spacing_m: 0.25,
        };
        this.center_camera_on_car();
        this
    }

    fn center_camera_on_car(&mut self) {
        let pos = self.world_pos();
        self.camera_pos = pos;
    }

    fn world_pos(&self) -> Vec2 {
        Vec2::new(self.car.position[0], self.car.position[1])
    }

    unsafe fn reset(&mut self) {
        self.car = ffi::lcc_car_create(self.preset.to_c_enum());
        self.plots = Plots::new();
        self.telemetry.clear();
        self.center_camera_on_car();

        self.path_trace.clear();
        self.skid_segments.clear();
        self.have_last_wheel_world = false;
    }

    unsafe fn handle_input(&mut self, ctx: &egui::Context) {
        let input = ctx.input(|i| i.clone());

        let left = input.key_down(egui::Key::A);
        let right = input.key_down(egui::Key::D);
        let target_steer = match (left, right) {
            (true, false) => -1.0,
            (false, true) => 1.0,
            _ => 0.0,
        };
        self.input.steer = lerp(self.input.steer, target_steer, 0.1);

        let throttle_down = input.key_down(egui::Key::K);
        let target_throttle = if throttle_down { 1.0 } else { 0.0 };
        self.input.throttle = smooth_approach(self.input.throttle, target_throttle, 0.02, 0.02);

        let brake_down = input.key_down(egui::Key::J);
        let target_brake = if brake_down { 1.0 } else { 0.0 };
        self.input.brake = smooth_approach(self.input.brake, target_brake, 0.02, 0.02);

        let clutch_down = input.key_down(egui::Key::H);
        let target_clutch = if clutch_down { 1.0 } else { 0.0 };
        self.input.clutch = smooth_approach(self.input.clutch, target_clutch, 0.3, 0.5);

        let w = input.key_pressed(egui::Key::W);
        let s = input.key_pressed(egui::Key::S);
        if w && !self.input.last_w {
            ffi::lcc_car_shift_up(&mut self.car);
        }
        if s && !self.input.last_s {
            ffi::lcc_car_shift_down(&mut self.car);
        }
        self.input.last_w = w;
        self.input.last_s = s;

        ffi::lcc_car_set_inputs(
            &mut self.car,
            self.input.throttle,
            self.input.brake,
            self.input.steer,
            self.input.clutch,
        );
    }

    unsafe fn fixed_update(&mut self, dt: f32) {
        ffi::lcc_car_update(&mut self.car, dt);

        let speed_kmh = ffi::lcc_car_get_speed(&self.car);
        let rpm = ffi::lcc_car_get_engine_rpm(&self.car);

        // Telemetry sample as before
        let wheels = [
            {
                let w = self.car.wheels[0];
                WheelSample {
                    slip_ratio: w.slip_ratio,
                    slip_angle: w.slip_angle,
                    load: w.load,
                    temp: w.temperature,
                    surface_mu: w.surface_friction,
                }
            },
            {
                let w = self.car.wheels[1];
                WheelSample {
                    slip_ratio: w.slip_ratio,
                    slip_angle: w.slip_angle,
                    load: w.load,
                    temp: w.temperature,
                    surface_mu: w.surface_friction,
                }
            },
            {
                let w = self.car.wheels[2];
                WheelSample {
                    slip_ratio: w.slip_ratio,
                    slip_angle: w.slip_angle,
                    load: w.load,
                    temp: w.temperature,
                    surface_mu: w.surface_friction,
                }
            },
            {
                let w = self.car.wheels[3];
                WheelSample {
                    slip_ratio: w.slip_ratio,
                    slip_angle: w.slip_angle,
                    load: w.load,
                    temp: w.temperature,
                    surface_mu: w.surface_friction,
                }
            },
        ];

        let sample = TelemetrySample {
            t: self.car.simulation_time,
            pos_x: self.car.position[0],
            pos_y: self.car.position[1],
            vel_x: self.car.velocity[0],
            vel_y: self.car.velocity[1],
            speed_kmh,
            yaw: self.car.angle,
            yaw_rate: self.car.angular_velocity,
            engine_rpm: rpm,
            gear: self.car.transmission.current_gear,
            throttle: self.car.throttle_input,
            brake: self.car.brake_input,
            steering: self.car.steering_input,
            clutch: self.car.clutch_input,
            wheels,
        };
        self.telemetry.push(sample.clone());

        // Push to plots (more meaningful data)
        self.plots.push(
            rpm as f32,
            speed_kmh as f32,
            [
                self.car.throttle_input,
                self.car.brake_input,
                self.car.clutch_input,
                // normalize steer from [-1..1] to [0..1] to plot alongside other inputs
                (self.car.steering_input + 1.0) * 0.5,
            ],
            [
                self.car.wheels[0].slip_ratio as f32,
                self.car.wheels[1].slip_ratio as f32,
                self.car.wheels[2].slip_ratio as f32,
                self.car.wheels[3].slip_ratio as f32,
            ],
            self.car.angular_velocity as f32,
        );

        // Path trace (store car center position every ~0.25 m)
        let pos = self.world_pos();
        if self
            .path_trace
            .last()
            .map_or(true, |p| (*p - pos).length() > self.trace_point_spacing_m)
        {
            self.path_trace.push(pos);
            if self.path_trace.len() > self.max_trace_points {
                let overflow = self.path_trace.len() - self.max_trace_points;
                self.path_trace.drain(0..overflow);
            }
        }

        // Skid marks
        let rot = Mat2::from_angle(self.car.angle);
        let mut wheel_world = [Vec2::ZERO; 4];
        for i in 0..4 {
            let w = &self.car.wheels[i];
            let wp = Vec2::new(w.position[0], w.position[1]);
            wheel_world[i] = pos + rot * wp;
        }
        if !self.have_last_wheel_world {
            self.last_wheel_world = wheel_world;
            self.have_last_wheel_world = true;
        } else {
            for i in 0..4 {
                let w = &self.car.wheels[i];
                let sr = w.slip_ratio.abs() as f32;
                let sa = w.slip_angle.abs() as f32;
                // normalized slip severity
                let long = sr / self.slip_ratio_threshold;
                let lat = sa / self.slip_angle_threshold;
                let severity = long.max(lat).min(3.0); // cap
                let moving = speed_kmh > self.min_speed_for_skid_kmh;
                let loaded = w.load > 50.0; // prevent skids while airborne
                if self.show_skids && moving && loaded && severity >= 1.0 {
                    let strength = ((severity - 1.0) / 2.0).clamp(0.2, 1.0); // map into 0.2..1.0
                    self.skid_segments.push(SkidSegment {
                        p0: self.last_wheel_world[i],
                        p1: wheel_world[i],
                        strength,
                        ttl: self.skid_ttl,
                    });
                }
                self.last_wheel_world[i] = wheel_world[i];
            }
        }

        // Fade old skid segments
        for seg in &mut self.skid_segments {
            seg.ttl -= dt;
        }
        self.skid_segments.retain(|s| s.ttl > 0.0);
    }

    unsafe fn step_sim(&mut self, ctx: &egui::Context) {
        let now = Instant::now();
        let mut dt = (now - self.last_tick).as_secs_f32();
        self.last_tick = now;

        dt = (dt * self.sim_speed).clamp(0.0, 0.1);
        if self.paused {
            self.handle_input(ctx);
            return;
        }

        self.accumulator += dt;
        while self.accumulator >= self.fixed_dt {
            self.fixed_update(self.fixed_dt);
            self.accumulator -= self.fixed_dt;
        }

        if self.follow_camera {
            self.camera_pos = self.world_pos();
        }

        self.handle_input(ctx);
    }

    fn draw_world(&self, ui: &mut egui::Ui) {
        let available = ui.available_rect_before_wrap();
        let painter = ui.painter_at(available);
        let rect = available;
        let center = rect.center();

        draw_grid(&painter, rect, self.camera_pos, self.zoom);
        // Path trace
        if self.show_trace && self.path_trace.len() > 1 {
            let to_screen = |w: Vec2| -> Pos2 {
                let rel = w - self.camera_pos;
                let screen = Vec2::new(rel.x * self.zoom, -rel.y * self.zoom);
                Pos2::new(center.x + screen.x, center.y + screen.y)
            };
            let pts: Vec<Pos2> = self.path_trace.iter().map(|&p| to_screen(p)).collect();
            painter.add(egui::Shape::line(
                pts,
                Stroke {
                    width: 1.5,
                    color: Color32::from_rgb(60, 200, 255).gamma_multiply(0.55),
                },
            ));
        }

        // Skid marks (fade with time and strength)
        if self.show_skids && !self.skid_segments.is_empty() {
            let to_screen = |w: Vec2| -> Pos2 {
                let rel = w - self.camera_pos;
                let screen = Vec2::new(rel.x * self.zoom, -rel.y * self.zoom);
                Pos2::new(center.x + screen.x, center.y + screen.y)
            };
            for seg in &self.skid_segments {
                let life = (seg.ttl / self.skid_ttl).clamp(0.0, 1.0);
                let alpha = (200.0 * life * seg.strength) as u8;
                let width = 2.0 + 1.5 * seg.strength;
                let col = Color32::from_black_alpha(alpha);
                painter.line_segment(
                    [to_screen(seg.p0), to_screen(seg.p1)],
                    Stroke::new(width, col),
                );
            }
        }

        unsafe {
            let to_screen = |w: Vec2| -> Pos2 {
                let rel = w - self.camera_pos;
                let screen = Vec2::new(rel.x * self.zoom, -rel.y * self.zoom);
                Pos2::new(center.x + screen.x, center.y + screen.y)
            };

            let pos = Vec2::new(self.car.position[0], self.car.position[1]);
            let vel = Vec2::new(self.car.velocity[0], self.car.velocity[1]);
            let p0 = to_screen(pos);
            let p1 = to_screen(pos + vel * 0.5);
            painter.line_segment([p0, p1], Stroke::new(2.0, Color32::LIGHT_BLUE));

            let angle = self.car.angle;
            let wb = self.car.wheelbase;
            let tw = self.car.track_width;
            let body_len = wb + 1.0;
            let body_w = tw + 0.4;

            let rot = Mat2::from_angle(angle);
            let half = Vec2::new(body_len * 0.5, body_w * 0.5);

            let corners = [
                Vec2::new(half.x, -half.y),
                Vec2::new(half.x, half.y),
                Vec2::new(-half.x, half.y),
                Vec2::new(-half.x, -half.y),
            ]
            .map(|local| pos + rot * local);

            painter.add(egui::Shape::convex_polygon(
                corners.map(to_screen).to_vec(),
                Color32::from_black_alpha(20),
                Stroke::new(2.0, Color32::from_rgb(180, 200, 220)),
            ));

            for i in 0..4 {
                let w = &self.car.wheels[i];
                let wp = Vec2::new(w.position[0], w.position[1]);
                let wheel_world = pos + rot * wp;
                let steer_angle = w.steer_angle + angle;

                draw_wheel(&painter, &to_screen, wheel_world, steer_angle, 0.7, 0.28, w);
            }
        }
    }
    fn draw_side_panel(&mut self, ui: &mut egui::Ui) {
        ui.spacing_mut().slider_width = 180.0;

        ui.horizontal(|ui| {
            ui.heading(RichText::new("libccar demo").color(Color32::LIGHT_GREEN));
            ui.label(format!("v{}", *VERSION));
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.button("Reset").clicked() {
                    unsafe {
                        self.reset();
                    }
                }
            });
        });
        ui.separator();

        egui::ComboBox::from_label("Preset")
            .selected_text(self.preset.as_label())
            .show_ui(ui, |ui| {
                for p in ViewPreset::all() {
                    ui.selectable_value(&mut self.preset, *p, p.as_label());
                }
            });
        if ui.button("Apply Preset").clicked() {
            unsafe {
                self.reset();
            }
        }

        ui.add(egui::Slider::new(&mut self.sim_speed, 0.1..=4.0).text("Sim speed"));
        ui.horizontal(|ui| {
            if ui.toggle_value(&mut self.paused, "Pause").clicked() {}
            if ui.button("Step").clicked() {
                unsafe {
                    self.fixed_update(self.fixed_dt);
                }
            }
        });
        ui.checkbox(&mut self.follow_camera, "Follow camera");
        ui.add(egui::Slider::new(&mut self.zoom, 8.0..=120.0).text("Zoom"));

        ui.separator();

        unsafe {
            let rpm = ffi::lcc_car_get_engine_rpm(&self.car);
            let speed = ffi::lcc_car_get_speed(&self.car);
            let gear = self.car.transmission.current_gear;
            ui.label(format!("Speed: {:6.1} km/h", speed));
            ui.label(format!("Engine RPM: {:6.0}", rpm));
            ui.label(format!("Gear: {}", gear));
            ui.label(format!("Yaw: {:5.2} rad", self.car.angle));
        }

        ui.separator();

        ui.collapsing("Controls", |ui| {
            ui.monospace("a/d = steer\nw/s = shift\nh = clutch\nj = brake\nk = throttle");
        });
        ui.separator();
        ui.collapsing("Visuals", |ui| {
            ui.checkbox(&mut self.show_trace, "Show path trace");
            ui.checkbox(&mut self.show_skids, "Show skid marks");
            ui.add(
                egui::Slider::new(&mut self.trace_point_spacing_m, 0.05..=1.0)
                    .text("Trace spacing (m)"),
            );
            ui.horizontal(|ui| {
                if ui.button("Clear traces").clicked() {
                    self.path_trace.clear();
                    self.skid_segments.clear();
                }
            });
            ui.add(
                egui::Slider::new(&mut self.slip_ratio_threshold, 0.05..=0.5)
                    .text("Skid slip ratio threshold"),
            );
            ui.add(
                egui::Slider::new(&mut self.slip_angle_threshold, 0.05..=0.5)
                    .text("Skid slip angle threshold (rad)"),
            );
            ui.add(
                egui::Slider::new(&mut self.min_speed_for_skid_kmh, 0.0..=40.0)
                    .text("Min speed for skids (km/h)"),
            );
            ui.add(egui::Slider::new(&mut self.skid_ttl, 1.0..=12.0).text("Skid mark fade (s)"));
        });
        ui.separator();
        ui.collapsing("Telemetry", |ui| {
            ui.horizontal(|ui| {
                if ui
                    .button(if self.telemetry.recording {
                        "Stop recording"
                    } else {
                        "Start recording"
                    })
                    .clicked()
                {
                    self.telemetry.recording = !self.telemetry.recording;
                }
                if ui.button("Clear").clicked() {
                    self.telemetry.clear();
                }
            });
            if ui.button("Export CSV").clicked() {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("CSV", &["csv"])
                    .save_file()
                {
                    if let Err(e) = self.telemetry.export_csv(&path) {
                        eprintln!("CSV export failed: {e:?}");
                    }
                }
            }
            if ui.button("Export JSON").clicked() {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("JSON", &["json"])
                    .save_file()
                {
                    if let Err(e) = self.telemetry.export_json(&path) {
                        eprintln!("JSON export failed: {e:?}");
                    }
                }
            }
            ui.label(format!("Samples: {}", self.telemetry.data.len()));
        });
    }

    fn draw_bottom_panel(&mut self, ui: &mut egui::Ui) {
        // ...existing code...
        StripBuilder::new(ui)
            .sizes(Size::exact(200.0), 1)
            .vertical(|mut strip| {
                strip.cell(|ui| {
                    // Two columns that fill height
                    StripBuilder::new(ui)
                        .sizes(Size::relative(0.5), 2)
                        .horizontal(|mut hstrip| {
                            // Left column: RPM Speed
                            hstrip.cell(|ui| {
                                let col_h = ui.available_height();
                                let half = (col_h - 6.0) * 0.5;

                                // RPM plot
                                Plot::new("rpm_plot")
                                    .legend(Legend::default())
                                    .height(half)
                                    .show(ui, |pui| {
                                        let pts = PlotPoints::from_iter(
                                            self.plots
                                                .rpm
                                                .iter()
                                                .map(|&(x, y)| [x as f64, y as f64]),
                                        );
                                        pui.line(
                                            Line::new("RPM", pts).color(Color32::LIGHT_YELLOW),
                                        );
                                    });

                                // Speed plot
                                Plot::new("speed_plot")
                                    .legend(Legend::default())
                                    .height(half)
                                    .show(ui, |pui| {
                                        let pts = PlotPoints::from_iter(
                                            self.plots
                                                .speed
                                                .iter()
                                                .map(|&(x, y)| [x as f64, y as f64]),
                                        );
                                        pui.line(
                                            Line::new("Speed (km/h)", pts)
                                                .color(Color32::LIGHT_GREEN),
                                        );
                                    });
                            });

                            // Right column: Inputs + Slip ratios
                            hstrip.cell(|ui| {
                                let col_h = ui.available_height();
                                let half = (col_h - 6.0) * 0.5;

                                // Driver inputs (0..1)
                                Plot::new("inputs_plot")
                                    .legend(Legend::default())
                                    .height(half)
                                    .show(ui, |pui| {
                                        let names = ["Throttle", "Brake", "Clutch", "Steer"];
                                        let colors = [
                                            Color32::from_rgb(100, 220, 100),
                                            Color32::from_rgb(230, 100, 100),
                                            Color32::from_rgb(100, 160, 230),
                                            Color32::from_rgb(240, 200, 80),
                                        ];
                                        for i in 0..4 {
                                            let pts = PlotPoints::from_iter(
                                                self.plots.inputs[i]
                                                    .iter()
                                                    .map(|&(x, y)| [x as f64, y as f64]),
                                            );
                                            pui.line(Line::new(names[i], pts).color(colors[i]));
                                        }
                                        // Remove include_y (no longer in egui_plot)
                                        // pui.include_y(0.0);
                                        // pui.include_y(1.0);
                                    });

                                // Tire slip ratio per wheel
                                Plot::new("slip_plot")
                                    .legend(Legend::default())
                                    .height(half)
                                    .show(ui, |pui| {
                                        let colors = [
                                            Color32::from_rgb(255, 80, 80),   // FL
                                            Color32::from_rgb(255, 150, 50),  // FR
                                            Color32::from_rgb(50, 200, 255),  // RL
                                            Color32::from_rgb(180, 120, 255), // RR
                                        ];
                                        for i in 0..4 {
                                            let pts = PlotPoints::from_iter(
                                                self.plots.slip_ratios[i]
                                                    .iter()
                                                    .map(|&(x, y)| [x as f64, y as f64]),
                                            );
                                            pui.line(
                                                Line::new(format!("W{i} slip ratio"), pts)
                                                    .color(colors[i]),
                                            );
                                        }
                                        // show +/- 1 slip band
                                        pui.hline(HLine::new("", 0.0).color(Color32::GRAY));
                                        // Remove include_y (no longer in egui_plot)
                                        // pui.include_y(-1.0);
                                        // pui.include_y(1.0);
                                    });
                            });
                        });
                });
            });
    }

    fn draw_hud(&self, ui: &mut egui::Ui) {
        let r = ui.available_rect_before_wrap();
        let painter = ui.painter_at(r);
        let origin = Pos2::new(r.left() + 130.0, r.top() + 130.0);

        unsafe {
            let rpm = ffi::lcc_car_get_engine_rpm(&self.car);
            let redline = self.car.engine.redline_rpm.max(1.0);
            draw_rpm_gauge(&painter, origin, rpm, redline);

            let tb_origin = Pos2::new(origin.x + 160.0, origin.y - 60.0);
            draw_bar(
                &painter,
                tb_origin,
                "Throttle",
                self.car.throttle_input,
                Color32::LIGHT_GREEN,
            );
            draw_bar(
                &painter,
                Pos2::new(tb_origin.x, tb_origin.y + 24.0),
                "Brake",
                self.car.brake_input,
                Color32::LIGHT_RED,
            );
            draw_bar(
                &painter,
                Pos2::new(tb_origin.x, tb_origin.y + 48.0),
                "Clutch",
                self.car.clutch_input,
                Color32::LIGHT_BLUE,
            );
            draw_bar(
                &painter,
                Pos2::new(tb_origin.x, tb_origin.y + 72.0),
                "Steer",
                (self.car.steering_input + 1.0) / 2.0,
                Color32::YELLOW,
            );

            let gear = self.car.transmission.current_gear;
            let gtxt = if gear == -1 {
                "R".to_string()
            } else if gear == 0 {
                "N".to_string()
            } else {
                format!("{}", gear)
            };
            painter.text(
                Pos2::new(origin.x - 60.0, origin.y + 70.0),
                Align2::CENTER_CENTER,
                format!("Gear {}", gtxt),
                egui::FontId::proportional(18.0),
                Color32::WHITE,
            );

            let base = Pos2::new(origin.x + 320.0, origin.y - 80.0);
            let labels = ["FL", "FR", "RL", "RR"];
            for i in 0..4 {
                let w = &self.car.wheels[i];
                draw_wheel_info(
                    &painter,
                    Pos2::new(base.x, base.y + i as f32 * 30.0),
                    labels[i],
                    w,
                );
            }
        }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
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
            .default_width(280.0)
            .show(ctx, |ui| {
                self.draw_side_panel(ui);
            });

        egui::CentralPanel::default()
            .frame(egui::Frame::none().fill(Color32::from_gray(30)))
            .show(ctx, |ui| {
                self.draw_world(ui);
                self.draw_hud(ui);
            });

        ctx.request_repaint();
    }
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t.clamp(0.0, 1.0)
}

fn smooth_approach(v: f32, target: f32, rise: f32, fall: f32) -> f32 {
    let k = if target > v { rise } else { fall };
    lerp(v, target, k)
}

fn draw_grid(painter: &Painter, rect: Rect, cam: Vec2, zoom: f32) {
    let grid_color = Color32::from_gray(50);
    let bold_color = Color32::from_gray(100);
    let step_world = 1.0;
    let step = step_world * zoom;

    let center = rect.center();

    let to_screen = |w: Vec2| -> Pos2 {
        let rel = w - cam;
        Pos2::new(center.x + rel.x * zoom, center.y - rel.y * zoom)
    };

    let half_w = rect.width() / 2.0;
    let half_h = rect.height() / 2.0;
    let start_x = -((half_w / step).ceil() as i32);
    let end_x = (half_w / step).ceil() as i32;
    let start_y = -((half_h / step).ceil() as i32);
    let end_y = (half_h / step).ceil() as i32;

    for gx in start_x..=end_x {
        let wx = gx as f32 * step_world + cam.x.round();
        let p0 = to_screen(Vec2::new(wx, cam.y - (end_y as f32 + 1.0)));
        let p1 = to_screen(Vec2::new(wx, cam.y + (end_y as f32 + 1.0)));
        let bold = wx.abs() < 0.01;
        painter.line_segment(
            [p0, p1],
            Stroke::new(
                if bold { 2.0 } else { 1.0 },
                if bold { bold_color } else { grid_color },
            ),
        );
    }
    for gy in start_y..=end_y {
        let wy = gy as f32 * step_world + cam.y.round();
        let p0 = to_screen(Vec2::new(cam.x - (end_x as f32 + 1.0), wy));
        let p1 = to_screen(Vec2::new(cam.x + (end_x as f32 + 1.0), wy));
        let bold = wy.abs() < 0.01;
        painter.line_segment(
            [p0, p1],
            Stroke::new(
                if bold { 2.0 } else { 1.0 },
                if bold { bold_color } else { grid_color },
            ),
        );
    }
}

unsafe fn draw_wheel(
    painter: &egui::Painter,
    to_screen: &dyn Fn(Vec2) -> Pos2,
    center: Vec2,
    angle: f32,
    length_m: f32,
    width_m: f32,
    w: &ffi::lcc_wheel_state_t,
) {
    let l = length_m;
    let wth = width_m;
    let rot = Mat2::from_angle(angle);
    let half = Vec2::new(l * 0.5, wth * 0.5);
    let corners = [
        Vec2::new(half.x, -half.y),
        Vec2::new(half.x, half.y),
        Vec2::new(-half.x, half.y),
        Vec2::new(-half.x, -half.y),
    ]
    .map(|local| center + rot * local);

    let pts = vec![
        to_screen(corners[0]),
        to_screen(corners[1]),
        to_screen(corners[2]),
        to_screen(corners[3]),
    ];

    let fill = Color32::from_black_alpha(40);
    let outline = Color32::from_gray(200);
    painter.add(egui::Shape::convex_polygon(
        pts,
        fill,
        Stroke::new(1.5, outline),
    ));

    let slip_dir_local = Vec2::new(0.0, w.slip_angle.tan().clamp(-2.0, 2.0));
    let slip_world = center + rot * slip_dir_local * 0.6;
    painter.line_segment(
        [to_screen(center), to_screen(slip_world)],
        Stroke::new(2.0, Color32::from_rgb(255, 140, 0)),
    );

    let long_dir = center + rot * Vec2::new(w.slip_ratio.clamp(-1.5, 1.5), 0.0) * 0.6;
    painter.line_segment(
        [to_screen(center), to_screen(long_dir)],
        Stroke::new(2.0, Color32::from_rgb(0, 220, 180)),
    );

    let temp = w.temperature;
    let mu = w.surface_friction;
    let color = Color32::from_rgb(
        (temp.clamp(20.0, 150.0) - 20.0) as u8 * 255 / 130,
        (mu.clamp(0.3, 1.3) * 200.0) as u8,
        180,
    );
    painter.circle_filled(to_screen(center), 4.0, color);
}

fn draw_rpm_gauge(painter: &egui::Painter, origin: Pos2, rpm: f32, redline: f32) {
    let radius = 60.0;
    let bg = Color32::from_gray(40);

    // Outer circle
    painter.circle(
        origin,
        radius,
        Color32::from_black_alpha(10),
        Stroke::new(2.0, bg),
    );

    // Arc range
    let start_angle = -0.75 * PI;
    let end_angle = 0.75 * PI;

    // Color-coded arc (green->yellow->red)
    let arc = |from_t: f32, to_t: f32, col: Color32, width: f32| {
        let steps = 32;
        let mut pts = Vec::with_capacity(steps + 1);
        for i in 0..=steps {
            let tt = lerp(from_t, to_t, i as f32 / steps as f32);
            let ang = start_angle + (end_angle - start_angle) * tt;
            let r = radius - 4.0;
            pts.push(Pos2::new(
                origin.x + r * ang.cos(),
                origin.y + r * ang.sin(),
            ));
        }
        painter.add(egui::Shape::line(pts, Stroke::new(width, col)));
    };

    // Zones relative to redline
    arc(0.0, 0.7, Color32::from_rgb(90, 200, 90), 5.0); // green
    arc(0.7, 0.9, Color32::from_rgb(240, 200, 80), 5.0); // yellow
    arc(0.9, 1.0, Color32::from_rgb(220, 70, 70), 5.0); // red

    // Tick marks every 1/8th
    for i in 0..=8 {
        let t = i as f32 / 8.0;
        let ang = start_angle + (end_angle - start_angle) * t;
        let r0 = radius - 2.0;
        let r1 = if i % 2 == 0 {
            radius - 12.0
        } else {
            radius - 8.0
        };
        let p0 = Pos2::new(origin.x + r0 * ang.cos(), origin.y + r0 * ang.sin());
        let p1 = Pos2::new(origin.x + r1 * ang.cos(), origin.y + r1 * ang.sin());
        painter.line_segment([p0, p1], Stroke::new(2.0, Color32::from_gray(120)));
    }

    // Needle
    let t = (rpm / redline).clamp(0.0, 1.0);
    let needle_angle = start_angle + (end_angle - start_angle) * t;
    let needle_len = radius * 0.9;
    let tip = Pos2::new(
        origin.x + needle_len * needle_angle.cos(),
        origin.y + needle_len * needle_angle.sin(),
    );
    painter.line_segment([origin, tip], Stroke::new(3.0, Color32::LIGHT_RED));

    // Labels
    let label = format!("{:5.0} rpm", rpm);
    painter.text(
        Pos2::new(origin.x, origin.y + radius + 14.0),
        Align2::CENTER_CENTER,
        label,
        egui::FontId::proportional(14.0),
        Color32::WHITE,
    );
    painter.text(
        Pos2::new(origin.x, origin.y - radius - 16.0),
        Align2::CENTER_CENTER,
        format!("Redline: {:.0}", redline),
        egui::FontId::proportional(12.0),
        Color32::from_gray(180),
    );
}

fn draw_bar(painter: &egui::Painter, pos: Pos2, label: &str, value01: f32, color: Color32) {
    let w = 140.0;
    let h = 16.0;
    let rect = Rect::from_min_size(pos, vec2(w, h));
    painter.rect(
        rect,
        3.0,
        Color32::from_black_alpha(20),
        Stroke::new(1.0, Color32::from_gray(70)),
        egui::StrokeKind::Inside, // <-- Add this argument
    );
    let fill = Rect::from_min_size(pos, vec2(w * value01.clamp(0.0, 1.0), h));
    painter.rect(
        fill,
        3.0,
        color.gamma_multiply(0.5),
        Stroke::NONE,
        egui::StrokeKind::Inside, // <-- Add this argument
    );
    painter.text(
        Pos2::new(rect.left() - 6.0, rect.center().y),
        Align2::RIGHT_CENTER,
        label,
        egui::FontId::proportional(12.0),
        Color32::GRAY,
    );
}

unsafe fn draw_wheel_info(
    painter: &egui::Painter,
    pos: Pos2,
    label: &str,
    w: &ffi::lcc_wheel_state_t,
) {
    let txt = format!(
        "{label}  SR {:5.2}  SA {:5.2}  Load {:5.0}N  T {:4.1}°C  μ {:4.2}",
        w.slip_ratio, w.slip_angle, w.load, w.temperature, w.surface_friction
    );
    painter.text(
        pos,
        Align2::LEFT_TOP,
        txt,
        egui::FontId::monospace(12.0),
        Color32::WHITE,
    );
}

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
        Box::new(|_cc| Ok(Box::new(unsafe { App::new(ViewPreset::Sports) }))),
    )
}
