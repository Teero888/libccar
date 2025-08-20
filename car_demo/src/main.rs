use eframe::egui::{self, Align2, Pos2};
use libloading::Library;
use std::collections::HashMap;
use std::ffi::CStr;
use std::os::raw::{c_char, c_float, c_int};
use std::path::Path;
use std::time::{Duration, Instant};

const STEERING_RATE: f32 = 4.0;
const STEERING_CENTERING_RATE: f32 = 5.0;
const THROTTLE_RATE: f32 = 3.0;
const BRAKE_RATE: f32 = 5.0;

const LCC_DRIVE_RWD: c_int = 0;
const LCC_DRIVE_FWD: c_int = 1;
const LCC_DRIVE_AWD: c_int = 2;
const LCC_MAX_GEARS: usize = 8;

#[repr(C)]
#[derive(Clone, Copy, Debug)]
struct LccWheel {
    radius: c_float,
    angular_velocity: c_float,
    steer_angle: c_float,
    grip: c_float,
    position: [c_float; 2],
    load: c_float,
    slip_angle: c_float,
    slip_ratio: c_float,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
struct LccCar {
    wheels: [LccWheel; 4],

    position: [c_float; 2],
    velocity: [c_float; 2],
    prev_velocity: [c_float; 2],
    throttle: c_float,
    brake: c_float,
    steering: c_float,
    angle: c_float,
    angular_velocity: c_float,

    mass: c_float,
    inertia: c_float,
    wheelbase: c_float,
    track_width: c_float,
    cg_height: c_float,
    cg_position: c_float,

    drag_coeff: c_float,
    frontal_area: c_float,
    air_density: c_float,
    downforce_coeff: c_float,
    downforce_area: c_float,

    max_steer_angle: c_float,

    engine_power: c_float,
    drive_type: c_int,
    num_gears: c_int,
    gear: c_int,
    gear_ratios: [c_float; LCC_MAX_GEARS],
    final_drive: c_float,
    transmission_efficiency: c_float,

    idle_rpm: c_float,
    peak_rpm: c_float,
    redline_rpm: c_float,

    brake_bias: c_float,
    brake_torque_max: c_float,

    wheel_mass: c_float,
    lateral_friction_scale: c_float,

    long_transfer_factor: c_float,
    lat_transfer_factor: c_float,
    wheel_inertia_bias: c_float,

    engine_omega: c_float,
    flywheel_inertia: c_float,
    engine_friction_coeff: c_float,
    clutch: c_float,
    clutch_engagement_rate: c_float,
    clutch_max_torque: c_float,
    clutch_stiffness: c_float,
    auto_clutch: c_int,
}

impl Default for LccCar {
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

struct InputState {
    target_throttle: f32,
    target_brake: f32,
    target_steering: f32,

    current_throttle: f32,
    current_brake: f32,
    current_steering: f32,

    clutch_inc: bool,
    clutch_dec: bool,
    keys_pressed: HashMap<egui::Key, bool>,
}

impl Default for InputState {
    fn default() -> Self {
        Self {
            target_throttle: 0.0,
            target_brake: 0.0,
            target_steering: 0.0,
            current_throttle: 0.0,
            current_brake: 0.0,
            current_steering: 0.0,
            clutch_inc: false,
            clutch_dec: false,
            keys_pressed: HashMap::new(),
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
enum CameraMode {
    Free,
    Follow,
}

fn effective_mu(load: f32, mu_base: f32) -> f32 {
    let ref_load = 3000.0_f32;
    let fz = load.max(1.0);
    let mut mu = mu_base * (fz / ref_load).powf(-0.15);
    mu = mu.min(1.4 * mu_base).max(0.6 * mu_base);
    mu
}

#[derive(Clone, Copy, Debug)]
struct TireForceEst {
    fx_local: f32,
    fy_local: f32,
    fx_world: f32,
    fy_world: f32,
    cap: f32,
    util: f32,
    mu_eff: f32,
}

struct CarSimApp {
    lib: Option<Library>,
    lib_path: String,
    car: LccCar,
    input: InputState,
    version: String,
    has_clutch_api: bool,

    auto_clutch_enabled: bool,
    manual_clutch: f32,

    last_update: Instant,
    error: Option<String>,
    paused: bool,
    auto_reload: bool,
    last_reload: Instant,
    reload_interval: Duration,
    last_lib_modified: Option<std::time::SystemTime>,
    camera_pos: egui::Pos2,
    zoom: f32,
    trail_points: Vec<egui::Pos2>,
    max_trail_points: usize,
    show_forces: bool,
    camera_mode: CameraMode,

    show_tire_forces: bool,
    show_traction_circles: bool,
    show_yaw_moment: bool,
    show_accel: bool,

    last_dt: f32,
    accel_world: [f32; 2],

    wheel_radius: f32,
    wheel_grip: f32,
    mass: f32,
    wheel_base: f32,
    track_width: f32,
    engine_power: f32,
}

impl CarSimApp {
    fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let lib_path = if cfg!(target_os = "windows") {
            "./libccar.dll".to_string()
        } else if cfg!(target_os = "macos") {
            "./libccar.dylib".to_string()
        } else {
            "./libccar.so".to_string()
        };

        let mut app = CarSimApp {
            lib: None,
            lib_path,
            car: LccCar::default(),
            input: InputState::default(),
            version: "Unknown".to_string(),
            has_clutch_api: false,
            auto_clutch_enabled: true,
            manual_clutch: 1.0,
            last_update: Instant::now(),
            last_reload: Instant::now(),
            reload_interval: Duration::from_secs(1),
            error: None,
            camera_pos: egui::pos2(0.0, 0.0),
            zoom: 25.0,
            trail_points: Vec::new(),
            max_trail_points: 500,
            show_forces: false,
            paused: false,
            auto_reload: true,
            last_lib_modified: None,
            camera_mode: CameraMode::Follow,

            show_tire_forces: false,
            show_traction_circles: false,
            show_yaw_moment: false,
            show_accel: false,
            last_dt: 1.0 / 60.0,
            accel_world: [0.0, 0.0],

            wheel_radius: 0.35,
            wheel_grip: 2.5,
            mass: 1200.0,
            wheel_base: 2.5,
            track_width: 1.5,
            engine_power: 300000.0,
        };
        app.try_load_library();
        app
    }

    fn try_load_library(&mut self) {
        if !Path::new(&self.lib_path).exists() {
            self.error = Some(format!("Library file {} not found", self.lib_path));
            self.lib = None;
            return;
        }

        if let Ok(metadata) = std::fs::metadata(&self.lib_path) {
            if let Ok(modified) = metadata.modified() {
                if let Some(last_modified) = self.last_lib_modified {
                    if modified <= last_modified && self.lib.is_some() {
                        return;
                    }
                }
                self.last_lib_modified = Some(modified);
            }
        }

        self.lib = None;

        match unsafe { Library::new(&self.lib_path) } {
            Ok(lib) => {
                self.error = None;
                let was_initialized = self.car.mass > 0.0;
                self.has_clutch_api = false;
                self.lib = Some(lib);

                if let Some(lib) = &self.lib {
                    let has_enable = unsafe {
                        lib.get::<unsafe extern "C" fn(*mut LccCar, c_int)>(
                            b"lcc_car_enable_auto_clutch\0",
                        )
                    }
                    .is_ok();
                    let has_set = unsafe {
                        lib.get::<unsafe extern "C" fn(*mut LccCar, c_float)>(
                            b"lcc_car_set_clutch\0",
                        )
                    }
                    .is_ok();
                    self.has_clutch_api = has_enable && has_set;
                }

                if !was_initialized {
                    self.reset_car_to_defaults();
                }
                self.reload_version();
            }
            Err(e) => {
                self.error = Some(format!("Failed to load library: {}", e));
                self.lib = None;
            }
        }
    }

    fn reset_car_to_defaults(&mut self) {
        if let Some(lib) = &self.lib {
            if let Ok(lcc_car_init) = unsafe {
                lib.get::<unsafe extern "C" fn(
                    c_float,
                    c_float,
                    c_float,
                    c_float,
                    c_float,
                    c_float,
                ) -> LccCar>(b"lcc_car_init\0")
            } {
                self.car = unsafe {
                    lcc_car_init(
                        self.wheel_radius,
                        self.wheel_grip,
                        self.mass,
                        self.wheel_base,
                        self.track_width,
                        self.engine_power,
                    )
                };

                self.auto_clutch_enabled = self.car.auto_clutch != 0;
                self.manual_clutch = self.car.clutch;
            }
        }
    }

    fn reload_version(&mut self) {
        if let Some(lib) = &self.lib {
            if let Ok(lcc_get_version) =
                unsafe { lib.get::<unsafe extern "C" fn() -> *const c_char>(b"lcc_get_version\0") }
            {
                self.version = unsafe {
                    CStr::from_ptr(lcc_get_version())
                        .to_str()
                        .unwrap_or("Unknown")
                        .to_string()
                };
            }
        }
    }

    fn update_input(&mut self, ctx: &egui::Context) {
        ctx.input(|i| {
            for event in &i.events {
                if let egui::Event::Key { key, pressed, .. } = event {
                    self.input.keys_pressed.insert(*key, *pressed);
                }
            }
        });

        let is_pressed = |key1, key2| {
            self.input.keys_pressed.get(key1).copied().unwrap_or(false)
                || self.input.keys_pressed.get(key2).copied().unwrap_or(false)
        };

        let pos_throttle = if is_pressed(&egui::Key::ArrowUp, &egui::Key::W) {
            1.0
        } else {
            0.0
        };
        let neg_throttle = if is_pressed(&egui::Key::Z, &egui::Key::X) {
            1.0
        } else {
            0.0
        };
        self.input.target_throttle = ((pos_throttle - neg_throttle) as f32).clamp(-1.0, 1.0);

        self.input.target_brake = if is_pressed(&egui::Key::ArrowDown, &egui::Key::S) {
            1.0
        } else {
            0.0
        };

        let mut steering_target = 0.0;
        if is_pressed(&egui::Key::ArrowLeft, &egui::Key::A) {
            steering_target -= 1.0;
        }
        if is_pressed(&egui::Key::ArrowRight, &egui::Key::D) {
            steering_target += 1.0;
        }
        self.input.target_steering = steering_target;

        self.input.clutch_dec = self
            .input
            .keys_pressed
            .get(&egui::Key::Q)
            .copied()
            .unwrap_or(false);
        self.input.clutch_inc = self
            .input
            .keys_pressed
            .get(&egui::Key::E)
            .copied()
            .unwrap_or(false);
    }

    fn smooth_inputs(&mut self, dt: f32) {
        let steer_rate = if self.input.target_steering.abs() < 0.01
            && self.input.current_steering.abs() > 0.01
        {
            STEERING_CENTERING_RATE
        } else {
            STEERING_RATE
        };
        let delta_steer = self.input.target_steering - self.input.current_steering;
        self.input.current_steering += delta_steer * steer_rate * dt;
        self.input.current_steering = self.input.current_steering.clamp(-1.0, 1.0);

        let delta_throttle = self.input.target_throttle - self.input.current_throttle;
        self.input.current_throttle += delta_throttle * THROTTLE_RATE * dt;
        self.input.current_throttle = self.input.current_throttle.clamp(-1.0, 1.0);

        let delta_brake = self.input.target_brake - self.input.current_brake;
        self.input.current_brake += delta_brake * BRAKE_RATE * dt;
        self.input.current_brake = self.input.current_brake.clamp(0.0, 1.0);
    }

    fn apply_realtime_param_edits(&mut self) {
        let ng = self.car.num_gears.clamp(1, LCC_MAX_GEARS as c_int) as usize;
        self.car.num_gears = ng as c_int;
        if self.car.gear < 0 {
            self.car.gear = 0;
        }
        if (self.car.gear as usize) >= ng {
            self.car.gear = (ng as c_int - 1).max(0);
        }
        if ![LCC_DRIVE_RWD, LCC_DRIVE_FWD, LCC_DRIVE_AWD].contains(&self.car.drive_type) {
            self.car.drive_type = LCC_DRIVE_RWD;
        }
        if self.car.air_density <= 0.0 {
            self.car.air_density = 1.225;
        }

        self.car.flywheel_inertia = self.car.flywheel_inertia.max(0.01);
        self.car.engine_friction_coeff = self.car.engine_friction_coeff.max(0.0);
        self.car.clutch_max_torque = self.car.clutch_max_torque.max(0.0);
        self.car.clutch_stiffness = self.car.clutch_stiffness.max(0.0);
        self.car.clutch_engagement_rate = self.car.clutch_engagement_rate.clamp(0.1, 100.0);
    }

    fn update_car_physics(&mut self, dt: f32) {
        self.apply_realtime_param_edits();

        if !self.auto_clutch_enabled {
            let rate = self.car.clutch_engagement_rate as f32;
            let mut target = self.manual_clutch;
            if self.input.clutch_dec {
                target = 0.0;
            } else if self.input.clutch_inc {
                target = 1.0;
            }

            let step = (rate * dt).clamp(0.0, 1.0);
            self.manual_clutch = self.manual_clutch + (target - self.manual_clutch) * step;
            self.manual_clutch = self.manual_clutch.clamp(0.0, 1.0);
        }

        if let Some(lib) = &self.lib {
            if let Ok(set_input) = unsafe {
                lib.get::<unsafe extern "C" fn(*mut LccCar, c_float, c_float, c_float)>(
                    b"lcc_car_set_input\0",
                )
            } {
                unsafe {
                    set_input(
                        &mut self.car,
                        self.input.current_throttle,
                        self.input.current_brake,
                        self.input.current_steering,
                    );
                }
            }

            if self.has_clutch_api {
                if let Ok(enable_auto) = unsafe {
                    lib.get::<unsafe extern "C" fn(*mut LccCar, c_int)>(
                        b"lcc_car_enable_auto_clutch\0",
                    )
                } {
                    unsafe {
                        enable_auto(&mut self.car, if self.auto_clutch_enabled { 1 } else { 0 });
                    }
                }
                if !self.auto_clutch_enabled {
                    if let Ok(set_clutch) = unsafe {
                        lib.get::<unsafe extern "C" fn(*mut LccCar, c_float)>(
                            b"lcc_car_set_clutch\0",
                        )
                    } {
                        unsafe {
                            set_clutch(&mut self.car, self.manual_clutch);
                        }
                    }
                }
            } else {
                self.car.auto_clutch = if self.auto_clutch_enabled { 1 } else { 0 };
                if !self.auto_clutch_enabled {
                    self.car.clutch = self.manual_clutch;
                }
            }

            if let Ok(update) = unsafe {
                lib.get::<unsafe extern "C" fn(*mut LccCar, c_float)>(b"lcc_car_update\0")
            } {
                unsafe { update(&mut self.car, dt) };
            }
        }

        self.trail_points
            .push(egui::Pos2::new(self.car.position[0], self.car.position[1]));
        if self.trail_points.len() > self.max_trail_points {
            self.trail_points.remove(0);
        }
    }

    fn world_to_screen(&self, world_pos: egui::Pos2, rect: egui::Rect) -> egui::Pos2 {
        let relative_pos = world_pos.to_vec2() - self.camera_pos.to_vec2();
        let screen_x = rect.center().x + relative_pos.x * self.zoom;
        let screen_y = rect.center().y - relative_pos.y * self.zoom;
        egui::Pos2::new(screen_x, screen_y)
    }

    fn screen_to_world(&self, screen_pos: egui::Pos2, rect: egui::Rect) -> egui::Pos2 {
        let relative_screen_pos = egui::vec2(
            (screen_pos.x - rect.center().x) / self.zoom,
            -(screen_pos.y - rect.center().y) / self.zoom,
        );

        self.camera_pos + relative_screen_pos
    }

    fn draw_simulation_view(&mut self, ui: &mut egui::Ui) {
        let available_rect = ui.available_rect_before_wrap();
        let response = ui.allocate_rect(available_rect, egui::Sense::click_and_drag());

        if response.hovered() {
            let scroll_delta = ui.input(|i| i.raw_scroll_delta.y);
            if scroll_delta != 0.0 {
                if let Some(hover_pos) = if self.camera_mode == CameraMode::Follow {
                    Some(Pos2::new(
                        available_rect.width() / 2.0,
                        available_rect.height() / 2.0,
                    ))
                } else {
                    ui.input(|i| i.pointer.hover_pos())
                } {
                    let world_pos_before_zoom = self.screen_to_world(hover_pos, available_rect);
                    self.zoom *= (1.0 + scroll_delta * 0.001).max(0.1);
                    self.zoom = self.zoom.clamp(2.0, 200.0);

                    let world_pos_after_zoom = self.screen_to_world(hover_pos, available_rect);
                    self.camera_pos += world_pos_before_zoom - world_pos_after_zoom;
                }
            }
        }

        if self.camera_mode == CameraMode::Free
            && (response.dragged_by(egui::PointerButton::Primary)
                || response.dragged_by(egui::PointerButton::Middle))
        {
            self.camera_pos.x -= response.drag_delta().x / self.zoom;
            self.camera_pos.y += response.drag_delta().y / self.zoom;
        }

        if self.lib.is_some() {
            let painter = ui.painter_at(available_rect);
            self.draw_grid(&painter, available_rect);
            self.draw_car_trail(&painter, available_rect);
            self.draw_car(&painter, available_rect);
            self.draw_hud_overlay(&painter, available_rect);
        } else {
            ui.centered_and_justified(|ui| {
                ui.label("Library not loaded. Check path and errors.");
            });
        }
    }

    fn draw_grid(&self, painter: &egui::Painter, rect: egui::Rect) {
        let grid_color = egui::Color32::from_gray(50);
        let thick_grid_color = egui::Color32::from_gray(80);

        let top_left_world = self.screen_to_world(rect.min, rect);
        let bottom_right_world = self.screen_to_world(rect.max, rect);

        let min_spacing_pixels = 50.0;
        let base_spacing = (min_spacing_pixels / self.zoom).log10().ceil();
        let spacing = 10.0f32.powf(base_spacing);

        let start_x = (top_left_world.x / spacing).floor() * spacing;
        let mut x = start_x;
        while x < bottom_right_world.x {
            let p1 = self.world_to_screen(egui::pos2(x, top_left_world.y), rect);
            let p2 = self.world_to_screen(egui::pos2(x, bottom_right_world.y), rect);
            let is_thick = x.abs() < 1e-6 || (x / (spacing * 5.0)).fract() == 0.0;
            painter.line_segment(
                [p1, p2],
                egui::Stroke::new(
                    if is_thick { 1.5 } else { 0.5 },
                    if is_thick {
                        thick_grid_color
                    } else {
                        grid_color
                    },
                ),
            );
            x += spacing;
        }

        let start_y = (bottom_right_world.y / spacing).floor() * spacing;
        let mut y = start_y;
        while y < top_left_world.y {
            let p1 = self.world_to_screen(egui::pos2(top_left_world.x, y), rect);
            let p2 = self.world_to_screen(egui::pos2(bottom_right_world.x, y), rect);
            let is_thick = y.abs() < 1e-6 || (y / (spacing * 5.0)).fract() == 0.0;
            painter.line_segment(
                [p1, p2],
                egui::Stroke::new(
                    if is_thick { 1.5 } else { 0.5 },
                    if is_thick {
                        thick_grid_color
                    } else {
                        grid_color
                    },
                ),
            );
            y += spacing;
        }
    }

    fn draw_car_trail(&self, painter: &egui::Painter, rect: egui::Rect) {
        if self.trail_points.len() > 1 {
            let trail_color = egui::Color32::from_rgba_unmultiplied(255, 255, 0, 100);
            let points: Vec<egui::Pos2> = self
                .trail_points
                .iter()
                .map(|&p| self.world_to_screen(p, rect))
                .collect();
            painter.add(egui::Shape::line(
                points,
                egui::Stroke::new(2.0, trail_color),
            ));
        }
    }

    fn is_wheel_driven(&self, idx: usize) -> bool {
        match self.car.drive_type {
            x if x == LCC_DRIVE_RWD => idx >= 2,
            x if x == LCC_DRIVE_FWD => idx < 2,
            _ => true,
        }
    }

    fn estimate_wheel_forces(&self, idx: usize) -> Option<TireForceEst> {
        let w = &self.car.wheels[idx];
        let load = w.load.max(1.0);
        if load < 1.0 {
            return None;
        }
        let mu_eff = effective_mu(load, w.grip);
        let fz = load;

        let (c_lat, d_lat, e_lat) = (1.35_f32, 1.0_f32, -1.3_f32);
        let (c_long, d_long, e_long) = (1.65_f32, 1.0_f32, -0.5_f32);
        let fz_ref = 4000.0_f32;
        let cf_ref = 20000.0_f32;
        let kx_ref = 25000.0_f32;

        let cf = cf_ref * (fz / fz_ref).powf(0.8);
        let kx = kx_ref * (fz / fz_ref).powf(0.8);

        let denom_lat = (mu_eff * fz * c_lat * d_lat).max(1e-3);
        let denom_long = (mu_eff * fz * c_long * d_long).max(1e-3);
        let b_lat = cf / denom_lat;
        let b_long = kx / denom_long;

        let alpha = w.slip_angle;
        let kappa = w.slip_ratio.clamp(-1.0, 1.0);

        let t_lat = b_lat * alpha;
        let fy0 = -d_lat
            * mu_eff
            * fz
            * (c_lat * (t_lat - e_lat * (t_lat - t_lat.atan())).atan()).sin()
            * self.car.lateral_friction_scale;

        let t_long = b_long * kappa;
        let fx0 = d_long
            * mu_eff
            * fz
            * (c_long * (t_long - e_long * (t_long - t_long.atan())).atan()).sin();

        let cap = mu_eff * fz;
        let nx = fx0 / cap.max(1.0);
        let ny = fy0 / cap.max(1.0);
        let p = 1.6_f32;
        let util = (nx.abs().powf(p) + ny.abs().powf(p)).powf(1.0 / p);
        let scale = if util > 1.0 { 1.0 / util } else { 1.0 };
        let fx = fx0 * scale;
        let fy = fy0 * scale;

        let wheel_world_angle = self.car.angle + w.steer_angle;
        let c = wheel_world_angle.cos();
        let s = wheel_world_angle.sin();
        let fx_world = fx * c - fy * s;
        let fy_world = fx * s + fy * c;

        Some(TireForceEst {
            fx_local: fx,
            fy_local: fy,
            fx_world,
            fy_world,
            cap,
            util: util.min(1.5),
            mu_eff,
        })
    }

    fn draw_car(&self, painter: &egui::Painter, rect: egui::Rect) {
        let car_world_pos = egui::pos2(self.car.position[0], self.car.position[1]);
        let car_screen_pos = self.world_to_screen(car_world_pos, rect);

        let car_length = self.car.wheelbase * self.zoom;
        let car_width = self.car.track_width * self.zoom;
        let half_length = car_length / 2.0;
        let half_width = car_width / 2.0;

        let rot = egui::emath::Rot2::from_angle(-self.car.angle);
        let body_points: Vec<egui::Pos2> = vec![
            egui::pos2(-half_length, -half_width),
            egui::pos2(half_length, -half_width),
            egui::pos2(half_length, half_width),
            egui::pos2(-half_length, half_width),
        ]
        .into_iter()
        .map(|p| car_screen_pos + rot * p.to_vec2())
        .collect();

        painter.add(egui::Shape::convex_polygon(
            body_points,
            egui::Color32::from_rgb(60, 120, 200),
            egui::Stroke::new(2.0, egui::Color32::BLACK),
        ));

        let mut yaw_mz_est = 0.0_f32;

        for (i, wheel) in self.car.wheels.iter().enumerate() {
            let wheel_local_pos = egui::vec2(wheel.position[0], wheel.position[1]);
            let rotated_offset = rot * wheel_local_pos;
            let wheel_screen_pos = car_screen_pos + rotated_offset * self.zoom;

            let static_wheel_load = (self.car.mass * 9.81 / 3.0) as f32;
            let load_factor = (wheel.load / static_wheel_load as c_float).clamp(0.0, 2.0) as f32;
            let wheel_color: egui::Color32 = egui::lerp(
                egui::Rgba::from_rgb(0.0, 1.0, 0.0)..=egui::Rgba::from_rgb(1.0, 0.0, 0.0),
                load_factor,
            )
            .into();

            let wheel_radius_screen = wheel.radius * self.zoom;

            painter.circle(
                wheel_screen_pos,
                wheel_radius_screen,
                wheel_color,
                egui::Stroke::new(1.5, egui::Color32::BLACK),
            );

            if self.is_wheel_driven(i) {
                painter.circle_stroke(
                    wheel_screen_pos,
                    wheel_radius_screen * 1.15,
                    egui::Stroke::new(2.0, egui::Color32::YELLOW),
                );
            }

            let total_steer_angle = self.car.angle + wheel.steer_angle;
            let dir = egui::vec2(total_steer_angle.cos(), -total_steer_angle.sin());
            let line_end = wheel_screen_pos + dir * wheel_radius_screen * 1.5;
            painter.line_segment(
                [wheel_screen_pos, line_end],
                egui::Stroke::new(2.0, egui::Color32::from_rgb(20, 20, 20)),
            );

            let slip = wheel.slip_angle.clamp(-0.7, 0.7);
            let perp = egui::vec2(-(total_steer_angle).sin(), -(total_steer_angle).cos());
            let slip_len = wheel_radius_screen * 1.2 * slip.abs() as f32;
            let slip_end = wheel_screen_pos + perp * slip_len * slip.signum() as f32;
            painter.arrow(
                wheel_screen_pos,
                slip_end - wheel_screen_pos,
                egui::Stroke::new(2.0, egui::Color32::RED),
            );

            if let Some(est) = self.estimate_wheel_forces(i) {
                let wp_world = egui::Vec2::new(
                    wheel.position[0] * self.car.angle.cos()
                        - wheel.position[1] * self.car.angle.sin(),
                    wheel.position[0] * self.car.angle.sin()
                        + wheel.position[1] * self.car.angle.cos(),
                );

                yaw_mz_est += wp_world.x * est.fy_world - wp_world.y * est.fx_world;

                let dir_long = egui::vec2(total_steer_angle.cos(), -total_steer_angle.sin());
                let dir_lat = egui::vec2(-total_steer_angle.sin(), -total_steer_angle.cos());

                if self.show_tire_forces {
                    let r = wheel_radius_screen * 1.2;
                    let cap = est.cap.max(1.0);
                    let nx = est.fx_local / cap;
                    let ny = est.fy_local / cap;
                    let tip = wheel_screen_pos + dir_long * (nx * r) + dir_lat * (ny * r);
                    let col = if est.util < 0.7 {
                        egui::Color32::from_rgb(50, 220, 50)
                    } else if est.util < 1.0 {
                        egui::Color32::from_rgb(250, 200, 0)
                    } else {
                        egui::Color32::from_rgb(240, 60, 60)
                    };
                    painter.arrow(
                        wheel_screen_pos,
                        tip - wheel_screen_pos,
                        egui::Stroke::new(2.0, col),
                    );
                }

                if self.show_traction_circles {
                    let r = wheel_radius_screen * 1.15;

                    painter.circle_stroke(
                        wheel_screen_pos,
                        r,
                        egui::Stroke::new(1.0, egui::Color32::from_gray(40)),
                    );

                    painter.line_segment(
                        [
                            wheel_screen_pos - dir_long * r,
                            wheel_screen_pos + dir_long * r,
                        ],
                        egui::Stroke::new(1.0, egui::Color32::from_gray(60)),
                    );
                    painter.line_segment(
                        [
                            wheel_screen_pos - dir_lat * r,
                            wheel_screen_pos + dir_lat * r,
                        ],
                        egui::Stroke::new(1.0, egui::Color32::from_gray(60)),
                    );

                    let cap = est.cap.max(1.0);
                    let nx = est.fx_local / cap;
                    let ny = est.fy_local / cap;
                    let pt = wheel_screen_pos + dir_long * (nx * r) + dir_lat * (ny * r);
                    let col = if est.util < 0.7 {
                        egui::Color32::from_rgb(50, 220, 50)
                    } else if est.util < 1.0 {
                        egui::Color32::from_rgb(250, 200, 0)
                    } else {
                        egui::Color32::from_rgb(240, 60, 60)
                    };
                    painter.circle_filled(pt, 3.0, col);
                }
            }
        }

        if self.show_forces {
            let vel_vec = egui::vec2(self.car.velocity[0], self.car.velocity[1]) * 0.25 * self.zoom;
            painter.arrow(
                car_screen_pos,
                egui::vec2(vel_vec.x, -vel_vec.y),
                egui::Stroke::new(2.0, egui::Color32::CYAN),
            );

            let speed = (self.car.velocity[0].hypot(self.car.velocity[1])) as f32;
            let df = 0.5
                * self.car.air_density as f32
                * self.car.downforce_coeff as f32
                * self.car.downforce_area as f32
                * speed
                * speed;
            if df > 1.0 {
                let df_len = (df / (self.car.mass as f32 * 9.81)).clamp(0.0, 2.0) * 50.0;
                painter.arrow(
                    car_screen_pos,
                    egui::vec2(0.0, df_len),
                    egui::Stroke::new(2.0, egui::Color32::from_rgb(200, 0, 200)),
                );
            }
        }

        if self.show_accel {
            let a = egui::vec2(self.accel_world[0], self.accel_world[1]) * 0.2 * self.zoom;
            if a.length_sq() > 1e-6 {
                painter.arrow(
                    car_screen_pos,
                    egui::vec2(a.x, -a.y),
                    egui::Stroke::new(2.0, egui::Color32::from_rgb(255, 0, 255)),
                );
            }
        }

        if self.show_yaw_moment {
            let scale_ref = (self.car.mass as f32 * 9.81 * self.car.wheelbase) as f32;
            let t = (yaw_mz_est / scale_ref).clamp(-1.5, 1.5);
            if t.abs() > 1e-3 {
                let dir_sign = if t >= 0.0 { 1.0 } else { -1.0 };
                let len = (t.abs()) * 60.0;
                let tang_world = egui::vec2(-self.car.angle.sin(), self.car.angle.cos());
                let tang_screen = egui::vec2(tang_world.x, -tang_world.y) * dir_sign * len;
                painter.arrow(
                    car_screen_pos,
                    tang_screen,
                    egui::Stroke::new(
                        2.0,
                        if dir_sign > 0.0 {
                            egui::Color32::LIGHT_GREEN
                        } else {
                            egui::Color32::RED
                        },
                    ),
                );
            }
        }
    }

    fn drivetrain_ratio(&self) -> f32 {
        let ng = self.car.num_gears.clamp(1, LCC_MAX_GEARS as c_int) as usize;
        let g = (self.car.gear as usize).min(ng.saturating_sub(1));
        self.car.gear_ratios[g] * self.car.final_drive
    }

    fn engine_rpm(&self) -> f32 {
        (self.car.engine_omega as f32) * (60.0 / (2.0 * std::f32::consts::PI))
    }

    fn draw_hud_overlay(&self, painter: &egui::Painter, rect: egui::Rect) {
        let speed = (self.car.velocity[0].hypot(self.car.velocity[1])) as f32;
        let speed_kmh = speed * 3.6;
        let rpm = self.engine_rpm().clamp(0.0, self.car.redline_rpm.max(1.0));
        let ng = self.car.num_gears.max(1) as usize;
        let gear_idx = (self.car.gear.max(0) as usize).min(ng.saturating_sub(1));
        let total_ratio = self.drivetrain_ratio();

        let drive_str = match self.car.drive_type {
            x if x == LCC_DRIVE_RWD => "RWD",
            x if x == LCC_DRIVE_FWD => "FWD",
            _ => "AWD",
        };

        let drag = 0.5
            * self.car.air_density as f32
            * self.car.drag_coeff as f32
            * self.car.frontal_area as f32
            * speed
            * speed;
        let downforce = 0.5
            * self.car.air_density as f32
            * self.car.downforce_coeff as f32
            * self.car.downforce_area as f32
            * speed
            * speed;
        let accel = (self.accel_world[0].hypot(self.accel_world[1])) as f32;

        let clutch_val = if self.auto_clutch_enabled {
            self.car.clutch as f32
        } else {
            self.manual_clutch
        };

        let text = format!(
            "v={:5.1} km/h | Gear {:>1}/{} | Ratio {:.2} | RPM {:>5.0} | {} | Drag {:>6.0} N | DF {:>6.0} N | acc={:>4.1} m/s² | Clutch {} {:.2}",
            speed_kmh,
            gear_idx + 1,
            ng,
            total_ratio,
            rpm,
            drive_str,
            drag,
            downforce,
            accel,
            if self.auto_clutch_enabled { "AUTO" } else { "MAN" },
            clutch_val
        );

        let font = egui::FontId::monospace(16.0);
        painter.text(
            rect.left_top() + egui::vec2(10.0, 10.0),
            Align2::LEFT_TOP,
            text,
            font,
            egui::Color32::WHITE,
        );

        let bar_w = 240.0;
        let bar_h = 10.0;
        let x = rect.left_top().x + 10.0;
        let y = rect.left_top().y + 35.0;
        let t = (rpm / self.car.redline_rpm.max(1.0)).clamp(0.0, 1.0);
        let filled_w = bar_w * t;
        painter.rect_filled(
            egui::Rect::from_min_size(egui::pos2(x, y), egui::vec2(bar_w, bar_h)),
            2.0,
            egui::Color32::from_gray(40),
        );
        painter.rect_filled(
            egui::Rect::from_min_size(egui::pos2(x, y), egui::vec2(filled_w, bar_h)),
            2.0,
            if t > 0.85 {
                egui::Color32::RED
            } else {
                egui::Color32::LIGHT_GREEN
            },
        );
    }

    fn draw_side_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Car Simulation");
        ui.label(format!("Library version: {}", self.version));
        if let Some(error) = &self.error {
            ui.colored_label(egui::Color32::RED, error);
        }

        ui.separator();

        ui.collapsing("Controls", |ui| {
            ui.label("W/Up: Throttle, Z/X: Engine Brake (negative throttle)");
            ui.label("S/Down: Wheel Brakes");
            ui.label("A/Left, D/Right: Steering");
            ui.label("Q: Disengage clutch (hold)");
            ui.label("E: Engage clutch (hold)");
            ui.label("C: Toggle Auto-Clutch");
            ui.label("Mouse Drag: Pan View (in Free mode)");
            ui.label("Mouse Wheel: Zoom View");
            ui.label("Spacebar: Pause/Resume");
            ui.label("R: Reset Car");
        });

        ui.separator();

        ui.horizontal(|ui| {
            if ui
                .button(if self.paused {
                    "▶ Resume"
                } else {
                    "⏸ Pause"
                })
                .clicked()
            {
                self.paused = !self.paused;
            }
            if ui.button("🔄 Reset Car").clicked() {
                self.reset_car_to_defaults();
                self.trail_points.clear();
            }
        });
        ui.checkbox(&mut self.auto_reload, "Auto-reload library");
        if ui.button("Reload Library Now").clicked() {
            self.last_lib_modified = None;
            self.try_load_library();
        }

        ui.separator();

        ui.collapsing("Camera", |ui| {
            ui.horizontal(|ui| {
                ui.label("Mode:");
                ui.selectable_value(&mut self.camera_mode, CameraMode::Free, "Free");
                ui.selectable_value(&mut self.camera_mode, CameraMode::Follow, "Follow");
            });
        });

        ui.separator();

        ui.collapsing("Visualization", |ui| {
            ui.checkbox(&mut self.show_forces, "Show Velocity/Downforce");
            ui.checkbox(&mut self.show_accel, "Show Acceleration");
            ui.checkbox(&mut self.show_tire_forces, "Show Tire Forces");
            ui.checkbox(&mut self.show_traction_circles, "Show Traction Circles");
            ui.checkbox(&mut self.show_yaw_moment, "Show Yaw Moment");
            ui.add(egui::Slider::new(&mut self.max_trail_points, 10..=2000).text("Trail Length"));
        })
        .header_response
        .context_menu(|ui| {
            if ui.button("Reset Visualization").clicked() {
                self.show_forces = true;
                self.show_accel = true;
                self.show_tire_forces = true;
                self.show_traction_circles = true;
                self.show_yaw_moment = true;
                self.max_trail_points = 500;
                ui.close();
            }
        });

        ui.separator();

        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.collapsing("Real-time Data", |ui| {
                let speed_kmh =
                    (self.car.velocity[0].powi(2) + self.car.velocity[1].powi(2)).sqrt() * 3.6;
                ui.label(format!("Speed: {:.1} km/h", speed_kmh));
                ui.label(format!(
                    "Position: ({:.1}, {:.1}) m",
                    self.car.position[0], self.car.position[1]
                ));
                ui.label(format!("Angle: {:.1}°", self.car.angle.to_degrees()));
                ui.label(format!(
                    "Angular Vel: {:.2} rad/s",
                    self.car.angular_velocity
                ));
                ui.separator();
                ui.label("Inputs (applied to car):");
                ui.label(format!("  Throttle: {:.2}", self.car.throttle));
                ui.label(format!("  Brake: {:.2}", self.car.brake));
                ui.label(format!("  Steering: {:.2}", self.car.steering));
                ui.separator();
                ui.label(format!(
                    "Gear: {}/{}",
                    (self.car.gear + 1).max(1),
                    self.car.num_gears
                ));
                ui.label(format!("Engine RPM: {:.0}", self.engine_rpm()));
                ui.label(format!(
                    "Clutch: {} {:.2}",
                    if self.auto_clutch_enabled {
                        "AUTO"
                    } else {
                        "MAN"
                    },
                    if self.auto_clutch_enabled {
                        self.car.clutch as f32
                    } else {
                        self.manual_clutch
                    }
                ));
            });

            ui.separator();

            ui.collapsing("Init Parameters (require Reset)", |ui| {
                ui.add(egui::Slider::new(&mut self.wheel_radius, 0.1..=1.0).text("wheel radius"));
                ui.add(egui::Slider::new(&mut self.wheel_grip, 0.0..=10.0).text("wheel grip (mu)"));
                ui.add(egui::Slider::new(&mut self.mass, 200.0..=2000.0).text("mass (kg)"));
                ui.add(egui::Slider::new(&mut self.wheel_base, 1.5..=5.0).text("wheelbase (m)"));
                ui.add(egui::Slider::new(&mut self.track_width, 1.0..=3.0).text("track width (m)"));
                ui.add(
                    egui::Slider::new(&mut self.engine_power, 1_000.0..=1_000_000.0)
                        .text("engine power (W)"),
                );
                if ui.button("Apply & Reset").clicked() {
                    self.reset_car_to_defaults();
                    self.trail_points.clear();
                }
            });

            ui.separator();

            ui.collapsing("Powertrain", |ui| {
                ui.horizontal(|ui| {
                    ui.label("Drive type:");
                    if ui
                        .selectable_label(self.car.drive_type == LCC_DRIVE_RWD, "RWD")
                        .clicked()
                    {
                        self.car.drive_type = LCC_DRIVE_RWD;
                    }
                    if ui
                        .selectable_label(self.car.drive_type == LCC_DRIVE_FWD, "FWD")
                        .clicked()
                    {
                        self.car.drive_type = LCC_DRIVE_FWD;
                    }
                    if ui
                        .selectable_label(self.car.drive_type == LCC_DRIVE_AWD, "AWD")
                        .clicked()
                    {
                        self.car.drive_type = LCC_DRIVE_AWD;
                    }
                });
                ui.add(
                    egui::Slider::new(&mut self.car.num_gears, 1..=LCC_MAX_GEARS as i32)
                        .text("num gears"),
                );
                ui.add(
                    egui::Slider::new(&mut self.car.gear, 0..=(self.car.num_gears - 1).max(0))
                        .text("current gear (override)"),
                );
                ui.add(egui::Slider::new(&mut self.car.final_drive, 1.0..=6.0).text("final drive"));
                ui.add(
                    egui::Slider::new(&mut self.car.transmission_efficiency, 0.5..=1.0)
                        .text("transmission efficiency"),
                );
                for i in 0..(self.car.num_gears.max(1) as usize) {
                    ui.add(
                        egui::Slider::new(&mut self.car.gear_ratios[i], 0.5..=5.0)
                            .text(format!("gear {} ratio", i + 1)),
                    );
                }
                ui.label(format!(
                    "Total Ratio (gear {}): {:.2}",
                    (self.car.gear + 1).max(1),
                    self.drivetrain_ratio()
                ));
                ui.label(format!("Engine RPM: {:.0}", self.engine_rpm()));

                ui.separator();
                ui.checkbox(&mut self.auto_clutch_enabled, "Auto clutch (C)");
                if !self.auto_clutch_enabled {
                    ui.add(
                        egui::Slider::new(&mut self.manual_clutch, 0.0..=1.0)
                            .text("manual clutch (0=disengaged, 1=engaged) [Q/E]"),
                    );
                }
                ui.add(
                    egui::Slider::new(&mut self.car.flywheel_inertia, 0.05..=1.0)
                        .text("flywheel inertia (kg·m²)"),
                );
                ui.add(
                    egui::Slider::new(&mut self.car.engine_friction_coeff, 0.0..=1.0)
                        .text("engine friction coeff"),
                );
                ui.add(
                    egui::Slider::new(&mut self.car.clutch_max_torque, 100.0..=3000.0)
                        .text("clutch max torque (Nm)"),
                );
                ui.add(
                    egui::Slider::new(&mut self.car.clutch_stiffness, 0.0..=200.0)
                        .text("clutch stiffness (Nm/(rad/s))"),
                );
                ui.add(
                    egui::Slider::new(&mut self.car.clutch_engagement_rate, 1.0..=30.0)
                        .text("clutch engagement rate (1/s)"),
                );
                if !self.has_clutch_api {
                    ui.colored_label(
                        egui::Color32::YELLOW,
                        "Clutch API not found in library: falling back to struct fields",
                    );
                }
            });

            ui.separator();

            ui.collapsing("Aero", |ui| {
                ui.add(
                    egui::Slider::new(&mut self.car.drag_coeff, 0.15..=1.2).text("drag coeff (Cd)"),
                );
                ui.add(
                    egui::Slider::new(&mut self.car.frontal_area, 0.5..=4.0)
                        .text("frontal area (m²)"),
                );
                ui.add(
                    egui::Slider::new(&mut self.car.air_density, 0.5..=1.5)
                        .text("air density (kg/m³)"),
                );
                ui.add(
                    egui::Slider::new(&mut self.car.downforce_coeff, 0.0..=3.0)
                        .text("downforce coeff (Cl)"),
                );
                ui.add(
                    egui::Slider::new(&mut self.car.downforce_area, 0.0..=4.0)
                        .text("downforce area (m²)"),
                );
            });

            ui.separator();

            ui.collapsing("Brakes & Tires", |ui| {
                ui.add(
                    egui::Slider::new(&mut self.car.brake_bias, 0.0..=1.0)
                        .text("brake bias (front)"),
                );
                ui.add(
                    egui::Slider::new(&mut self.car.brake_torque_max, 1000.0..=30000.0)
                        .text("brake torque max (Nm)"),
                );
                ui.add(
                    egui::Slider::new(&mut self.car.lateral_friction_scale, 0.1..=2.0)
                        .text("lateral friction scale"),
                );
                ui.add(
                    egui::Slider::new(&mut self.car.wheel_mass, 5.0..=40.0).text("wheel mass (kg)"),
                );
                ui.add(
                    egui::Slider::new(&mut self.car.max_steer_angle, 0.2..=1.2)
                        .text("max steer angle (rad)"),
                );
                ui.add(egui::Slider::new(&mut self.car.cg_height, 0.1..=1.0).text("CG height (m)"));
                ui.add(
                    egui::Slider::new(&mut self.car.cg_position, 0.3..=0.7)
                        .text("CG front fraction"),
                );
            });

            ui.separator();

            ui.collapsing("Load Transfer Tuning", |ui| {
                ui.add(
                    egui::Slider::new(&mut self.car.long_transfer_factor, 0.0..=1.5)
                        .text("long transfer factor"),
                );
                ui.add(
                    egui::Slider::new(&mut self.car.lat_transfer_factor, 0.0..=1.5)
                        .text("lat transfer factor"),
                );
                ui.add(
                    egui::Slider::new(&mut self.car.wheel_inertia_bias, 1.0..=10.0)
                        .text("wheel inertia bias"),
                );
            });

            ui.separator();

            ui.collapsing("Wheel Data", |ui| {
                let wheel_names = ["Front Left", "Front Right", "Rear Left", "Rear Right"];
                for (i, wheel_name) in wheel_names.iter().enumerate() {
                    ui.collapsing(*wheel_name, |ui| {
                        let wheel = &self.car.wheels[i];
                        ui.label(format!("Load: {:.0} N", wheel.load));
                        ui.label(format!("Slip Angle: {:.2}°", wheel.slip_angle.to_degrees()));
                        ui.label(format!("Slip Ratio: {:.3}", wheel.slip_ratio));
                        ui.label(format!("Ang. Vel: {:.1} rad/s", wheel.angular_velocity));
                        if self.is_wheel_driven(i) {
                            ui.label("Driven: yes");
                        } else {
                            ui.label("Driven: no");
                        }
                        if let Some(est) = self.estimate_wheel_forces(i) {
                            ui.separator();
                            ui.label(format!("μ_eff: {:.2}", est.mu_eff));
                            ui.label(format!(
                                "Utilization: {:>5.1}%",
                                (est.util * 100.0).min(150.0)
                            ));
                            ui.label(format!("Fx_local: {:>7.0} N", est.fx_local));
                            ui.label(format!("Fy_local: {:>7.0} N", est.fy_local));
                        }
                    });
                }
            });
        });
    }
}

impl eframe::App for CarSimApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.input(|i| {
            if i.key_pressed(egui::Key::Space) {
                self.paused = !self.paused;
            }
            if i.key_pressed(egui::Key::R) {
                self.reset_car_to_defaults();
                self.trail_points.clear();
            }
            if i.key_pressed(egui::Key::C) {
                self.auto_clutch_enabled = !self.auto_clutch_enabled;
            }
        });

        if self.auto_reload && self.last_reload.elapsed() >= self.reload_interval {
            self.try_load_library();
            self.last_reload = Instant::now();
        }

        let now = Instant::now();

        let dt = (now - self.last_update)
            .as_secs_f32()
            .clamp(1.0 / 300.0, 1.0 / 30.0);

        self.update_input(ctx);
        self.smooth_inputs(dt);

        let v0 = [self.car.velocity[0], self.car.velocity[1]];

        if !self.paused && self.lib.is_some() {
            self.update_car_physics(dt);
        }

        let v1 = [self.car.velocity[0], self.car.velocity[1]];
        if dt > 0.0 {
            self.accel_world = [(v1[0] - v0[0]) / dt, (v1[1] - v0[1]) / dt];
            self.last_dt = dt;
        }
        self.last_update = now;

        match self.camera_mode {
            CameraMode::Follow => {
                self.camera_pos = egui::pos2(self.car.position[0], self.car.position[1]);
            }
            CameraMode::Free => {}
        }

        egui::SidePanel::right("side_panel")
            .resizable(true)
            .default_width(360.0)
            .show(ctx, |ui| {
                self.draw_side_panel(ui);
            });

        egui::CentralPanel::default()
            .frame(egui::Frame::dark_canvas(&ctx.style()))
            .show(ctx, |ui| {
                self.draw_simulation_view(ui);
            });

        ctx.request_repaint();
    }
}

fn main() -> Result<(), eframe::Error> {
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 800.0])
            .with_title("Interactive Car Physics Simulation"),
        ..Default::default()
    };

    eframe::run_native(
        "Car Physics Simulation",
        native_options,
        Box::new(|cc| Ok(Box::new(CarSimApp::new(cc)))),
    )
}
