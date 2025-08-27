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
use serde::{Deserialize, Serialize};
use std::{f32::consts::PI, time::Instant};

mod ffi {
    #![allow(non_camel_case_types, non_snake_case, non_upper_case_globals, unused)]
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

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct CarCfg {
    mass: f32,
    inertia: f32, // if <= 0, auto-calc from mass, wheelbase, track_width
    wheelbase: f32,
    track_width: f32,
    cg_height: f32,
    cg_position: f32,
    front_brake_bias: f32,
    max_brake_torque: f32,
    air_density: f32,
    ambient_temp: f32,
    surface_friction: f32,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct EngineCfg {
    max_power: f32,
    max_torque: f32,
    idle_rpm: f32,
    max_rpm: f32,
    redline_rpm: f32,
    inertia: f32,
    friction: f32,
    response_time: f32,
    peak_torque_rpm: f32,
    peak_power_rpm: f32,
    engine_brake_coeff: f32,
    friction_quadratic: f32,
    idle_torque: f32,
    stall_rpm: f32,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct TransCfg {
    num_gears: i32,
    gear_ratios: Vec<f32>, // len == num_gears
    final_drive: f32,
    reverse_ratio: f32,
    efficiency: f32,
    drive_type: String, // "RWD"|"FWD"|"AWD"
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct DiffCfg {
    preload: f32,
    power_factor: f32,
    coast_factor: f32,
    viscous_coefficient: f32,
    bias_limit: f32,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct AeroCfg {
    drag_coefficient: f32,
    frontal_area: f32,
    downforce_coefficient: f32,
    downforce_area: f32,
    aero_balance_front: f32,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct TireCfg {
    radius: f32,
    width: f32,
    aspect_ratio: f32,
    pressure: f32,
    nominal_load: f32,
    peak_friction: f32,
    slip_friction: f32,
    stiffness: f32,
    cornering_stiffness: f32,
    camber_stiffness: f32,
    rolling_resistance: f32,
    relax_length_long: f32,
    relax_length_lat: f32,
    load_sensitivity: f32,
    mu_min: f32,
    mu_max: f32,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct WheelCfg {
    pos_x: f32,
    pos_y: f32,
    rotational_inertia: f32,
    camber_angle: f32,
    surface_friction: f32,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct CarConfig {
    car: CarCfg,
    engine: EngineCfg,
    trans: TransCfg,
    diff: DiffCfg,
    aero: AeroCfg,
    tires: [TireCfg; 4],
    wheels: [WheelCfg; 4],
}

impl CarConfig {
    unsafe fn from_car(car: &ffi::lcc_car_t) -> Self {
        let mut cfg = CarConfig::default();

        cfg.car = CarCfg {
            mass: car.mass,
            inertia: car.inertia,
            wheelbase: car.wheelbase,
            track_width: car.track_width,
            cg_height: car.cg_height,
            cg_position: car.cg_position,
            front_brake_bias: car.front_brake_bias,
            max_brake_torque: car.max_brake_torque,
            air_density: car.air_density,
            ambient_temp: car.ambient_temp,
            surface_friction: car.surface_friction,
        };

        cfg.engine = EngineCfg {
            max_power: car.engine.max_power,
            max_torque: car.engine.max_torque,
            idle_rpm: car.engine.idle_rpm,
            max_rpm: car.engine.max_rpm,
            redline_rpm: car.engine.redline_rpm,
            inertia: car.engine.inertia,
            friction: car.engine.friction,
            response_time: car.engine.response_time,
            peak_torque_rpm: car.engine.peak_torque_rpm,
            peak_power_rpm: car.engine.peak_power_rpm,
            engine_brake_coeff: car.engine.engine_brake_coeff,
            friction_quadratic: car.engine.friction_quadratic,
            idle_torque: car.engine.idle_torque,
            stall_rpm: car.engine.stall_rpm,
        };

        let drive_type = match car.transmission.drive_type {
            x if x == ffi::lcc_drive_t_LCC_DRIVE_RWD => "RWD",
            x if x == ffi::lcc_drive_t_LCC_DRIVE_FWD => "FWD",
            _ => "AWD",
        }
        .to_string();

        let mut gear_ratios = Vec::new();
        let ng = car.transmission.num_gears as usize;
        for i in 0..ng.min(8) {
            gear_ratios.push(car.transmission.gear_ratios[i]);
        }
        cfg.trans = TransCfg {
            num_gears: car.transmission.num_gears,
            gear_ratios,
            final_drive: car.transmission.final_drive,
            reverse_ratio: car.transmission.reverse_ratio,
            efficiency: car.transmission.efficiency,
            drive_type,
        };

        cfg.diff = DiffCfg {
            preload: car.differential.preload,
            power_factor: car.differential.power_factor,
            coast_factor: car.differential.coast_factor,
            viscous_coefficient: car.differential.viscous_coefficient,
            bias_limit: car.differential.bias_limit,
        };

        cfg.aero = AeroCfg {
            drag_coefficient: car.aerodynamics.drag_coefficient,
            frontal_area: car.aerodynamics.frontal_area,
            downforce_coefficient: car.aerodynamics.downforce_coefficient,
            downforce_area: car.aerodynamics.downforce_area,
            aero_balance_front: car.aerodynamics.aero_balance_front,
        };

        for i in 0..4 {
            let tp = &car.tire_params[i];
            cfg.tires[i] = TireCfg {
                radius: tp.radius,
                width: tp.width,
                aspect_ratio: tp.aspect_ratio,
                pressure: tp.pressure,
                nominal_load: tp.nominal_load,
                peak_friction: tp.peak_friction,
                slip_friction: tp.slip_friction,
                stiffness: tp.stiffness,
                cornering_stiffness: tp.cornering_stiffness,
                camber_stiffness: tp.camber_stiffness,
                rolling_resistance: tp.rolling_resistance,
                relax_length_long: tp.relax_length_long,
                relax_length_lat: tp.relax_length_lat,
                load_sensitivity: tp.load_sensitivity,
                mu_min: tp.mu_min,
                mu_max: tp.mu_max,
            };
            let w = &car.wheels[i];
            cfg.wheels[i] = WheelCfg {
                pos_x: w.position[0],
                pos_y: w.position[1],
                rotational_inertia: w.rotational_inertia,
                camber_angle: w.camber_angle,
                surface_friction: w.surface_friction,
            };
        }
        cfg
    }

    unsafe fn apply_to_car(&self, car: &mut ffi::lcc_car_t) {
        // Top-level
        car.mass = self.car.mass;
        car.wheelbase = self.car.wheelbase;
        car.track_width = self.car.track_width;
        car.cg_height = self.car.cg_height;
        car.cg_position = self.car.cg_position;
        car.front_brake_bias = self.car.front_brake_bias.clamp(0.0, 1.0);
        car.max_brake_torque = self.car.max_brake_torque.max(0.0);
        car.air_density = self.car.air_density.max(0.5);
        car.ambient_temp = self.car.ambient_temp;
        car.surface_friction = self.car.surface_friction.max(0.05);

        // Engine
        car.engine.max_power = self.engine.max_power.max(0.0);
        car.engine.max_torque = self.engine.max_torque.max(0.0);
        car.engine.idle_rpm = self.engine.idle_rpm.max(200.0);
        car.engine.max_rpm = self.engine.max_rpm.max(car.engine.idle_rpm);
        car.engine.redline_rpm = self.engine.redline_rpm.max(car.engine.max_rpm);
        car.engine.inertia = self.engine.inertia.max(0.01);
        car.engine.friction = self.engine.friction.max(0.0);
        car.engine.response_time = self.engine.response_time.max(0.01);
        car.engine.peak_torque_rpm = self.engine.peak_torque_rpm.max(500.0);
        car.engine.peak_power_rpm = self.engine.peak_power_rpm.max(500.0);
        car.engine.engine_brake_coeff = self.engine.engine_brake_coeff.max(0.0);
        car.engine.friction_quadratic = self.engine.friction_quadratic.max(0.0);
        car.engine.idle_torque = self.engine.idle_torque.max(0.0);
        car.engine.stall_rpm = self.engine.stall_rpm.max(300.0);
        // Keep current rpm/throttle

        // Transmission
        car.transmission.num_gears = self.trans.num_gears.clamp(0, 8);
        for i in 0..8 {
            let v = *self.trans.gear_ratios.get(i).unwrap_or(&0.0);
            car.transmission.gear_ratios[i] = v;
        }
        car.transmission.final_drive = self.trans.final_drive.max(0.0);
        car.transmission.reverse_ratio = self.trans.reverse_ratio.max(0.0);
        car.transmission.efficiency = self.trans.efficiency.clamp(0.0, 1.0);
        car.transmission.drive_type = match self.trans.drive_type.as_str() {
            "FWD" => ffi::lcc_drive_t_LCC_DRIVE_FWD,
            "AWD" => ffi::lcc_drive_t_LCC_DRIVE_AWD,
            _ => ffi::lcc_drive_t_LCC_DRIVE_RWD,
        };

        // Differential
        car.differential.preload = self.diff.preload.max(0.0);
        car.differential.power_factor = self.diff.power_factor.max(0.0);
        car.differential.coast_factor = self.diff.coast_factor.max(0.0);
        car.differential.viscous_coefficient = self.diff.viscous_coefficient.max(0.0);
        car.differential.bias_limit = self.diff.bias_limit.max(0.0);

        // Aero
        car.aerodynamics.drag_coefficient = self.aero.drag_coefficient.max(0.0);
        car.aerodynamics.frontal_area = self.aero.frontal_area.max(0.1);
        car.aerodynamics.downforce_coefficient = self.aero.downforce_coefficient;
        car.aerodynamics.downforce_area = self.aero.downforce_area.max(0.0);
        car.aerodynamics.aero_balance_front = self.aero.aero_balance_front.clamp(0.0, 1.0);

        // Tires/Wheels
        for i in 0..4 {
            let tp = &mut car.tire_params[i];
            let src = &self.tires[i];
            tp.radius = src.radius.max(0.05);
            tp.width = src.width.max(0.05);
            tp.aspect_ratio = src.aspect_ratio.max(0.1);
            tp.pressure = src.pressure.max(0.0);
            tp.nominal_load = src.nominal_load.max(1.0);
            tp.peak_friction = src.peak_friction.max(0.1);
            tp.slip_friction = src.slip_friction.max(0.1);
            tp.stiffness = src.stiffness.max(0.0);
            tp.cornering_stiffness = src.cornering_stiffness.max(0.0);
            tp.camber_stiffness = src.camber_stiffness.max(0.0);
            tp.rolling_resistance = src.rolling_resistance.max(0.0);
            tp.relax_length_long = src.relax_length_long.max(0.01);
            tp.relax_length_lat = src.relax_length_lat.max(0.01);
            tp.load_sensitivity = src.load_sensitivity.clamp(0.0, 1.0);
            tp.mu_min = src.mu_min.max(0.0);
            tp.mu_max = src.mu_max.max(tp.mu_min);

            let w = &mut car.wheels[i];
            let wc = &self.wheels[i];
            w.position[0] = wc.pos_x;
            w.position[1] = wc.pos_y;
            w.rotational_inertia = wc.rotational_inertia.max(0.01);
            w.camber_angle = wc.camber_angle;
            w.surface_friction = wc.surface_friction.max(0.05);
        }

        // Inertia: use provided if > 0, else recompute
        car.inertia = if self.car.inertia > 0.0 {
            self.car.inertia
        } else {
            car.mass * (car.wheelbase * car.wheelbase + car.track_width * car.track_width) / 12.0
        };

        // Re-init load smoothing based on static distribution at rest
        let w = car.mass * 9.81;
        let wf = w * (1.0 - car.cg_position);
        let wr = w * (car.cg_position);
        car.Fz_smooth[0] = 0.5 * wf;
        car.Fz_smooth[1] = 0.5 * wf;
        car.Fz_smooth[2] = 0.5 * wr;
        car.Fz_smooth[3] = 0.5 * wr;
        for i in 0..4 {
            car.wheels[i].load = car.Fz_smooth[i];
        }
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
    slip_ratio_threshold: f32,
    slip_angle_threshold: f32,
    min_speed_for_skid_kmh: f32,

    last_wheel_world: [Vec2; 4],
    have_last_wheel_world: bool,

    // Electrics / ignition
    key_pos_ui: ffi::lcc_key_state_t, // OFF/RUN selection
    accessory_load_watts: f32,        // user accessories

    // Config editor
    config: CarConfig,
}

#[derive(Clone, Copy)]
struct SkidSegment {
    p0: Vec2,
    p1: Vec2,
    strength: f32, // 0..1
    ttl: f32,
}

struct Plots {
    rpm: Vec<(f32, f32)>,
    speed: Vec<(f32, f32)>,
    wheel_omega: [Vec<(f32, f32)>; 4],
    slip_ratios: [Vec<(f32, f32)>; 4],
    yaw_rate: Vec<(f32, f32)>,
    start_time: Instant,
    max_len: usize,
}

impl Plots {
    fn new() -> Self {
        Self {
            rpm: Vec::new(),
            speed: Vec::new(),
            wheel_omega: [Vec::new(), Vec::new(), Vec::new(), Vec::new()],
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
            self.wheel_omega[i].push((t, inputs[i]));
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
            if self.wheel_omega[i].len() > self.max_len {
                self.wheel_omega[i].remove(0);
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

            key_pos_ui: ffi::lcc_key_state_t_LCC_KEY_OFF,
            accessory_load_watts: 0.0,

            max_trace_points: 4000,
            path_trace: Vec::new(),
            show_trace: true,
            trace_point_spacing_m: 0.25,

            config: CarConfig::from_car(&car),
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

        // refresh config from car
        self.config = CarConfig::from_car(&self.car);
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
        self.input.clutch = smooth_approach(self.input.clutch, target_clutch, 0.05, 0.05);

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
        if input.key_down(egui::Key::Space) {
            ffi::lcc_car_set_ignition(&mut self.car, ffi::lcc_ignition_state_t_LCC_IGNITION_ON);
        } else {
            ffi::lcc_car_set_ignition(&mut self.car, ffi::lcc_ignition_state_t_LCC_IGNITION_OFF);
        }
        ffi::lcc_car_set_keypos(&mut self.car, self.key_pos_ui);

        // Accessory load to DC bus
        ffi::lcc_car_set_accessory_load(&mut self.car, self.accessory_load_watts.max(0.0));
    }

    unsafe fn fixed_update(&mut self, dt: f32) {
        ffi::lcc_car_update(&mut self.car, dt);

        let speed_kmh = ffi::lcc_car_get_speed(&self.car);
        let rpm = ffi::lcc_car_get_engine_rpm(&self.car);

        // Telemetry
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

        // Plots
        self.plots.push(
            rpm as f32,
            speed_kmh as f32,
            [
                self.car.wheels[0].angular_velocity,
                self.car.wheels[1].angular_velocity,
                self.car.wheels[2].angular_velocity,
                self.car.wheels[3].angular_velocity,
            ],
            [
                self.car.wheels[0].slip_ratio as f32,
                self.car.wheels[1].slip_ratio as f32,
                self.car.wheels[2].slip_ratio as f32,
                self.car.wheels[3].slip_ratio as f32,
            ],
            self.car.angular_velocity as f32,
        );

        // Path trace
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
                let long = sr / self.slip_ratio_threshold;
                let lat = sa / self.slip_angle_threshold;
                let severity = long.max(lat).min(3.0);
                let moving = speed_kmh > self.min_speed_for_skid_kmh;
                let loaded = w.load > 50.0;
                if self.show_skids && moving && loaded && severity >= 1.0 {
                    let strength = ((severity - 1.0) / 2.0).clamp(0.2, 1.0);
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

        self.handle_input(ctx);

        if self.paused {
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

        // Skid marks
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

    fn draw_config_editor(&mut self, ui: &mut egui::Ui) {
        ui.heading("Car setup");
        ui.separator();

        ui.horizontal(|ui| {
            if ui.button("Apply to car").clicked() {
                unsafe { self.config.apply_to_car(&mut self.car) }
            }
            if ui.button("Read from car").clicked() {
                unsafe { self.config = CarConfig::from_car(&self.car) }
            }
        });

        ui.separator();

        egui::CollapsingHeader::new("General")
            .default_open(true)
            .show(ui, |ui| {
                egui::Grid::new("car_grid")
                    .num_columns(2)
                    .striped(true)
                    .show(ui, |ui| {
                        ui.label("Mass (kg)");
                        ui.add(egui::DragValue::new(&mut self.config.car.mass).speed(1.0));
                        ui.end_row();
                        ui.label("Inertia (<=0 auto)");
                        ui.add(egui::DragValue::new(&mut self.config.car.inertia).speed(1.0));
                        ui.end_row();
                        ui.label("Wheelbase (m)");
                        if ui
                            .add(egui::DragValue::new(&mut self.config.car.wheelbase).speed(0.01))
                            .changed()
                        {
                            let wb = self.config.car.wheelbase;
                            let tw = self.config.car.track_width;
                            let half_wb = wb * 0.5;
                            let half_tw = tw * 0.5;
                            self.config.wheels[0].pos_x = half_wb;
                            self.config.wheels[1].pos_x = half_wb;
                            self.config.wheels[2].pos_x = -half_wb;
                            self.config.wheels[3].pos_x = -half_wb;
                            self.config.wheels[0].pos_y = half_tw;
                            self.config.wheels[1].pos_y = -half_tw;
                            self.config.wheels[2].pos_y = half_tw;
                            self.config.wheels[3].pos_y = -half_tw;
                        }
                        ui.end_row();
                        ui.label("Track width (m)");
                        if ui
                            .add(egui::DragValue::new(&mut self.config.car.track_width).speed(0.01))
                            .changed()
                        {
                            let wb = self.config.car.wheelbase;
                            let tw = self.config.car.track_width;
                            let half_wb = wb * 0.5;
                            let half_tw = tw * 0.5;
                            self.config.wheels[0].pos_x = half_wb;
                            self.config.wheels[1].pos_x = half_wb;
                            self.config.wheels[2].pos_x = -half_wb;
                            self.config.wheels[3].pos_x = -half_wb;
                            self.config.wheels[0].pos_y = half_tw;
                            self.config.wheels[1].pos_y = -half_tw;
                            self.config.wheels[2].pos_y = half_tw;
                            self.config.wheels[3].pos_y = -half_tw;
                        }
                        ui.end_row();
                        ui.label("CG height (m)");
                        ui.add(egui::DragValue::new(&mut self.config.car.cg_height).speed(0.005));
                        ui.end_row();
                        ui.label("CG position (0..1)");
                        ui.add(egui::DragValue::new(&mut self.config.car.cg_position).speed(0.005));
                        ui.end_row();
                        ui.label("Front brake bias");
                        ui.add(
                            egui::DragValue::new(&mut self.config.car.front_brake_bias)
                                .speed(0.005),
                        );
                        ui.end_row();
                        ui.label("Max brake torque (Nm)");
                        ui.add(
                            egui::DragValue::new(&mut self.config.car.max_brake_torque).speed(10.0),
                        );
                        ui.end_row();
                        ui.label("Air density");
                        ui.add(egui::DragValue::new(&mut self.config.car.air_density).speed(0.01));
                        ui.end_row();
                        ui.label("Ambient temp (C)");
                        ui.add(egui::DragValue::new(&mut self.config.car.ambient_temp).speed(0.5));
                        ui.end_row();
                        ui.label("Surface friction");
                        ui.add(
                            egui::DragValue::new(&mut self.config.car.surface_friction).speed(0.01),
                        );
                        ui.end_row();
                    });
            });

        egui::CollapsingHeader::new("Engine")
            .default_open(false)
            .show(ui, |ui| {
                egui::Grid::new("engine_grid")
                    .num_columns(2)
                    .striped(true)
                    .show(ui, |ui| {
                        macro_rules! row {
                            ($label:expr, $field:expr, $spd:expr) => {{
                                ui.label($label);
                                ui.add(egui::DragValue::new(&mut $field).speed($spd));
                                ui.end_row();
                            }};
                        }
                        row!("Max power (W)", self.config.engine.max_power, 50.0);
                        row!("Max torque (Nm)", self.config.engine.max_torque, 1.0);
                        row!("Idle RPM", self.config.engine.idle_rpm, 10.0);
                        row!("Max RPM", self.config.engine.max_rpm, 10.0);
                        row!("Redline RPM", self.config.engine.redline_rpm, 10.0);
                        row!("Inertia", self.config.engine.inertia, 0.005);
                        row!("Friction", self.config.engine.friction, 0.001);
                        row!("Response time (s)", self.config.engine.response_time, 0.005);
                        row!("Peak torque RPM", self.config.engine.peak_torque_rpm, 10.0);
                        row!("Peak power RPM", self.config.engine.peak_power_rpm, 10.0);
                        row!(
                            "Engine brake coeff",
                            self.config.engine.engine_brake_coeff,
                            0.001
                        );
                        row!(
                            "Friction quadratic",
                            self.config.engine.friction_quadratic,
                            1e-5
                        );
                        row!("Idle torque", self.config.engine.idle_torque, 0.5);
                        row!("Stall RPM", self.config.engine.stall_rpm, 10.0);
                    });
            });

        egui::CollapsingHeader::new("Transmission")
            .default_open(false)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Drive type");
                    egui::ComboBox::from_id_salt("drive_type")
                        .selected_text(&self.config.trans.drive_type)
                        .show_ui(ui, |ui| {
                            ui.selectable_value(
                                &mut self.config.trans.drive_type,
                                "RWD".into(),
                                "RWD",
                            );
                            ui.selectable_value(
                                &mut self.config.trans.drive_type,
                                "FWD".into(),
                                "FWD",
                            );
                            ui.selectable_value(
                                &mut self.config.trans.drive_type,
                                "AWD".into(),
                                "AWD",
                            );
                        });
                });
                ui.add(
                    egui::Slider::new(&mut self.config.trans.num_gears, 0..=8).text("Num gears"),
                );
                while self.config.trans.gear_ratios.len() < self.config.trans.num_gears as usize {
                    self.config.trans.gear_ratios.push(1.0);
                }
                while self.config.trans.gear_ratios.len() > self.config.trans.num_gears as usize {
                    self.config.trans.gear_ratios.pop();
                }
                for i in 0..self.config.trans.gear_ratios.len() {
                    ui.horizontal(|ui| {
                        ui.label(format!("Gear {} ratio", i + 1));
                        ui.add(
                            egui::DragValue::new(&mut self.config.trans.gear_ratios[i]).speed(0.01),
                        );
                    });
                }
                ui.separator();
                egui::Grid::new("trans_grid")
                    .num_columns(2)
                    .striped(true)
                    .show(ui, |ui| {
                        ui.label("Final drive");
                        ui.add(
                            egui::DragValue::new(&mut self.config.trans.final_drive).speed(0.01),
                        );
                        ui.end_row();
                        ui.label("Reverse ratio");
                        ui.add(
                            egui::DragValue::new(&mut self.config.trans.reverse_ratio).speed(0.01),
                        );
                        ui.end_row();
                        ui.label("Efficiency");
                        ui.add(egui::DragValue::new(&mut self.config.trans.efficiency).speed(0.01));
                        ui.end_row();
                    });
            });

        egui::CollapsingHeader::new("Differential")
            .default_open(false)
            .show(ui, |ui| {
                egui::Grid::new("diff_grid")
                    .num_columns(2)
                    .striped(true)
                    .show(ui, |ui| {
                        ui.label("Preload (Nm)");
                        ui.add(egui::DragValue::new(&mut self.config.diff.preload).speed(1.0));
                        ui.end_row();
                        ui.label("Power factor");
                        ui.add(
                            egui::DragValue::new(&mut self.config.diff.power_factor).speed(0.01),
                        );
                        ui.end_row();
                        ui.label("Coast factor");
                        ui.add(
                            egui::DragValue::new(&mut self.config.diff.coast_factor).speed(0.01),
                        );
                        ui.end_row();
                        ui.label("Viscous coeff");
                        ui.add(
                            egui::DragValue::new(&mut self.config.diff.viscous_coefficient)
                                .speed(0.1),
                        );
                        ui.end_row();
                        ui.label("Bias limit (Nm)");
                        ui.add(egui::DragValue::new(&mut self.config.diff.bias_limit).speed(1.0));
                        ui.end_row();
                    });
            });

        egui::CollapsingHeader::new("Aerodynamics")
            .default_open(false)
            .show(ui, |ui| {
                egui::Grid::new("aero_grid")
                    .num_columns(2)
                    .striped(true)
                    .show(ui, |ui| {
                        ui.label("Cd");
                        ui.add(
                            egui::DragValue::new(&mut self.config.aero.drag_coefficient)
                                .speed(0.005),
                        );
                        ui.end_row();
                        ui.label("Frontal area (m^2)");
                        ui.add(
                            egui::DragValue::new(&mut self.config.aero.frontal_area).speed(0.01),
                        );
                        ui.end_row();
                        ui.label("Cl (downforce)");
                        ui.add(
                            egui::DragValue::new(&mut self.config.aero.downforce_coefficient)
                                .speed(0.01),
                        );
                        ui.end_row();
                        ui.label("Downforce area (m^2)");
                        ui.add(
                            egui::DragValue::new(&mut self.config.aero.downforce_area).speed(0.01),
                        );
                        ui.end_row();
                        ui.label("Aero balance front (0..1)");
                        ui.add(
                            egui::DragValue::new(&mut self.config.aero.aero_balance_front)
                                .speed(0.01),
                        );
                        ui.end_row();
                    });
            });

        egui::CollapsingHeader::new("Tires")
            .default_open(false)
            .show(ui, |ui| {
                for i in 0..4 {
                    egui::CollapsingHeader::new(format!("Tire {}", i)).show(ui, |ui| {
                        let t = &mut self.config.tires[i];
                        egui::Grid::new(format!("tire_grid_{i}"))
                            .num_columns(2)
                            .striped(true)
                            .show(ui, |ui| {
                                macro_rules! g {
                                    ($a:expr,$b:expr,$s:expr) => {{
                                        ui.label($a);
                                        ui.add(egui::DragValue::new(&mut $b).speed($s));
                                        ui.end_row();
                                    }};
                                }
                                g!("Radius (m)", t.radius, 0.001);
                                g!("Width (m)", t.width, 0.001);
                                g!("Aspect ratio", t.aspect_ratio, 0.005);
                                g!("Pressure (kPa)", t.pressure, 0.5);
                                g!("Nominal load (N)", t.nominal_load, 5.0);
                                g!("Peak friction", t.peak_friction, 0.005);
                                g!("Slip friction", t.slip_friction, 0.005);
                                g!("Long stiffness", t.stiffness, 10.0);
                                g!("Cornering stiffness", t.cornering_stiffness, 10.0);
                                g!("Camber stiffness", t.camber_stiffness, 10.0);
                                g!("Rolling resistance", t.rolling_resistance, 0.0001);
                                g!("Relax length long (m)", t.relax_length_long, 0.005);
                                g!("Relax length lat (m)", t.relax_length_lat, 0.005);
                                g!("Load sensitivity", t.load_sensitivity, 0.005);
                                g!("mu_min", t.mu_min, 0.005);
                                g!("mu_max", t.mu_max, 0.005);
                            });
                    });
                }
            });

        egui::CollapsingHeader::new("Wheels")
            .default_open(false)
            .show(ui, |ui| {
                for i in 0..4 {
                    egui::CollapsingHeader::new(format!("Wheel {}", i)).show(ui, |ui| {
                        let w = &mut self.config.wheels[i];
                        egui::Grid::new(format!("wheel_grid_{i}"))
                            .num_columns(2)
                            .striped(true)
                            .show(ui, |ui| {
                                ui.label("Pos X (m)");
                                ui.add(egui::DragValue::new(&mut w.pos_x).speed(0.005));
                                ui.end_row();
                                ui.label("Pos Y (m)");
                                ui.add(egui::DragValue::new(&mut w.pos_y).speed(0.005));
                                ui.end_row();
                                ui.label("Rot inertia");
                                ui.add(
                                    egui::DragValue::new(&mut w.rotational_inertia).speed(0.005),
                                );
                                ui.end_row();
                                ui.label("Camber (rad)");
                                ui.add(egui::DragValue::new(&mut w.camber_angle).speed(0.001));
                                ui.end_row();
                                ui.label("Surface μ");
                                ui.add(egui::DragValue::new(&mut w.surface_friction).speed(0.005));
                                ui.end_row();
                            });
                    });
                }
            });
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
    ui.monospace("a/d = steer\nw/s = shift\nh = clutch\nj = brake\nk = throttle\nSpace = hold START\n(Ignition OFF/RUN via UI)");
});
        ui.separator();
        ui.collapsing("Ignition & Electrics", |ui| {
            // Key selector (OFF/RUN). START is momentary via Space.
            ui.horizontal(|ui| {
                ui.label("Key:");
                let mut choose = |label: &str, v| {
                    ui.selectable_value(&mut self.key_pos_ui, v, label);
                };
                choose("OFF", ffi::lcc_key_state_t_LCC_KEY_OFF);
                choose("RUN", ffi::lcc_key_state_t_LCC_KEY_RUN);
                ui.label("(hold Space = START)");
            });

            // Accessory load slider (applies every frame)
            ui.add(
                egui::Slider::new(&mut self.accessory_load_watts, 0.0..=700.0)
                    .text("Accessory load (W)"),
            );

            unsafe {
                let batt_v = ffi::lcc_car_get_battery_voltage(&self.car);
                let soc = ffi::lcc_car_get_battery_soc(&self.car);
                ui.label(format!(
                    "Battery: {:4.2} V  |  SOC: {:3.0}%",
                    batt_v,
                    soc * 100.0
                ));
                ui.label(format!(
                    "Electrical load: {:5.0} W",
                    self.car.electrical_load_W
                ));
                ui.label(format!(
                    "Alternator out: {:5.0} W",
                    self.car.alternator_out_W
                ));

                let eng_state = match self.car.engine.state {
                    0 => "OFF",
                    1 => "CRANKING",
                    2 => "RUNNING",
                    _ => "?",
                };
                let fuelcut = self.car.engine.fuel_cut_active != 0;
                let sparkcut = self.car.engine.spark_cut_active != 0;
                ui.label(format!(
                    "Engine: {} {} {}",
                    eng_state,
                    if fuelcut { "[FuelCut]" } else { " " },
                    if sparkcut { "[SparkCut]" } else { " " },
                ));
            }
        });

        ui.separator();
        ui.collapsing("Fuel", |ui| unsafe {
            let cap = ffi::lcc_car_get_fuel_capacity_L(&self.car);
            let mut lvl = ffi::lcc_car_get_fuel_level_L(&self.car);
            ui.label(format!("Fuel: {:5.1} / {:5.1} L", lvl, cap));
            ui.horizontal(|ui| {
                if ui.button("Refill full").clicked() {
                    ffi::lcc_car_set_fuel_level(&mut self.car, cap);
                }
                if ui.button("+5 L").clicked() {
                    ffi::lcc_car_refuel(&mut self.car, 5.0);
                }
            });
            if ui
                .add(egui::Slider::new(&mut lvl, 0.0..=cap).text("Set level (L)"))
                .changed()
            {
                ffi::lcc_car_set_fuel_level(&mut self.car, lvl);
            }
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

        ui.separator();
        ui.collapsing("Setup / Config", |ui| {
            self.draw_config_editor(ui);
        });
    }

    fn draw_bottom_panel(&mut self, ui: &mut egui::Ui) {
        StripBuilder::new(ui)
            .sizes(Size::exact(200.0), 1)
            .vertical(|mut strip| {
                strip.cell(|ui| {
                    StripBuilder::new(ui)
                        .sizes(Size::relative(0.5), 2)
                        .horizontal(|mut hstrip| {
                            // Left: RPM/Speed
                            hstrip.cell(|ui| {
                                let col_h = ui.available_height();
                                let half = (col_h - 6.0) * 0.5;

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

                            // Right: Inputs + Slip ratios
                            hstrip.cell(|ui| {
                                let col_h = ui.available_height();
                                let half = (col_h - 6.0) * 0.5;

                                Plot::new("inputs_plot")
                                    .legend(Legend::default())
                                    .height(half)
                                    .show(ui, |pui| {
                                        let colors = [
                                            Color32::from_rgb(100, 220, 100),
                                            Color32::from_rgb(230, 100, 100),
                                            Color32::from_rgb(100, 160, 230),
                                            Color32::from_rgb(240, 200, 80),
                                        ];
                                        for i in 0..4 {
                                            let pts = PlotPoints::from_iter(
                                                self.plots.wheel_omega[i]
                                                    .iter()
                                                    .map(|&(x, y)| [x as f64, y as f64]),
                                            );
                                            pui.line(
                                                Line::new(format!("W{i} omega"), pts)
                                                    .color(colors[i]),
                                            );
                                        }
                                    });

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
                                        pui.hline(HLine::new("", 0.0).color(Color32::GRAY));
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

            let base2 = Pos2::new(origin.x + 160.0, origin.y + 50.0);

            // Fuel bar: L / capacity
            let cap = ffi::lcc_car_get_fuel_capacity_L(&self.car);
            let lvl = ffi::lcc_car_get_fuel_level_L(&self.car);
            draw_bar(
                &painter,
                Pos2::new(base2.x, base2.y + 24.0),
                "Fuel",
                (lvl / cap).clamp(0.0, 1.0),
                Color32::from_rgb(200, 180, 60),
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
            .default_width(320.0)
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

    painter.circle(
        origin,
        radius,
        Color32::from_black_alpha(10),
        Stroke::new(2.0, bg),
    );

    let start_angle = -0.75 * PI;
    let end_angle = 0.75 * PI;

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

    arc(0.0, 0.7, Color32::from_rgb(90, 200, 90), 5.0);
    arc(0.7, 0.9, Color32::from_rgb(240, 200, 80), 5.0);
    arc(0.9, 1.0, Color32::from_rgb(220, 70, 70), 5.0);

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

    let t = (rpm / redline).clamp(0.0, 1.0);
    let needle_angle = start_angle + (end_angle - start_angle) * t;
    let needle_len = radius * 0.9;
    let tip = Pos2::new(
        origin.x + needle_len * needle_angle.cos(),
        origin.y + needle_len * needle_angle.sin(),
    );
    painter.line_segment([origin, tip], Stroke::new(3.0, Color32::LIGHT_RED));

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
        egui::StrokeKind::Inside,
    );
    let fill = Rect::from_min_size(pos, vec2(w * value01.clamp(0.0, 1.0), h));
    painter.rect(
        fill,
        3.0,
        color.gamma_multiply(0.5),
        Stroke::NONE,
        egui::StrokeKind::Inside,
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
        "{label}  SR {:5.2}  SA {:5.2}  w {:8.2} Load {:5.0}N  T {:4.1}°C  μ {:4.2}",
        w.slip_ratio, w.slip_angle, w.angular_velocity, w.load, w.temperature, w.surface_friction
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
