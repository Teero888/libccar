#![allow(
    clippy::missing_safety_doc,
    non_camel_case_types,
    non_snake_case,
    non_upper_case_globals
)]

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
use std::{f32::consts::PI, ffi::CStr, mem, time::Instant};

mod ffi {
    #![allow(non_camel_case_types, non_snake_case, non_upper_case_globals, unused)]
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}

use crate::ffi::{
    lcc_abs_mode_e_LCC_ABS_OFF as LCC_ABS_OFF, lcc_abs_mode_e_LCC_ABS_ON as LCC_ABS_ON,
    lcc_drivetrain_layout_e_LCC_LAYOUT_AWD as LCC_LAYOUT_AWD,
    lcc_drivetrain_layout_e_LCC_LAYOUT_FWD as LCC_LAYOUT_FWD,
    lcc_drivetrain_layout_e_LCC_LAYOUT_RWD as LCC_LAYOUT_RWD,
    lcc_esc_mode_e_LCC_ESC_OFF as LCC_ESC_OFF, lcc_esc_mode_e_LCC_ESC_ON as LCC_ESC_ON,
    lcc_tc_mode_e_LCC_TC_OFF as LCC_TC_OFF, lcc_tc_mode_e_LCC_TC_ON as LCC_TC_ON,
    lcc_transmission_type_e_LCC_TRANS_DCT as LCC_TRANS_DCT,
    lcc_transmission_type_e_LCC_TRANS_MANUAL as LCC_TRANS_MANUAL,
};

static VERSION: Lazy<String> = Lazy::new(|| unsafe {
    let cstr = ffi::lcc_version_string();
    if cstr.is_null() {
        "unknown".to_string()
    } else {
        CStr::from_ptr(cstr).to_string_lossy().to_string()
    }
});

#[derive(Clone, Copy, PartialEq)]
enum ViewPreset {
    Economy,
    Midsize,
    Sports,
    Supercar,
    Hypercar,
    Drift,
}
impl ViewPreset {
    fn all() -> &'static [ViewPreset] {
        &[
            ViewPreset::Economy,
            ViewPreset::Midsize,
            ViewPreset::Sports,
            ViewPreset::Supercar,
            ViewPreset::Hypercar,
            ViewPreset::Drift,
        ]
    }
    fn as_label(self) -> &'static str {
        match self {
            ViewPreset::Economy => "Economy",
            ViewPreset::Midsize => "Midsize",
            ViewPreset::Sports => "Sports",
            ViewPreset::Supercar => "Supercar",
            ViewPreset::Hypercar => "Hypercar",
            ViewPreset::Drift => "Drift",
        }
    }
}

#[derive(Serialize, Clone)]
struct WheelSample {
    // kinematics
    slip_ratio: f32, // [-] (as reported)
    slip_angle: f32, // [rad] (as reported)
    omega: f32,      // [rad/s]
    // forces
    f_long: f32, // [N] tire_force_long_n
    f_lat: f32,  // [N] tire_force_lat_n
    load: f32,   // [N] normal_force_n
    // torques
    t_drive: f32, // [Nm]
    t_brake: f32, // [Nm]
    // tire/env
    temp: f32,   // [C]
    est_mu: f32, // estimated mu (see logic below)
    fcap: f32,   // mu*Fz [N]
}
#[derive(Serialize, Clone)]
struct TelemetrySample {
    // time & pose
    t: f32,
    dt: f32,
    pos_x: f32,
    pos_y: f32,
    vel_x: f32,
    vel_y: f32,
    speed_mps: f32,
    speed_kmh: f32,
    yaw: f32,
    yaw_rate: f32,

    // engine / trans / inputs
    engine_rpm: f32,

    gear: i32,
    throttle: f32,  // command (your input/control)
    brake: f32,     // command
    steering: f32,  // command
    clutch: f32,    // command
    handbrake: f32, // command

    // aids state at time of sample
    abs_on: bool,
    tc_on: bool,
    esc_on: bool,

    // low-speed “glue” flag (sim < 0.25 m/s)
    glue: bool,

    wheels: [WheelSample; 4],
}

struct TelemetryRec {
    recording: bool,
    data: Vec<TelemetrySample>,
    max_points: usize,
}
impl TelemetryRec {
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
            "dt",
            "pos_x",
            "pos_y",
            "vel_x",
            "vel_y",
            "speed_mps",
            "speed_kmh",
            "yaw",
            "yaw_rate",
            "engine_rpm",
            "gear",
            "throttle_cmd",
            "brake_cmd",
            "steering_cmd",
            "clutch_cmd",
            "handbrake_cmd",
            "abs_on",
            "tc_on",
            "esc_on",
            "glue",
            // wheel 0
            "w0_slip_ratio",
            "w0_slip_angle",
            "w0_omega",
            "w0_Fx",
            "w0_Fy",
            "w0_Fz",
            "w0_Tdrv",
            "w0_Tbr",
            "w0_temp",
            "w0_mu",
            "w0_Fcap",
            // wheel 1
            "w1_slip_ratio",
            "w1_slip_angle",
            "w1_omega",
            "w1_Fx",
            "w1_Fy",
            "w1_Fz",
            "w1_Tdrv",
            "w1_Tbr",
            "w1_temp",
            "w1_mu",
            "w1_Fcap",
            // wheel 2
            "w2_slip_ratio",
            "w2_slip_angle",
            "w2_omega",
            "w2_Fx",
            "w2_Fy",
            "w2_Fz",
            "w2_Tdrv",
            "w2_Tbr",
            "w2_temp",
            "w2_mu",
            "w2_Fcap",
            // wheel 3
            "w3_slip_ratio",
            "w3_slip_angle",
            "w3_omega",
            "w3_Fx",
            "w3_Fy",
            "w3_Fz",
            "w3_Tdrv",
            "w3_Tbr",
            "w3_temp",
            "w3_mu",
            "w3_Fcap",
        ])?;

        for s in &self.data {
            let rec = vec![
                s.t,
                s.dt,
                s.pos_x,
                s.pos_y,
                s.vel_x,
                s.vel_y,
                s.speed_mps,
                s.speed_kmh,
                s.yaw,
                s.yaw_rate,
                s.engine_rpm,
                s.gear as f32,
                s.throttle,
                s.brake,
                s.steering,
                s.clutch,
                s.handbrake,
                (s.abs_on as i32) as f32,
                (s.tc_on as i32) as f32,
                (s.esc_on as i32) as f32,
                (s.glue as i32) as f32,
                // wheel 0
                s.wheels[0].slip_ratio,
                s.wheels[0].slip_angle,
                s.wheels[0].omega,
                s.wheels[0].f_long,
                s.wheels[0].f_lat,
                s.wheels[0].load,
                s.wheels[0].t_drive,
                s.wheels[0].t_brake,
                s.wheels[0].temp,
                s.wheels[0].est_mu,
                s.wheels[0].fcap,
                // wheel 1
                s.wheels[1].slip_ratio,
                s.wheels[1].slip_angle,
                s.wheels[1].omega,
                s.wheels[1].f_long,
                s.wheels[1].f_lat,
                s.wheels[1].load,
                s.wheels[1].t_drive,
                s.wheels[1].t_brake,
                s.wheels[1].temp,
                s.wheels[1].est_mu,
                s.wheels[1].fcap,
                // wheel 2
                s.wheels[2].slip_ratio,
                s.wheels[2].slip_angle,
                s.wheels[2].omega,
                s.wheels[2].f_long,
                s.wheels[2].f_lat,
                s.wheels[2].load,
                s.wheels[2].t_drive,
                s.wheels[2].t_brake,
                s.wheels[2].temp,
                s.wheels[2].est_mu,
                s.wheels[2].fcap,
                // wheel 3
                s.wheels[3].slip_ratio,
                s.wheels[3].slip_angle,
                s.wheels[3].omega,
                s.wheels[3].f_long,
                s.wheels[3].f_lat,
                s.wheels[3].load,
                s.wheels[3].t_drive,
                s.wheels[3].t_brake,
                s.wheels[3].temp,
                s.wheels[3].est_mu,
                s.wheels[3].fcap,
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
    mass_kg: f32,
    wheelbase: f32,
    track_front: f32,
    track_rear: f32,
    cg_height: f32,
    final_drive: f32,
    layout: i32, // LCC_LAYOUT_FWD/RWD/AWD/4X4
    // aero
    cd: f32,
    frontal_area: f32,
    cl_front: f32,
    cl_rear: f32,
    // steering
    max_steer_deg: f32,
    ackermann: f32,
    // tire mu scale
    global_mu: f32,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct TireCfg {
    mu_nominal: f32,
    pressure_kpa: f32,
    rolling_resistance: f32,
    load_sens: f32,
    radius_m: f32,
    width_m: f32,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct WheelCfg {
    pos_x: f32,
    pos_y: f32,
    steerable: bool,
    driven: bool,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct CarConfig {
    car: CarCfg,
    tires: [TireCfg; 4],
    wheels: [WheelCfg; 4],
}

impl CarConfig {
    fn from_desc(desc: &ffi::lcc_car_desc_t) -> Self {
        let mut cfg = CarConfig::default();
        cfg.car = CarCfg {
            mass_kg: desc.chassis.mass_kg,
            wheelbase: desc.chassis.wheelbase_m,
            track_front: desc.chassis.track_front_m,
            track_rear: desc.chassis.track_rear_m,
            cg_height: desc.chassis.cg_height_m,
            final_drive: desc.transmission.final_drive_ratio,
            layout: desc.driveline.layout as i32,
            cd: desc.aero.drag_coefficient,
            frontal_area: desc.aero.frontal_area_m2,
            cl_front: desc.aero.lift_coefficient_front,
            cl_rear: desc.aero.lift_coefficient_rear,
            max_steer_deg: desc.steering.max_steer_deg,
            ackermann: desc.steering.ackermann_factor,
            global_mu: 1.0, // environment global friction scale (edited via env)
        };
        for i in 0..4usize {
            cfg.tires[i] = TireCfg {
                mu_nominal: desc.tires[i].mu_nominal,
                pressure_kpa: desc.tires[i].pressure_kpa,
                rolling_resistance: desc.tires[i].rolling_resistance,
                load_sens: desc.tires[i].load_sensitivity,
                radius_m: desc.wheels[i].radius_m,
                width_m: desc.wheels[i].width_m,
            };
            cfg.wheels[i] = WheelCfg {
                pos_x: desc.wheels[i].position_local[0],
                pos_y: desc.wheels[i].position_local[1],
                steerable: desc.wheels[i].steerable != 0,
                driven: desc.wheels[i].driven != 0,
            };
        }
        cfg
    }
    fn apply_runtime(
        &self,
        car: *mut ffi::lcc_car_t,
        desc: &mut ffi::lcc_car_desc_t,
        env: &mut ffi::lcc_environment_t,
    ) {
        unsafe {
            // environment friction scale
            env.global_friction_scale = self.car.global_mu;
            ffi::lcc_car_set_environment(car, env as *const _);

            // tires/wheels basic params per wheel
            for i in 0..4 {
                // wheel radius/width is in wheel_desc, but at runtime we'll only change tires + brakes/susp later if needed
                let mut t = desc.tires[i];
                t.mu_nominal = self.tires[i].mu_nominal;
                t.pressure_kpa = self.tires[i].pressure_kpa;
                t.rolling_resistance = self.tires[i].rolling_resistance;
                t.load_sensitivity = self.tires[i].load_sens;
                let _ = ffi::lcc_car_set_tire_params(car, i as i32, &t as *const _);

                let mut w = desc.wheels[i];
                w.radius_m = self.tires[i].radius_m.max(0.05);
                w.width_m = self.tires[i].width_m.max(0.05);
                // No direct setter for wheel dims, but we can recreate for structural changes. For runtime, skip.
                desc.tires[i] = t;
                desc.wheels[i] = w;
            }

            // steering params
            desc.steering.max_steer_deg = self.car.max_steer_deg;
            desc.steering.ackermann_factor = self.car.ackermann;
            let _ = ffi::lcc_car_set_steering_params(car, &desc.steering as *const _);

            // aero
            desc.aero.drag_coefficient = self.car.cd;
            desc.aero.frontal_area_m2 = self.car.frontal_area;
            desc.aero.lift_coefficient_front = self.car.cl_front;
            desc.aero.lift_coefficient_rear = self.car.cl_rear;
            // No dedicated runtime setter; used internally in step from desc snapshot.

            // final drive (runtime)
            let ratios = desc.transmission.gear_ratios.as_ptr();
            let count = desc.transmission.gear_count;
            let _ = ffi::lcc_car_set_gear_ratios(car, ratios, count, self.car.final_drive);
            desc.transmission.final_drive_ratio = self.car.final_drive;

            // driveline layout affects driven flag; need recreate for correct wheel driven flags.
        }
    }
}

struct InputState {
    throttle: f32,
    brake: f32,
    steer: f32,
    clutch: f32,
    handbrake: f32,
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
            handbrake: 0.0,
            last_w: false,
            last_s: false,
        }
    }
}

#[derive(Clone, Copy)]
struct SkidSegment {
    p0: Vec2,
    p1: Vec2,
    strength: f32,
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
    fn push(&mut self, rpm: f32, speed_kmh: f32, omega: [f32; 4], slip: [f32; 4], yaw_rate: f32) {
        let t = self.t();
        self.rpm.push((t, rpm));
        self.speed.push((t, speed_kmh));
        self.yaw_rate.push((t, yaw_rate));
        for i in 0..4 {
            self.wheel_omega[i].push((t, omega[i]));
            self.slip_ratios[i].push((t, slip[i]));
            if self.wheel_omega[i].len() > self.max_len {
                self.wheel_omega[i].remove(0);
            }
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
    }
}

struct App {
    car: *mut ffi::lcc_car_t,
    desc: ffi::lcc_car_desc_t,
    env: ffi::lcc_environment_t,
    controls: ffi::lcc_controls_t,

    preset: ViewPreset,
    last_tick: Instant,
    accumulator: f32,
    fixed_dt: f32,
    paused: bool,
    sim_speed: f32,
    input: InputState,
    telemetry_rec: TelemetryRec,
    plots: Plots,
    zoom: f32,
    follow_camera: bool,
    camera_pos: Vec2,

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

    // driver aids toggles
    abs_on: bool,
    tc_on: bool,
    esc_on: bool,

    // config editor
    config: CarConfig,
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
    unsafe fn car_ref(&self) -> &ffi::lcc_car_t {
        &*self.car
    }
}

impl App {
    unsafe fn make_desc_from_preset(preset: ViewPreset) -> ffi::lcc_car_desc_t {
        let mut d: ffi::lcc_car_desc_t = mem::zeroed();
        ffi::lcc_car_desc_init_defaults(&mut d);

        fn make_curve<const N: usize>(pts: [(f32, f32); N]) -> ffi::lcc_curve1d_t {
            let mut v = Vec::<ffi::lcc_curve1d_point_t>::with_capacity(N);
            for (x, y) in pts {
                v.push(ffi::lcc_curve1d_point_t { x, y });
            }
            let b = Box::leak(v.into_boxed_slice());
            ffi::lcc_curve1d_t {
                points: b.as_ptr(),
                count: N as i32,
            }
        }

        const WEAK_WOT: [(f32, f32); 11] = [
            (800.0, 120.0),
            (1200.0, 150.0),
            (1800.0, 175.0),
            (2400.0, 195.0),
            (3000.0, 210.0),
            (3600.0, 220.0),
            (4200.0, 225.0),
            (4800.0, 225.0),
            (5400.0, 215.0),
            (6000.0, 200.0),
            (6800.0, 180.0),
        ];
        const WEAK_FRIC: [(f32, f32); 8] = [
            (0.0, 8.0),
            (1000.0, 10.0),
            (2000.0, 14.0),
            (3000.0, 18.0),
            (4000.0, 24.0),
            (5000.0, 32.0),
            (6000.0, 42.0),
            (7000.0, 54.0),
        ];
        const DRIFT_WOT: [(f32, f32); 7] = [
            (1000.0, 400.0),
            (2000.0, 550.0),
            (3000.0, 650.0),
            (4000.0, 660.0),
            (5000.0, 640.0),
            (6000.0, 580.0),
            (7000.0, 500.0),
        ];
        const DRIFT_FRIC: [(f32, f32); 8] = [
            (0.0, 15.0),
            (1000.0, 20.0),
            (2000.0, 25.0),
            (3000.0, 32.0),
            (4000.0, 40.0),
            (5000.0, 50.0),
            (6000.0, 65.0),
            (7000.0, 80.0),
        ];

        match preset {
            ViewPreset::Economy => {
                d.chassis.mass_kg = 1100.0;
                d.driveline.layout = LCC_LAYOUT_FWD;
                d.engine.redline_rpm = 6200.0;
                d.tires[0].mu_nominal = 0.95;
                d.tires[1].mu_nominal = 0.95;
                d.tires[2].mu_nominal = 0.95;
                d.tires[3].mu_nominal = 0.95;
            }
            ViewPreset::Midsize => {
                d.chassis.mass_kg = 1400.0;
                d.driveline.layout = LCC_LAYOUT_FWD;
                d.engine.redline_rpm = 6500.0;
                for i in 0..4 {
                    d.tires[i].mu_nominal = 1.05;
                }
            }
            ViewPreset::Sports => {
                d.chassis.mass_kg = 1350.0;
                d.driveline.layout = LCC_LAYOUT_RWD;
                d.engine.redline_rpm = 7000.0;
                for i in 0..4 {
                    d.tires[i].mu_nominal = 1.15;
                    d.tires[i].pressure_kpa = 230.0;
                }
                d.transmission.type_ = LCC_TRANS_MANUAL;
                d.transmission.final_drive_ratio = 4.1;
            }
            ViewPreset::Supercar => {
                d.chassis.mass_kg = 1450.0;
                d.driveline.layout = LCC_LAYOUT_AWD;
                d.engine.redline_rpm = 8000.0;
                for i in 0..4 {
                    d.tires[i].mu_nominal = 1.35;
                    d.tires[i].pressure_kpa = 240.0;
                }
                d.aero.lift_coefficient_front = -0.3;
                d.aero.lift_coefficient_rear = -0.6;
                d.transmission.type_ = LCC_TRANS_DCT;
                d.transmission.final_drive_ratio = 3.5;
            }
            ViewPreset::Hypercar => {
                d.chassis.mass_kg = 1250.0;
                d.driveline.layout = LCC_LAYOUT_AWD;
                d.engine.redline_rpm = 8500.0;
                for i in 0..4 {
                    d.tires[i].mu_nominal = 1.5;
                    d.tires[i].pressure_kpa = 250.0;
                }
                d.aero.lift_coefficient_front = -0.5;
                d.aero.lift_coefficient_rear = -1.0;
                d.transmission.type_ = LCC_TRANS_MANUAL;
                d.transmission.final_drive_ratio = 3.3;
            }
            ViewPreset::Drift => {
                d.chassis.mass_kg = 1300.0;
                d.driveline.layout = LCC_LAYOUT_RWD;
                d.engine.redline_rpm = 7200.0;
                d.engine.idle_rpm = 650.0;
                d.starter.power_w = 1500.0;
                for i in 0..4 {
                    d.tires[i].mu_nominal = 1.15;
                }
                d.aero.lift_coefficient_front = 0.0;
                d.aero.lift_coefficient_rear = 0.0;
                d.transmission.type_ = LCC_TRANS_MANUAL;
                d.transmission.final_drive_ratio = 3.7;
            }
        }

        if preset == ViewPreset::Drift {
            d.engine.wot_torque_nm_vs_rpm = make_curve(DRIFT_WOT);
            d.engine.friction_torque_nm_vs_rpm = make_curve(DRIFT_FRIC);
        } else {
            d.engine.wot_torque_nm_vs_rpm = make_curve(WEAK_WOT);
            d.engine.friction_torque_nm_vs_rpm = make_curve(WEAK_FRIC);
        }

        d
    }

    unsafe fn new(preset: ViewPreset) -> Self {
        let mut env: ffi::lcc_environment_t = mem::zeroed();
        ffi::lcc_environment_init_defaults(&mut env);

        let mut desc = Self::make_desc_from_preset(preset);
        desc.environment = env;

        let car = ffi::lcc_car_create(&desc as *const _);

        let mut this = Self {
            car,
            desc,
            env,
            controls: mem::zeroed(),
            preset,
            last_tick: Instant::now(),
            accumulator: 0.0,
            fixed_dt: 1.0 / 120.0,
            paused: false,
            sim_speed: 1.0,
            input: InputState::new(),
            telemetry_rec: TelemetryRec::new(),
            show_trace: true,
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
            abs_on: false,
            tc_on: false,
            esc_on: false,
            config: CarConfig::from_desc(&desc),
            path_trace: Vec::new(),
            max_trace_points: 4000,
            trace_point_spacing_m: 0.25,
        };
        // Set driver aids
        ffi::lcc_car_set_abs(this.car, if this.abs_on { LCC_ABS_ON } else { LCC_ABS_OFF });
        ffi::lcc_car_set_tc(this.car, if this.tc_on { LCC_TC_ON } else { LCC_TC_OFF });
        ffi::lcc_car_set_esc(this.car, if this.esc_on { LCC_ESC_ON } else { LCC_ESC_OFF });
        this.center_camera_on_car();
        this
    }

    unsafe fn recreate_from_desc(&mut self) {
        if !self.car.is_null() {
            ffi::lcc_car_destroy(self.car);
        }
        self.desc.environment = self.env;
        self.car = ffi::lcc_car_create(&self.desc as *const _);
        // aids
        ffi::lcc_car_set_abs(self.car, if self.abs_on { LCC_ABS_ON } else { LCC_ABS_OFF });
        ffi::lcc_car_set_tc(self.car, if self.tc_on { LCC_TC_ON } else { LCC_TC_OFF });
        ffi::lcc_car_set_esc(self.car, if self.esc_on { LCC_ESC_ON } else { LCC_ESC_OFF });
        self.telemetry_rec.clear();
        self.plots = Plots::new();
        self.path_trace.clear();
        self.skid_segments.clear();
        self.have_last_wheel_world = false;
        self.center_camera_on_car();
    }

    fn center_camera_on_car(&mut self) {
        let pos = self.world_pos();
        self.camera_pos = pos;
    }

    fn world_pos(&self) -> Vec2 {
        unsafe {
            let cs = &self.car_ref().car_state;
            Vec2::new(cs.pos_world[0], cs.pos_world[1])
        }
    }

    unsafe fn reset(&mut self) {
        self.desc = Self::make_desc_from_preset(self.preset);
        self.env = self.desc.environment;
        self.recreate_from_desc();
        self.config = CarConfig::from_desc(&self.desc);
    }

    unsafe fn handle_input(&mut self, ctx: &egui::Context, dt: f32) {
        let input = ctx.input(|i| i.clone());

        let target_steer = if input.key_down(egui::Key::A) {
            -1.0
        } else if input.key_down(egui::Key::D) {
            1.0
        } else {
            0.0
        };
        self.input.steer = linear_approach(self.input.steer, target_steer, 10.0, dt);

        let throttle_down = input.key_down(egui::Key::K);
        let target_throttle = if throttle_down { 1.0 } else { 0.0 };
        self.input.throttle = linear_approach(self.input.throttle, target_throttle, 3.0, dt);

        let handbrake_down = input.key_down(egui::Key::C);
        self.input.handbrake = if handbrake_down { 1.0 } else { 0.0 };

        let brake_down = input.key_down(egui::Key::J);
        let target_brake = if brake_down { 1.0 } else { 0.0 };
        self.input.brake = linear_approach(self.input.brake, target_brake, 4.0, dt);

        let clutch_down = input.key_down(egui::Key::H);
        let target_clutch = if clutch_down { 1.0 } else { 0.0 };
        self.input.clutch = linear_approach(self.input.clutch, target_clutch, 8.0, dt);

        let w = input.key_pressed(egui::Key::W);
        let s = input.key_pressed(egui::Key::S);
        if w && !self.input.last_w {
            let _ = ffi::lcc_car_shift_up(self.car);
        }
        if s && !self.input.last_s {
            let _ = ffi::lcc_car_shift_down(self.car);
        }
        self.input.last_w = w;
        self.input.last_s = s;

        // starter (space) + ignition on
        self.controls.ignition_switch = 1;
        self.controls.starter = if input.key_down(egui::Key::Space) {
            1
        } else {
            0
        };

        self.controls.steer = self.input.steer;
        self.controls.throttle = self.input.throttle;
        self.controls.brake = self.input.brake;
        self.controls.clutch = self.input.clutch;
        self.controls.handbrake = self.input.handbrake;
        ffi::lcc_car_set_controls(self.car, &self.controls as *const _);
    }
    unsafe fn fixed_update(&mut self, dt: f32) {
        let _ = ffi::lcc_car_step(self.car, dt);

        let cs = self.car_ref().car_state;
        let es = self.car_ref().engine_state;
        let ts = self.car_ref().trans_state;
        let ws = self.car_ref().wheel_states;

        let speed_kmh = cs.speed_mps * 3.6;
        let rpm = es.rpm;

        let omega = [
            ws[0].omega_radps,
            ws[1].omega_radps,
            ws[2].omega_radps,
            ws[3].omega_radps,
        ];
        let slip = [
            ws[0].slip_ratio,
            ws[1].slip_ratio,
            ws[2].slip_ratio,
            ws[3].slip_ratio,
        ];
        self.plots
            .push(rpm, speed_kmh, omega, slip, cs.yaw_rate_radps);

        // helper: estimate mu and mu*Fz as in lib
        let est_mu_fc = |i: usize, wst: &ffi::lcc_wheel_state_t| -> (f32, f32) {
            let fz = wst.normal_force_n.max(0.0);
            let mut mu = self.desc.tires[i].mu_nominal + self.desc.tires[i].load_sensitivity * fz;
            mu = mu.clamp(0.2, 3.0);
            mu *= self.env.global_friction_scale.clamp(0.1, 2.0);
            (mu, mu * fz)
        };

        let wheel_sample = |i: usize| -> WheelSample {
            let wst = &ws[i];
            let (mu, fcap) = est_mu_fc(i, wst);
            WheelSample {
                slip_ratio: wst.slip_ratio,
                slip_angle: wst.slip_angle_rad,
                omega: wst.omega_radps,
                f_long: wst.tire_force_long_n,
                f_lat: wst.tire_force_lat_n,
                load: wst.normal_force_n,
                t_drive: wst.drive_torque_nm,
                t_brake: wst.brake_torque_nm,
                temp: wst.tire_temp_c,
                est_mu: mu,
                fcap,
            }
        };

        let glue = cs.speed_mps < 0.25;

        // compute wheel world positions from pose
        let pos = Vec2::new(cs.pos_world[0], cs.pos_world[1]);
        let rot = Mat2::from_angle(cs.yaw_rad);
        let wheel_world: [Vec2; 4] = (0..4)
            .map(|i| {
                let local = Vec2::new(
                    self.desc.wheels[i].position_local[0],
                    self.desc.wheels[i].position_local[1],
                );
                pos + rot * local
            })
            .collect::<Vec<Vec2>>()
            .try_into()
            .unwrap_or([Vec2::ZERO; 4]);

        let sample = TelemetrySample {
            t: cs.time_s as f32,
            dt,
            pos_x: cs.pos_world[0],
            pos_y: cs.pos_world[1],
            vel_x: cs.vel_world[0],
            vel_y: cs.vel_world[1],
            speed_mps: cs.speed_mps,
            speed_kmh,
            yaw: cs.yaw_rad,
            yaw_rate: cs.yaw_rate_radps,

            engine_rpm: es.rpm,
            gear: ts.gear_index,
            throttle: self.controls.throttle,
            brake: self.controls.brake,
            steering: self.controls.steer,
            clutch: self.controls.clutch,
            handbrake: self.controls.handbrake,

            abs_on: self.abs_on,
            tc_on: self.tc_on,
            esc_on: self.esc_on,
            glue,

            wheels: [
                wheel_sample(0),
                wheel_sample(1),
                wheel_sample(2),
                wheel_sample(3),
            ],
        };
        self.telemetry_rec.push(sample);

        // Path trace
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
        if !self.have_last_wheel_world {
            self.last_wheel_world = wheel_world;
            self.have_last_wheel_world = true;
        } else {
            for i in 0..4 {
                let w = &ws[i];
                let sr = w.slip_ratio.abs();
                let sa = w.slip_angle_rad.abs();
                let long = (sr / self.slip_ratio_threshold).max(0.0);
                let lat = (sa / self.slip_angle_threshold).max(0.0);
                let severity = long.max(lat).min(3.0);
                let moving = speed_kmh > self.min_speed_for_skid_kmh;
                let loaded = w.normal_force_n > 50.0;
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
        self.handle_input(ctx, dt);

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
            let car = self.car_ref();
            let cs = &car.car_state;

            let pos = Vec2::new(cs.pos_world[0], cs.pos_world[1]);
            let p0 = to_screen_point(pos, self.camera_pos, self.zoom, center);
            let vel = Vec2::new(cs.vel_world[0], cs.vel_world[1]);
            let p1 = to_screen_point(pos + vel * 0.5, self.camera_pos, self.zoom, center);
            painter.line_segment([p0, p1], Stroke::new(2.0, Color32::LIGHT_BLUE));

            let yaw = cs.yaw_rad;
            let rot = Mat2::from_angle(yaw);

            let body_len = self
                .desc
                .chassis
                .length_m
                .max(self.desc.chassis.wheelbase_m + 0.7);
            let body_w = self.desc.chassis.width_m.max(
                self.desc
                    .chassis
                    .track_front_m
                    .max(self.desc.chassis.track_rear_m)
                    + 0.4,
            );
            let half = Vec2::new(body_len * 0.5, body_w * 0.5);
            let corners = [
                Vec2::new(half.x, -half.y),
                Vec2::new(half.x, half.y),
                Vec2::new(-half.x, half.y),
                Vec2::new(-half.x, -half.y),
            ]
            .map(|local| pos + rot * local);

            painter.add(egui::Shape::convex_polygon(
                corners
                    .iter()
                    .map(|&w| to_screen_point(w, self.camera_pos, self.zoom, center))
                    .collect(),
                Color32::from_black_alpha(20),
                Stroke::new(2.0, Color32::from_rgb(180, 200, 220)),
            ));

            for i in 0..4usize {
                let local = Vec2::new(
                    self.desc.wheels[i].position_local[0],
                    self.desc.wheels[i].position_local[1],
                );
                let wheel_world = pos + rot * local;

                let ws = &car.wheel_states[i];

                let angle = yaw + self.car_ref().wheel_steer_rad[i];
                draw_wheel_vis(
                    &painter,
                    &|w| to_screen_point(w, self.camera_pos, self.zoom, center),
                    wheel_world,
                    angle,
                    self.desc.wheels[i].radius_m * 2.0,
                    self.desc.wheels[i].width_m,
                    ws,
                );
            }
        }
    }

    fn draw_config_editor(&mut self, ui: &mut egui::Ui) {
        ui.heading("Car setup (runtime & recreate)");
        ui.separator();

        ui.horizontal(|ui| {
            if ui.button("Apply runtime fields").clicked() {
                self.config
                    .apply_runtime(self.car, &mut self.desc, &mut self.env);
            }
            if ui.button("Recreate car from editor").clicked() {
                // copy editor -> desc and recreate
                // General
                self.desc.chassis.mass_kg = self.config.car.mass_kg;
                self.desc.chassis.wheelbase_m = self.config.car.wheelbase;
                self.desc.chassis.track_front_m = self.config.car.track_front;
                self.desc.chassis.track_rear_m = self.config.car.track_rear;
                self.desc.chassis.cg_height_m = self.config.car.cg_height;
                // fuck bindgen https://github.com/rust-lang/rust-bindgen/issues/1966
                #[cfg(target_os = "windows")]
                {
                    self.desc.driveline.layout = self.config.car.layout;
                }
                #[cfg(not(target_os = "windows"))]
                {
                    self.desc.driveline.layout = self.config.car.layout as u32;
                }
                self.desc.transmission.final_drive_ratio = self.config.car.final_drive;

                self.desc.aero.drag_coefficient = self.config.car.cd;
                self.desc.aero.frontal_area_m2 = self.config.car.frontal_area;
                self.desc.aero.lift_coefficient_front = self.config.car.cl_front;
                self.desc.aero.lift_coefficient_rear = self.config.car.cl_rear;

                self.desc.steering.max_steer_deg = self.config.car.max_steer_deg;
                self.desc.steering.ackermann_factor = self.config.car.ackermann;

                // Wheels/tires geometry
                for i in 0..4 {
                    self.desc.wheels[i].position_local[0] = self.config.wheels[i].pos_x;
                    self.desc.wheels[i].position_local[1] = self.config.wheels[i].pos_y;
                    self.desc.wheels[i].steerable = if self.config.wheels[i].steerable {
                        1
                    } else {
                        0
                    };
                    self.desc.wheels[i].driven = if self.config.wheels[i].driven { 1 } else { 0 };
                    self.desc.wheels[i].radius_m = self.config.tires[i].radius_m;
                    self.desc.wheels[i].width_m = self.config.tires[i].width_m;

                    self.desc.tires[i].mu_nominal = self.config.tires[i].mu_nominal;
                    self.desc.tires[i].pressure_kpa = self.config.tires[i].pressure_kpa;
                    self.desc.tires[i].rolling_resistance = self.config.tires[i].rolling_resistance;
                    self.desc.tires[i].load_sensitivity = self.config.tires[i].load_sens;
                }

                unsafe {
                    self.recreate_from_desc();
                }
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
                        ui.add(egui::DragValue::new(&mut self.config.car.mass_kg).speed(1.0));
                        ui.end_row();
                        ui.label("Wheelbase (m)");
                        if ui
                            .add(egui::DragValue::new(&mut self.config.car.wheelbase).speed(0.01))
                            .changed()
                        {
                            let half_wb = self.config.car.wheelbase * 0.5;
                            self.config.wheels[0].pos_x = half_wb;
                            self.config.wheels[1].pos_x = half_wb;
                            self.config.wheels[2].pos_x = -half_wb;
                            self.config.wheels[3].pos_x = -half_wb;
                        }
                        ui.end_row();
                        ui.label("Track front (m)");
                        if ui
                            .add(egui::DragValue::new(&mut self.config.car.track_front).speed(0.01))
                            .changed()
                        {
                            let hf = self.config.car.track_front * 0.5;
                            self.config.wheels[0].pos_y = hf;
                            self.config.wheels[1].pos_y = -hf;
                        }
                        ui.end_row();
                        ui.label("Track rear (m)");
                        if ui
                            .add(egui::DragValue::new(&mut self.config.car.track_rear).speed(0.01))
                            .changed()
                        {
                            let hr = self.config.car.track_rear * 0.5;
                            self.config.wheels[2].pos_y = hr;
                            self.config.wheels[3].pos_y = -hr;
                        }
                        ui.end_row();
                        ui.label("CG height (m)");
                        ui.add(egui::DragValue::new(&mut self.config.car.cg_height).speed(0.005));
                        ui.end_row();
                        ui.label("Final drive");
                        ui.add(egui::DragValue::new(&mut self.config.car.final_drive).speed(0.01));
                        ui.end_row();

                        ui.label("Layout");
                        egui::ComboBox::from_id_salt("layout")
                            .selected_text(match self.config.car.layout {
                                x if x == LCC_LAYOUT_FWD as i32 => "FWD",
                                x if x == LCC_LAYOUT_RWD as i32 => "RWD",
                                x if x == LCC_LAYOUT_AWD as i32 => "AWD",
                                _ => "FWD",
                            })
                            .show_ui(ui, |ui| {
                                ui.selectable_value(
                                    &mut self.config.car.layout,
                                    LCC_LAYOUT_FWD as i32,
                                    "FWD",
                                );
                                ui.selectable_value(
                                    &mut self.config.car.layout,
                                    LCC_LAYOUT_RWD as i32,
                                    "RWD",
                                );
                                ui.selectable_value(
                                    &mut self.config.car.layout,
                                    LCC_LAYOUT_AWD as i32,
                                    "AWD",
                                );
                            });
                        ui.end_row();

                        ui.label("Cd");
                        ui.add(egui::DragValue::new(&mut self.config.car.cd).speed(0.005));
                        ui.end_row();
                        ui.label("Frontal area (m^2)");
                        ui.add(egui::DragValue::new(&mut self.config.car.frontal_area).speed(0.01));
                        ui.end_row();
                        ui.label("Cl front");
                        ui.add(egui::DragValue::new(&mut self.config.car.cl_front).speed(0.01));
                        ui.end_row();
                        ui.label("Cl rear");
                        ui.add(egui::DragValue::new(&mut self.config.car.cl_rear).speed(0.01));
                        ui.end_row();

                        ui.label("Max steer (deg)");
                        ui.add(egui::DragValue::new(&mut self.config.car.max_steer_deg).speed(0.1));
                        ui.end_row();
                        ui.label("Ackermann");
                        ui.add(egui::DragValue::new(&mut self.config.car.ackermann).speed(0.01));
                        ui.end_row();

                        ui.label("Global μ scale");
                        ui.add(egui::DragValue::new(&mut self.config.car.global_mu).speed(0.01));
                        ui.end_row();
                    });
            });

        egui::CollapsingHeader::new("Tires & wheels")
            .default_open(false)
            .show(ui, |ui| {
                for i in 0..4 {
                    egui::CollapsingHeader::new(format!("Wheel {}", i)).show(ui, |ui| {
                        egui::Grid::new(format!("wheel_grid_{i}"))
                            .num_columns(2)
                            .striped(true)
                            .show(ui, |ui| {
                                ui.label("Pos X (m)");
                                ui.add(
                                    egui::DragValue::new(&mut self.config.wheels[i].pos_x)
                                        .speed(0.005),
                                );
                                ui.end_row();
                                ui.label("Pos Y (m)");
                                ui.add(
                                    egui::DragValue::new(&mut self.config.wheels[i].pos_y)
                                        .speed(0.005),
                                );
                                ui.end_row();
                                ui.label("Steerable");
                                ui.checkbox(&mut self.config.wheels[i].steerable, "");
                                ui.end_row();
                                ui.label("Driven");
                                ui.checkbox(&mut self.config.wheels[i].driven, "");
                                ui.end_row();

                                ui.label("Tire radius (m)");
                                ui.add(
                                    egui::DragValue::new(&mut self.config.tires[i].radius_m)
                                        .speed(0.001),
                                );
                                ui.end_row();
                                ui.label("Tire width (m)");
                                ui.add(
                                    egui::DragValue::new(&mut self.config.tires[i].width_m)
                                        .speed(0.001),
                                );
                                ui.end_row();
                                ui.label("μ nominal");
                                ui.add(
                                    egui::DragValue::new(&mut self.config.tires[i].mu_nominal)
                                        .speed(0.005),
                                );
                                ui.end_row();
                                ui.label("Pressure (kPa)");
                                ui.add(
                                    egui::DragValue::new(&mut self.config.tires[i].pressure_kpa)
                                        .speed(0.5),
                                );
                                ui.end_row();
                                ui.label("Rolling resistance");
                                ui.add(
                                    egui::DragValue::new(
                                        &mut self.config.tires[i].rolling_resistance,
                                    )
                                    .speed(0.0005),
                                );
                                ui.end_row();
                                ui.label("Load sensitivity");
                                ui.add(
                                    egui::DragValue::new(&mut self.config.tires[i].load_sens)
                                        .speed(0.0005),
                                );
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
            let car = self.car_ref();
            let cs = &car.car_state;
            let es = &car.engine_state;
            let ts = &car.trans_state;

            ui.label(format!("Speed: {:6.1} km/h", cs.speed_mps * 3.6));
            ui.label(format!("Engine RPM: {:6.0}", es.rpm));
            ui.label(format!("Gear: {}", ts.gear_index));
            ui.label(format!("Yaw: {:5.2} rad", cs.yaw_rad));
        }
        ui.separator();
        ui.collapsing("Controls", |ui| {
            ui.monospace("a/d = steer\nw/s = shift\nh = clutch\nj = brake\nk = throttle\nSpace = hold START (ign ON)");
        });

        ui.separator();
        ui.collapsing("Driver Aids", |ui| {
            let mut changed = false;
            changed |= ui.checkbox(&mut self.abs_on, "ABS").changed();
            changed |= ui.checkbox(&mut self.tc_on, "TC").changed();
            changed |= ui.checkbox(&mut self.esc_on, "ESC").changed();
            if changed {
                unsafe {
                    ffi::lcc_car_set_abs(
                        self.car,
                        if self.abs_on { LCC_ABS_ON } else { LCC_ABS_OFF },
                    );
                    ffi::lcc_car_set_tc(self.car, if self.tc_on { LCC_TC_ON } else { LCC_TC_OFF });
                    ffi::lcc_car_set_esc(
                        self.car,
                        if self.esc_on { LCC_ESC_ON } else { LCC_ESC_OFF },
                    );
                }
            }
        });

        ui.separator();
        ui.collapsing("Fuel / Electrics", |ui| unsafe {
            let car = self.car_ref();
            let fuel = &car.fuel_state;
            let elec = &car.elec_state;

            let cap = self.desc.fuel.tank_capacity_l;
            let lvl = fuel.fuel_l;
            ui.label(format!("Fuel: {:5.1} / {:5.1} L", lvl, cap));
            ui.horizontal(|ui| {
                if ui.button("Refill full").clicked() {
                    let _ = ffi::lcc_car_set_fuel(self.car, cap);
                }
                if ui.button("+5 L").clicked() {
                    let _ = ffi::lcc_car_refuel(self.car, 5.0);
                }
            });
            ui.label(format!("Battery SOC: {:3.0}%", elec.battery_soc * 100.0));
        });

        ui.separator();
        ui.collapsing("Telemetry", |ui| {
            ui.horizontal(|ui| {
                if ui
                    .button(if self.telemetry_rec.recording {
                        "Stop recording"
                    } else {
                        "Start recording"
                    })
                    .clicked()
                {
                    self.telemetry_rec.recording = !self.telemetry_rec.recording;
                }
                if ui.button("Clear").clicked() {
                    self.telemetry_rec.clear();
                }
            });
            if ui.button("Export CSV").clicked() {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("CSV", &["csv"])
                    .save_file()
                {
                    if let Err(e) = self.telemetry_rec.export_csv(&path) {
                        eprintln!("CSV export failed: {e:?}");
                    }
                }
            }
            if ui.button("Export JSON").clicked() {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("JSON", &["json"])
                    .save_file()
                {
                    if let Err(e) = self.telemetry_rec.export_json(&path) {
                        eprintln!("JSON export failed: {e:?}");
                    }
                }
            }
            ui.label(format!("Samples: {}", self.telemetry_rec.data.len()));
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

                            hstrip.cell(|ui| {
                                let col_h = ui.available_height();
                                let half = (col_h - 6.0) * 0.5;

                                Plot::new("omega_plot")
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
                                                Line::new(format!("W{i} ω (rad/s)"), pts)
                                                    .color(colors[i]),
                                            );
                                        }
                                    });

                                Plot::new("slip_plot")
                                    .legend(Legend::default())
                                    .height(half)
                                    .show(ui, |pui| {
                                        let colors = [
                                            Color32::from_rgb(255, 80, 80),
                                            Color32::from_rgb(255, 150, 50),
                                            Color32::from_rgb(50, 200, 255),
                                            Color32::from_rgb(180, 120, 255),
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
            let car = self.car_ref();
            let es = &car.engine_state;
            let ts = &car.trans_state;

            let rpm = es.rpm;
            let redline = self.desc.engine.redline_rpm.max(1.0);
            draw_rpm_gauge(&painter, origin, rpm, redline);

            let tb_origin = Pos2::new(origin.x + 160.0, origin.y - 60.0);
            draw_bar(
                &painter,
                tb_origin,
                "Throttle",
                self.controls.throttle,
                Color32::LIGHT_GREEN,
            );
            draw_bar(
                &painter,
                Pos2::new(tb_origin.x, tb_origin.y + 24.0),
                "Brake",
                self.controls.brake,
                Color32::LIGHT_RED,
            );
            draw_bar(
                &painter,
                Pos2::new(tb_origin.x, tb_origin.y + 48.0),
                "Handbrake",
                self.controls.handbrake,
                Color32::DARK_RED,
            );
            draw_bar(
                &painter,
                Pos2::new(tb_origin.x, tb_origin.y + 72.0),
                "Clutch",
                self.controls.clutch,
                Color32::LIGHT_BLUE,
            );
            draw_bar(
                &painter,
                Pos2::new(tb_origin.x, tb_origin.y + 96.0),
                "Steer",
                (self.controls.steer + 1.0) / 2.0,
                Color32::YELLOW,
            );

            let gear = ts.gear_index;
            let gtxt = if gear == 0 {
                "R".to_string()
            } else if gear == 1 {
                "N".to_string()
            } else {
                format!("{}", gear - 1)
            };
            painter.text(
                Pos2::new(origin.x - 60.0, origin.y + 70.0),
                Align2::CENTER_CENTER,
                format!("Gear {}", gtxt),
                egui::FontId::proportional(18.0),
                Color32::WHITE,
            );

            let base2 = Pos2::new(origin.x + 160.0, origin.y + 50.0);
            let cap = self.desc.fuel.tank_capacity_l;
            let lvl = car.fuel_state.fuel_l;
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
                let w = &car.wheel_states[i];
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

fn to_screen_point(w: Vec2, cam: Vec2, zoom: f32, center: Pos2) -> Pos2 {
    let rel = w - cam;
    let screen = Vec2::new(rel.x * zoom, -rel.y * zoom);
    Pos2::new(center.x + screen.x, center.y + screen.y)
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t.clamp(0.0, 1.0)
}
fn linear_approach(current: f32, target: f32, rate: f32, dt: f32) -> f32 {
    let diff = target - current;
    let step = rate * dt;
    if diff.abs() <= step {
        target
    } else {
        current + diff.signum() * step
    }
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

unsafe fn draw_wheel_vis(
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

    // slip vectors
    let slip_dir_local = Vec2::new(0.0, w.slip_angle_rad.tan().clamp(-2.0, 2.0));
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

    // heat-ish color dot
    let temp = w.tire_temp_c;
    let color = Color32::from_rgb(
        ((temp.clamp(20.0, 150.0) - 20.0) / 130.0 * 255.0) as u8,
        180,
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
        "{label}  SR {:5.2}  SA {:5.2}  ω {:8.2}  Fz {:5.0}N  T {:4.1}°C  wear {:4.2}",
        w.slip_ratio, w.slip_angle_rad, w.omega_radps, w.normal_force_n, w.tire_temp_c, w.tire_wear
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
        Box::new(|_cc| Ok(Box::new(unsafe { App::new(ViewPreset::Hypercar) }))),
    )
}
