use crate::{
    app::App,
    config::{CarConfig, ViewPreset},
    ffi_bindings::{ffi, LCC_ABS_OFF, LCC_ABS_ON, LCC_ESC_OFF, LCC_ESC_ON, LCC_TC_OFF, LCC_TC_ON},
    ffi_bindings::{LCC_LAYOUT_AWD, LCC_LAYOUT_FWD, LCC_LAYOUT_RWD},
    ffi_bindings::{LCC_TRANS_DCT, LCC_TRANS_MANUAL},
    input::InputState,
    telemetry::{TelemetryRec, TelemetrySample, WheelSample},
    track::Track, // Import Track
    ui::{Plots, SkidSegment},
};
use glam::{Mat2, Vec2};
use std::{mem, time::Instant};

// Sets the initial pos of the car based on the selected track.
unsafe fn set_initial_car_pos(
    car_ptr: *mut ffi::lcc_car_t,
    selected_track_idx: usize,
    tracks: &[Track],
) {
    let initial_pos: Vec2;
    let initial_yaw: f32; // Default facing positive X

    if selected_track_idx > 0 {
        let track = &tracks[selected_track_idx - 1];
        let spawn_distance_behind_start = 10.0;
        let start_dir = track.get_direction(0.0);
        let start_pos = track.get_position(0.0);
        initial_pos = start_pos - start_dir * spawn_distance_behind_start;
        initial_yaw = start_dir.y.atan2(start_dir.x);
    } else {
        initial_pos = Vec2::new(0.0, 0.0);
        initial_yaw = 0.0; // Face +X
    }

    let _ = ffi::lcc_car_set_pos(car_ptr, initial_pos.as_ref().as_ptr(), initial_yaw);
}

impl App {
    pub unsafe fn make_desc_from_preset(preset: ViewPreset) -> ffi::lcc_car_desc_t {
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

    pub unsafe fn new(preset: ViewPreset) -> Self {
        let mut env: ffi::lcc_environment_t = mem::zeroed();
        ffi::lcc_environment_init_defaults(&mut env);

        let mut desc = Self::make_desc_from_preset(preset);
        desc.environment = env;

        let car = ffi::lcc_car_create(&desc as *const _);

        // Steering wheel/gamepad init
        let gilrs = gilrs::Gilrs::new()
            .map_err(|e| eprintln!("Failed to init gilrs: {e}"))
            .ok();
        let active_gamepad = gilrs
            .as_ref()
            .and_then(|g| g.gamepads().next().map(|(id, _)| id));
        if active_gamepad.is_some() {
            println!("Steering wheel/gamepad connected!");
        }

        // Initialize track data before setting pos
        let tracks = crate::track::get_predefined_tracks();
        let selected_track_idx = 0; // Default to "None" initially

        set_initial_car_pos(car, selected_track_idx, &tracks);

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
            camera_pos: Vec2::ZERO, // Will be centered shortly
            skid_segments: Vec::new(),
            skid_ttl: 6.0,
            show_skids: true,
            last_wheel_world: [Vec2::ZERO; 4],
            have_last_wheel_world: false,
            tracks, // Use the initialized tracks
            selected_track: selected_track_idx,
            lap_times: Vec::new(),
            lap_start_time: None,
            current_checkpoint: 0,
            last_pos: Vec2::ZERO, // Will be set by set_initial_car_pos indirectly
            have_last_pos: false,
            abs_on: false,
            tc_on: false,
            esc_on: false,
            config: CarConfig::from_desc(&desc),
            path_trace: Vec::new(),
            max_trace_points: 4000,
            trace_point_spacing_m: 0.25,
            gilrs,
            active_gamepad,
        };
        // Set driver aids
        ffi::lcc_car_set_abs(this.car, if this.abs_on { LCC_ABS_ON } else { LCC_ABS_OFF });
        ffi::lcc_car_set_tc(this.car, if this.tc_on { LCC_TC_ON } else { LCC_TC_OFF });
        ffi::lcc_car_set_esc(this.car, if this.esc_on { LCC_ESC_ON } else { LCC_ESC_OFF });

        this.center_camera_on_car();
        this.last_pos = this.world_pos();
        this.have_last_pos = true;

        this
    }

    pub unsafe fn recreate_from_desc(&mut self) {
        if !self.car.is_null() {
            ffi::lcc_car_destroy(self.car);
        }
        self.desc.environment = self.env;
        self.car = ffi::lcc_car_create(&self.desc as *const _);

        set_initial_car_pos(self.car, self.selected_track, &self.tracks);

        // aids
        ffi::lcc_car_set_abs(self.car, if self.abs_on { LCC_ABS_ON } else { LCC_ABS_OFF });
        ffi::lcc_car_set_tc(self.car, if self.tc_on { LCC_TC_ON } else { LCC_TC_OFF });
        ffi::lcc_car_set_esc(self.car, if self.esc_on { LCC_ESC_ON } else { LCC_ESC_OFF });

        // Reset other state
        self.telemetry_rec.clear();
        self.plots = Plots::new();
        self.path_trace.clear();
        self.skid_segments.clear();
        self.have_last_wheel_world = false;
        self.lap_times.clear();
        self.lap_start_time = None;
        self.current_checkpoint = 0;
        self.center_camera_on_car();
        self.last_pos = self.world_pos();
        self.have_last_pos = true;
    }

    pub fn center_camera_on_car(&mut self) {
        let pos = self.world_pos();
        self.camera_pos = pos;
    }

    pub fn world_pos(&self) -> Vec2 {
        unsafe {
            let cs = &self.car_ref().car_state;
            Vec2::new(cs.pos_world[0], cs.pos_world[1])
        }
    }

    pub unsafe fn reset(&mut self) {
        // We keep the selected track, but reset the car desc to the preset
        self.desc = Self::make_desc_from_preset(self.preset);
        self.env = self.desc.environment; // Update env if preset changed it
        self.recreate_from_desc(); // This now handles setting the pos based on selected_track
        self.config = CarConfig::from_desc(&self.desc);
    }

    pub unsafe fn fixed_update(&mut self, dt: f32) {
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

        let glue = cs.speed_mps < 0.25;

        let pos = Vec2::new(cs.pos_world[0], cs.pos_world[1]);

        // Lap timing logic
        if self.selected_track > 0 {
            let track = self.tracks[self.selected_track - 1].clone(); // Clone to avoid borrow issue TODO: make this not shit idk how to avoid the borrow checker. worst enemy
            if self.have_last_pos {
                self.check_lap_timing(self.last_pos, pos, &track);
            }
        }
        self.last_pos = pos;

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
                let severity = (w.slip_ratio.powi(2) + w.slip_angle_rad.powi(2)).sqrt() * 2.0;
                if self.show_skids {
                    let strength = severity.min(1.0);
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

    pub unsafe fn step_sim(&mut self, ctx: &eframe::egui::Context) {
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
}
