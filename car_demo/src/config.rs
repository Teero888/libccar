use crate::ffi_bindings::ffi;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, PartialEq)]
pub enum ViewPreset {
    Economy,
    Midsize,
    Sports,
    Supercar,
    Hypercar,
    Drift,
}
impl ViewPreset {
    pub fn all() -> &'static [ViewPreset] {
        &[
            ViewPreset::Economy,
            ViewPreset::Midsize,
            ViewPreset::Sports,
            ViewPreset::Supercar,
            ViewPreset::Hypercar,
            ViewPreset::Drift,
        ]
    }
    pub fn as_label(self) -> &'static str {
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

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct CarCfg {
    pub mass_kg: f32,
    pub wheelbase: f32,
    pub track_front: f32,
    pub track_rear: f32,
    pub cg_height: f32,
    pub final_drive: f32,
    pub layout: i32, // LCC_LAYOUT_FWD/RWD/AWD/4X4
    // aero
    pub cd: f32,
    pub frontal_area: f32,
    pub cl_front: f32,
    pub cl_rear: f32,
    // steering
    pub max_steer_deg: f32,
    pub ackermann: f32,
    // tire mu scale
    pub global_mu: f32,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TireCfg {
    pub mu_nominal: f32,
    pub pressure_kpa: f32,
    pub rolling_resistance: f32,
    pub load_sens: f32,
    pub radius_m: f32,
    pub width_m: f32,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct WheelCfg {
    pub pos_x: f32,
    pub pos_y: f32,
    pub steerable: bool,
    pub driven: bool,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct CarConfig {
    pub car: CarCfg,
    pub tires: [TireCfg; 4],
    pub wheels: [WheelCfg; 4],
}

impl CarConfig {
    pub fn from_desc(desc: &ffi::lcc_car_desc_t) -> Self {
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
    pub fn apply_runtime(
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
