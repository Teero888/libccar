use serde::Serialize;

#[derive(Serialize, Clone)]
pub struct WheelSample {
    // kinematics
    pub slip_ratio: f32, // [-] (as reported)
    pub slip_angle: f32, // [rad] (as reported)
    pub omega: f32,      // [rad/s]
    // forces
    pub f_long: f32, // [N] tire_force_long_n
    pub f_lat: f32,  // [N] tire_force_lat_n
    pub load: f32,   // [N] normal_force_n
    // torques
    pub t_drive: f32, // [Nm]
    pub t_brake: f32, // [Nm]
    // tire/env
    pub temp: f32,   // [C]
    pub est_mu: f32, // estimated mu (see logic below)
    pub fcap: f32,   // mu*Fz [N]
}
#[derive(Serialize, Clone)]
pub struct TelemetrySample {
    // time & pos
    pub t: f32,
    pub dt: f32,
    pub pos_x: f32,
    pub pos_y: f32,
    pub vel_x: f32,
    pub vel_y: f32,
    pub speed_mps: f32,
    pub speed_kmh: f32,
    pub yaw: f32,
    pub yaw_rate: f32,

    // engine / trans / inputs
    pub engine_rpm: f32,

    pub gear: i32,
    pub throttle: f32,  // command (your input/control)
    pub brake: f32,     // command
    pub steering: f32,  // command
    pub clutch: f32,    // command
    pub handbrake: f32, // command

    // aids state at time of sample
    pub abs_on: bool,
    pub tc_on: bool,
    pub esc_on: bool,

    // low-speed “glue” flag (sim < 0.25 m/s)
    pub glue: bool,

    pub wheels: [WheelSample; 4],
}

pub struct TelemetryRec {
    pub recording: bool,
    pub data: Vec<TelemetrySample>,
    pub max_points: usize,
}
impl TelemetryRec {
    pub fn new() -> Self {
        Self {
            recording: false,
            data: Vec::new(),
            max_points: 20000,
        }
    }
    pub fn clear(&mut self) {
        self.data.clear();
    }
    pub fn push(&mut self, s: TelemetrySample) {
        if self.recording {
            self.data.push(s);
            if self.data.len() > self.max_points {
                self.data.drain(0..(self.data.len() - self.max_points));
            }
        }
    }
    pub fn export_csv(&self, path: &std::path::Path) -> anyhow::Result<()> {
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
    pub fn export_json(&self, path: &std::path::Path) -> anyhow::Result<()> {
        let s = serde_json::to_string_pretty(&self.data)?;
        std::fs::write(path, s)?;
        Ok(())
    }
}
