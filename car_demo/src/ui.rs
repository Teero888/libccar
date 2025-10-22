use crate::{
    app::App,
    config::ViewPreset,
    ffi_bindings::{
        ffi, LCC_ABS_OFF, LCC_ABS_ON, LCC_ESC_OFF, LCC_ESC_ON, LCC_LAYOUT_AWD, LCC_LAYOUT_FWD,
        LCC_LAYOUT_RWD, LCC_TC_OFF, LCC_TC_ON, VERSION,
    },
    util::{lerp, to_screen_point},
};
use eframe::{
    egui,
    egui::{vec2, Align2, Color32, Painter, Pos2, Rect, RichText, Stroke},
};
use egui_extras::{Size, StripBuilder};
use egui_plot::{HLine, Legend, Line, Plot, PlotPoints};
use glam::{Mat2, Vec2};
use std::{f32::consts::PI, time::Instant};

#[derive(Clone, Copy)]
pub struct SkidSegment {
    pub p0: Vec2,
    pub p1: Vec2,
    pub strength: f32,
    pub ttl: f32,
}

pub struct Plots {
    pub rpm: Vec<(f32, f32)>,
    pub speed: Vec<(f32, f32)>,
    pub wheel_omega: [Vec<(f32, f32)>; 4],
    pub slip_ratios: [Vec<(f32, f32)>; 4],
    pub yaw_rate: Vec<(f32, f32)>,
    pub start_time: Instant,
    pub max_len: usize,
}
impl Plots {
    pub fn new() -> Self {
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
    pub fn t(&self) -> f32 {
        (Instant::now() - self.start_time).as_secs_f32()
    }
    pub fn push(
        &mut self,
        rpm: f32,
        speed_kmh: f32,
        omega: [f32; 4],
        slip: [f32; 4],
        yaw_rate: f32,
    ) {
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

// App impl block for UI methods

impl App {
    pub fn draw_world(&mut self, ui: &mut egui::Ui) {
        let rect = ui.available_rect_before_wrap(); // Get rect for drawing bounds
        let painter = ui.painter_at(rect);
        let center = rect.center();

        if !self.follow_camera {
            let input = ui.input(|i| i.clone()); // Clone input state
                                                 // Check if dragging within the panel's current clip rectangle
            if input.pointer.primary_down() && ui.rect_contains_pointer(rect) {
                let delta = input.pointer.delta();
                if delta != egui::Vec2::ZERO {
                    let world_delta_x = delta.x / self.zoom;
                    let world_delta_y = -delta.y / self.zoom;
                    self.camera_pos.x -= world_delta_x;
                    self.camera_pos.y -= world_delta_y;
                }
            }
        }

        draw_grid(&painter, rect, self.camera_pos, self.zoom);

        if self.selected_track > 0 {
            let track = &self.tracks[self.selected_track - 1];
            let to_screen = |w: Vec2| -> Pos2 {
                let rel = w - self.camera_pos;
                let screen = Vec2::new(rel.x * self.zoom, -rel.y * self.zoom);
                Pos2::new(center.x + screen.x, center.y + screen.y)
            };

            let half_w = track.width * 0.5;
            let len = track.points.len();

            if len > 2 {
                let track_color = Color32::from_gray(80);
                let line_color = Color32::from_gray(200);

                let mut vtx_left = Vec::with_capacity(len);
                let mut vtx_right = Vec::with_capacity(len);

                for i in 0..len {
                    let p_curr = track.points[i];
                    // Calculate 't' corresponding to this point index
                    let t = (i as f32 / len as f32) * track.length;
                    let normal = track.get_normal(t); // Use the track's normal calculation

                    vtx_left.push(egui::epaint::Vertex {
                        pos: to_screen(p_curr + normal * half_w),
                        uv: egui::Pos2::ZERO,
                        color: track_color,
                    });
                    vtx_right.push(egui::epaint::Vertex {
                        pos: to_screen(p_curr - normal * half_w),
                        uv: egui::Pos2::ZERO,
                        color: track_color,
                    });
                }

                let mut mesh = egui::epaint::Mesh::default();
                for i in 0..len {
                    let next_i = (i + 1) % len;

                    let v_left_curr = vtx_left[i];
                    let v_right_curr = vtx_right[i];
                    let v_left_next = vtx_left[next_i];
                    let v_right_next = vtx_right[next_i];

                    let base_idx = mesh.vertices.len() as u32;
                    mesh.vertices.extend_from_slice(&[
                        v_left_curr,
                        v_right_curr,
                        v_left_next,
                        v_right_next,
                    ]);

                    mesh.add_triangle(base_idx, base_idx + 1, base_idx + 2);
                    mesh.add_triangle(base_idx + 1, base_idx + 3, base_idx + 2);
                }
                painter.add(egui::Shape::Mesh(mesh.into()));

                let mut left_line_points: Vec<Pos2> = vtx_left.iter().map(|v| v.pos).collect();
                if len > 0 {
                    left_line_points.push(vtx_left[0].pos);
                }

                let mut right_line_points: Vec<Pos2> = vtx_right.iter().map(|v| v.pos).collect();
                if len > 0 {
                    right_line_points.push(vtx_right[0].pos);
                }

                let white_line = Stroke::new(1.5, line_color);
                painter.add(egui::Shape::line(left_line_points, white_line));
                painter.add(egui::Shape::line(right_line_points, white_line));
            }

            for (i, (p1, p2)) in track.checkpoints.iter().enumerate() {
                let color = if i == self.current_checkpoint && self.lap_start_time.is_some() {
                    Color32::LIGHT_BLUE
                } else {
                    Color32::from_rgb(200, 200, 0)
                };
                painter.line_segment(
                    [to_screen(*p1), to_screen(*p2)],
                    Stroke::new(2.0, color.gamma_multiply(0.7)),
                );
            }
            let sf_p1 = to_screen(track.start_finish.0);
            let sf_p2 = to_screen(track.start_finish.1);
            painter.line_segment([sf_p1, sf_p2], Stroke::new(3.0, Color32::LIGHT_GREEN));
        }

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

    pub fn draw_config_editor(&mut self, ui: &mut egui::Ui) {
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

    pub fn draw_side_panel(&mut self, ui: &mut egui::Ui) {
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

        egui::ComboBox::from_label("Track")
            .selected_text(if self.selected_track == 0 {
                "None"
            } else {
                self.tracks[self.selected_track - 1].name
            })
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut self.selected_track, 0, "None");
                for i in 0..self.tracks.len() {
                    // Use (i+1) as the ID so 0 can be "None"
                    ui.selectable_value(&mut self.selected_track, i + 1, self.tracks[i].name);
                }
            });

        if ui.button("Apply Preset & Track").clicked() {
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
        ui.add(egui::Slider::new(&mut self.zoom, 2.0..=120.0).text("Zoom"));

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
        ui.collapsing("Lap Times", |ui| {
            if self.selected_track == 0 {
                ui.label("Select a track to start timing.");
            } else {
                let track = &self.tracks[self.selected_track - 1];
                // Show current lap time
                if let Some(start) = self.lap_start_time {
                    let current_lap_time = start.elapsed().as_secs_f32();
                    ui.label(RichText::new(format!("Current: {:.3}s", current_lap_time)).strong());
                } else {
                    ui.label("Cross start line to begin...");
                }
                // Show checkpoint progress
                ui.label(format!(
                    "Checkpoints: {} / {}",
                    self.current_checkpoint,
                    track.checkpoints.len()
                ));

                ui.separator();
                ui.label("Completed Laps:");
                // Show best lap
                if let Some(best) = self
                    .lap_times
                    .iter()
                    .min_by(|a, b| a.partial_cmp(b).unwrap())
                {
                    if *best > 0.0 {
                        ui.label(
                            RichText::new(format!("Best: {:.3}s", best))
                                .color(Color32::LIGHT_YELLOW),
                        );
                    }
                }
                // Show lap history
                egui::ScrollArea::vertical()
                    .max_height(100.0)
                    .show(ui, |ui| {
                        if self.lap_times.is_empty() {
                            ui.label("No laps completed.");
                        }
                        // Show all laps in reverse order
                        for (i, time) in self.lap_times.iter().enumerate().rev() {
                            ui.label(format!("Lap {}: {:.3}s", i + 1, time));
                        }
                    });
                if ui.button("Clear Laps").clicked() {
                    self.lap_times.clear();
                    self.lap_start_time = None;
                    self.current_checkpoint = 0;
                }
            }
        });

        ui.separator();
        ui.collapsing("Controls", |ui| {
            ui.monospace(
                "a/d = steer\n\
                 w/s = shift up/down\n\
                 k = throttle\n\
                 j = brake\n\
                 h = clutch\n\
                 c = handbrake\n\
                 r = reset\n\
                 Space = hold START (ign ON)",
            );
            ui.monospace(
                "\nGamepad/Wheel: \n\
                 Steer: L-Stick X / Wheel\n\
                 Gas: R-Trigger / Throttle Pedal\n\
                 Brake: L-Trigger / Brake Pedal\n\
                 Clutch: 'A' (South) / Clutch Pedal\n\
                 Shift Up: R-Bumper (RB) / Paddle\n\
                 Shift Down: L-Bumper (LB) / Paddle\n\
                 Handbrake: 'B' (East)\n\
                 Start: 'Start' Button\n\
                 Reset: 'Select' (Back) Button",
            );
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

    pub fn draw_bottom_panel(&mut self, ui: &mut egui::Ui) {
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

    pub fn draw_hud(&self, ui: &mut egui::Ui) {
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
