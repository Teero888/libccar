use eframe::egui::{self, Pos2};
use libloading::Library;
use std::collections::HashMap;
use std::ffi::CStr;
use std::os::raw::{c_char, c_float};
use std::path::Path;
use std::time::{Duration, Instant};

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
    max_steer_angle: c_float,
    engine_power: c_float,
    brake_force: c_float,
    wheel_mass: c_float,
    cornering_stiffness: c_float,
    lateral_friction_scale: c_float,
}

impl Default for LccCar {
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

struct InputState {
    throttle: f32,
    brake: f32,
    steering: f32,
    keys_pressed: HashMap<egui::Key, bool>,
}

impl Default for InputState {
    fn default() -> Self {
        Self {
            throttle: 0.0,
            brake: 0.0,
            steering: 0.0,
            keys_pressed: HashMap::new(),
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
enum CameraMode {
    Free,
    Follow,
}

struct CarSimApp {
    lib: Option<Library>,
    lib_path: String,
    car: LccCar,
    input: InputState,
    version: String,
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
                self.lib = Some(lib);

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

        self.input.throttle = if is_pressed(&egui::Key::ArrowUp, &egui::Key::W) {
            1.0
        } else {
            0.0
        };
        self.input.brake = if is_pressed(&egui::Key::ArrowDown, &egui::Key::S) {
            1.0
        } else {
            0.0
        };

        let mut steering = 0.0;
        if is_pressed(&egui::Key::ArrowLeft, &egui::Key::A) {
            steering -= 1.0;
        }
        if is_pressed(&egui::Key::ArrowRight, &egui::Key::D) {
            steering += 1.0;
        }
        self.input.steering = steering;
    }

    fn update_car_physics(&mut self, dt: f32) {
        if let Some(lib) = &self.lib {
            if let Ok(set_input) = unsafe {
                lib.get::<unsafe extern "C" fn(*mut LccCar, c_float, c_float, c_float)>(
                    b"lcc_car_set_input\0",
                )
            } {
                unsafe {
                    set_input(
                        &mut self.car,
                        self.input.throttle,
                        self.input.brake,
                        self.input.steering,
                    );
                }
            }

            if let Ok(update) = unsafe {
                lib.get::<unsafe extern "C" fn(*mut LccCar, c_float)>(b"lcc_car_update\0")
            } {
                unsafe {
                    update(&mut self.car, dt);
                }
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

        for (_i, wheel) in self.car.wheels.iter().enumerate() {
            let wheel_local_pos = egui::vec2(wheel.position[0], wheel.position[1]);
            let rotated_offset = rot * wheel_local_pos;
            let wheel_screen_pos = car_screen_pos + rotated_offset * self.zoom;

            let static_wheel_load = (self.car.mass * 9.81) as f32;
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

            let total_steer_angle = self.car.angle + wheel.steer_angle;
            let line_end = wheel_screen_pos
                + egui::vec2(total_steer_angle.cos(), -total_steer_angle.sin())
                    * wheel_radius_screen
                    * 1.5;
            painter.line_segment(
                [wheel_screen_pos, line_end],
                egui::Stroke::new(2.0, egui::Color32::from_rgb(20, 20, 20)),
            );
        }

        if self.show_forces {
            let vel_vec = egui::vec2(self.car.velocity[0], self.car.velocity[1]) * 0.1 * self.zoom;
            painter.arrow(
                car_screen_pos,
                egui::vec2(vel_vec.x, -vel_vec.y),
                egui::Stroke::new(2.0, egui::Color32::CYAN),
            );
        }
    }

    fn draw_side_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Car Simulation");
        ui.label(format!("Library version: {}", self.version));
        if let Some(error) = &self.error {
            ui.colored_label(egui::Color32::RED, error);
        }

        ui.separator();

        ui.collapsing("Controls", |ui| {
            ui.label("WASD/Arrows: Drive");
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
            ui.checkbox(&mut self.show_forces, "Show Velocity Vector");
            ui.add(egui::Slider::new(&mut self.max_trail_points, 10..=2000).text("Trail Length"));
        })
        .header_response
        .context_menu(|ui| {
            if ui.button("Reset Visualization").clicked() {
                self.show_forces = false;
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
                ui.label("Inputs:");
                ui.label(format!("  Throttle: {:.2}", self.car.throttle));
                ui.label(format!("  Brake: {:.2}", self.car.brake));
                ui.label(format!("  Steering: {:.2}", self.car.steering));
            });

            ui.separator();

            ui.collapsing("Physics Parameters", |ui| {
                ui.add(egui::Slider::new(&mut self.wheel_radius, 0.1..=1.0).text("wheel radius"));
                ui.add(egui::Slider::new(&mut self.wheel_grip, 0.0..=4.0).text("wheel grip"));
                ui.add(egui::Slider::new(&mut self.mass, 200.0..=2000.0).text("mass (kg)"));
                ui.add(egui::Slider::new(&mut self.wheel_base, 1.5..=4.0).text("wheelbase (m)"));
                ui.add(egui::Slider::new(&mut self.track_width, 1.0..=2.5).text("Track Width (m)"));
                ui.add(
                    egui::Slider::new(&mut self.engine_power, 1_000.0..=1000_000.0)
                        .text("Engine Power (W)"),
                );
                if ui.button("Apply to library").clicked() {
                    self.reset_car_to_defaults();
                    self.trail_points.clear();
                }
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
        });

        if self.auto_reload && self.last_reload.elapsed() >= self.reload_interval {
            self.try_load_library();
            self.last_reload = Instant::now();
        }

        self.update_input(ctx);
        if !self.paused && self.lib.is_some() {
            let now = Instant::now();
            let dt = (now - self.last_update).as_secs_f32().min(1.0 / 60.0);
            self.update_car_physics(dt);
            self.last_update = now;
        }

        match self.camera_mode {
            CameraMode::Follow => {
                self.camera_pos = egui::pos2(self.car.position[0], self.car.position[1]);
            }
            CameraMode::Free => {}
        }

        egui::SidePanel::right("side_panel")
            .resizable(true)
            .default_width(300.0)
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
