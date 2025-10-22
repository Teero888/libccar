use crate::{app::App, ffi_bindings::ffi, util::linear_approach};
use eframe::egui;

pub struct InputState {
    pub throttle: f32,
    pub brake: f32,
    pub steer: f32,
    pub clutch: f32,
    pub handbrake: f32,
    pub last_w: bool,
    pub last_s: bool,
}
impl InputState {
    pub fn new() -> Self {
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

impl App {
    pub unsafe fn handle_input(&mut self, ctx: &egui::Context, dt: f32) {
        let mut steer_wheel: f32 = 0.0;
        let mut throttle_wheel: f32 = 0.0;
        let mut brake_wheel: f32 = 0.0;
        let mut clutch_wheel: f32 = 0.0;
        let mut shift_up_pressed = false;
        let mut shift_down_pressed = false;

        // Gamepad/Steering Wheel Input
        if let (Some(gilrs), Some(gamepad_id)) = (self.gilrs.as_ref(), self.active_gamepad) {
            if let Some(gamepad) = gilrs.connected_gamepad(gamepad_id) {
                // Steering
                steer_wheel = gamepad.value(gilrs::Axis::LeftStickX);

                // Throttle
                throttle_wheel = gamepad
                    .button_data(gilrs::Button::RightTrigger2)
                    .map_or(0.0, |data| data.value());
                // Brake
                brake_wheel = gamepad
                    .button_data(gilrs::Button::LeftTrigger2)
                    .map_or(0.0, |data| data.value());

                // Clutch
                if gamepad.is_pressed(gilrs::Button::South) {
                    clutch_wheel = 1.0;
                }

                // Shifting
                shift_up_pressed = gamepad.is_pressed(gilrs::Button::RightTrigger);
                shift_down_pressed = gamepad.is_pressed(gilrs::Button::LeftTrigger);
            }
        }

        // Keyboard Input (as fallback)
        let input = ctx.input(|i| i.clone());

        // Combine Inputs

        // Steering: Direct from wheel, smoothed from keyboard
        let target_steer_key = if input.key_down(egui::Key::A) {
            -1.0
        } else if input.key_down(egui::Key::D) {
            1.0
        } else {
            0.0
        };

        if steer_wheel.abs() > 0.05 {
            // Deadzone
            self.input.steer = steer_wheel; // Direct input from wheel
        } else {
            // Smoothed input from keyboard
            self.input.steer = linear_approach(self.input.steer, target_steer_key, 10.0, dt);
        }

        // Throttle: Smoothed from wheel or keyboard
        let target_throttle = if throttle_wheel > 0.01 {
            throttle_wheel
        } else if input.key_down(egui::Key::K) {
            1.0
        } else {
            0.0
        };
        self.input.throttle = linear_approach(self.input.throttle, target_throttle, 3.0, dt);

        // Brake: Smoothed from wheel or keyboard
        let target_brake = if brake_wheel > 0.01 {
            brake_wheel
        } else if input.key_down(egui::Key::J) {
            1.0
        } else {
            0.0
        };
        self.input.brake = linear_approach(self.input.brake, target_brake, 4.0, dt);

        // Clutch: Smoothed from wheel or keyboard
        let target_clutch = if clutch_wheel > 0.01 {
            clutch_wheel
        } else if input.key_down(egui::Key::H) {
            1.0
        } else {
            0.0
        };
        self.input.clutch = linear_approach(self.input.clutch, target_clutch, 8.0, dt);

        // Handbrake: Keyboard only for simplicity
        let handbrake_down = input.key_down(egui::Key::C);
        self.input.handbrake = if handbrake_down { 1.0 } else { 0.0 };

        // Shifting: Combine gamepad and keyboard
        let w = input.key_pressed(egui::Key::W);
        let s = input.key_pressed(egui::Key::S);

        if (w || shift_up_pressed) && !self.input.last_w {
            let _ = ffi::lcc_car_shift_up(self.car);
        }
        if (s || shift_down_pressed) && !self.input.last_s {
            let _ = ffi::lcc_car_shift_down(self.car);
        }
        self.input.last_w = w || shift_up_pressed;
        self.input.last_s = s || shift_down_pressed;

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
}
