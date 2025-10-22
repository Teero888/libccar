use eframe::egui::Pos2;
use glam::Vec2;

pub fn to_screen_point(w: Vec2, cam: Vec2, zoom: f32, center: Pos2) -> Pos2 {
    let rel = w - cam;
    let screen = Vec2::new(rel.x * zoom, -rel.y * zoom);
    Pos2::new(center.x + screen.x, center.y + screen.y)
}

pub fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t.clamp(0.0, 1.0)
}
pub fn linear_approach(current: f32, target: f32, rate: f32, dt: f32) -> f32 {
    let diff = target - current;
    let step = rate * dt;
    if diff.abs() <= step {
        target
    } else {
        current + diff.signum() * step
    }
}
