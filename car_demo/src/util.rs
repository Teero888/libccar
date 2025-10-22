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

#[inline(always)]
fn ccw(a: Vec2, b: Vec2, c: Vec2) -> i32 {
    let val = (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y);
    if val.abs() < 1e-6 {
        0 // collinear
    } else if val > 0.0 {
        1 // clockwise
    } else {
        -1 // counter-clockwise
    }
}

pub fn line_segments_intersect(p1: Vec2, q1: Vec2, p2: Vec2, q2: Vec2) -> bool {
    let o1 = ccw(p1, q1, p2);
    let o2 = ccw(p1, q1, q2);
    let o3 = ccw(p2, q2, p1);
    let o4 = ccw(p2, q2, q1);
    if o1 != o2 && o3 != o4 {
        return true;
    }
    let on_segment = |p: Vec2, q: Vec2, r: Vec2| -> bool {
        q.x <= p.x.max(r.x)
            && q.x >= p.x.min(r.x)
            && q.y <= p.y.max(r.y)
            && q.y >= p.y.min(r.y)
    };
    if o1 == 0 && on_segment(p1, p2, q1) {
        return true;
    }
    if o2 == 0 && on_segment(p1, q2, q1) {
        return true;
    }
    if o3 == 0 && on_segment(p2, p1, q2) {
        return true;
    }
    if o4 == 0 && on_segment(p2, q1, q2) {
        return true;
    }
    false
}