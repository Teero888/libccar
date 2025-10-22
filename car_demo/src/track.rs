use glam::Vec2;
use splines::{Interpolation, Key, Spline};

#[derive(Clone)]
pub struct Track {
    pub name: &'static str,
    pub points: Vec<Vec2>,
    pub length: f32,
    pub width: f32,
    pub start_finish: (Vec2, Vec2),
    pub checkpoints: Vec<(Vec2, Vec2)>,
    spline: Spline<f32, Vec2>,
}

impl Track {
    pub fn get_direction(&self, t: f32) -> Vec2 {
        let t_norm = t % self.length; // Ensure t wraps around the track length
        let delta = 0.1;

        let t_before = if t_norm < delta {
            self.length - (delta - t_norm)
        } else {
            t_norm - delta
        };
        let t_after = if t_norm > self.length - delta {
            (t_norm + delta) - self.length
        } else {
            t_norm + delta
        };

        let p_before = self.spline.sample(t_before).unwrap_or(Vec2::ZERO);
        let p_after = self.spline.sample(t_after).unwrap_or(Vec2::ZERO);

        (p_after - p_before).normalize_or_zero()
    }

    // Calculates the perpendicular normal vector of the track at a given distance 't'.
    pub fn get_normal(&self, t: f32) -> Vec2 {
        let dir = self.get_direction(t);
        Vec2::new(-dir.y, dir.x) // Perpendicular
    }

    // Gets the world position on the centerline at a given distance 't'.
    pub fn get_position(&self, t: f32) -> Vec2 {
        let t_norm = t % self.length;
        self.spline.sample(t_norm).unwrap_or(Vec2::ZERO)
    }
}

// Returns a list of all predefined tracks.
pub fn get_predefined_tracks() -> Vec<Track> {
    vec![track_oval(), track_road_course(), track_complex()]
}

// Helper function to generate a complete Track from a set of control points.
fn create_track_from_control_points(
    name: &'static str,
    control_points: Vec<Vec2>,
    width: f32,
    num_checkpoints: usize,
    sample_resolution_m: f32,
) -> Track {
    if control_points.len() < 3 {
        panic!("Track requires at least 3 control points.");
    }

    let n = control_points.len();
    let mut keys = Vec::new();
    let mut cumulative_distance = 0.0;

    let ghost_start_dist = control_points[n - 1].distance(control_points[0]);
    keys.push(Key::new(
        -ghost_start_dist,
        control_points[n - 1],
        Interpolation::CatmullRom,
    ));

    keys.push(Key::new(0.0, control_points[0], Interpolation::CatmullRom)); // Real start point at t=0
    for i in 1..n {
        let p1 = control_points[i - 1];
        let p2 = control_points[i];
        cumulative_distance += p1.distance(p2);
        keys.push(Key::new(cumulative_distance, p2, Interpolation::CatmullRom));
    }

    // Add the start point again to close the loop mathematically for the spline
    let dist_to_start = control_points[n - 1].distance(control_points[0]);
    cumulative_distance += dist_to_start;
    keys.push(Key::new(
        cumulative_distance,
        control_points[0],
        Interpolation::CatmullRom,
    ));

    let actual_track_length = cumulative_distance;

    // Add the second point as a ghost point after the end
    let ghost_end_dist = control_points[0].distance(control_points[1]);
    keys.push(Key::new(
        cumulative_distance + ghost_end_dist,
        control_points[1],
        Interpolation::CatmullRom,
    ));

    let spline = Spline::from_iter(keys);
    let half_w = width * 0.5;

    let mut sampled_centerline = Vec::new();
    let num_samples = (actual_track_length / sample_resolution_m).ceil() as usize;
    for i in 0..num_samples {
        // Loop up to num_samples (exclusive)
        let t = (i as f32 / num_samples as f32) * actual_track_length; // Use actual length
        if let Some(pos) = spline.sample(t) {
            sampled_centerline.push(pos);
        }
    }

    // We need this because get_normal/get_position depend on the final track length and spline.
    let temp_track = Track {
        name,
        points: vec![],              // points filled later
        length: actual_track_length, // This value is now correct
        width,
        start_finish: (Vec2::ZERO, Vec2::ZERO), // filled later
        checkpoints: vec![],                    // filled later
        spline: spline.clone(),                 // Clone spline for the final struct
    };

    let start_pos = temp_track.get_position(0.0);
    let start_normal = temp_track.get_normal(0.0);
    let start_finish = (
        start_pos - start_normal * half_w,
        start_pos + start_normal * half_w,
    );

    let mut checkpoints = Vec::new();
    let checkpoint_spacing = actual_track_length / (num_checkpoints + 1) as f32;
    for i in 1..=num_checkpoints {
        let t = checkpoint_spacing * i as f32;
        let pos = temp_track.get_position(t);
        let normal = temp_track.get_normal(t);
        let cp = (pos - normal * half_w, pos + normal * half_w);
        checkpoints.push(cp);
    }

    Track {
        name,
        points: sampled_centerline,
        length: actual_track_length,
        width,
        start_finish,
        checkpoints,
        spline,
    }
}

fn track_oval() -> Track {
    let shift = Vec2::new(5.0, -75.0);
    let control_points = vec![
        Vec2::new(0.0, 75.0) + shift,
        Vec2::new(150.0, 75.0) + shift,
        Vec2::new(150.0, -75.0) + shift,
        Vec2::new(0.0, -75.0) + shift,
        Vec2::new(-150.0, -75.0) + shift,
        Vec2::new(-150.0, 75.0) + shift,
    ];
    create_track_from_control_points("Spline Oval", control_points, 10.0, 8, 1.0)
}

fn track_road_course() -> Track {
    let shift = Vec2::new(10.0, 0.0);
    let control_points = vec![
        Vec2::new(0.0, 0.0) + shift,
        Vec2::new(150.0, 0.0) + shift,
        Vec2::new(200.0, 50.0) + shift,
        Vec2::new(200.0, 100.0) + shift,
        Vec2::new(150.0, 150.0) + shift,
        Vec2::new(50.0, 150.0) + shift,
        Vec2::new(0.0, 100.0) + shift,
    ];
    create_track_from_control_points("Spline Course", control_points, 8.0, 10, 1.0)
}

fn track_complex() -> Track {
    let shift = Vec2::new(10.0, 0.0);
    let control_points = vec![
        Vec2::new(0.0, 0.0) + shift,
        Vec2::new(100.0, 0.0) + shift,
        Vec2::new(180.0, 80.0) + shift,
        Vec2::new(150.0, 200.0) + shift,
        Vec2::new(50.0, 250.0) + shift,
        Vec2::new(-100.0, 200.0) + shift,
        Vec2::new(-150.0, 100.0) + shift,
        Vec2::new(-100.0, 50.0) + shift,
        Vec2::new(-50.0, 100.0) + shift,
    ];
    create_track_from_control_points("Complex Loop", control_points, 8.0, 15, 0.5)
}
