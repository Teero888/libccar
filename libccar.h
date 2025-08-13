#ifndef LIBCCAR_H
#define LIBCCAR_H

/*
  libccar — simple 2D top-down car simulation

  What this does
  --------------
  - Single-file C99 library (header + implementation) for quick experiments.
  - Rear-wheel drive with throttle, brake, and steering inputs.
  - Linear lateral tire model with basic longitudinal slip.
  - Load transfer from longitudinal/lateral acceleration and CG height.
  - Aerodynamic drag, wheel inertias, and simple substepped integration.
  - Coordinates/units are SI-ish (meters, seconds, kg, N).

  License — "Do-What-You-Want (except claim it's yours) License"
  ----------------------------------------------------------------
  Copyright (c) 2025 Teero

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

    1) You must not misrepresent the origin of the Software. You may not claim
       that you wrote the original Software or that it is your own creation.
    2) This notice must be included in all copies or substantial portions
       of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define LCC_VERSION "0.3.2"
#define LCC_PI      3.14159265358979323846f
#define LCC_GRAVITY 9.81f

typedef struct {
  float radius;
  float angular_velocity;
  float steer_angle;
  float grip;
  float position[2];
  float load;
  float slip_angle;
  float slip_ratio;
} lcc_wheel_t;

typedef struct {
  lcc_wheel_t wheels[4];
  float       position[2];
  float       velocity[2];
  float       prev_velocity[2];
  float       throttle;
  float       brake;
  float       steering;
  float       angle;
  float       angular_velocity;
  float       mass;
  float       inertia;
  float       wheelbase;
  float       track_width;
  float       cg_height;
  float       cg_position;
  float       drag_coeff;
  float       frontal_area;
  float       max_steer_angle;
  float       engine_power;
  float       brake_force;
  float       wheel_mass;
  float       cornering_stiffness;
  float       lateral_friction_scale;
} lcc_car_t;

typedef struct {
  float longitudinal;
  float lateral;
  float torque;
} lcc_wheel_force_t;

lcc_car_t   lcc_car_init(float wheel_radius, float wheel_grip, float mass, float wheelbase, float track_width, float engine_power);
void        lcc_car_set_input(lcc_car_t *car, float throttle, float brake, float steering);
void        lcc_car_update(lcc_car_t *car, float dt);
const char *lcc_get_version(void);
#define LIBCCAR_IMPLEMENTATION
#ifdef LIBCCAR_IMPLEMENTATION

static float lcc_clamp(float v, float a, float b) {
  if (v < a)
    return a;
  if (v > b)
    return b;
  return v;
}

static float lcc_len2(const float v[2]) { return v[0] * v[0] + v[1] * v[1]; }
static float lcc_length(const float v[2]) { return sqrtf(lcc_len2(v)); }

static void lcc_rotate_vec(float v[2], float ang) {
  float c = cosf(ang), s = sinf(ang);
  float x = v[0] * c - v[1] * s;
  float y = v[0] * s + v[1] * c;
  v[0] = x;
  v[1] = y;
}

static float lcc_calc_slip_angle(const lcc_car_t *car, int wi) {
  const lcc_wheel_t *w = &car->wheels[wi];
  float              wp[2] = {w->position[0], w->position[1]};
  lcc_rotate_vec(wp, car->angle);
  float wheel_vx = car->velocity[0] - car->angular_velocity * wp[1];
  float wheel_vy = car->velocity[1] + car->angular_velocity * wp[0];
  float wheel_world_angle = car->angle + w->steer_angle;
  float longitudinal = wheel_vx * cosf(wheel_world_angle) + wheel_vy * sinf(wheel_world_angle);
  float lateral = -wheel_vx * sinf(wheel_world_angle) + wheel_vy * cosf(wheel_world_angle);
  if (fabsf(longitudinal) < 0.01f && fabsf(lateral) < 0.01f)
    return 0.0f;
  return atan2f(lateral, longitudinal);
}

static float lcc_calc_slip_ratio(const lcc_car_t *car, int wi) {
  const lcc_wheel_t *w = &car->wheels[wi];
  float              wp[2] = {w->position[0], w->position[1]};
  lcc_rotate_vec(wp, car->angle);
  float wheel_vx = car->velocity[0] - car->angular_velocity * wp[1];
  float wheel_vy = car->velocity[1] + car->angular_velocity * wp[0];
  float wheel_world_angle = car->angle + w->steer_angle;
  float v_long = wheel_vx * cosf(wheel_world_angle) + wheel_vy * sinf(wheel_world_angle);
  float tire_speed = w->angular_velocity * w->radius;
  float eps = 0.1f;
  float denom = fmaxf(fabsf(v_long), fabsf(tire_speed));
  denom = fmaxf(denom, eps);
  float r = (tire_speed - v_long) / denom;
  return lcc_clamp(r, -5.0f, 5.0f);
}

static float lcc_lateral_force_linear(float cornering_coeff_perN, float alpha, float load, float mu, float lateral_scale) {
  float K = cornering_coeff_perN * load;
  float Fy = -K * alpha * lateral_scale;
  float maxFy = mu * load;
  if (Fy > maxFy)
    Fy = maxFy;
  if (Fy < -maxFy)
    Fy = -maxFy;
  return Fy;
}

static float lcc_longitudinal_force_simple(float slip_ratio, float load, float mu) {
  const float base_long_stiffness = 15.0f * load;
  float       Fx = base_long_stiffness * slip_ratio;
  float       maxFx = mu * load;
  if (Fx > maxFx)
    Fx = maxFx;
  if (Fx < -maxFx)
    Fx = -maxFx;
  return Fx;
}

static void lcc_compute_loads(lcc_car_t *car, const float accel_world[2]) {
  float total_load = car->mass * LCC_GRAVITY;
  float front_axle_static = total_load * (1.0f - car->cg_position);
  float rear_axle_static = total_load * car->cg_position;
  float front_per_wheel = front_axle_static * 0.5f;
  float rear_per_wheel = rear_axle_static * 0.5f;
  float long_transfer = car->mass * accel_world[0] * car->cg_height / car->wheelbase;
  float lat_transfer = car->mass * accel_world[1] * car->cg_height / car->track_width;
  car->wheels[0].load = front_per_wheel - 0.5f * long_transfer - 0.5f * lat_transfer;
  car->wheels[1].load = front_per_wheel - 0.5f * long_transfer + 0.5f * lat_transfer;
  car->wheels[2].load = rear_per_wheel + 0.5f * long_transfer - 0.5f * lat_transfer;
  car->wheels[3].load = rear_per_wheel + 0.5f * long_transfer + 0.5f * lat_transfer;
  for (int i = 0; i < 4; ++i) {
    if (car->wheels[i].load < 0.0f)
      car->wheels[i].load = 0.0f;
  }
}

lcc_car_t lcc_car_init(float wheel_radius, float wheel_grip, float mass, float wheelbase, float track_width, float engine_power) {
  lcc_car_t car;
  if (wheel_grip <= 0.0f)
    wheel_grip = 1.1f;
  for (int i = 0; i < 4; ++i) {
    car.wheels[i].radius = wheel_radius;
    car.wheels[i].angular_velocity = 0.0f;
    car.wheels[i].steer_angle = 0.0f;
    car.wheels[i].grip = wheel_grip;
    car.wheels[i].position[0] = 0.0f;
    car.wheels[i].position[1] = 0.0f;
    car.wheels[i].load = 0.0f;
    car.wheels[i].slip_angle = 0.0f;
    car.wheels[i].slip_ratio = 0.0f;
  }
  float half_wb = 0.5f * wheelbase;
  float half_tw = 0.5f * track_width;
  car.wheels[0].position[0] = half_wb;
  car.wheels[0].position[1] = -half_tw;
  car.wheels[1].position[0] = half_wb;
  car.wheels[1].position[1] = half_tw;
  car.wheels[2].position[0] = -half_wb;
  car.wheels[2].position[1] = -half_tw;
  car.wheels[3].position[0] = -half_wb;
  car.wheels[3].position[1] = half_tw;
  car.position[0] = car.position[1] = 0.0f;
  car.velocity[0] = car.velocity[1] = 0.0f;
  car.prev_velocity[0] = car.prev_velocity[1] = 0.0f;
  car.throttle = car.brake = car.steering = 0.0f;

  car.angle = car.angular_velocity = 0.0f;
  car.mass = mass;
  car.inertia = mass * (wheelbase * wheelbase + track_width * track_width) / 12.0f;
  car.wheelbase = wheelbase;
  car.track_width = track_width;
  car.cg_height = 0.45f;
  car.cg_position = 0.5f;
  car.drag_coeff = 0.3f;
  car.frontal_area = 1.8f;
  car.max_steer_angle = 0.7f;
  car.engine_power = engine_power;
  car.brake_force = 20000.0f;
  car.wheel_mass = 10.0f;
  car.cornering_stiffness = 8.0f;
  car.lateral_friction_scale = 1.0f;
  float static_load_front = car.mass * LCC_GRAVITY * (1.0f - car.cg_position) * 0.5f;
  float static_load_rear = car.mass * LCC_GRAVITY * car.cg_position * 0.5f;
  car.wheels[0].load = static_load_front;
  car.wheels[1].load = static_load_front;
  car.wheels[2].load = static_load_rear;
  car.wheels[3].load = static_load_rear;
  return car;
}

void lcc_car_set_input(lcc_car_t *car, float throttle, float brake, float steering) {
  car->throttle = lcc_clamp(throttle, -1.0f, 1.0f);
  car->brake = lcc_clamp(brake, 0.0f, 1.0f);
  car->steering = lcc_clamp(steering, -1.0f, 1.0f);
}

void lcc_car_update(lcc_car_t *car, float dt) {
  if (dt <= 0.0f)
    return;
  const float low_speed_threshold = 0.5f;
  const float wheel_omega_snap = 0.5f;
  const float max_wheel_alpha = 5000.0f;
  const float wheel_inertia_bias = 5.0f;
  const float angular_damping = 0.995f;
  const float linear_damping = 0.999f;
  const int   max_substeps = 6;
  const float target_substep_dt = 0.02f;
  int         steps = (int)ceilf(dt / target_substep_dt);
  if (steps < 1)
    steps = 1;
  if (steps > max_substeps)
    steps = max_substeps;
  float subdt = dt / (float)steps;
  float steer = -car->steering * car->max_steer_angle;
  car->wheels[0].steer_angle = steer;
  car->wheels[1].steer_angle = steer;
#define CLAMP(a, lo, hi) ((a) < (lo) ? (lo) : ((a) > (hi) ? (hi) : (a)))
  for (int s = 0; s < steps; ++s) {
    float vehicle_speed = lcc_length(car->velocity);
    float total_force[2] = {0.0f, 0.0f};
    float total_torque = 0.0f;
    for (int i = 0; i < 4; ++i) {
      lcc_wheel_t *w = &car->wheels[i];
      float        wp[2] = {w->position[0], w->position[1]};
      lcc_rotate_vec(wp, car->angle);
      float wheel_vx = car->velocity[0] - car->angular_velocity * wp[1];
      float wheel_vy = car->velocity[1] + car->angular_velocity * wp[0];

      float wheel_world_angle = car->angle + w->steer_angle;
      float local_speed = sqrtf(wheel_vx * wheel_vx + wheel_vy * wheel_vy);
      if (local_speed > 0.05f && vehicle_speed > 0.05f) {
        w->slip_angle = lcc_calc_slip_angle(car, i);
        w->slip_ratio = lcc_calc_slip_ratio(car, i);
      } else {
        w->slip_angle = 0.0f;
        w->slip_ratio = 0.0f;
      }
      float Fy = lcc_lateral_force_linear(car->cornering_stiffness, w->slip_angle, w->load, w->grip, car->lateral_friction_scale);
      float Fx_tire = lcc_longitudinal_force_simple(w->slip_ratio, w->load, w->grip);

      if (vehicle_speed < low_speed_threshold) {
        float fac = CLAMP(vehicle_speed / low_speed_threshold, 0.0f, 1.0f);
        Fy *= fac;
        Fx_tire *= fac;
      }
      float drive_force = 0.0f;
      if (i >= 2) {
        float vehicle_speed_mag = fmaxf(vehicle_speed, 0.5f);
        drive_force = (car->throttle * car->engine_power) / vehicle_speed_mag;
        if (vehicle_speed < low_speed_threshold) {
          float fac = CLAMP(vehicle_speed / low_speed_threshold, 0.1f, 1.0f);
          drive_force *= fac;
        }
      }

      float Fx_total = Fx_tire + drive_force;
      float world_Fx = Fx_total * cosf(wheel_world_angle) - Fy * sinf(wheel_world_angle);
      float world_Fy = Fx_total * sinf(wheel_world_angle) + Fy * cosf(wheel_world_angle);

      total_force[0] += world_Fx;
      total_force[1] += world_Fy;
      total_torque += wp[0] * world_Fy - wp[1] * world_Fx;

      float Iw = 0.5f * car->wheel_mass * w->radius * w->radius * wheel_inertia_bias;
      float vehicle_speed_mag = lcc_length(car->velocity);
      float speed_for_power = fmaxf(vehicle_speed_mag, 0.5f);

      float engine_force_equiv = (car->throttle * car->engine_power) / speed_for_power;
      float engine_force_on_wheel = (i >= 2) ? (engine_force_equiv * 0.5f) : 0.0f;
      float engine_torque = engine_force_on_wheel * w->radius;

      float reaction_torque = Fx_tire * w->radius;
      float raw_brake_torque = car->brake * car->brake_force * 0.25f * w->radius;
      float brake_torque = 0.0f;

      if (car->brake > 0.0f) {
        if (w->angular_velocity > 0.0f)
          brake_torque = raw_brake_torque;
        else if (w->angular_velocity < 0.0f)
          brake_torque = -raw_brake_torque;
      }
      float net_torque = engine_torque - reaction_torque - brake_torque;
      float alpha = net_torque / fmaxf(Iw, 1e-6f);
      alpha = CLAMP(alpha, -max_wheel_alpha, max_wheel_alpha);
      w->angular_velocity += alpha * subdt;
      float       v_long = wheel_vx * cosf(wheel_world_angle) + wheel_vy * sinf(wheel_world_angle);
      float       wheel_ground_speed = v_long / w->radius;
      const float coupling = 25.0f;
      float       target_omega = wheel_ground_speed;
      float       lerp_t = CLAMP(subdt * coupling, 0.0f, 1.0f);
      w->angular_velocity = w->angular_velocity * (1.0f - lerp_t) + target_omega * lerp_t;
      w->angular_velocity *= 0.998f;
      if (fabsf(w->angular_velocity) < wheel_omega_snap && car->brake > 0.1f)
        w->angular_velocity = 0.0f;

      const float maxomega = 2000.0f;
      if (w->angular_velocity > maxomega)
        w->angular_velocity = maxomega;
      if (w->angular_velocity < -maxomega)
        w->angular_velocity = -maxomega;
    }
    float speed = lcc_length(car->velocity);
    if (speed > 0.01f) {
      float drag = 0.5f * 1.225f * car->drag_coeff * car->frontal_area * speed * speed;
      float drag_dir[2] = {-car->velocity[0] / speed, -car->velocity[1] / speed};
      total_force[0] += drag * drag_dir[0];
      total_force[1] += drag * drag_dir[1];
    }
    float accel[2] = {total_force[0] / car->mass, total_force[1] / car->mass};
    car->velocity[0] += accel[0] * subdt;
    car->velocity[1] += accel[1] * subdt;
    float angular_accel = total_torque / fmaxf(car->inertia, 1e-6f);
    car->angular_velocity += angular_accel * subdt;
    car->position[0] += car->velocity[0] * subdt;
    car->position[1] += car->velocity[1] * subdt;
    car->angle += car->angular_velocity * subdt;
    car->angle = fmodf(car->angle, 2.0f * LCC_PI);
    if (car->angle < 0.0f)
      car->angle += 2.0f * LCC_PI;
    float acceleration[2] = {(car->velocity[0] - car->prev_velocity[0]) / (subdt * 8.f), (car->velocity[1] - car->prev_velocity[1]) / (subdt * 8.f)};
    lcc_compute_loads(car, acceleration);
    car->angular_velocity *= angular_damping;
    car->velocity[0] *= linear_damping;
    car->velocity[1] *= linear_damping;
    car->prev_velocity[0] = car->velocity[0];
    car->prev_velocity[1] = car->velocity[1];
  }
#undef CLAMP
}

const char *lcc_get_version(void) { return LCC_VERSION; }

#endif

#ifdef __cplusplus
}
#endif

#endif
