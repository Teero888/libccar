/*
  libccar — simple 2D top-down car simulation

  What this does
  --------------
  - Single-file C99 library (header + implementation) for quick experiments.
  - u dont deserve to know aka. im lazy. (wip)
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

#ifndef LIBCCAR_H
#define LIBCCAR_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#define LCC_VERSION     "0.3.6"
#define LCC_PI          (3.14159265358979323846f)
#define LCC_GRAVITY     (9.81f)
#define LCC_AIR_DENSITY (1.225f)
#define LCC_EPS         (1e-6f)

typedef enum { LCC_PRESET_ECONOMY, LCC_PRESET_MIDSIZE, LCC_PRESET_SPORTS, LCC_PRESET_SUPERCAR, LCC_PRESET_HYPERCAR, LCC_PRESET_TRACK, LCC_PRESET_CUSTOM } lcc_preset_t;

typedef struct {
  float radius;
  float width;
  float aspect_ratio;
  float pressure;
  float nominal_load;
  float peak_friction;
  float slip_friction;
  float stiffness;
  float cornering_stiffness;
  float camber_stiffness;
  float rolling_resistance;
  float temperature;
  float wear;
} lcc_tire_params_t;

typedef struct {
  float angular_velocity;
  float steer_angle;
  float camber_angle;
  float slip_angle;
  float slip_ratio;
  float load;
  float position[2];
  float suspension_deflection;
  float suspension_velocity;
  float temperature;
  float surface_friction;

  float Fx;
  float Fy;
  float drive_torque;
  float brake_torque;
  float rotational_inertia;
} lcc_wheel_state_t;

typedef struct {
  float max_power;
  float max_torque;
  float idle_rpm;
  float max_rpm;
  float redline_rpm;
  float inertia;
  float friction;
  float response_time;
  float current_rpm;
  float throttle;

  float output_torque;
} lcc_engine_t;

typedef struct {
  int   num_gears;
  float gear_ratios[8];
  float final_drive;
  float reverse_ratio;
  float efficiency;
  int   current_gear;
  float clutch_engagement;
  float clutch_inertia;
  float clutch_friction;

  int drive_type;
} lcc_transmission_t;

typedef struct {
  float ratio;
  float preload;
  float power_factor;
  float coast_factor;
  float inertia;
  float friction;
} lcc_differential_t;

typedef struct {
  float drag_coefficient;
  float frontal_area;
  float downforce_coefficient;
  float downforce_area;
  float lift_coefficient;
} lcc_aerodynamics_t;

typedef struct {
  float stiffness;
  float damping;
  float travel;
  float anti_roll_bar;
  float ride_height;
} lcc_suspension_t;

typedef struct {

  float mass;
  float inertia;
  float wheelbase;
  float track_width;
  float cg_height;
  float cg_position;

  float position[2];
  float velocity[2];
  float prev_velocity[2];
  float angle;
  float angular_velocity;

  lcc_engine_t       engine;
  lcc_transmission_t transmission;
  lcc_differential_t differential;
  lcc_aerodynamics_t aerodynamics;
  lcc_suspension_t   suspension;

  lcc_tire_params_t tire_params[4];
  lcc_wheel_state_t wheels[4];

  float throttle_input;
  float brake_input;
  float steering_input;
  float clutch_input;

  float front_brake_bias;
  float max_brake_torque;

  float air_density;
  float ambient_temp;
  float surface_friction;

  float timestep;
  float simulation_time;
} lcc_car_t;

lcc_car_t   lcc_car_create(lcc_preset_t preset);
void        lcc_car_destroy(lcc_car_t *car);
void        lcc_car_set_inputs(lcc_car_t *car, float throttle, float brake, float steering, float clutch);
void        lcc_car_update(lcc_car_t *car, float dt);
void        lcc_car_shift_up(lcc_car_t *car);
void        lcc_car_shift_down(lcc_car_t *car);
void        lcc_car_set_gear(lcc_car_t *car, int gear);
float       lcc_car_get_speed(const lcc_car_t *car);
float       lcc_car_get_engine_rpm(const lcc_car_t *car);
const char *lcc_get_version(void);

#define LIBCCAR_IMPLEMENTATION
#ifdef LIBCCAR_IMPLEMENTATION

static float lcc_clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static float lcc_smoothstep(float x) {
  x = lcc_clamp(x, 0.0f, 1.0f);
  return x * x * (3.0f - 2.0f * x);
}
static float lcc_lerp(float a, float b, float t) { return a + t * (b - a); }
static float lcc_sign(float x) { return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f); }
static float lcc_length(const float v[2]) { return sqrtf(v[0] * v[0] + v[1] * v[1]); }
static void  lcc_normalize2(float v[2]) {
  float len = lcc_length(v);
  if (len > LCC_EPS) {
    v[0] /= len;
    v[1] /= len;
  }
}
static void lcc_rotate2(const float v[2], float angle, float out[2]) {
  float c = cosf(angle), s = sinf(angle);
  out[0] = v[0] * c - v[1] * s;
  out[1] = v[0] * s + v[1] * c;
}

static void lcc_engine_physics(lcc_car_t *car) {
  lcc_engine_t *e = &car->engine;

  float rpm = lcc_clamp(e->current_rpm, e->idle_rpm, e->redline_rpm);
  float rpm_norm = (rpm - e->idle_rpm) / fmaxf(e->max_rpm - e->idle_rpm, 1.0f);
  rpm_norm = lcc_clamp(rpm_norm, 0.0f, 1.0f);

  const float idle_frac = 0.35f;
  float       torque_curve;
  if (rpm_norm < 0.3f) {
    torque_curve = lcc_lerp(idle_frac, 1.0f, lcc_smoothstep(rpm_norm / 0.3f));
  } else if (rpm_norm < 0.8f) {
    torque_curve = 1.0f;
  } else {
    float t = lcc_clamp((rpm_norm - 0.8f) / 0.2f, 0.0f, 1.0f);
    torque_curve = lcc_lerp(1.0f, 0.6f, t);
  }

  e->output_torque = e->max_torque * torque_curve * lcc_clamp(e->throttle, 0.0f, 1.0f);
}

static void lcc_transmission_physics(lcc_car_t *car) {
  lcc_transmission_t *t = &car->transmission;
  lcc_engine_t       *e = &car->engine;

  float target = lcc_clamp(car->clutch_input, 0.0f, 1.0f);
  float rate_up = 10.0f, rate_down = 2.0f;
  float rate = (target > t->clutch_engagement ? rate_up : rate_down);
  t->clutch_engagement = lcc_lerp(t->clutch_engagement, target, lcc_clamp(rate * car->timestep, 0.0f, 1.0f));

  float gear_ratio = 0.0f;
  if (t->current_gear > 0)
    gear_ratio = t->gear_ratios[t->current_gear - 1];
  else if (t->current_gear < 0)
    gear_ratio = -t->reverse_ratio;

  float total_ratio = gear_ratio * t->final_drive;

  int driven_mask[4] = {0, 0, 0, 0};
  int driven_count = 0;
  if (t->current_gear != 0 && t->drive_type == 0) {
    driven_mask[2] = driven_mask[3] = 1;
    driven_count = 2;
  } else if (t->current_gear != 0 && t->drive_type == 1) {
    driven_mask[0] = driven_mask[1] = 1;
    driven_count = 2;
  } else if (t->current_gear != 0 && t->drive_type == 2) {
    driven_mask[0] = driven_mask[1] = driven_mask[2] = driven_mask[3] = 1;
    driven_count = 4;
  }

  float omega_shaft = 0.0f;
  if (driven_count > 0 && fabsf(total_ratio) > LCC_EPS) {
    float omega_wheels_avg = 0.0f;
    for (int i = 0; i < 4; i++)
      if (driven_mask[i])
        omega_wheels_avg += car->wheels[i].angular_velocity;
    omega_wheels_avg /= (float)driven_count;
    omega_shaft = omega_wheels_avg * total_ratio;
  }

  float omega_e = e->current_rpm * (2.0f * LCC_PI / 60.0f);
  float engine_friction_torque = e->friction * omega_e;

  float delta_omega = omega_e - omega_shaft;
  float visc_torque = t->clutch_friction * delta_omega * t->clutch_engagement;
  float T_c = visc_torque;

  float T_eng_avail = fmaxf(e->output_torque, 0.0f);

  if (T_c > T_eng_avail)
    T_c = T_eng_avail;

  float T_engine_net = e->output_torque - engine_friction_torque - T_c;
  omega_e += (T_engine_net / fmaxf(e->inertia, 1e-3f)) * car->timestep;

  float rpm_new = omega_e * (60.0f / (2.0f * LCC_PI));
  rpm_new = lcc_clamp(rpm_new, e->idle_rpm, e->redline_rpm);
  e->current_rpm = rpm_new;
  omega_e = e->current_rpm * (2.0f * LCC_PI / 60.0f);

  for (int i = 0; i < 4; i++)
    car->wheels[i].drive_torque = 0.0f;

  if (driven_count > 0 && fabsf(total_ratio) > LCC_EPS) {
    float T_at_axle_total = T_c * total_ratio * t->efficiency;
    float per_wheel = T_at_axle_total / (float)driven_count;
    for (int i = 0; i < 4; i++)
      if (driven_mask[i])
        car->wheels[i].drive_torque += per_wheel;
  }

  float brake = lcc_clamp(car->brake_input, 0.0f, 1.0f);
  float T_brake_front = brake * car->max_brake_torque * car->front_brake_bias;
  float T_brake_rear = brake * car->max_brake_torque * (1.0f - car->front_brake_bias);
  car->wheels[0].brake_torque = T_brake_front * 0.5f;
  car->wheels[1].brake_torque = T_brake_front * 0.5f;
  car->wheels[2].brake_torque = T_brake_rear * 0.5f;
  car->wheels[3].brake_torque = T_brake_rear * 0.5f;
}

static void lcc_aerodynamics_physics(lcc_car_t *car) {
  float v_mag = lcc_length(car->velocity);
  if (v_mag > 0.05f) {
    float rho = car->air_density;
    float Cd = car->aerodynamics.drag_coefficient;
    float Af = car->aerodynamics.frontal_area;

    float F_drag = 0.5f * rho * Cd * Af * v_mag * v_mag;
    float drag_dir[2] = {-car->velocity[0], -car->velocity[1]};
    lcc_normalize2(drag_dir);

    car->velocity[0] += (F_drag * drag_dir[0] / car->mass) * car->timestep;
    car->velocity[1] += (F_drag * drag_dir[1] / car->mass) * car->timestep;
  }

  float ClA = car->aerodynamics.downforce_coefficient * car->aerodynamics.downforce_area;
  if (ClA > 0.0f) {
    float v_mag = lcc_length(car->velocity);
    float downforce = 0.5f * car->air_density * ClA * v_mag * v_mag;

    float front = downforce * 0.6f, rear = downforce * 0.4f;
    car->wheels[0].load += front * 0.5f;
    car->wheels[1].load += front * 0.5f;
    car->wheels[2].load += rear * 0.5f;
    car->wheels[3].load += rear * 0.5f;
  }
}

static void lcc_suspension_physics(lcc_car_t *car) {

  float Fz_front_static = car->mass * LCC_GRAVITY * (1.0f - car->cg_position);
  float Fz_rear_static = car->mass * LCC_GRAVITY * (car->cg_position);

  float dt = fmaxf(car->timestep, LCC_EPS);
  float a_world[2] = {(car->velocity[0] - car->prev_velocity[0]) / dt, (car->velocity[1] - car->prev_velocity[1]) / dt};

  float a_body[2];
  float neg_angle = -car->angle;
  a_body[0] = a_world[0] * cosf(neg_angle) - a_world[1] * sinf(neg_angle);
  a_body[1] = a_world[0] * sinf(neg_angle) + a_world[1] * cosf(neg_angle);

  float dF_long = car->mass * a_body[0] * car->cg_height / fmaxf(car->wheelbase, LCC_EPS);

  float dF_lat_total = car->mass * a_body[1] * car->cg_height / fmaxf(car->track_width, LCC_EPS);

  float Fz_FL = 0.5f * Fz_front_static;
  float Fz_FR = 0.5f * Fz_front_static;
  float Fz_RL = 0.5f * Fz_rear_static;
  float Fz_RR = 0.5f * Fz_rear_static;

  Fz_FL -= 0.5f * dF_long;
  Fz_FR -= 0.5f * dF_long;
  Fz_RL += 0.5f * dF_long;
  Fz_RR += 0.5f * dF_long;

  Fz_FL += 0.5f * dF_lat_total;
  Fz_RL += 0.5f * dF_lat_total;
  Fz_FR -= 0.5f * dF_lat_total;
  Fz_RR -= 0.5f * dF_lat_total;

  float k = car->suspension.stiffness;
  float c = car->suspension.damping;
  float max_def = car->suspension.travel * 0.5f;

  float Fz_static[4] = {Fz_FL, Fz_FR, Fz_RL, Fz_RR};
  for (int i = 0; i < 4; i++) {
    lcc_wheel_state_t *w = &car->wheels[i];

    float F_spring = -k * w->suspension_deflection - c * w->suspension_velocity;

    float Fz = Fz_static[i] + F_spring;
    if (Fz < 0.0f)
      Fz = 0.0f;

    float residual = (Fz_static[i] - Fz);
    float acc_defl = residual / fmaxf(k, 1.0f);
    w->suspension_velocity += acc_defl * dt;
    w->suspension_deflection += w->suspension_velocity * dt;

    if (w->suspension_deflection < -max_def) {
      w->suspension_deflection = -max_def;
      w->suspension_velocity = 0.0f;
    }
    if (w->suspension_deflection > max_def) {
      w->suspension_deflection = max_def;
      w->suspension_velocity = 0.0f;
    }

    w->load = Fz;
  }
}

static void lcc_tire_physics(lcc_car_t *car, int i) {
  lcc_wheel_state_t *w = &car->wheels[i];
  lcc_tire_params_t *t = &car->tire_params[i];

  float ca = cosf(car->angle), sa = sinf(car->angle);
  float r_world_x = w->position[0] * ca - w->position[1] * sa;
  float r_world_y = w->position[0] * sa + w->position[1] * ca;

  float wheel_vel_world[2] = {car->velocity[0] - car->angular_velocity * r_world_y, car->velocity[1] + car->angular_velocity * r_world_x};

  float wheel_angle = car->angle + w->steer_angle;
  float c = cosf(wheel_angle), s = sinf(wheel_angle);
  float v_long = wheel_vel_world[0] * c + wheel_vel_world[1] * s;
  float v_lat = -wheel_vel_world[0] * s + wheel_vel_world[1] * c;

  float omega_r = w->angular_velocity * t->radius;
  float slip_rel = omega_r - v_long;

  float T_drive = w->drive_torque;

  float Iw = fmaxf(w->rotational_inertia, 1e-6f);
  float omega_reg = fmaxf(LCC_EPS, (car->timestep * w->brake_torque) / Iw);
  float omega_abs = fabsf(w->angular_velocity);
  float brake_dir = (omega_abs > 0.0f) ? (w->angular_velocity / sqrtf(w->angular_velocity * w->angular_velocity + omega_reg * omega_reg)) : 0.0f;
  float T_brake_eff = w->brake_torque * brake_dir;

  float v_scale = sqrtf(v_long * v_long + omega_r * omega_r) + LCC_EPS;
  float kappa = (omega_r - v_long) / v_scale;
  kappa = lcc_clamp(kappa, -1.5f, 1.5f);

  float alpha = atan2f(v_lat, fabsf(v_long) + LCC_EPS);

  float mu_surface = fmaxf(w->surface_friction, 0.1f) * fmaxf(car->surface_friction, 0.1f);
  float temp_term = 1.0f - 0.002f * (w->temperature - 80.0f);
  temp_term = lcc_clamp(temp_term, 0.5f, 1.05f);
  float wear_term = lcc_clamp(1.0f - 0.6f * t->wear, 0.4f, 1.0f);
  float mu_tire = lcc_lerp(t->slip_friction, t->peak_friction, lcc_clamp(temp_term, 0.0f, 1.0f)) * wear_term;
  float mu = mu_surface * mu_tire;
  float Fz = fmaxf(w->load, 0.0f);

  float Dy = mu * Fz;
  float Cy = 1.3f;
  float By = t->cornering_stiffness / fmaxf(Cy * Dy, 1.0f);
  float Fy_pure = -Dy * sinf(Cy * atanf(By * alpha));
  Fy_pure += -t->camber_stiffness * w->camber_angle;

  float Dx = mu * Fz;
  float Cx = 1.3f;
  float Bx = t->stiffness / fmaxf(Cx * Dx, 1.0f);
  float Fx0 = Dx * sinf(Cx * atanf(Bx * kappa));

  float Fmax = mu * Fz;
  float Fy0 = Fy_pure;
  float usage = sqrtf(Fx0 * Fx0 + Fy0 * Fy0) / (Fmax + LCC_EPS);
  if (usage > 1.0f) {
    Fx0 /= usage;
    Fy0 /= usage;
  }

  float Frr = t->rolling_resistance * Fz;
  float rr_dir = -(slip_rel) / (fmaxf(fabsf(slip_rel), v_scale) + LCC_EPS);
  float rr_scale = v_scale / (v_scale + 1.0f);

  float Fx = Fx0 + rr_scale * Frr * rr_dir;
  float Fy = Fy0;

  w->Fx = Fx;
  w->Fy = Fy;
  w->slip_ratio = kappa;
  w->slip_angle = alpha;

  float T_tire = -Fx * t->radius;
  float T_net = T_drive - T_brake_eff + T_tire;
  w->angular_velocity += (T_net / Iw) * car->timestep;

  float v_slip_long = fabsf(slip_rel);
  float v_slip_lat = fabsf(v_lat);
  float slip_power = fabsf(Fx) * v_slip_long + fabsf(Fy) * v_slip_lat;

  float motion_gate = v_scale / (v_scale + 1.0f);

  const float c_heat = 3.0e-4f;
  float       k_cool = 0.05f + 0.01f * lcc_length(car->velocity);
  float       dTdt = c_heat * slip_power * motion_gate - k_cool * (w->temperature - car->ambient_temp);
  w->temperature += dTdt * car->timestep;
  w->temperature = lcc_clamp(w->temperature, car->ambient_temp, 150.0f);

  t->wear = lcc_clamp(t->wear + (slip_power * car->timestep) * 1e-7f, 0.0f, 1.0f);
}

static void lcc_vehicle_dynamics(lcc_car_t *car) {
  float total_F[2] = {0.0f, 0.0f};
  float total_Mz = 0.0f;

  for (int i = 0; i < 4; i++) {
    lcc_wheel_state_t *w = &car->wheels[i];

    lcc_tire_physics(car, i);

    float wheel_angle_world = car->angle + w->steer_angle;
    float cw = cosf(wheel_angle_world), sw = sinf(wheel_angle_world);

    float Fx_world = w->Fx * cw - w->Fy * sw;
    float Fy_world = w->Fx * sw + w->Fy * cw;

    total_F[0] += Fx_world;
    total_F[1] += Fy_world;

    float ca = cosf(car->angle), sa = sinf(car->angle);
    float Fx_body = Fx_world * ca + Fy_world * sa;
    float Fy_body = -Fx_world * sa + Fy_world * ca;

    total_Mz += w->position[0] * Fy_body - w->position[1] * Fx_body;
  }

  car->prev_velocity[0] = car->velocity[0];
  car->prev_velocity[1] = car->velocity[1];

  float ax = total_F[0] / car->mass;
  float ay = total_F[1] / car->mass;
  car->velocity[0] += ax * car->timestep;
  car->velocity[1] += ay * car->timestep;

  car->position[0] += car->velocity[0] * car->timestep;
  car->position[1] += car->velocity[1] * car->timestep;

  float yaw_acc = total_Mz / fmaxf(car->inertia, 1e-3f);
  car->angular_velocity += yaw_acc * car->timestep;
  car->angle += car->angular_velocity * car->timestep;

  while (car->angle > LCC_PI)
    car->angle -= 2.0f * LCC_PI;
  while (car->angle < -LCC_PI)
    car->angle += 2.0f * LCC_PI;

  {
    const float v_stop = 0.10f;
    const float w_stop = 0.20f;
    const float brake_min = 0.4f;

    float vmag = lcc_length(car->velocity);
    if (vmag < v_stop && car->brake_input > brake_min) {

      car->velocity[0] = car->velocity[1] = 0.0f;
      car->angular_velocity = 0.0f;

      for (int i = 0; i < 4; ++i) {
        if (fabsf(car->wheels[i].angular_velocity) < w_stop)
          car->wheels[i].angular_velocity = 0.0f;
        car->wheels[i].Fx = 0.0f;
        car->wheels[i].Fy = 0.0f;
        car->wheels[i].slip_ratio = 0.0f;
        car->wheels[i].slip_angle = 0.0f;
      }
    }
  }
}

lcc_car_t lcc_car_create(lcc_preset_t preset) {
  lcc_car_t car;

  for (size_t i = 0; i < sizeof(lcc_car_t); ++i)
    ((uint8_t *)&car)[i] = 0;

  car.mass = 1500.0f;
  car.wheelbase = 2.7f;
  car.track_width = 1.6f;
  car.cg_height = 0.5f;
  car.cg_position = 0.55f;
  car.air_density = LCC_AIR_DENSITY;
  car.ambient_temp = 20.0f;
  car.surface_friction = 1.0f;
  car.front_brake_bias = 0.65f;
  car.max_brake_torque = 8000.0f;

  for (int i = 0; i < 4; i++) {
    car.tire_params[i].radius = 0.32f;
    car.tire_params[i].width = 0.22f;
    car.tire_params[i].aspect_ratio = 0.5f;
    car.tire_params[i].pressure = 220.0f;
    car.tire_params[i].nominal_load = 3500.0f;
    car.tire_params[i].peak_friction = 1.2f;
    car.tire_params[i].slip_friction = 0.8f;
    car.tire_params[i].stiffness = 90000.0f;
    car.tire_params[i].cornering_stiffness = 120000.0f;
    car.tire_params[i].camber_stiffness = 30000.0f;
    car.tire_params[i].rolling_resistance = 0.015f;
    car.tire_params[i].temperature = car.ambient_temp;
    car.tire_params[i].wear = 0.0f;

    car.wheels[i].surface_friction = car.surface_friction;
    car.wheels[i].temperature = car.ambient_temp;
    car.wheels[i].rotational_inertia = 1.2f;
    car.wheels[i].drive_torque = 0.0f;
    car.wheels[i].brake_torque = 0.0f;
  }

  car.wheels[0].position[0] = car.wheelbase * 0.5f;
  car.wheels[0].position[1] = car.track_width * 0.5f;
  car.wheels[1].position[0] = car.wheelbase * 0.5f;
  car.wheels[1].position[1] = -car.track_width * 0.5f;
  car.wheels[2].position[0] = -car.wheelbase * 0.5f;
  car.wheels[2].position[1] = car.track_width * 0.5f;
  car.wheels[3].position[0] = -car.wheelbase * 0.5f;
  car.wheels[3].position[1] = -car.track_width * 0.5f;

  car.engine.max_power = 200000.0f;
  car.engine.max_torque = 400.0f;
  car.engine.idle_rpm = 800.0f;
  car.engine.max_rpm = 6000.0f;
  car.engine.redline_rpm = 6500.0f;
  car.engine.inertia = 0.2f;
  car.engine.friction = 0.05f;
  car.engine.response_time = 0.1f;
  car.engine.current_rpm = car.engine.idle_rpm;
  car.engine.throttle = 0.0f;
  car.engine.output_torque = 0.0f;

  car.transmission.num_gears = 6;
  car.transmission.gear_ratios[0] = 3.5f;
  car.transmission.gear_ratios[1] = 2.0f;
  car.transmission.gear_ratios[2] = 1.4f;
  car.transmission.gear_ratios[3] = 1.1f;
  car.transmission.gear_ratios[4] = 0.9f;
  car.transmission.gear_ratios[5] = 0.7f;
  car.transmission.final_drive = 3.7f;
  car.transmission.reverse_ratio = 3.2f;
  car.transmission.efficiency = 0.92f;
  car.transmission.current_gear = 0;
  car.transmission.clutch_engagement = 0.0f;
  car.transmission.clutch_inertia = 0.05f;
  car.transmission.clutch_friction = 5.0f;
  car.transmission.drive_type = 0;

  car.differential.ratio = 1.0f;
  car.differential.preload = 50.0f;
  car.differential.power_factor = 0.3f;
  car.differential.coast_factor = 0.2f;
  car.differential.inertia = 0.05f;
  car.differential.friction = 0.05f;

  car.aerodynamics.drag_coefficient = 0.3f;
  car.aerodynamics.frontal_area = 2.0f;
  car.aerodynamics.downforce_coefficient = 0.1f;
  car.aerodynamics.downforce_area = 2.0f;
  car.aerodynamics.lift_coefficient = 0.0f;

  car.suspension.stiffness = 30000.0f;
  car.suspension.damping = 3000.0f;
  car.suspension.travel = 0.15f;
  car.suspension.anti_roll_bar = 5000.0f;
  car.suspension.ride_height = 0.2f;

  car.inertia = car.mass * (car.wheelbase * car.wheelbase + car.track_width * car.track_width) / 12.0f;

  switch (preset) {
  case LCC_PRESET_ECONOMY:
    car.mass = 1350.0f;
    car.engine.max_power = 110000.0f;
    car.engine.max_torque = 200.0f;
    car.transmission.drive_type = 1;
    car.aerodynamics.drag_coefficient = 0.32f;
    break;
  case LCC_PRESET_MIDSIZE:
    car.mass = 1600.0f;
    car.engine.max_power = 190000.0f;
    car.engine.max_torque = 350.0f;
    car.transmission.drive_type = 0;
    car.aerodynamics.drag_coefficient = 0.29f;
    break;
  case LCC_PRESET_SPORTS:
    car.mass = 1450.0f;
    car.engine.max_power = 350000.0f;
    car.engine.max_torque = 500.0f;
    car.engine.max_rpm = 7000.0f;
    car.engine.redline_rpm = 7500.0f;
    car.transmission.drive_type = 0;
    car.aerodynamics.drag_coefficient = 0.28f;
    car.aerodynamics.downforce_coefficient = 0.2f;
    car.suspension.stiffness = 40000.0f;
    car.suspension.damping = 4000.0f;
    break;
  case LCC_PRESET_SUPERCAR:
    car.mass = 1400.0f;
    car.engine.max_power = 530000.0f;
    car.engine.max_torque = 700.0f;
    car.engine.max_rpm = 7500.0f;
    car.engine.redline_rpm = 8000.0f;
    car.transmission.drive_type = 0;
    car.aerodynamics.drag_coefficient = 0.32f;
    car.aerodynamics.downforce_coefficient = 0.35f;
    car.suspension.stiffness = 50000.0f;
    car.suspension.damping = 5000.0f;
    break;
  case LCC_PRESET_HYPERCAR:
    car.mass = 1950.0f;
    car.engine.max_power = 1100000.0f;
    car.engine.max_torque = 1200.0f;
    car.engine.max_rpm = 6700.0f;
    car.engine.redline_rpm = 7100.0f;
    car.transmission.drive_type = 2;
    car.aerodynamics.drag_coefficient = 0.38f;
    car.aerodynamics.downforce_coefficient = 0.25f;
    car.suspension.stiffness = 60000.0f;
    car.suspension.damping = 6000.0f;
    break;
  case LCC_PRESET_TRACK:
    car.mass = 750.0f;
    car.engine.max_power = 450000.0f;
    car.engine.max_torque = 400.0f;
    car.engine.max_rpm = 10000.0f;
    car.engine.redline_rpm = 11000.0f;
    car.transmission.drive_type = 0;
    car.aerodynamics.drag_coefficient = 0.9f;
    car.aerodynamics.downforce_coefficient = 2.5f;
    car.suspension.stiffness = 80000.0f;
    car.suspension.damping = 8000.0f;
    break;
  default:
    break;
  }

  car.inertia = car.mass * (car.wheelbase * car.wheelbase + car.track_width * car.track_width) / 12.0f;

  return car;
}

void lcc_car_destroy(lcc_car_t *car) { (void)car; }

void lcc_car_set_inputs(lcc_car_t *car, float throttle, float brake, float steering, float clutch) {
  car->throttle_input = lcc_clamp(throttle, -1.0f, 1.0f);
  car->brake_input = lcc_clamp(brake, 0.0f, 1.0f);
  car->steering_input = lcc_clamp(steering, -1.0f, 1.0f);
  car->clutch_input = lcc_clamp(clutch, 0.0f, 1.0f);
}

void lcc_car_update(lcc_car_t *car, float dt) {
  if (dt <= 0.0f)
    return;

  car->timestep = dt;
  car->simulation_time += dt;

  float max_steer_angle = 0.6981317008;
  float sa = -lcc_clamp(car->steering_input, -1.0f, 1.0f) * max_steer_angle;
  car->wheels[0].steer_angle = sa;
  car->wheels[1].steer_angle = sa;
  car->wheels[2].steer_angle = 0.0f;
  car->wheels[3].steer_angle = 0.0f;

  float target_throttle = lcc_clamp(car->throttle_input, 0.0f, 1.0f);
  float tau = fmaxf(car->engine.response_time, 1e-3f);
  float k = 1.0f - expf(-dt / tau);
  car->engine.throttle = lcc_lerp(car->engine.throttle, target_throttle, k);

  lcc_engine_physics(car);
  lcc_suspension_physics(car);
  lcc_aerodynamics_physics(car);
  lcc_transmission_physics(car);
  lcc_vehicle_dynamics(car);
}

void lcc_car_shift_up(lcc_car_t *car) {
  if (car->transmission.current_gear < car->transmission.num_gears)
    car->transmission.current_gear++;
}

void lcc_car_shift_down(lcc_car_t *car) {
  if (car->transmission.current_gear > 0) {
    car->transmission.current_gear--;
  } else if (car->transmission.current_gear == 0) {
    car->transmission.current_gear = -1;
  }
}

void lcc_car_set_gear(lcc_car_t *car, int gear) {
  if (gear >= -1 && gear <= car->transmission.num_gears)
    car->transmission.current_gear = gear;
}

float lcc_car_get_speed(const lcc_car_t *car) { return lcc_length(car->velocity) * 3.6f; }

float lcc_car_get_engine_rpm(const lcc_car_t *car) { return car->engine.current_rpm; }

const char *lcc_get_version(void) { return LCC_VERSION; }

#endif
#endif
