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
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN,
  THE SOFTWARE.
*/

#ifndef LIBCCAR_H
#define LIBCCAR_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define LCC_VERSION "0.3.4"
#define LCC_PI      3.14159265358979323846f
#define LCC_GRAVITY 9.81f

typedef struct {
  float radius;           /* m */
  float angular_velocity; /* rad/s */
  float steer_angle;      /* rad */
  float grip;             /* friction coefficient (mu) */
  float position[2];      /* m, local position of wheel center relative to CG (x forward, y left) */
  float load;             /* N, normal force */
  float slip_angle;       /* rad */
  float slip_ratio;       /* dimensionless */
} lcc_wheel_t;

typedef struct {
  lcc_wheel_t wheels[4];               /* **the wheels** (don't really need them in my opinion) */
  float       position[2];             /* m, world position */
  float       velocity[2];             /* m/s, world velocity */
  float       prev_velocity[2];        /* m/s, previous velocity for acceleration */
  float       throttle;                /* -1..1, negative applies engine braking */
  float       brake;                   /* 0..1 */
  float       steering;                /* -1..1 */
  float       angle;                   /* rad, yaw */
  float       angular_velocity;        /* rad/s, yaw rate */
  float       mass;                    /* kg */
  float       inertia;                 /* kg*m^2, yaw inertia */
  float       wheelbase;               /* m */
  float       track_width;             /* m */
  float       cg_height;               /* m, center of gravity height above ground */
  float       cg_position;             /* 0..1, fraction from front axle to rear */
  float       drag_coeff;              /* dimensionless, Cd */
  float       frontal_area;            /* m^2 */
  float       air_density;             /* kg/m^3 */
  float       downforce_coeff;         /* dimensionless, Cl */
  float       downforce_area;          /* m^2 */
  float       max_steer_angle;         /* rad */
  float       engine_power;            /* W, peak power */
  int         drive_type;              /* 0=RWD, 1=FWD, 2=AWD */
  int         num_gears;               /* this comment is here so clang-format makes it pretty :3 */
  int         gear;                    /* current gear index */
  float       gear_ratios[8];          /* this comment is here so clang-format makes it pretty :3 */
  float       final_drive;             /* this comment is here so clang-format makes it pretty :3 */
  float       transmission_efficiency; /* 0..1 */
  float       idle_rpm;                /* RPM */
  float       peak_rpm;                /* RPM, peak power (sadly not over 9000) */
  float       redline_rpm;             /* RPM */
  float       brake_bias;              /* 0..1, fraction to front axle */
  float       brake_torque_max;        /* Nm, total system capability */
  float       wheel_mass;              /* kg, per wheel for rotational inertia */
  float       lateral_friction_scale;  /* dimensionless */
  float       long_transfer_factor;    /* dimensionless, suspension compliance for longitudinal transfer */
  float       lat_transfer_factor;     /* dimensionless, suspension compliance for lateral transfer */
  float       wheel_inertia_bias;      /* dimensionless, multiplier on wheel inertia */
} lcc_car_t;

typedef struct {
  float longitudinal; /* N */
  float lateral;      /* N */
  float torque;       /* Nm */
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

static float lcc_signf(float v) { return (v > 0.0f) - (v < 0.0f); }

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
  float omega_r = w->angular_velocity * w->radius;
  float slip_ratio = 0.0f;
  if (fabsf(v_long) < 0.1f && fabsf(omega_r) < 0.1f)
    slip_ratio = 0.0f;
  else if (fabsf(omega_r) > fabsf(v_long))
    slip_ratio = (omega_r - v_long) / fmaxf(fabsf(omega_r), 0.1f);
  else
    slip_ratio = (omega_r - v_long) / fmaxf(fabsf(v_long), 0.1f);
  return lcc_clamp(slip_ratio, -1.0f, 1.0f);
}

static float lcc_effective_mu(float load, float mu_base) {
  const float ref_load = 3000.0f;
  float       Fz = fmaxf(load, 1.0f);
  float       mu = mu_base * powf(Fz / ref_load, -0.15f);
  if (mu > 1.4f * mu_base)
    mu = 1.4f * mu_base;
  if (mu < 0.6f * mu_base)
    mu = 0.6f * mu_base;
  return mu;
}

static void lcc_tire_forces(float slip_angle, float slip_ratio, float load, float mu, float *out_Fx, float *out_Fy, float lateral_scale) {
  *out_Fx = 0.0f;
  *out_Fy = 0.0f;
  if (load < 1.0f)
    return;
  float       Fz = fmaxf(load, 1.0f);
  float       mu_eff = lcc_effective_mu(Fz, mu);
  const float C_lat = 1.35f, D_lat = 1.0f, E_lat = -1.3f;
  const float C_long = 1.65f, D_long = 1.0f, E_long = -0.5f;
  const float Fz_ref = 4000.0f;
  const float Cf_ref = 20000.0f;
  const float Kx_ref = 25000.0f;
  float       Cf = Cf_ref * powf(Fz / Fz_ref, 0.8f);
  float       Kx = Kx_ref * powf(Fz / Fz_ref, 0.8f);
  float       denom_lat = fmaxf(mu_eff * Fz * C_lat * D_lat, 1e-3f);
  float       denom_long = fmaxf(mu_eff * Fz * C_long * D_long, 1e-3f);
  float       B_lat = Cf / denom_lat;
  float       B_long = Kx / denom_long;
  float       alpha = slip_angle;
  float       t_lat = B_lat * alpha;
  float       Fy0 = (-D_lat * mu_eff * Fz * sinf(C_lat * atanf(t_lat - E_lat * (t_lat - atanf(t_lat))))) * lateral_scale;
  float       kappa = lcc_clamp(slip_ratio, -1.0f, 1.0f);
  float       t_long = B_long * kappa;
  float       Fx0 = D_long * mu_eff * Fz * sinf(C_long * atanf(t_long - E_long * (t_long - atanf(t_long))));
  float       cap = mu_eff * Fz;
  float       nx = Fx0 / fmaxf(cap, 1.0f);
  float       ny = Fy0 / fmaxf(cap, 1.0f);
  const float p = 1.6f;
  float       util = powf(powf(fabsf(nx), p) + powf(fabsf(ny), p), 1.0f / p);
  if (util > 1.0f) {
    float scale = 1.0f / util;
    Fx0 *= scale;
    Fy0 *= scale;
  }
  *out_Fx = Fx0;
  *out_Fy = Fy0;
}

static float lcc_calculate_downforce(const lcc_car_t *car, float speed) {
  if (car->downforce_coeff <= 0.0f || car->downforce_area <= 0.0f)
    return 0.0f;
  return 0.5f * car->air_density * car->downforce_coeff * car->downforce_area * speed * speed;
}

static void lcc_compute_loads(lcc_car_t *car, const float accel_world[2], float speed) {
  const float df_front_share = 0.45f;
  const float phi_f = 0.55f;

  float a_local[2] = {accel_world[0], accel_world[1]};
  lcc_rotate_vec(a_local, -car->angle);
  float total_load = car->mass * LCC_GRAVITY;
  float downforce = lcc_calculate_downforce(car, speed);
  float front_axle_static = total_load * (1.0f - car->cg_position) + df_front_share * downforce;
  float rear_axle_static = total_load * car->cg_position + (1.0f - df_front_share) * downforce;
  float long_transfer = (car->mass * a_local[0] * car->cg_height / fmaxf(car->wheelbase, 0.001f)) * car->long_transfer_factor;
  float lat_transfer = (car->mass * a_local[1] * car->cg_height / fmaxf(car->track_width, 0.001f)) * car->lat_transfer_factor;
  float lat_f = phi_f * lat_transfer;
  float lat_r = (1.0f - phi_f) * lat_transfer;
  float front_axle = front_axle_static - long_transfer;
  float rear_axle = rear_axle_static + long_transfer;
  float fl = 0.5f * front_axle - 0.5f * lat_f;
  float fr = 0.5f * front_axle + 0.5f * lat_f;
  float rl = 0.5f * rear_axle - 0.5f * lat_r;
  float rr = 0.5f * rear_axle + 0.5f * lat_r;
  car->wheels[0].load = fmaxf(fl, 10.0f);
  car->wheels[1].load = fmaxf(fr, 10.0f);
  car->wheels[2].load = fmaxf(rl, 10.0f);
  car->wheels[3].load = fmaxf(rr, 10.0f);
}

static float lcc_engine_torque(const lcc_car_t *car, float rpm, float throttle_pos) {
  const float idle = car->idle_rpm;
  const float pwr_rpm = car->peak_rpm;
  const float red = car->redline_rpm;
  rpm = fmaxf(rpm, idle);
  rpm = fminf(rpm, red);
  float tpk_rpm = 0.65f * pwr_rpm;
  float omega_pwr = pwr_rpm * 2.0f * LCC_PI / 60.0f;
  float T_pwr = car->engine_power / fmaxf(omega_pwr, 1.0f);
  float T_peak = 1.10f * T_pwr;
  float T_out = 0.0f;
  if (rpm <= tpk_rpm) {
    float x = (rpm - idle) / fmaxf(tpk_rpm - idle, 1.0f);
    x = lcc_clamp(x, 0.0f, 1.0f);
    float s = x * x * (3.0f - 2.0f * x);
    T_out = (0.4f * T_peak) + s * (0.6f * T_peak);
  } else if (rpm <= pwr_rpm) {
    float x = (rpm - tpk_rpm) / fmaxf(pwr_rpm - tpk_rpm, 1.0f);
    x = lcc_clamp(x, 0.0f, 1.0f);
    T_out = T_peak * (1.0f - 0.15f * x);
  } else {
    float omega = rpm * 2.0f * LCC_PI / 60.0f;
    T_out = car->engine_power / fmaxf(omega, 1.0f);
  }
  T_out *= lcc_clamp(throttle_pos, 0.0f, 1.0f);
  return T_out;
}

static int lcc_is_wheel_driven(const lcc_car_t *car, int wi) {
  switch (car->drive_type) {
  case 0:
    return wi >= 2;
  case 1:
    return wi < 2;
  case 2:
  default:
    return 1;
  }
}

static void lcc_split_axle_torque(const lcc_car_t *car, int iL, int iR, float axle_torque, float drive_torque[4]) {
  const lcc_wheel_t *wL = &car->wheels[iL];
  const lcc_wheel_t *wR = &car->wheels[iR];

  float muL = lcc_effective_mu(wL->load, wL->grip);
  float muR = lcc_effective_mu(wR->load, wR->grip);
  float capL = muL * wL->load * wL->radius;
  float capR = muR * wR->load * wR->radius;
  float sum = fmaxf(wL->load + wR->load, 1e-3f);
  float TL = axle_torque * (wL->load / sum);
  float TR = axle_torque - TL;

  const float TBR = 2.5f;
  if (fabsf(TR) > 1e-6f) {
    float ratio = fabsf(TL / TR);
    if (ratio > TBR)
      TL = lcc_signf(TL) * TBR * fabsf(TR);
    if (ratio < 1.0f / TBR)
      TR = lcc_signf(TR) * TBR * fabsf(TL);
  }
  TL = lcc_clamp(TL, -capL, capL);
  TR = lcc_clamp(TR, -capR, capR);
  drive_torque[iL] += TL;
  drive_torque[iR] += TR;
}

lcc_car_t lcc_car_init(float wheel_radius, float wheel_grip, float mass, float wheelbase, float track_width, float engine_power) {
  lcc_car_t car = {0};
  for (int i = 0; i < 4; ++i) {
    car.wheels[i].radius = wheel_radius;
    car.wheels[i].grip = (wheel_grip > 0.0f) ? wheel_grip : 1.20f;
    car.wheels[i].position[0] = 0.0f;
    car.wheels[i].position[1] = 0.0f;
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
  car.mass = mass;
  car.inertia = mass * (wheelbase * wheelbase + track_width * track_width) / 12.0f;
  car.wheelbase = wheelbase;
  car.track_width = track_width;
  car.cg_height = 0.52f;
  car.cg_position = 0.55f;
  car.drag_coeff = 0.29f;
  car.frontal_area = 2.1f;
  car.air_density = 1.225f;
  car.downforce_coeff = 0.0f;
  car.downforce_area = 1.5f;
  car.max_steer_angle = 0.50f;
  car.engine_power = engine_power;
  car.drive_type = 0;
  car.num_gears = 6;
  car.gear_ratios[0] = 4.11f;
  car.gear_ratios[1] = 2.32f;
  car.gear_ratios[2] = 1.54f;
  car.gear_ratios[3] = 1.18f;
  car.gear_ratios[4] = 1.00f;
  car.gear_ratios[5] = 0.84f;
  car.final_drive = 3.46f;
  car.transmission_efficiency = 0.90f;
  car.idle_rpm = 800.0f;
  car.peak_rpm = 6500.0f;
  car.redline_rpm = 7400.0f;
  car.brake_bias = 0.60f;
  car.brake_torque_max = 22000.0f;
  car.wheel_mass = 12.0f;
  car.lateral_friction_scale = 1.0f;
  car.long_transfer_factor = 1.0f;
  car.lat_transfer_factor = 1.0f;
  car.wheel_inertia_bias = 1.0f;
  float total_load = car.mass * LCC_GRAVITY;
  float front_axle_static = total_load * (1.0f - car.cg_position);
  float rear_axle_static = total_load * car.cg_position;
  float static_load_front = front_axle_static * 0.5f;
  float static_load_rear = rear_axle_static * 0.5f;
  car.wheels[0].load = static_load_front;
  car.wheels[1].load = static_load_front;
  car.wheels[2].load = static_load_rear;
  car.wheels[3].load = static_load_rear;
  return car;
}

void lcc_car_set_input(lcc_car_t *car, float throttle, float brake, float steering) {
  if (!car)
    return;
  car->throttle = lcc_clamp(throttle, -1.0f, 1.0f);
  car->brake = lcc_clamp(brake, 0.0f, 1.0f);
  car->steering = lcc_clamp(steering, -1.0f, 1.0f);
}

void lcc_car_update(lcc_car_t *car, float dt) {
  if (!car || dt <= 0.0f)
    return;
  const float low_speed_threshold = 0.5f;
  const float wheel_omega_snap = 1.5f;
  const float max_wheel_alpha = 5000.0f;
  const float angular_damping = 0.998f;
  const float linear_damping = 0.999f;
  const int   max_substeps = 5;
  const float target_substep_dt = 0.01f;

  int steps = (int)ceilf(dt / target_substep_dt);
  if (steps < 1)
    steps = 1;
  if (steps > max_substeps)
    steps = max_substeps;
  float subdt = dt / (float)steps;
  float steer = -car->steering * car->max_steer_angle;
  car->wheels[0].steer_angle = steer;
  car->wheels[1].steer_angle = steer;
  car->wheels[2].steer_angle = 0.0f;
  car->wheels[3].steer_angle = 0.0f;
  for (int s = 0; s < steps; ++s) {
    float vehicle_speed = lcc_length(car->velocity);
    float drivetrain_ratio = car->gear_ratios[car->gear] * car->final_drive;
    int   driven_count = 0;
    float avg_wheel_omega = 0.0f;
    for (int i = 0; i < 4; ++i) {
      if (lcc_is_wheel_driven(car, i)) {
        avg_wheel_omega += fabsf(car->wheels[i].angular_velocity);
        driven_count++;
      }
    }
    if (driven_count > 0)
      avg_wheel_omega /= (float)driven_count;
    float engine_rpm = avg_wheel_omega * (60.0f / (2.0f * LCC_PI)) * drivetrain_ratio;
    float upshift_rpm = 0.98f * car->redline_rpm;
    float downshift_rpm = fmaxf(1.4f * car->idle_rpm, 0.35f * car->peak_rpm);
    if (engine_rpm > upshift_rpm && car->gear < car->num_gears - 1) {
      car->gear++;
      drivetrain_ratio = car->gear_ratios[car->gear] * car->final_drive;
      engine_rpm = avg_wheel_omega * (60.0f / (2.0f * LCC_PI)) * drivetrain_ratio;
    } else if (engine_rpm < downshift_rpm && car->gear > 0) {
      car->gear--;
      drivetrain_ratio = car->gear_ratios[car->gear] * car->final_drive;
      engine_rpm = avg_wheel_omega * (60.0f / (2.0f * LCC_PI)) * drivetrain_ratio;
    }
    float throttle_pos = fmaxf(0.0f, car->throttle);
    float throttle_neg = -fminf(0.0f, car->throttle);
    float eng_torque = lcc_engine_torque(car, engine_rpm, throttle_pos);
    float T_pwr = car->engine_power / fmaxf(car->peak_rpm * 2.0f * LCC_PI / 60.0f, 1.0f);
    float T_peak_for_brake = 1.10f * T_pwr;
    float rpm_norm = lcc_clamp((engine_rpm - car->idle_rpm) / fmaxf(car->redline_rpm - car->idle_rpm, 1.0f), 0.0f, 1.0f);
    float eng_brake_factor = 0.2f + 0.6f * rpm_norm * rpm_norm;
    float eng_brake_torque = throttle_neg * eng_brake_factor * T_peak_for_brake;
    float axle_torque = (eng_torque - eng_brake_torque) * drivetrain_ratio * car->transmission_efficiency;
    float drive_torque[4] = {0};
    if (axle_torque != 0.0f) {
      if (car->drive_type == 0)
        lcc_split_axle_torque(car, 2, 3, axle_torque, drive_torque);
      else if (car->drive_type == 1)
        lcc_split_axle_torque(car, 0, 1, axle_torque, drive_torque);
      else {
        lcc_split_axle_torque(car, 0, 1, 0.5f * axle_torque, drive_torque);
        lcc_split_axle_torque(car, 2, 3, 0.5f * axle_torque, drive_torque);
      }
    }
    float brake_torque[4] = {0};
    if (car->brake > 0.0f) {
      float front_pair = car->brake_bias * car->brake_torque_max * car->brake;
      float rear_pair = (1.0f - car->brake_bias) * car->brake_torque_max * car->brake;
      brake_torque[0] = 0.5f * front_pair;
      brake_torque[1] = 0.5f * front_pair;
      brake_torque[2] = 0.5f * rear_pair;
      brake_torque[3] = 0.5f * rear_pair;
    }
    float total_force[2] = {0.0f, 0.0f};
    float total_torque = 0.0f;
    for (int i = 0; i < 4; ++i) {
      lcc_wheel_t *w = &car->wheels[i];
      float        wp[2] = {w->position[0], w->position[1]};
      lcc_rotate_vec(wp, car->angle);
      float wheel_vx = car->velocity[0] - car->angular_velocity * wp[1];
      float wheel_vy = car->velocity[1] + car->angular_velocity * wp[0];
      float wheel_world_angle = car->angle + w->steer_angle;
      float c = cosf(wheel_world_angle), s = sinf(wheel_world_angle);
      float v_long = wheel_vx * c + wheel_vy * s;
      float v_lat = -wheel_vx * s + wheel_vy * c;
      float speed_local = sqrtf(v_long * v_long + v_lat * v_lat);
      if (speed_local > 0.05f || fabsf(w->angular_velocity) > 0.1f) {
        w->slip_angle = lcc_calc_slip_angle(car, i);
        w->slip_ratio = lcc_calc_slip_ratio(car, i);
      } else {
        w->slip_angle = 0.0f;
        w->slip_ratio = 0.0f;
      }
      float brake_t = brake_torque[i];
      if (car->brake > 0.0f && brake_t > 0.0f) {
        float sr = fabsf(w->slip_ratio);
        if (sr > 0.2f) {
          float scale = lcc_clamp(1.0f - 2.0f * (sr - 0.2f), 0.3f, 1.0f);
          brake_t *= scale;
        }
      }
      float Fx_tire = 0.0f, Fy_tire = 0.0f;
      lcc_tire_forces(w->slip_angle, w->slip_ratio, w->load, w->grip, &Fx_tire, &Fy_tire, car->lateral_friction_scale);
      if (vehicle_speed < low_speed_threshold) {
        float fac = lcc_clamp(vehicle_speed / low_speed_threshold, 0.0f, 1.0f);
        Fy_tire *= fac;
        Fx_tire *= (0.5f + 0.5f * fac);
      }
      float world_Fx = Fx_tire * c - Fy_tire * s;
      float world_Fy = Fx_tire * s + Fy_tire * c;
      total_force[0] += world_Fx;
      total_force[1] += world_Fy;
      total_torque += wp[0] * world_Fy - wp[1] * world_Fx;
      float Iw = car->wheel_mass * w->radius * w->radius * car->wheel_inertia_bias;
      Iw = fmaxf(Iw, 1e-5f);
      float reaction_torque = Fx_tire * w->radius;
      float brake_dir = 0.0f;
      if (car->brake > 0.0f && brake_t > 0.0f) {
        if (fabsf(w->angular_velocity) > 0.5f) {
          brake_dir = lcc_signf(w->angular_velocity);
        } else if (fabsf(v_long) > 0.1f) {
          brake_dir = lcc_signf(v_long) * 1.0f;
        } else {
          brake_dir = 1.0f;
        }
      }
      float net_torque = drive_torque[i] - reaction_torque - brake_dir * brake_t;
      float alpha = net_torque / Iw;
      alpha = lcc_clamp(alpha, -max_wheel_alpha, max_wheel_alpha);
      w->angular_velocity += alpha * subdt;
      float coupling = 0.5f;
      float target_omega = v_long / fmaxf(w->radius, 1e-4f);
      float lerp_t = lcc_clamp(subdt * coupling, 0.0f, 1.0f);
      w->angular_velocity = w->angular_velocity * (1.0f - lerp_t) + target_omega * lerp_t;
      w->angular_velocity *= 0.998f;
      if (fabsf(w->angular_velocity) < wheel_omega_snap && car->brake > 0.2f && fabsf(v_long) < 0.5f) {
        w->angular_velocity = 0.0f;
      }
      const float maxomega = 2000.0f;
      w->angular_velocity = lcc_clamp(w->angular_velocity, -maxomega, maxomega);
    }
    float speed = lcc_length(car->velocity);
    if (speed > 0.01f) {
      float drag = 0.5f * car->air_density * car->drag_coeff * car->frontal_area * speed * speed;
      total_force[0] += -drag * car->velocity[0] / speed;
      total_force[1] += -drag * car->velocity[1] / speed;
    }
    float accel[2] = {total_force[0] / car->mass, total_force[1] / car->mass};
    car->velocity[0] += accel[0] * subdt;
    car->velocity[1] += accel[1] * subdt;
    car->position[0] += car->velocity[0] * subdt;
    car->position[1] += car->velocity[1] * subdt;
    float angular_accel = total_torque / fmaxf(car->inertia, 1e-6f);
    car->angular_velocity += angular_accel * subdt;
    car->angle += car->angular_velocity * subdt;
    car->angle = fmodf(car->angle, 2.0f * LCC_PI);
    if (car->angle < 0.0f)
      car->angle += 2.0f * LCC_PI;
    float speed_post = lcc_length(car->velocity);
    lcc_compute_loads(car, accel, speed_post);
    car->angular_velocity *= angular_damping;
    car->velocity[0] *= linear_damping;
    car->velocity[1] *= linear_damping;
    car->prev_velocity[0] = car->velocity[0];
    car->prev_velocity[1] = car->velocity[1];
  }
}

const char *lcc_get_version(void) { return LCC_VERSION; }

#endif

#endif
