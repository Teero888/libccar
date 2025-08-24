/*
  libccar — simple 2D top-down car simulation

  What this does
  --------------
  - Single-file C99 library for quick experiments.
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
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#define LCC_VERSION     "0.3.7"
#define LCC_PI          (3.14159265358979323846f)
#define LCC_GRAVITY     (9.81f)
#define LCC_AIR_DENSITY (1.225f)
#define LCC_EPS         (1e-6f)
#define LCC_RAD_PER_RPM (2.0f * LCC_PI / 60.0f)

typedef enum {
  LCC_PRESET_ECONOMY,
  LCC_PRESET_MIDSIZE,
  LCC_PRESET_SPORTS,
  LCC_PRESET_SUPERCAR,
  LCC_PRESET_HYPERCAR,
} lcc_preset_t;

typedef enum {
  LCC_DRIVE_RWD,
  LCC_DRIVE_FWD,
  LCC_DRIVE_AWD,
} lcc_drive_t;

/* ------------------------ Parameters ------------------------ */
typedef struct {
  /* geometry */
  float radius; /* m */
  float width; /* m */
  float aspect_ratio; /* [-] */
  float pressure; /* kPa nominal */
  float nominal_load; /* N */

  /* friction model */
  float peak_friction; /* mu_peak baseline at nominal load, ref temp */
  float slip_friction; /* low-slip mu */
  float stiffness; /* long stiffness ~ N per slip unit (pre MF shape) */
  float cornering_stiffness; /* lat stiffness ~ N/rad (pre MF shape) */
  float camber_stiffness; /* N/rad */
  float rolling_resistance; /* Crr */
  float temperature; /* degC (state) */
  float wear; /* 0..1 (state) */

  /* dynamics */
  float relax_length_long; /* m */
  float relax_length_lat; /* m */
  float load_sensitivity; /* 0..1 reduces mu with load */
  float mu_min, mu_max; /* clamps */
} lcc_tire_params_t;

typedef struct {
  float angular_velocity; /* rad/s */
  float steer_angle; /* rad */
  float camber_angle; /* rad (+ = top leaning in) */
  float slip_angle; /* filtered alpha (rad) */
  float slip_ratio; /* filtered kappa (-) */
  float load; /* N */
  float position[2]; /* m in body frame (x fwd, y left) */
  float temperature; /* degC */
  float surface_friction; /* multiplier for surface */

  /* outputs/actuators */
  float Fx, Fy; /* N in tire frame (long +ve forward, lat +ve left) */
  float drive_torque; /* Nm applied at hub */
  float brake_torque; /* Nm resistive magnitude from brakes */
  float rotational_inertia; /* kg*m^2 */
} lcc_wheel_state_t;

typedef struct {
  float max_power; /* W */
  float max_torque; /* Nm */
  float idle_rpm, max_rpm, redline_rpm;
  float inertia; /* kg*m^2 (crank equivalent) */
  float friction; /* linear viscous Nm/(rad/s) */
  float response_time; /* throttle lag (s) */
  float current_rpm;
  float throttle; /* 0..1 internal filtered */
  float output_torque; /* Nm from torque curve (pre losses) */

  /* curve shaping :> */
  float peak_torque_rpm;
  float peak_power_rpm;
  float engine_brake_coeff; /* Nm/(rad/s) scaled by (1-throttle) */
  float friction_quadratic; /* Nm/(rad/s)^2 */
  float idle_torque; /* Nm assist near idle */
  float stall_rpm; /* rpm clamp */
} lcc_engine_t;

typedef struct {
  int         num_gears; /* 1..8 */
  float       gear_ratios[8];
  float       final_drive;
  float       reverse_ratio;
  float       efficiency; /* 0..1 */
  int         current_gear; /* -1..num_gears; 0=neutral */
  lcc_drive_t drive_type;
} lcc_transmission_t;

typedef struct {
  float preload; /* Nm baseline locking */
  float power_factor; /* Nm/Nm times input torque on power */
  float coast_factor; /* Nm/Nm on coast */
  float viscous_coefficient; /* Nm/(rad/s) vs wheel speed difference */
  float bias_limit; /* Nm max locking action */
} lcc_differential_t;

typedef struct {
  float drag_coefficient; /* Cd */
  float frontal_area; /* m^2 */
  float downforce_coefficient; /* Cl (positive produces downforce) */
  float downforce_area; /* m^2 */
  float lift_coefficient; /* (unused; keep for compatibility) */
  float aero_balance_front; /* 0..1 fraction of DF on front axle */
} lcc_aerodynamics_t;

/* ------------------------ Vehicle ------------------------ */
typedef struct {
  float              mass; /* kg */
  float              inertia; /* yaw inertia kg*m^2 */
  float              wheelbase; /* m */
  float              track_width; /* m */
  float              cg_height; /* m above ground */
  float              cg_position; /* 0..1 fraction from front axle (0=at front axle, 1=at rear axle) */
  float              position[2]; /* world (m) */
  float              velocity[2]; /* world (m/s) */
  float              angle; /* yaw rad */
  float              angular_velocity; /* yaw rate rad/s */
  lcc_engine_t       engine;
  lcc_transmission_t transmission;
  lcc_differential_t differential;
  lcc_aerodynamics_t aerodynamics;
  lcc_tire_params_t  tire_params[4];
  lcc_wheel_state_t  wheels[4];

  /* inputs 0..1 except steering -1..1 */
  float throttle_input, brake_input, steering_input, clutch_input;
  float front_brake_bias; /* 0..1 */
  float max_brake_torque; /* Nm total */
  float air_density; /* kg/m^3 */
  float ambient_temp; /* degC */
  float surface_friction; /* global multiplier */
  float timestep; /* s  */
  float simulation_time; /* s */
  float _Fz_smooth[4]; /* N filtered loads to avoid spikes */
} lcc_car_t;

/* ------------------------ API ------------------------ */
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

/* ------------------------ Implementation ------------------------ */
#define LIBCCAR_IMPLEMENTATION
#ifdef LIBCCAR_IMPLEMENTATION
/* ---------- math helpers ---------- */
static float lcc_clamp(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static float lcc_lerp(float a, float b, float t) {
  return a + t * (b - a);
}

static float lcc_sign(float x) {
  return (x > 0.0f) - (x < 0.0f);
}

static float lcc_length2(const float v[2]) {
  return v[0] * v[0] + v[1] * v[1];
}

static float lcc_length(const float v[2]) {
  return sqrtf(lcc_length2(v));
}

static void lcc_norm2(float v[2]) {
  float L = lcc_length(v);
  if(L > LCC_EPS) {
    v[0] /= L;
    v[1] /= L;
  }
}

static void lcc_body_to_world(float cx, float sx, const float vb[2], float out[2]) {
  out[0] = vb[0] * cx - vb[1] * sx;
  out[1] = vb[0] * sx + vb[1] * cx;
}

static void lcc_world_to_body(float cx, float sx, const float vw[2], float out[2]) {
  out[0] = vw[0] * cx + vw[1] * sx;
  out[1] = -vw[0] * sx + vw[1] * cx;
}

/* ---------- engine ---------- */
static float lcc_engine_torque_curve(const lcc_engine_t *e, float rpm, float thr) {
  float rpm_cl = lcc_clamp(rpm, e->stall_rpm, e->redline_rpm);
  float Trpm   = e->peak_torque_rpm > 0 ? e->peak_torque_rpm : (0.5f * (e->idle_rpm + e->max_rpm));
  float Prpm   = e->peak_power_rpm > 0 ? e->peak_power_rpm : (0.85f * e->redline_rpm);
  float t_norm;
  if(rpm_cl <= Trpm) {
    float x = (rpm_cl - e->idle_rpm) / fmaxf(Trpm - e->idle_rpm, 1.0f);
    /* smooth inverted parabola from idle to peak torque */
    t_norm = 0.2f + 0.8f * (1.0f - (x - 1.0f) * (x - 1.0f));
  } else if(rpm_cl < Prpm) {
    float x = (rpm_cl - Trpm) / fmaxf(Prpm - Trpm, 1.0f);
    t_norm  = 1.0f - 0.15f * x;
  } else {
    float x = (rpm_cl - Prpm) / fmaxf(e->redline_rpm - Prpm, 1.0f);
    t_norm  = lcc_clamp(0.85f - 0.85f * x, 0.3f, 0.9f);
  }
  float thr_shaped = thr * thr * (3.0f - 2.0f * thr);
  float torque_cmd = e->max_torque * t_norm * thr_shaped;
  return fmaxf(0.0f, torque_cmd);
}

static void lcc_engine_physics(lcc_car_t *car) {
  lcc_engine_t *e   = &car->engine;
  float         rpm = lcc_clamp(e->current_rpm, e->stall_rpm, e->redline_rpm);

  float throttle_torque = lcc_engine_torque_curve(e, rpm, lcc_clamp(e->throttle, 0.0f, 1.0f));

  float idle_assist = 0.0f;
  // Apply idle torque only when throttle is off and RPM is near or below idle
  if(e->throttle < 0.05f && rpm < e->idle_rpm * 1.1f) {
    // Gently phase in idle torque to prevent stalling
    float deficit_rpm = (e->idle_rpm * 1.1f) - rpm;
    float idle_factor = lcc_clamp(deficit_rpm / (e->idle_rpm * 1.1f - e->stall_rpm), 0.0f, 1.0f);
    idle_assist       = e->idle_torque * idle_factor;
  }

  e->output_torque = throttle_torque + idle_assist;
}

/* ---------- differential (simple clutch-type) ---------- */
static void lcc_lsd_split(const lcc_differential_t *d, float axle_Tin, float omega_left, float omega_right, float *outL, float *outR) {
  float Tl          = 0.5f * axle_Tin;
  float Tr          = 0.5f * axle_Tin;
  float domega      = omega_left - omega_right;
  float lock_factor = (axle_Tin >= 0.0f) ? d->power_factor : d->coast_factor;
  float T_lock      = d->preload + lock_factor * fabsf(axle_Tin) + d->viscous_coefficient * fabsf(domega);
  if(d->bias_limit > 0.0f) T_lock = fminf(T_lock, d->bias_limit);

  /* oppose speed difference */
  float sgn = lcc_sign(domega);
  Tl -= 0.5f * T_lock * sgn;
  Tr += 0.5f * T_lock * sgn;
  *outL = Tl;
  *outR = Tr;
}

/* ---------- driveline ---------- */
static float lcc_safe_avg(const float *vals, int n) {
  float s = 0.0f;
  int   k = 0;
  for(int i = 0; i < n; ++i) {
    if(isfinite(vals[i])) {
      s += vals[i];
      k++;
    }
  }
  return (k > 0) ? s / (float)k : 0.0f;
}

static void lcc_transmission_physics(lcc_car_t *car) {
  lcc_transmission_t *t = &car->transmission;
  lcc_engine_t       *e = &car->engine;

  /* ---------------- Brakes ---------------- */
  float b                     = lcc_clamp(car->brake_input, 0.0f, 1.0f);
  float Tb_total              = fmaxf(0.0f, car->max_brake_torque) * b;
  float bias                  = lcc_clamp(car->front_brake_bias, 0.0f, 1.0f);
  float Tb_front_pw           = 0.5f * Tb_total * bias;
  float Tb_rear_pw            = 0.5f * Tb_total * (1.0f - bias);
  car->wheels[0].brake_torque = Tb_front_pw;
  car->wheels[1].brake_torque = Tb_front_pw;
  car->wheels[2].brake_torque = Tb_rear_pw;
  car->wheels[3].brake_torque = Tb_rear_pw;
  for(int i = 0; i < 4; ++i) car->wheels[i].drive_torque = 0.0f;

  /* ---------------- Gear selection ---------------- */
  int   g  = t->current_gear;
  float gr = 0.0f;
  if(g > 0 && g <= t->num_gears) gr = t->gear_ratios[g - 1];
  else if(g == -1)
    gr = -t->reverse_ratio;
  else
    gr = 0.0f;
  float Rf  = t->final_drive;
  float eff = lcc_clamp(t->efficiency, 0.0f, 1.0f);

  /* ---------------- Engine state ---------------- */
  float dt      = car->timestep;
  float omega_e = e->current_rpm * LCC_RAD_PER_RPM;
  if(!isfinite(omega_e)) omega_e = e->idle_rpm * LCC_RAD_PER_RPM;
  float thr = lcc_clamp(e->throttle, 0.0f, 1.0f);

  /* This is the total generated torque from lcc_engine_physics */
  float T_out = fmaxf(0.0f, e->output_torque);

  /* Losses (oppose rotation) */
  float omega_abs    = fabsf(omega_e);
  float T_resist_mag = e->friction * omega_abs + e->friction_quadratic * omega_abs * omega_abs + e->engine_brake_coeff * (1.0f - thr) * omega_abs;
  float T_resist     = T_resist_mag * lcc_sign(omega_e);

  /* ---------------- Clutch engagement ----------------*/
  float E = 1.0f - lcc_clamp(car->clutch_input, 0.0f, 1.0f);
  if(gr == 0.0f || Rf == 0.0f || E < 1e-3f) {
    float Ieng         = fmaxf(e->inertia, 1e-4f);
    float T_net_engine = T_out - T_resist;
    float domega_e     = lcc_clamp((T_net_engine / Ieng) * dt, -5000.0f, 5000.0f);
    omega_e            = fmaxf(0.0f, omega_e + domega_e);
    e->current_rpm     = lcc_clamp(omega_e / LCC_RAD_PER_RPM, 0.0f, e->redline_rpm);
    return;
  }

  /* ---------------- Driven wheels and shaft speeds ---------------- */
  int drivenMask[4] = { 0, 0, 0, 0 };
  switch(t->drive_type) {
  case LCC_DRIVE_RWD: drivenMask[2] = drivenMask[3] = 1; break;
  case LCC_DRIVE_FWD: drivenMask[0] = drivenMask[1] = 1; break;
  case LCC_DRIVE_AWD:
    for(int i = 0; i < 4; ++i) drivenMask[i] = 1;
    break;
  default: break;
  }
  float sum_w = 0.0f;
  int   cnt_w = 0;
  for(int i = 0; i < 4; ++i)
    if(drivenMask[i]) {
      sum_w += car->wheels[i].angular_velocity;
      cnt_w++;
    }
  float avg_w_omega = (cnt_w > 0) ? sum_w / (float)cnt_w : 0.0f;
  float omega_out   = avg_w_omega * Rf;
  float omega_in    = omega_out * gr;
  float omega_diff  = omega_e - omega_in;

  /* ---------------- Clutch model ----------------*/
  float Tcap_base = 50.0f + 1.5f * fmaxf(100.0f, e->max_torque);
  float T_cap     = Tcap_base * (0.05f + 0.95f * E);
  float Ksync     = 0.15f * Tcap_base;

  float T_sync  = Ksync * omega_diff;
  float T_c_pre = T_out + T_sync;

  /* Prevent forward drive with closed throttle and engaged clutch (engine braking) */
  if(thr < 0.02f && g != 0 && E > 0.7f && omega_in > omega_e) T_c_pre = fminf(T_c_pre, 0.0f);
  float T_c = lcc_clamp(T_c_pre, -T_cap, T_cap);

  /* ---------------- Torque to wheels ---------------- */
  float T_prop         = T_c * gr * eff;
  float T_wheels_total = T_prop * Rf;
  float TfL = 0, TfR = 0, TrL = 0, TrR = 0;
  switch(t->drive_type) {
  case LCC_DRIVE_RWD:
    lcc_lsd_split(&car->differential, T_wheels_total, car->wheels[2].angular_velocity, car->wheels[3].angular_velocity, &TrL, &TrR);
    break;
  case LCC_DRIVE_FWD:
    lcc_lsd_split(&car->differential, T_wheels_total, car->wheels[0].angular_velocity, car->wheels[1].angular_velocity, &TfL, &TfR);
    break;
  case LCC_DRIVE_AWD: {
    float T_axle = 0.5f * T_wheels_total;
    lcc_lsd_split(&car->differential, T_axle, car->wheels[0].angular_velocity, car->wheels[1].angular_velocity, &TfL, &TfR);
    lcc_lsd_split(&car->differential, T_axle, car->wheels[2].angular_velocity, car->wheels[3].angular_velocity, &TrL, &TrR);
    break;
  }
  default: break;
  }
  car->wheels[0].drive_torque = TfL;
  car->wheels[1].drive_torque = TfR;
  car->wheels[2].drive_torque = TrL;
  car->wheels[3].drive_torque = TrR;

  /* ---------------- Engine integration ---------------- */
  float Ieng         = fmaxf(e->inertia, 1e-4f);
  float T_net_engine = T_out - T_resist - T_c;
  if(e->current_rpm >= e->redline_rpm && T_net_engine > 0.0f) T_net_engine = 0.0f;
  float domega_e = lcc_clamp((T_net_engine / Ieng) * dt, -5000.0f, 5000.0f);
  omega_e        = fmaxf(0.0f, omega_e + domega_e);
  e->current_rpm = lcc_clamp(omega_e / LCC_RAD_PER_RPM, 0.0f, e->redline_rpm);
}

/* ---------- aero forces ---------- */
static void lcc_aero_forces(const lcc_car_t *car, float out_Fdrag_world[2], float *out_DF_front, float *out_DF_rear) {
  float vmag2 = lcc_length2((float *)car->velocity);
  if(vmag2 < 1e-5f) {
    out_Fdrag_world[0] = out_Fdrag_world[1] = 0.0f;
    *out_DF_front = *out_DF_rear = 0.0f;
    return;
  }
  float vmag     = sqrtf(vmag2);
  float rho      = car->air_density;
  float q        = 0.5f * rho * vmag2; /* dynamic pressure */
  float Fd       = q * car->aerodynamics.drag_coefficient * car->aerodynamics.frontal_area;
  float dfA      = car->aerodynamics.downforce_coefficient * car->aerodynamics.downforce_area;
  float DF_total = q * dfA; /* positive downward */
  float vdir[2]  = { -car->velocity[0], -car->velocity[1] };
  lcc_norm2(vdir);
  out_Fdrag_world[0] = Fd * vdir[0];
  out_Fdrag_world[1] = Fd * vdir[1];
  float fb           = lcc_clamp(car->aerodynamics.aero_balance_front, 0.0f, 1.0f);
  *out_DF_front      = DF_total * fb;
  *out_DF_rear       = DF_total * (1.0f - fb);
}

/* ---------- load transfer (quasi-static + smoothing) ---------- */
static void lcc_compute_loads(lcc_car_t *car, float ax_body, float ay_body, float DF_front, float DF_rear) {
  /* static axle loads */
  float W         = car->mass * LCC_GRAVITY;
  float Wf_static = W * (1.0f - car->cg_position);
  float Wr_static = W * (car->cg_position);

  /* longitudinal transfer (to rear with +ax) */
  float dF_long = (car->mass * ax_body * car->cg_height) / fmaxf(car->wheelbase, 0.1f);
  /* lateral transfer total magnitude */
  float dF_lat_total = (car->mass * ay_body * car->cg_height) / fmaxf(car->track_width, 0.1f);

  /* distribute lateral transfer 50/50 between axles by default.
     If you later add separate front/rear roll stiffness, change lambda. */
  float lambda       = 0.5f;
  float dF_lat_front = dF_lat_total * lambda;
  float dF_lat_rear  = dF_lat_total * (1.0f - lambda);

  /* base per-wheel before aero */
  float Fz_FL = 0.5f * Wf_static - 0.5f * dF_long - 0.5f * dF_lat_front;
  float Fz_FR = 0.5f * Wf_static - 0.5f * dF_long + 0.5f * dF_lat_front;
  float Fz_RL = 0.5f * Wr_static + 0.5f * dF_long - 0.5f * dF_lat_rear;
  float Fz_RR = 0.5f * Wr_static + 0.5f * dF_long + 0.5f * dF_lat_rear;

  /* aero DF per axle on both wheels */
  Fz_FL += 0.5f * DF_front;
  Fz_FR += 0.5f * DF_front;
  Fz_RL += 0.5f * DF_rear;
  Fz_RR += 0.5f * DF_rear;

  /* clamp to non-negative and apply gentle smoothing to avoid chattering */
  float Fz_target[4] = { fmaxf(0.0f, Fz_FL), fmaxf(0.0f, Fz_FR), fmaxf(0.0f, Fz_RL), fmaxf(0.0f, Fz_RR) };
  float alpha        = 1.0f - expf(-car->timestep * 20.0f); /* ~50 ms time constant */
  for(int i = 0; i < 4; ++i) {
    car->_Fz_smooth[i]  = lcc_lerp(car->_Fz_smooth[i], Fz_target[i], alpha);
    car->wheels[i].load = car->_Fz_smooth[i];
  }
}

/* ---------- tire model (pretty magical) ---------- */
static void lcc_tire_step(lcc_car_t *car, int i) {
  lcc_wheel_state_t *w  = &car->wheels[i];
  lcc_tire_params_t *tp = &car->tire_params[i];

  /* wheel contact velocity at hub */
  float ca = cosf(car->angle), sa = sinf(car->angle);

  /* wheel world offset */
  float rWx = w->position[0] * ca - w->position[1] * sa;
  float rWy = w->position[0] * sa + w->position[1] * ca;

  /* velocity at contact patch in world = V + omega_z x r */
  float vWx = car->velocity[0] - car->angular_velocity * rWy;
  float vWy = car->velocity[1] + car->angular_velocity * rWx;

  /* rotate into wheel frame (steered) */
  float psi = car->angle + w->steer_angle;
  float c = cosf(psi), s = sinf(psi);
  float v_long = vWx * c + vWy * s;
  float v_lat  = -vWx * s + vWy * c;
  float v_abs  = sqrtf(v_long * v_long + v_lat * v_lat);

  /* slips with relaxation */
  float R         = fmaxf(tp->radius, 0.05f);
  float vref_long = fmaxf(fabsf(v_long), 0.5f);
  float kappa_tgt = (w->angular_velocity * R - v_long) / vref_long;
  kappa_tgt       = lcc_clamp(kappa_tgt, -2.5f, 2.5f);
  float alpha_tgt = atanf((fabsf(v_long) > 0.2f) ? (v_lat / fabsf(v_long)) : (v_lat / 0.2f));
  alpha_tgt       = lcc_clamp(alpha_tgt, -LCC_PI * 0.5f, LCC_PI * 0.5f);
  float Lx        = fmaxf(tp->relax_length_long, 0.05f);
  float Ly        = fmaxf(tp->relax_length_lat, 0.05f);
  float rate_k    = fminf(25.0f, vref_long / Lx); /* 1/s */
  float rate_a    = fminf(25.0f, v_abs / Ly); /* 1/s */
  w->slip_ratio += (kappa_tgt - w->slip_ratio) * rate_k * car->timestep;
  w->slip_angle += (alpha_tgt - w->slip_angle) * rate_a * car->timestep;

  /* effective mu with temperature/wear and load sensitivity */
  float mu_surface = fmaxf(w->surface_friction, 0.1f) * fmaxf(car->surface_friction, 0.1f);
  float Fz         = fmaxf(w->load, 0.0f);

  /* temp: best ~80C, degrade linearly away */
  float temp_term = 1.0f - 0.0025f * (w->temperature - 80.0f);
  temp_term       = lcc_clamp(temp_term, 0.5f, 1.05f);
  float wear_term = lcc_clamp(1.0f - 0.6f * tp->wear, 0.4f, 1.0f);
  float mu0       = lcc_lerp(tp->slip_friction, tp->peak_friction, lcc_clamp(temp_term, 0.0f, 1.0f)) * wear_term;
  float Ls        = lcc_clamp(tp->load_sensitivity, 0.0f, 0.9f);
  float mu_load   = mu0 * (1.0f - Ls * (Fz - tp->nominal_load) / fmaxf(tp->nominal_load, 1.0f));
  mu_load         = lcc_clamp(mu_load, fmaxf(0.1f, tp->mu_min), fminf(2.5f, fmaxf(mu0, tp->mu_max)));
  float mu        = mu_surface * mu_load;

  /* Magic-Formula-style shapes (lightweight) */
  float Dy     = mu * Fz;
  float Cy     = 1.35f;
  float By     = tp->cornering_stiffness / fmaxf(Cy * Dy, 1.0f);
  float Fy_lin = -Dy * sinf(Cy * atanf(By * w->slip_angle)) - tp->camber_stiffness * w->camber_angle;
  float Dx     = mu * Fz;
  float Cx     = 1.30f;
  float Bx     = tp->stiffness / fmaxf(Cx * Dx, 1.0f);
  float Fx_lin = Dx * sinf(Cx * atanf(Bx * w->slip_ratio));

  /* combined-slip weighting (friction ellipse with curvature) */
  float Fx0 = Fx_lin, Fy0 = Fy_lin;
  float denom = fmaxf(LCC_EPS, sqrtf(Fx0 * Fx0 + Fy0 * Fy0));
  float scale = fminf(1.0f, mu * Fz / denom);
  float Fx    = Fx0 * scale;
  float Fy    = Fy0 * scale;

  /* rolling resistance (small, sign-aware) */
  float Crr    = tp->rolling_resistance;
  float Frr    = Crr * Fz * (vref_long / (vref_long + 5.0f));
  float rr_dir = (fabsf(v_long) > 1e-3f) ? -lcc_sign(v_long) : -(lcc_sign(w->angular_velocity));
  Fx += Frr * rr_dir;

  /* wheel spin dynamics */
  float Iw        = fmaxf(w->rotational_inertia, 1e-4f);
  float omega_abs = fabsf(w->angular_velocity);

  /* brake opposes rotation, with smooth sign */
  float eps       = fmaxf(0.02f, car->timestep);
  float brake_dir = (omega_abs > 0.0f) ? (w->angular_velocity / sqrtf(omega_abs * omega_abs + eps * eps)) : 0.0f;
  float T_brake   = w->brake_torque * brake_dir;
  float T_contact = -Fx * R; /* road torque at hub */
  float T_net     = w->drive_torque + T_contact - T_brake;
  float domega    = (T_net / Iw) * car->timestep;

  domega = lcc_clamp(domega, -500.0f, 500.0f);
  w->angular_velocity += domega;

  /* lock wheels when braking at low speeds */
  if(car->brake_input > 0.1f && fabsf(w->angular_velocity) < 0.5f) {
    float static_torques = w->drive_torque + T_contact;
    if(fabsf(static_torques) < w->brake_torque) w->angular_velocity = 0.0f;
  }

  /* outputs */
  w->Fx = Fx;
  w->Fy = Fy;

  /* tire thermal/wear simple model */
  float       slip_power = fabsf(Fx) * (fabsf(w->angular_velocity * R - v_long)) + fabsf(Fy) * fabsf(v_lat);
  float       v_gate     = v_abs / (v_abs + 1.0f);
  const float c_heat     = 3.5e-4f; /* K/(N*m/s) */
  float       k_cool     = 0.04f + 0.012f * lcc_length((float *)car->velocity);
  float       dTdt       = c_heat * slip_power * v_gate - k_cool * (w->temperature - car->ambient_temp);
  w->temperature         = lcc_clamp(w->temperature + dTdt * car->timestep, car->ambient_temp, 160.0f);
  tp->wear               = lcc_clamp(tp->wear + (slip_power * car->timestep) * 1.0e-7f, 0.0f, 1.0f);
}

/* ---------- vehicle integration ---------- */
static void lcc_vehicle_step(lcc_car_t *car) {
  float cx = cosf(car->angle), sx = sinf(car->angle);
  /* sum tire forces in world */
  float Fw[2] = { 0.0f, 0.0f };
  float Mz    = 0.0f;
  for(int i = 0; i < 4; ++i) {
    lcc_wheel_state_t *w   = &car->wheels[i];
    float              psi = car->angle + w->steer_angle;
    float              cw = cosf(psi), sw = sinf(psi);
    float              Fxw = w->Fx * cw - w->Fy * sw;
    float              Fyw = w->Fx * sw + w->Fy * cw;
    Fw[0] += Fxw;
    Fw[1] += Fyw;
    float Fb[2];
    lcc_world_to_body(cx, sx, (float[2]){ Fxw, Fyw }, Fb);
    float rb[2] = { w->position[0], w->position[1] };
    Mz += rb[0] * Fb[1] - rb[1] * Fb[0];
  }
  /* aero */
  float Fdrag[2], DFf = 0.0f, DFr = 0.0f;
  lcc_aero_forces(car, Fdrag, &DFf, &DFr);
  Fw[0] += Fdrag[0];
  Fw[1] += Fdrag[1];
  /* accelerations */
  float ax = Fw[0] / car->mass;
  float ay = Fw[1] / car->mass;
  car->velocity[0] += ax * car->timestep;
  car->velocity[1] += ay * car->timestep;
  car->position[0] += car->velocity[0] * car->timestep;
  car->position[1] += car->velocity[1] * car->timestep;
  float yaw_acc = Mz / fmaxf(car->inertia, 1e-3f);
  car->angular_velocity += yaw_acc * car->timestep;
  car->angle += car->angular_velocity * car->timestep;

  /* wrap angle */
  if(car->angle > LCC_PI) car->angle -= 2.0f * LCC_PI;
  if(car->angle < -LCC_PI) car->angle += 2.0f * LCC_PI;

  /* compute body accelerations for next-step load transfer */
  float Vb[2];
  lcc_world_to_body(cx, sx, (float[2]){ ax, ay }, Vb);
  lcc_compute_loads(car, Vb[0], Vb[1], DFf, DFr);
}

/* ---------- ackermann steering model ---------- */
/* https://en.wikipedia.org/wiki/Ackermann_steering_geometry */
static void ackermann_steering(const lcc_car_t *car, float steering_input, float *out_angle_i, float *out_angle_o) {
  float max_sa = 40.0f * (LCC_PI / 180.0f);

  if(fabsf(steering_input) < LCC_EPS) {
    *out_angle_i = 0.0f;
    *out_angle_o = 0.0f;
  } else {
    float L           = car->wheelbase;
    float T           = car->track_width;
    float delta       = -steering_input * max_sa;
    float R           = L / tanf(fabsf(delta));
    float abs_angle_i = atanf(L / (R - 0.5f * T));
    float abs_angle_o = atanf(L / (R + 0.5f * T));
    float sign        = lcc_sign(delta);
    float angle_i     = abs_angle_i * sign;
    float angle_o     = abs_angle_o * sign;
    if(steering_input > 0.0f) {
      *out_angle_i = angle_o;
      *out_angle_o = angle_i;
    } else {
      *out_angle_i = angle_i;
      *out_angle_o = angle_o;
    }
  }
}

/* ---------- API impl ---------- */
void lcc_car_shift_up(lcc_car_t *car) {
  if(car->transmission.current_gear < car->transmission.num_gears) car->transmission.current_gear++;
}

void lcc_car_shift_down(lcc_car_t *car) {
  if(car->transmission.current_gear > -1) car->transmission.current_gear--;
}

void lcc_car_set_gear(lcc_car_t *car, int gear) {
  if(gear >= -1 && gear <= car->transmission.num_gears) car->transmission.current_gear = gear;
}

void lcc_car_set_inputs(lcc_car_t *car, float throttle, float brake, float steering, float clutch) {
  car->throttle_input = lcc_clamp(throttle, -1.0f, 1.0f);
  car->brake_input    = lcc_clamp(brake, 0.0f, 1.0f);
  car->steering_input = lcc_clamp(steering, -1.0f, 1.0f);
  car->clutch_input   = lcc_clamp(clutch, 0.0f, 1.0f);
}

void lcc_car_update(lcc_car_t *car, float dt) {
  if(dt <= 0.0f) return;
  car->timestep = dt;
  car->simulation_time += dt;

  ackermann_steering(car, car->steering_input, &car->wheels[0].steer_angle, &car->wheels[1].steer_angle);
  /* four wheel steering xd ackermann_steering(car, -car->steering_input, &car->wheels[3].steer_angle, &car->wheels[2].steer_angle); */

  /* throttle filter */
  float tau             = fmaxf(car->engine.response_time, 1e-3f);
  float k               = 1.0f - expf(-dt / tau);
  float target_throttle = lcc_clamp(car->throttle_input, 0.0f, 1.0f);
  car->engine.throttle  = lcc_lerp(car->engine.throttle, target_throttle, k);

  lcc_engine_physics(car);
  lcc_transmission_physics(car);
  for(int i = 0; i < 4; ++i) lcc_tire_step(car, i);
  lcc_vehicle_step(car);
}

float lcc_car_get_speed(const lcc_car_t *car) {
  return lcc_length((float *)car->velocity) * 3.6f;
}

float lcc_car_get_engine_rpm(const lcc_car_t *car) {
  return car->engine.current_rpm;
}

const char *lcc_get_version(void) {
  return LCC_VERSION;
}

/* ---------- presets & ctor ---------- */
lcc_car_t lcc_car_create(lcc_preset_t preset) {
  lcc_car_t car;
  memset(&car, 0, sizeof(car));
  car.mass             = 1500.0f;
  car.wheelbase        = 2.7f;
  car.track_width      = 1.6f;
  car.cg_height        = 0.50f;
  car.cg_position      = 0.55f; /* 55% towards rear axle */
  car.air_density      = LCC_AIR_DENSITY;
  car.ambient_temp     = 20.0f;
  car.surface_friction = 1.0f;
  car.front_brake_bias = 0.65f;
  car.max_brake_torque = 8000.0f;
  /* tire baseline */
  for(int i = 0; i < 4; ++i) {
    lcc_tire_params_t *tp   = &car.tire_params[i];
    tp->radius              = 0.32f;
    tp->width               = 0.22f;
    tp->aspect_ratio        = 0.5f;
    tp->pressure            = 220.0f;
    tp->nominal_load        = 3500.0f;
    tp->peak_friction       = 1.25f;
    tp->slip_friction       = 0.85f;
    tp->stiffness           = 90000.0f;
    tp->cornering_stiffness = 12000.0f;
    tp->camber_stiffness    = 30000.0f;
    tp->rolling_resistance  = 0.013f;
    tp->temperature         = car.ambient_temp;
    tp->wear                = 0.0f;
    tp->relax_length_long   = 0.30f;
    tp->relax_length_lat    = 0.50f;
    tp->load_sensitivity    = 0.25f;
    tp->mu_min              = 0.6f;
    tp->mu_max              = 1.9f;
    lcc_wheel_state_t *w    = &car.wheels[i];
    w->surface_friction     = car.surface_friction;
    w->temperature          = car.ambient_temp;
    w->rotational_inertia   = 1.2f;
    w->drive_torque         = 0.0f;
    w->brake_torque         = 0.0f;
  }

  /* wheel positions */
  car.wheels[0].position[0] = +car.wheelbase * 0.5f;
  car.wheels[0].position[1] = +car.track_width * 0.5f; /* FL */
  car.wheels[1].position[0] = +car.wheelbase * 0.5f;
  car.wheels[1].position[1] = -car.track_width * 0.5f; /* FR */
  car.wheels[2].position[0] = -car.wheelbase * 0.5f;
  car.wheels[2].position[1] = +car.track_width * 0.5f; /* RL */
  car.wheels[3].position[0] = -car.wheelbase * 0.5f;
  car.wheels[3].position[1] = -car.track_width * 0.5f; /* RR */

  /* engine */
  car.engine.max_power          = 200000.0f;
  car.engine.max_torque         = 400.0f;
  car.engine.idle_rpm           = 800.0f;
  car.engine.max_rpm            = 6000.0f;
  car.engine.redline_rpm        = 6500.0f;
  car.engine.inertia            = 0.20f;
  car.engine.friction           = 0.05f;
  car.engine.response_time      = 0.10f;
  car.engine.current_rpm        = car.engine.idle_rpm;
  car.engine.throttle           = 0.0f;
  car.engine.output_torque      = 0.0f;
  car.engine.peak_torque_rpm    = 3500.0f;
  car.engine.peak_power_rpm     = 5800.0f;
  car.engine.engine_brake_coeff = 0.08f;
  car.engine.friction_quadratic = 5e-4f;
  car.engine.idle_torque        = 30.0f;
  car.engine.stall_rpm          = 600.0f;

  /* gearbox */
  car.transmission.num_gears      = 6;
  car.transmission.gear_ratios[0] = 3.50f;
  car.transmission.gear_ratios[1] = 2.00f;
  car.transmission.gear_ratios[2] = 1.40f;
  car.transmission.gear_ratios[3] = 1.10f;
  car.transmission.gear_ratios[4] = 0.90f;
  car.transmission.gear_ratios[5] = 0.70f;
  car.transmission.final_drive    = 3.7f;
  car.transmission.reverse_ratio  = 3.2f;
  car.transmission.efficiency     = 0.92f;
  car.transmission.current_gear   = 0;
  car.transmission.drive_type     = LCC_DRIVE_RWD;

  /* diff */
  car.differential.preload             = 50.0f;
  car.differential.power_factor        = 0.25f;
  car.differential.coast_factor        = 0.20f;
  car.differential.viscous_coefficient = 5.0f;
  car.differential.bias_limit          = 600.0f;

  /* aero */
  car.aerodynamics.drag_coefficient      = 0.30f;
  car.aerodynamics.frontal_area          = 2.0f;
  car.aerodynamics.downforce_coefficient = 0.10f;
  car.aerodynamics.downforce_area        = 2.0f;
  car.aerodynamics.lift_coefficient      = 0.0f;
  car.aerodynamics.aero_balance_front    = 0.55f;

  /* inertia */
  car.inertia = car.mass * (car.wheelbase * car.wheelbase + car.track_width * car.track_width) / 12.0f;

  /* presets. values taken from gpt-5-high */
  switch(preset) {
  case LCC_PRESET_ECONOMY: { /* 2018 Honda Civic 1.5T */
    car.mass        = 1270.0f;
    car.wheelbase   = 2.70f;
    car.track_width = 1.56f;
    car.cg_height   = 0.52f;
    car.cg_position = 0.42f;

    car.engine.max_power          = 127000.0f;
    car.engine.max_torque         = 220.0f;
    car.engine.idle_rpm           = 750.0f;
    car.engine.max_rpm            = 6500.0f;
    car.engine.redline_rpm        = 6700.0f;
    car.engine.peak_torque_rpm    = 2000.0f;
    car.engine.peak_power_rpm     = 5600.0f;
    car.engine.inertia            = 0.18f;
    car.engine.response_time      = 0.12f;
    car.engine.friction           = 0.040f;
    car.engine.friction_quadratic = 3.0e-4f;
    car.engine.engine_brake_coeff = 0.060f;
    car.engine.idle_torque        = 25.0f;
    car.engine.stall_rpm          = 550.0f;

    car.transmission.num_gears      = 6;
    car.transmission.gear_ratios[0] = 3.64f;
    car.transmission.gear_ratios[1] = 2.08f;
    car.transmission.gear_ratios[2] = 1.36f;
    car.transmission.gear_ratios[3] = 1.03f;
    car.transmission.gear_ratios[4] = 0.86f;
    car.transmission.gear_ratios[5] = 0.69f;
    car.transmission.final_drive    = 4.10f;
    car.transmission.reverse_ratio  = 3.58f;
    car.transmission.efficiency     = 0.92f;
    car.transmission.drive_type     = LCC_DRIVE_FWD;

    car.differential.preload             = 0.0f;
    car.differential.power_factor        = 0.0f;
    car.differential.coast_factor        = 0.0f;
    car.differential.viscous_coefficient = 2.0f;
    car.differential.bias_limit          = 0.0f;

    car.aerodynamics.drag_coefficient      = 0.27f;
    car.aerodynamics.frontal_area          = 2.20f;
    car.aerodynamics.downforce_coefficient = 0.00f;
    car.aerodynamics.downforce_area        = 2.20f;
    car.aerodynamics.aero_balance_front    = 0.60f;

    car.front_brake_bias = 0.67f;
    car.max_brake_torque = 6000.0f;

    for(int i = 0; i < 4; ++i) {
      lcc_tire_params_t *tp            = &car.tire_params[i];
      tp->width                        = 0.215f;
      tp->aspect_ratio                 = 0.55f;
      tp->radius                       = 0.315f;
      tp->pressure                     = 230.0f;
      tp->nominal_load                 = car.mass * LCC_GRAVITY / 4.0f;
      tp->peak_friction                = 1.05f;
      tp->slip_friction                = 0.90f;
      tp->stiffness                    = 75000.0f;
      tp->cornering_stiffness          = 50000.0f;
      tp->camber_stiffness             = 18000.0f;
      tp->rolling_resistance           = 0.011f;
      tp->relax_length_long            = 0.30f;
      tp->relax_length_lat             = 0.55f;
      tp->load_sensitivity             = 0.25f;
      tp->mu_min                       = 0.75f;
      tp->mu_max                       = 1.15f;
      car.wheels[i].rotational_inertia = 1.00f;
    }
  } break;

  case LCC_PRESET_MIDSIZE: { /* Camry 2.5 */
    car.mass        = 1550.0f;
    car.wheelbase   = 2.82f;
    car.track_width = 1.595f;
    car.cg_height   = 0.52f;
    car.cg_position = 0.44f;

    car.engine.max_power          = 151000.0f;
    car.engine.max_torque         = 250.0f;
    car.engine.idle_rpm           = 680.0f;
    car.engine.max_rpm            = 6600.0f;
    car.engine.redline_rpm        = 6800.0f;
    car.engine.peak_torque_rpm    = 4100.0f;
    car.engine.peak_power_rpm     = 6600.0f;
    car.engine.inertia            = 0.20f;
    car.engine.response_time      = 0.12f;
    car.engine.friction           = 0.050f;
    car.engine.friction_quadratic = 3.0e-4f;
    car.engine.engine_brake_coeff = 0.070f;
    car.engine.idle_torque        = 26.0f;
    car.engine.stall_rpm          = 550.0f;

    car.transmission.num_gears      = 6;
    car.transmission.gear_ratios[0] = 3.54f;
    car.transmission.gear_ratios[1] = 2.05f;
    car.transmission.gear_ratios[2] = 1.39f;
    car.transmission.gear_ratios[3] = 1.00f;
    car.transmission.gear_ratios[4] = 0.73f;
    car.transmission.gear_ratios[5] = 0.59f;
    car.transmission.final_drive    = 3.36f;
    car.transmission.reverse_ratio  = 3.16f;
    car.transmission.efficiency     = 0.92f;
    car.transmission.drive_type     = LCC_DRIVE_FWD;

    car.differential.preload             = 0.0f;
    car.differential.power_factor        = 0.0f;
    car.differential.coast_factor        = 0.0f;
    car.differential.viscous_coefficient = 3.0f;
    car.differential.bias_limit          = 0.0f;

    car.aerodynamics.drag_coefficient      = 0.28f;
    car.aerodynamics.frontal_area          = 2.25f;
    car.aerodynamics.downforce_coefficient = 0.00f;
    car.aerodynamics.downforce_area        = 2.25f;
    car.aerodynamics.aero_balance_front    = 0.58f;

    car.front_brake_bias = 0.65f;
    car.max_brake_torque = 8000.0f;

    for(int i = 0; i < 4; ++i) {
      lcc_tire_params_t *tp            = &car.tire_params[i];
      tp->width                        = 0.225f;
      tp->aspect_ratio                 = 0.50f;
      tp->radius                       = 0.330f;
      tp->pressure                     = 230.0f;
      tp->nominal_load                 = car.mass * LCC_GRAVITY / 4.0f;
      tp->peak_friction                = 1.05f;
      tp->slip_friction                = 0.90f;
      tp->stiffness                    = 80000.0f;
      tp->cornering_stiffness          = 55000.0f;
      tp->camber_stiffness             = 20000.0f;
      tp->rolling_resistance           = 0.0105f;
      tp->relax_length_long            = 0.32f;
      tp->relax_length_lat             = 0.55f;
      tp->load_sensitivity             = 0.25f;
      tp->mu_min                       = 0.75f;
      tp->mu_max                       = 1.20f;
      car.wheels[i].rotational_inertia = 1.05f;
    }
  } break;

  case LCC_PRESET_SPORTS: { /* BMW M3 (F80) */
    car.mass        = 1575.0f;
    car.wheelbase   = 2.81f;
    car.track_width = 1.58f;
    car.cg_height   = 0.35f;
    car.cg_position = 0.47f;

    car.engine.max_power          = 317000.0f;
    car.engine.max_torque         = 550.0f;
    car.engine.idle_rpm           = 800.0f;
    car.engine.max_rpm            = 7500.0f;
    car.engine.redline_rpm        = 7600.0f;
    car.engine.peak_torque_rpm    = 3000.0f;
    car.engine.peak_power_rpm     = 7300.0f;
    car.engine.inertia            = 0.23f;
    car.engine.response_time      = 0.09f;
    car.engine.friction           = 0.060f;
    car.engine.friction_quadratic = 5.5e-4f;
    car.engine.engine_brake_coeff = 0.10f;
    car.engine.idle_torque        = 30.0f;
    car.engine.stall_rpm          = 650.0f;

    car.transmission.num_gears      = 6;
    car.transmission.gear_ratios[0] = 4.11f;
    car.transmission.gear_ratios[1] = 2.32f;
    car.transmission.gear_ratios[2] = 1.54f;
    car.transmission.gear_ratios[3] = 1.18f;
    car.transmission.gear_ratios[4] = 1.00f;
    car.transmission.gear_ratios[5] = 0.85f;
    car.transmission.final_drive    = 3.46f;
    car.transmission.reverse_ratio  = 3.68f;
    car.transmission.efficiency     = 0.92f;
    car.transmission.drive_type     = LCC_DRIVE_RWD;

    car.differential.preload             = 80.0f;
    car.differential.power_factor        = 0.35f;
    car.differential.coast_factor        = 0.25f;
    car.differential.viscous_coefficient = 5.0f;
    car.differential.bias_limit          = 1200.0f;

    car.aerodynamics.drag_coefficient      = 0.34f;
    car.aerodynamics.frontal_area          = 2.20f;
    car.aerodynamics.downforce_coefficient = 0.03f;
    car.aerodynamics.downforce_area        = 2.20f;
    car.aerodynamics.aero_balance_front    = 0.52f;

    car.front_brake_bias = 0.62f;
    car.max_brake_torque = 11000.0f;

    for(int i = 0; i < 4; ++i) {
      lcc_tire_params_t *tp            = &car.tire_params[i];
      tp->width                        = 0.265f;
      tp->aspect_ratio                 = 0.35f;
      tp->radius                       = 0.330f;
      tp->pressure                     = 230.0f;
      tp->nominal_load                 = car.mass * LCC_GRAVITY / 4.0f;
      tp->peak_friction                = 1.20f;
      tp->slip_friction                = 0.95f;
      tp->stiffness                    = 90000.0f;
      tp->cornering_stiffness          = 65000.0f;
      tp->camber_stiffness             = 40000.0f;
      tp->rolling_resistance           = 0.010f;
      tp->relax_length_long            = 0.33f;
      tp->relax_length_lat             = 0.50f;
      tp->load_sensitivity             = 0.25f;
      tp->mu_min                       = 0.85f;
      tp->mu_max                       = 1.35f;
      car.wheels[i].rotational_inertia = 1.10f;
    }
  } break;

  case LCC_PRESET_SUPERCAR: { /* Ferrari 488 GTB-ish */
    car.mass        = 1475.0f;
    car.wheelbase   = 2.65f;
    car.track_width = 1.67f;
    car.cg_height   = 0.34f;
    car.cg_position = 0.585f;

    car.engine.max_power          = 492000.0f;
    car.engine.max_torque         = 760.0f;
    car.engine.idle_rpm           = 800.0f;
    car.engine.max_rpm            = 8000.0f;
    car.engine.redline_rpm        = 8200.0f;
    car.engine.peak_torque_rpm    = 3000.0f;
    car.engine.peak_power_rpm     = 8000.0f;
    car.engine.inertia            = 0.24f;
    car.engine.response_time      = 0.07f;
    car.engine.friction           = 0.070f;
    car.engine.friction_quadratic = 8.0e-4f;
    car.engine.engine_brake_coeff = 0.12f;
    car.engine.idle_torque        = 32.0f;
    car.engine.stall_rpm          = 650.0f;

    car.transmission.num_gears      = 6;
    car.transmission.gear_ratios[0] = 3.13f;
    car.transmission.gear_ratios[1] = 2.18f;
    car.transmission.gear_ratios[2] = 1.56f;
    car.transmission.gear_ratios[3] = 1.19f;
    car.transmission.gear_ratios[4] = 0.94f;
    car.transmission.gear_ratios[5] = 0.76f;
    car.transmission.final_drive    = 3.54f;
    car.transmission.reverse_ratio  = 2.90f;
    car.transmission.efficiency     = 0.92f;
    car.transmission.drive_type     = LCC_DRIVE_RWD;

    car.differential.preload             = 100.0f;
    car.differential.power_factor        = 0.40f;
    car.differential.coast_factor        = 0.30f;
    car.differential.viscous_coefficient = 6.0f;
    car.differential.bias_limit          = 1500.0f;

    car.aerodynamics.drag_coefficient      = 0.33f;
    car.aerodynamics.frontal_area          = 2.00f;
    car.aerodynamics.downforce_coefficient = 0.30f;
    car.aerodynamics.downforce_area        = 2.00f;
    car.aerodynamics.aero_balance_front    = 0.46f;

    car.front_brake_bias = 0.60f;
    car.max_brake_torque = 14000.0f;

    for(int i = 0; i < 4; ++i) {
      lcc_tire_params_t *tp            = &car.tire_params[i];
      tp->width                        = 0.285f;
      tp->aspect_ratio                 = 0.30f;
      tp->radius                       = 0.335f;
      tp->pressure                     = 230.0f;
      tp->nominal_load                 = car.mass * LCC_GRAVITY / 4.0f;
      tp->peak_friction                = 1.35f;
      tp->slip_friction                = 1.05f;
      tp->stiffness                    = 100000.0f;
      tp->cornering_stiffness          = 80000.0f;
      tp->camber_stiffness             = 60000.0f;
      tp->rolling_resistance           = 0.012f;
      tp->relax_length_long            = 0.35f;
      tp->relax_length_lat             = 0.55f;
      tp->load_sensitivity             = 0.25f;
      tp->mu_min                       = 0.95f;
      tp->mu_max                       = 1.55f;
      car.wheels[i].rotational_inertia = 1.15f;
    }
  } break;

  case LCC_PRESET_HYPERCAR: { /* Bugatti Chiron */
    car.mass        = 1995.0f;
    car.wheelbase   = 2.71f;
    car.track_width = 1.66f;
    car.cg_height   = 0.36f;
    car.cg_position = 0.56f;

    car.engine.max_power          = 1103000.0f;
    car.engine.max_torque         = 1600.0f;
    car.engine.idle_rpm           = 800.0f;
    car.engine.max_rpm            = 6700.0f;
    car.engine.redline_rpm        = 6900.0f;
    car.engine.peak_torque_rpm    = 2000.0f;
    car.engine.peak_power_rpm     = 6600.0f;
    car.engine.inertia            = 0.35f;
    car.engine.response_time      = 0.09f;
    car.engine.friction           = 0.090f;
    car.engine.friction_quadratic = 1.1e-3f;
    car.engine.engine_brake_coeff = 0.14f;
    car.engine.idle_torque        = 40.0f;
    car.engine.stall_rpm          = 650.0f;

    car.transmission.num_gears      = 6;
    car.transmission.gear_ratios[0] = 3.286f;
    car.transmission.gear_ratios[1] = 2.130f;
    car.transmission.gear_ratios[2] = 1.556f;
    car.transmission.gear_ratios[3] = 1.157f;
    car.transmission.gear_ratios[4] = 0.852f;
    car.transmission.gear_ratios[5] = 0.628f;
    car.transmission.final_drive    = 2.80f;
    car.transmission.reverse_ratio  = 2.90f;
    car.transmission.efficiency     = 0.92f;
    car.transmission.drive_type     = LCC_DRIVE_AWD;

    car.differential.preload             = 120.0f;
    car.differential.power_factor        = 0.30f;
    car.differential.coast_factor        = 0.25f;
    car.differential.viscous_coefficient = 8.0f;
    car.differential.bias_limit          = 1800.0f;

    car.aerodynamics.drag_coefficient      = 0.35f;
    car.aerodynamics.frontal_area          = 2.07f;
    car.aerodynamics.downforce_coefficient = 0.25f;
    car.aerodynamics.downforce_area        = 2.00f;
    car.aerodynamics.aero_balance_front    = 0.45f;

    car.front_brake_bias = 0.58f;
    car.max_brake_torque = 16000.0f;

    for(int i = 0; i < 4; ++i) {
      lcc_tire_params_t *tp            = &car.tire_params[i];
      tp->width                        = 0.315f;
      tp->aspect_ratio                 = 0.30f;
      tp->radius                       = 0.360f;
      tp->pressure                     = 230.0f;
      tp->nominal_load                 = car.mass * LCC_GRAVITY / 4.0f;
      tp->peak_friction                = 1.25f;
      tp->slip_friction                = 1.00f;
      tp->stiffness                    = 110000.0f;
      tp->cornering_stiffness          = 80000.0f;
      tp->camber_stiffness             = 65000.0f;
      tp->rolling_resistance           = 0.011f;
      tp->relax_length_long            = 0.37f;
      tp->relax_length_lat             = 0.58f;
      tp->load_sensitivity             = 0.28f;
      tp->mu_min                       = 0.90f;
      tp->mu_max                       = 1.45f;
      car.wheels[i].rotational_inertia = 1.50f;
    }
  } break;

  default: break;
  }
  car.inertia = car.mass * (car.wheelbase * car.wheelbase + car.track_width * car.track_width) / 12.0f;
  /* initialize load filter with static + aero at rest */
  float dummyFdrag[2];
  float DFf = 0.0f, DFr = 0.0f;
  lcc_aero_forces(&car, dummyFdrag, &DFf, &DFr);
  float W           = car.mass * LCC_GRAVITY;
  float Wf          = W * (1.0f - car.cg_position) + DFf;
  float Wr          = W * (car.cg_position) + DFr;
  car._Fz_smooth[0] = car._Fz_smooth[1] = 0.5f * Wf;
  car._Fz_smooth[2] = car._Fz_smooth[3] = 0.5f * Wr;
  return car;
}

void lcc_car_destroy(lcc_car_t *car) {
  (void)car;
}
#endif /* LIBCCAR_IMPLEMENTATION */
#endif /* LIBCCAR_H */
