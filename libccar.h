/*
  libccar - "simple" C99 2D top-down car simulation

  What this does
  --------------
  - Single-file C99 library for quick experiments.
  - u dont deserve to know rn aka. im lazy. (wip)

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

/* version and constants {{{*/
#define LCC_VERSION     "0.6.0"
#define LCC_PI          (3.14159265358979323846f)
#define LCC_GRAVITY     (9.81f)  /* m/s^2 */
#define LCC_AIR_DENSITY (1.225f) /* kg/m^3 at sea level */
#define LCC_EPS         (1e-6f)
#define LCC_RAD_PER_RPM (2.0f * LCC_PI / 60.0f) /* rad/s per RPM */

/*}}}*/

/* enums {{{*/

/* built-in car presets */
typedef enum {
  LCC_PRESET_ECONOMY,
  LCC_PRESET_MIDSIZE,
  LCC_PRESET_SPORTS,
  LCC_PRESET_SUPERCAR,
  LCC_PRESET_HYPERCAR,
} lcc_preset_t;

/* drive configuration */
typedef enum {
  LCC_DRIVE_RWD, /* rear-wheel drive */
  LCC_DRIVE_FWD, /* front-wheel drive */
  LCC_DRIVE_AWD, /* all-wheel drive */
} lcc_drive_t;

/* engine state for the state machine */
typedef enum {
  LCC_ENGINE_OFF,      /* stopped: no power, no rotation unless back-driven by wheels */
  LCC_ENGINE_CRANKING, /* starter active: trying to start */
  LCC_ENGINE_RUNNING,  /* normal operation */
} lcc_engine_state_t;

/* rev limiter behavior */
typedef enum {
  LCC_REV_CUT_FUEL,  /* hard fuel cut */
  LCC_REV_CUT_SPARK, /* spark cut */
  LCC_REV_CUT_MIXED, /* fuel cut at hard limit + soft spark cut near redline */
} lcc_rev_limit_mode_t;

/* key states */
typedef enum {
  LCC_KEY_OFF, /* battery off-line (no ECU/pump), accessories off */
  LCC_KEY_RUN, /* electronics, ECU and fuel system active (engine can run if already started) */
} lcc_key_state_t;

/* ignition states */
typedef enum {
  LCC_IGNITION_OFF,
  LCC_IGNITION_ON, /* starts the car when key is in LCC_KEY_RUN */
} lcc_ignition_state_t;

/* tire parameters for each wheel */
typedef struct {
  /* geometry */
  float radius;       /* m */
  float width;        /* m */
  float aspect_ratio; /* ratio height/width */
  float pressure;     /* kPa nominal */
  float nominal_load; /* N */

  /* friction model baseline and stiffnesses */
  float peak_friction;       /* mu_peak baseline at nominal load and temperature */
  float slip_friction;       /* mu at small slip */
  float stiffness;           /* longitudinal stiffness scale (N per slip unit) */
  float cornering_stiffness; /* lateral stiffness scale (N/rad) */
  float camber_stiffness;    /* camber stiffness (N/rad) */
  float rolling_resistance;  /* Crr constant */
  float temperature;         /* degC current tire temperature */
  float wear;                /* 0..1 progressive wear */

  /* dynamics and load sensitivity */
  float relax_length_long; /* m, relaxation length for slip ratio */
  float relax_length_lat;  /* m, relaxation length for slip angle */
  float load_sensitivity;  /* 0..1 reduces mu with load */
  float mu_min, mu_max;    /* clamps on mu */
} lcc_tire_params_t;

/* wheel state (steering, camber, rotational state, forces) */
typedef struct {
  float angular_velocity; /* rad/s wheel spin */
  float steer_angle;      /* rad wheel steer angle */
  float camber_angle;     /* rad camber (+ = top toward vehicle) */
  float slip_angle;       /* filtered slip angle (rad) */
  float slip_ratio;       /* filtered slip ratio (-) */
  float load;             /* N vertical load */
  float position[2];      /* m in body frame (x forward, y left) */
  float temperature;      /* degC */
  float surface_friction; /* surface mu multiplier */

  /* outputs (forces in tire frame) and actuator/rotational parameters */
  float Fx, Fy;             /* N longitudinal/lateral */
  float drive_torque;       /* Nm from driveline */
  float brake_torque;       /* Nm braking (resistive) */
  float rotational_inertia; /* kg*m^2 */
} lcc_wheel_state_t;

/* engine model + state machine (compact flags for memory efficiency) */
typedef struct {
  /* base characteristics */
  float max_power;  /* W */
  float max_torque; /* Nm */
  float idle_rpm, max_rpm, redline_rpm;
  float inertia;       /* kg*m^2 (crank equivalent) */
  float friction;      /* Nm/(rad/s) viscous friction */
  float response_time; /* throttle lag (s) */
  float current_rpm;
  float throttle;      /* 0..1 filtered input */
  float output_torque; /* Nm produced (pre losses at crank) */

  /* torque curve shaping and losses */
  float peak_torque_rpm;
  float peak_power_rpm;
  float engine_brake_coeff; /* Nm/(rad/s) scales with (1-throttle) */
  float friction_quadratic; /* Nm/(rad/s)^2 */
  float idle_torque;        /* Nm extra assist near idle */
  float stall_rpm;          /* rpm below which stall is considered */

  /* limiters and cuts */
  float decel_fuel_cut_rpm;      /* rpm threshold for decel fuel cut */
  float decel_fuel_cut_throttle; /* throttle threshold for decel fuel cut */
  float rev_limiter_hyst;        /* rpm hysteresis below redline */
  float rev_limiter_soft_zone;   /* rpm band for soft spark cut */
  float rev_limiter_cut_ratio;   /* 0..1 fraction under soft spark cut */
  float min_start_rpm;           /* rpm needed to self-sustain */

  /* starter parameters and lockouts */
  float starter_torque;      /* Nm assisted at crank while START */
  float starter_power_watts; /* W electrical draw while START */
  float starter_efficiency;  /* mech/electrical ratio */

  /* idle controller parameters */
  float idle_target_rpm;      /* rpm target for idle */
  float idle_P, idle_I;       /* proportional and integral gains */
  float idle_integrator;      /* state */
  float idle_max_torque;      /* Nm clamp for idle control */
  float idle_throttle_window; /* active only if user throttle below this */

  /* compact state/flags (packed to reduce padding) */
  uint8_t state;                  /* lcc_engine_state_t (OFF/CRANKING/RUNNING) */
  uint8_t key_pos;                /* lcc_key_state_t (OFF/RUN/START) */
  uint8_t ignition;               /* lcc_ignitions_state_t (OFF/ON) accessed externally */
  uint8_t fuel_cut_active;        /* 0/1 */
  uint8_t spark_cut_active;       /* 0/1 */
  uint8_t decel_fuel_cut_enabled; /* 0/1 */
  uint8_t rev_limiter_mode;       /* lcc_rev_limit_mode_t */
  uint8_t _pad0;                  /* alignment */

  /* stall detection */
  float stall_time_accum; /* s under stall conditions so far */
  float stall_delay;      /* s before declaring stall */
  float last_net_torque;  /* Nm net torque at crank last step */
} lcc_engine_t;

/* transmission and drive configuration */
typedef struct {
  int         num_gears;      /* 1..8 */
  float       gear_ratios[8]; /* indexed gear 1..num_gears => [0..num_gears-1] */
  float       final_drive;    /* axle ratio */
  float       reverse_ratio;  /* reverse gear ratio */
  float       efficiency;     /* 0..1 overall efficiency */
  int         current_gear;   /* -1=R, 0=N, 1..num_gears */
  lcc_drive_t drive_type;     /* which axle(s) are driven */
} lcc_transmission_t;

/* simple clutch-type differential model (quasi torque-biasing) */
typedef struct {
  float preload;             /* Nm baseline locking */
  float power_factor;        /* Nm/Nm scales with input torque on power */
  float coast_factor;        /* Nm/Nm on coast */
  float viscous_coefficient; /* Nm/(rad/s) term opposing wheel speed diff */
  float bias_limit;          /* Nm max locking action */
} lcc_differential_t;

/* aerodynamic properties */
typedef struct {
  float drag_coefficient;      /* Cd */
  float frontal_area;          /* m^2 */
  float downforce_coefficient; /* Cl (positive downward) */
  float downforce_area;        /* m^2 */
  float aero_balance_front;    /* 0..1 fraction of downforce at front axle */
} lcc_aerodynamics_t;

/* electrical system: battery model */
typedef struct {
  float capacity_Ah;          /* nominal capacity */
  float soc;                  /* 0..1 state of charge */
  float internal_resistance;  /* ohm (effective) */
  float voltage;              /* V terminal voltage */
  float min_ignition_voltage; /* V below which ECU/spark disabled */
  float min_starter_voltage;  /* V below which starter inhibited */
  float temp_C;               /* degC (for future temp-dependent behaviors) */

  float accessory_load_watts; /* W user accessories */
  float ecu_load_watts;       /* W ECU draw when RUN/START */
  float fuel_pump_watts;      /* W pump draw when RUN/START */
  float parasitic_watts;      /* always-on quiescent draw when key OFF as well */
} lcc_battery_t;

/* alternator model (regulated to target voltage) */
typedef struct {
  float rated_power_W;    /* W max electrical power at high rpm */
  float efficiency;       /* electrical/mechanical */
  float cut_in_rpm;       /* alternator rpm threshold for any output */
  float pulley_ratio;     /* alternator_rpm = engine_rpm * ratio */
  float target_voltage;   /* regulator target voltage */
  float current_output_W; /* W last step output */
} lcc_alternator_t;

/* fuel system and consumption model */
typedef struct {
  float   tank_capacity_L;  /* L tank size */
  float   fuel_level_L;     /* L current level */
  float   density_kg_per_L; /* ~0.745 for gasoline */
  float   bsfc_best_gpkWh;  /* best (lower) g/kWh at optimal load */
  float   bsfc_worst_gpkWh; /* worst (higher) g/kWh at idle/very low load */
  uint8_t pump_ok;          /* 1 if pump enabled (RUN/START and voltage OK) */
  uint8_t _padF[3];         /* alignment */
} lcc_fuel_system_t;

/* vehicle state: pose, subsystems, wheels, and telemetry fields */
typedef struct {
  /* body mass properties */
  float mass;        /* kg */
  float inertia;     /* yaw inertia kg*m^2 */
  float wheelbase;   /* m */
  float track_width; /* m */
  float cg_height;   /* m above ground */
  float cg_position; /* 0..1 from front axle (0=front axle, 1=rear axle) */

  /* pose in world */
  float position[2];      /* m */
  float velocity[2];      /* m/s world frame */
  float angle;            /* yaw rad */
  float angular_velocity; /* yaw rate rad/s */

  /* subsystems */
  lcc_engine_t       engine;
  lcc_transmission_t transmission;
  lcc_differential_t differential;
  lcc_aerodynamics_t aerodynamics;
  lcc_tire_params_t  tire_params[4];
  lcc_wheel_state_t  wheels[4];

  /* electrical/fuel */
  lcc_battery_t     battery;
  lcc_alternator_t  alternator;
  lcc_fuel_system_t fuel;

  /* inputs (0..1 except steering -1..1) */
  float throttle_input, brake_input, steering_input, clutch_input;

  /* misc setup */
  float front_brake_bias; /* 0..1 front distribution */
  float max_brake_torque; /* Nm total system capacity */
  float air_density;      /* kg/m^3 */
  float ambient_temp;     /* degC */
  float surface_friction; /* global mu multiplier */

  /* integrator and filtered states */
  float timestep;        /* s */
  float simulation_time; /* s */
  float Fz_smooth[4];    /* N filtered vertical loads */
  float gbx_in_omega;    /* rad/s filtered propshaft speed (post final drive) */

  /* telemetry outputs */
  float electrical_load_W;      /* W total DC load */
  float alternator_out_W;       /* W alternator output */
  float battery_voltage;        /* V */
  float engine_mech_alt_torque; /* Nm mechanical load at crank from alternator */
} lcc_car_t;

/*}}}*/

/* public api {{{*/

/* construction / destruction */
lcc_car_t lcc_car_create(lcc_preset_t preset);
void      lcc_car_destroy(lcc_car_t *car);

/* inputs and stepping */
void lcc_car_set_inputs(lcc_car_t *car, float throttle, float brake, float steering, float clutch);
void lcc_car_update(lcc_car_t *car, float dt);

/* gearing and info */
void        lcc_car_shift_up(lcc_car_t *car);
void        lcc_car_shift_down(lcc_car_t *car);
void        lcc_car_set_gear(lcc_car_t *car, int gear);
float       lcc_car_get_speed(const lcc_car_t *car);      /* km/h */
float       lcc_car_get_engine_rpm(const lcc_car_t *car); /* rpm */
const char *lcc_get_version(void);

/* key position and subsystems */
void                 lcc_car_set_keypos(lcc_car_t *car, lcc_key_state_t key);
void                 lcc_car_set_ignition(lcc_car_t *car, lcc_ignition_state_t ignition);
lcc_key_state_t      lcc_car_get_keypos(const lcc_car_t *car);
lcc_ignition_state_t lcc_car_get_ignition(const lcc_car_t *car);
int                  lcc_car_engine_is_running(const lcc_car_t *car);

/* electrics and fuel getters/setters */
float lcc_car_get_battery_voltage(const lcc_car_t *car);
float lcc_car_get_battery_soc(const lcc_car_t *car);
void  lcc_car_set_accessory_load(lcc_car_t *car, float watts);
float lcc_car_get_fuel_level_L(const lcc_car_t *car);
float lcc_car_get_fuel_capacity_L(const lcc_car_t *car);
void  lcc_car_refuel(lcc_car_t *car, float liters);
void  lcc_car_set_fuel_level(lcc_car_t *car, float liters);

/*}}}*/

/* implementation */
#define LIBCCAR_IMPLEMENTATION
#ifdef LIBCCAR_IMPLEMENTATION

/* math helper {{{*/

/* clamp v between [lo,hi] */
static float lcc_clamp(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

/* linear interpolation between a and b by t in [0,1] */
static float lcc_lerp(float a, float b, float t) {
  return a + t * (b - a);
}

/* sign of x: -1, 0, or +1 (for float) */
static float lcc_sign(float x) {
  return (x > 0.0f) - (x < 0.0f);
}

/* vector helpers */
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

/* transform body->world and world->body using yaw cos/sin */
static void lcc_body_to_world(float cx, float sx, const float vb[2], float out[2]) {
  out[0] = vb[0] * cx - vb[1] * sx;
  out[1] = vb[0] * sx + vb[1] * cx;
}

static void lcc_world_to_body(float cx, float sx, const float vw[2], float out[2]) {
  out[0] = vw[0] * cx + vw[1] * sx;
  out[1] = -vw[0] * sx + vw[1] * cx;
}

/*}}}*/

/* engine and electrics {{{*/

/* torque curve: smooth approximation with rising torque to peak, then roll-off */
static float lcc_engine_torque_curve(const lcc_engine_t *e, float rpm, float thr) {
  float rpm_cl = lcc_clamp(rpm, e->stall_rpm, e->redline_rpm);
  float Trpm   = e->peak_torque_rpm > 0 ? e->peak_torque_rpm : (0.5f * (e->idle_rpm + e->max_rpm));
  float Prpm   = e->peak_power_rpm > 0 ? e->peak_power_rpm : (0.85f * e->redline_rpm);

  float t_norm; /* normalized torque shape 0..1 */
  if(rpm_cl <= Trpm) {
    /* smooth inverted parabola from idle to peak torque */
    float x = (rpm_cl - e->idle_rpm) / fmaxf(Trpm - e->idle_rpm, 1.0f);
    t_norm  = 0.2f + 0.8f * (1.0f - (x - 1.0f) * (x - 1.0f));
  } else if(rpm_cl < Prpm) {
    /* mild decrease toward peak power rpm */
    float x = (rpm_cl - Trpm) / fmaxf(Prpm - Trpm, 1.0f);
    t_norm  = 1.0f - 0.15f * x;
  } else {
    /* roll-off beyond peak power */
    float x = (rpm_cl - Prpm) / fmaxf(e->redline_rpm - Prpm, 1.0f);
    t_norm  = lcc_clamp(0.85f - 0.85f * x, 0.3f, 0.9f);
  }

  /* shape user throttle into a smoother curve */
  float thr_shaped = thr * thr * (3.0f - 2.0f * thr);

  /* torque command from curve and throttle */
  float torque_cmd = e->max_torque * t_norm * thr_shaped;

  return fmaxf(0.0f, torque_cmd);
}

/* battery open-circuit voltage model (simple linear) */
static float lcc_batt_ocv(float soc) {
  soc = lcc_clamp(soc, 0.0f, 1.0f);
  return 11.8f + soc * 1.0f; /* 11.8V at 0% -> 12.8V at 100% */
}

/* electrics step:
   - computes DC loads (accessory + ECU + fuel pump + starter if START)
   - computes alternator output based on rpm and regulator behavior
   - updates battery SOC and voltage (with internal resistance)
   - returns alternator mechanical torque load at crank (Nm)
*/
static float lcc_electrics_step(lcc_car_t *car, float dt, float engine_rpm, int engine_running) {
  lcc_battery_t     *bat = &car->battery;
  lcc_alternator_t  *alt = &car->alternator;
  lcc_engine_t      *e   = &car->engine;
  lcc_fuel_system_t *fu  = &car->fuel;

  /* base loads at current step */
  float Voc   = lcc_batt_ocv(bat->soc);
  float loads = fmaxf(0.0f, bat->parasitic_watts); // always-on tiny drain

  uint8_t key_run = (e->key_pos >= LCC_KEY_RUN);

  // accessories only in RUN
  if(key_run) loads += fmaxf(0.0f, bat->accessory_load_watts);

  // ECU + pump only when RUN and voltage OK
  if(key_run && bat->voltage > bat->min_ignition_voltage) {
    loads += bat->ecu_load_watts;
    if(fu->fuel_level_L > 0.01f) loads += bat->fuel_pump_watts;
  }

  /* starter electrical draw only while START and allowed by lockouts */
  int   gear            = car->transmission.current_gear;
  float veh_kph         = lcc_length((float *)car->velocity) * 3.6f;
  int   starter_allowed = (bat->voltage > bat->min_starter_voltage - 0.5f);
  int   starter_on      = (e->ignition && key_run && starter_allowed);
  if(starter_on) loads += e->starter_power_watts;

  /* alternator available electrical power based on alternator rpm */
  float alt_rpm   = engine_rpm * alt->pulley_ratio;
  float P_alt_max = 0.0f;
  if(engine_running && alt_rpm > alt->cut_in_rpm) {
    /* ramp from cut-in to full power over ~1500 alt rpm */
    float ramp = lcc_clamp((alt_rpm - alt->cut_in_rpm) / 1500.0f, 0.0f, 1.0f);
    P_alt_max  = alt->rated_power_W * ramp;
  }

  /* estimate load current using OCV (approximation) */
  float Vbat   = Voc;
  float I_load = (Vbat > 1.0f) ? (loads / Vbat) : 0.0f;

  /* regulator tries to maintain target voltage and charge low battery */
  float soc_err          = lcc_clamp(1.0f - bat->soc, 0.0f, 1.0f);
  float V_err            = lcc_clamp(alt->target_voltage - Vbat, 0.0f, 3.0f);
  float P_charge_desired = 200.0f * soc_err + 150.0f * (V_err / 3.0f);
  if(bat->soc < 0.2f) P_charge_desired += 300.0f; /* more aggressive when very low */

  float P_alt_elec = fminf(P_alt_max, loads + P_charge_desired);
  if(!engine_running) P_alt_elec = 0.0f;

  /* battery net power and updated terminal voltage with internal resistance */
  float P_batt         = loads - P_alt_elec; /* +ve means discharging */
  float Vapprox        = fmaxf(11.0f, Voc);
  float I              = (fabsf(P_batt) > 1e-3f) ? (P_batt / Vapprox) : 0.0f;
  float Vterm          = Voc - I * bat->internal_resistance;
  Vterm                = lcc_clamp(Vterm, 9.0f, 15.0f);
  bat->voltage         = Vterm;
  car->battery_voltage = Vterm;

  /* SOC integration (Ah-based) */
  float dAh      = I * (dt / 3600.0f); /* +ve discharging */
  float Ah_total = fmaxf(1e-3f, bat->capacity_Ah);
  bat->soc       = lcc_clamp(bat->soc - dAh / Ah_total, 0.0f, 1.0f);

  /* alternator mechanical load at crank */
  float omega_e = engine_rpm * LCC_RAD_PER_RPM;
  float T_alt   = 0.0f;
  if(omega_e > 5.0f && P_alt_elec > 0.0f && alt->efficiency > 0.05f) {
    float P_mech = P_alt_elec / alt->efficiency;
    T_alt        = P_mech / omega_e;
  }

  /* store telemetry and pump status */
  alt->current_output_W       = P_alt_elec;
  car->alternator_out_W       = P_alt_elec;
  car->electrical_load_W      = loads;
  car->engine_mech_alt_torque = T_alt;

  fu->pump_ok = (key_run && bat->voltage > bat->min_ignition_voltage && fu->fuel_level_L > 0.01f) ? 1 : 0;

  return T_alt;
}

/* limiters and cuts (rev limiter with hysteresis and decel fuel cut) */
static void lcc_engine_controls(lcc_engine_t *e, float rpm, int electrical_ok, int fuel_ok) {
  /* reset cuts each step; set below as needed */
  e->fuel_cut_active  = 0;
  e->spark_cut_active = 0;

  /* rev limiter logic */
  if(rpm >= e->redline_rpm) {
    /* hard limit: optional fuel cut, optional spark cut */
    if(e->rev_limiter_mode == LCC_REV_CUT_FUEL || e->rev_limiter_mode == LCC_REV_CUT_MIXED) e->fuel_cut_active = 1;
    if(e->rev_limiter_mode == LCC_REV_CUT_SPARK || e->rev_limiter_mode == LCC_REV_CUT_MIXED) e->spark_cut_active = 1;
  } else if(rpm >= e->redline_rpm - fmaxf(e->rev_limiter_hyst, 50.0f)) {
    /* soft zone near redline: apply partial spark cut proportionally */
    if(e->rev_limiter_mode != LCC_REV_CUT_FUEL) {
      float zone               = fmaxf(1.0f, e->rev_limiter_hyst);
      float alpha              = (rpm - (e->redline_rpm - zone)) / zone; /* 0..1 */
      e->spark_cut_active      = 1;
      e->rev_limiter_cut_ratio = lcc_clamp(alpha, 0.0f, 1.0f);
    }
  } else {
    e->rev_limiter_cut_ratio = 0.0f;
  }

  /* decel fuel cut: RUNNING, throttle low, rpm high */
  if(e->decel_fuel_cut_enabled && rpm > e->decel_fuel_cut_rpm && e->throttle < e->decel_fuel_cut_throttle && e->state == LCC_ENGINE_RUNNING &&
    e->key_pos >= LCC_KEY_RUN) {
    e->fuel_cut_active = 1;
  }

  /* disable combustion if not in RUN or if electrics/fuel are not OK */
  if(!electrical_ok || !fuel_ok || e->key_pos < LCC_KEY_RUN) {
    e->fuel_cut_active  = 1;
    e->spark_cut_active = 0;
  }
}

/* engine state machine:
   - enter CRANKING only when key is START (and lockouts permit)
   - transition to RUNNING only from CRANKING once rpm >= min_start_rpm and key is RUN/START
   - no bump-catch: engine never goes to RUNNING from OFF without CRANKING
*/
static void lcc_engine_state_update(lcc_car_t *car, float T_net_engine) {
  lcc_engine_t *e    = &car->engine;
  float         rpm  = e->current_rpm;
  e->last_net_torque = T_net_engine;

  /* basic ok flags */
  int electrical_ok = (car->battery.voltage > car->battery.min_ignition_voltage);
  int fuel_ok       = (car->fuel.fuel_level_L > 0.01f) && (car->fuel.pump_ok);

  /* starter lockouts */
  int   gear            = car->transmission.current_gear;
  float kph             = lcc_car_get_speed(car);
  int   starter_allowed = (car->battery.voltage > car->battery.min_starter_voltage);

  /* START => CRANKING */
  if(e->key_pos == LCC_KEY_RUN && e->ignition && electrical_ok && fuel_ok && starter_allowed) {
    e->state = LCC_ENGINE_CRANKING;
  } else if(e->state == LCC_ENGINE_CRANKING && e->key_pos < LCC_KEY_RUN) {
    /* starter released before start; if stopped, go OFF */
    if(rpm < 50.0f) e->state = LCC_ENGINE_OFF;
  }

  /* CRANKING => RUNNING when criteria are met */
  if(e->state == LCC_ENGINE_CRANKING && e->key_pos >= LCC_KEY_RUN && electrical_ok && fuel_ok) {
    if(rpm >= e->min_start_rpm) {
      e->state            = LCC_ENGINE_RUNNING;
      e->stall_time_accum = 0.0f;
    }
  }

  /* RUNNING => OFF (stall or key status change) */
  if(e->state == LCC_ENGINE_RUNNING) {
    /* stall conditions: low rpm and insufficient torque, or key/fuel/electrics loss */
    if((rpm < e->stall_rpm && T_net_engine <= 0.0f) || !electrical_ok || !fuel_ok || e->key_pos < LCC_KEY_RUN) {
      e->stall_time_accum += car->timestep;
      if(e->stall_time_accum > e->stall_delay || rpm < 0.5f * e->stall_rpm) {
        e->state            = LCC_ENGINE_OFF;
        e->stall_time_accum = 0.0f;
      }
    } else {
      e->stall_time_accum = 0.0f;
    }
  }
}

/* engine torque production step:
   - computes combustion torque (if enabled)
   - adds idle control torque and starter torque
   - applies spark cut in soft redline zone
   - sets engine.output_torque (pre-loads)
*/
static void lcc_engine_physics(lcc_car_t *car) {
  lcc_engine_t *e   = &car->engine;
  float         rpm = lcc_clamp(e->current_rpm, 0.0f, e->redline_rpm * 1.2f);

  /* check systems status */
  int electrical_ok = (car->battery.voltage > car->battery.min_ignition_voltage);
  int fuel_ok       = (car->fuel.fuel_level_L > 0.01f) && (car->fuel.pump_ok);

  /* apply limiters/cuts */
  lcc_engine_controls(e, rpm, electrical_ok, fuel_ok);

  /* combustion allowed only in RUN and under appropriate states */
  int combustion_enabled = 0;
  if(e->key_pos >= LCC_KEY_RUN && electrical_ok && fuel_ok) {
    if(e->state == LCC_ENGINE_RUNNING) combustion_enabled = !e->fuel_cut_active;
    else if(e->state == LCC_ENGINE_CRANKING && rpm >= 0.8f * e->min_start_rpm)
      combustion_enabled = 1;
  }

  float user_thr = lcc_clamp(e->throttle, 0.0f, 1.0f);

  /* idle control adds torque when throttle is below a small window */
  float T_idle = 0.0f;
  if(user_thr < e->idle_throttle_window && e->state == LCC_ENGINE_RUNNING && combustion_enabled) {
    float err = e->idle_target_rpm - rpm;
    if(err > 0.0f) {
      e->idle_integrator += err * e->idle_I * car->timestep;
      e->idle_integrator = lcc_clamp(e->idle_integrator, 0.0f, e->idle_max_torque);
      T_idle             = lcc_clamp(e->idle_P * err + e->idle_integrator, 0.0f, e->idle_max_torque);
    } else {
      /* above target: decay integrator */
      e->idle_integrator *= (1.0f - fminf(1.0f, car->timestep * 4.0f));
      T_idle = 0.0f;
    }
  } else {
    e->idle_integrator *= (1.0f - fminf(1.0f, car->timestep * 4.0f));
  }

  /* base combustion torque from torque curve */
  float T_comb = combustion_enabled ? lcc_engine_torque_curve(e, rpm, user_thr) : 0.0f;

  /* add idle torque */
  T_comb += T_idle;

  /* soft spark cut near redline reduces delivered torque */
  if(e->spark_cut_active && e->rev_limiter_cut_ratio > 0.0f) {
    float cut = lcc_clamp(e->rev_limiter_cut_ratio, 0.0f, 1.0f);
    T_comb *= (1.0f - 0.7f * cut);
  }

  /* starter mechanical torque when START with lockouts respected */
  int   gear            = car->transmission.current_gear;
  float kph             = lcc_car_get_speed(car);
  int   starter_allowed = (car->battery.voltage > car->battery.min_starter_voltage);
  float T_starter       = ((e->key_pos == LCC_KEY_RUN && e->ignition) && starter_allowed) ? e->starter_torque : 0.0f;

  /* mild idle assist near idle to prevent chattering */
  float T_idle_assist = 0.0f;
  if(combustion_enabled && rpm <= e->idle_rpm) {
    float deficit_rpm = e->idle_rpm - rpm;
    float idle_factor = lcc_clamp(deficit_rpm / fmaxf(e->idle_rpm - e->stall_rpm, 50.0f), 0.0f, 1.0f);
    T_idle_assist     = e->idle_torque * idle_factor;
  }

  e->output_torque = fmaxf(0.0f, T_comb + T_starter + T_idle_assist);
}

/*}}}*/

/* driveline {{{*/

/* limited-slip differential: splits axle torque based on preload, power/coast bias, and viscous term */
static void lcc_lsd_split(const lcc_differential_t *d, float axle_Tin, float omega_left, float omega_right, float *outL, float *outR) {
  float Tl          = 0.5f * axle_Tin;
  float Tr          = 0.5f * axle_Tin;
  float domega      = omega_left - omega_right;
  float lock_factor = (axle_Tin >= 0.0f) ? d->power_factor : d->coast_factor;
  float T_lock      = d->preload + lock_factor * fabsf(axle_Tin) + d->viscous_coefficient * fabsf(domega);
  if(d->bias_limit > 0.0f) T_lock = fminf(T_lock, d->bias_limit);

  /* locking opposes speed difference */
  float sgn = lcc_sign(domega);
  Tl -= 0.5f * T_lock * sgn;
  Tr += 0.5f * T_lock * sgn;

  *outL = Tl;
  *outR = Tr;
}

/* transmission physics:
   - distributes brake torques
   - computes clutch coupling and torque to wheels
   - integrates engine rpm (free-running or coupled)
   - calls electrics to add alternator load and update battery
   - updates engine state machine
*/
static void lcc_transmission_physics(lcc_car_t *car) {
  lcc_transmission_t *t = &car->transmission;
  lcc_engine_t       *e = &car->engine;

  /* brake torques split by bias (front/rear) */
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

  /* current gear ratio selection (includes reverse) */
  int   g      = t->current_gear;
  float gr_raw = 0.0f;
  if(g > 0 && g <= t->num_gears) gr_raw = t->gear_ratios[g - 1];
  else if(g == -1)
    gr_raw = -t->reverse_ratio;

  float Rf  = t->final_drive;
  float eff = lcc_clamp(t->efficiency, 0.0f, 1.0f);

  /* which wheels are driven */
  int drivenMask[4] = { 0, 0, 0, 0 };
  switch(t->drive_type) {
  case LCC_DRIVE_RWD: drivenMask[2] = drivenMask[3] = 1; break;
  case LCC_DRIVE_FWD: drivenMask[0] = drivenMask[1] = 1; break;
  case LCC_DRIVE_AWD:
    for(int i = 0; i < 4; ++i) drivenMask[i] = 1;
    break;
  default: break;
  }

  /* engine rotational state and available output torque */
  float dt      = car->timestep;
  float omega_e = e->current_rpm * LCC_RAD_PER_RPM;
  if(!isfinite(omega_e)) omega_e = e->idle_rpm * LCC_RAD_PER_RPM;

  float thr   = lcc_clamp(e->throttle, 0.0f, 1.0f);
  float T_out = fmaxf(0.0f, e->output_torque);

  /* electrics step (adds alternator mechanical load) */
  int   engine_running = (e->state == LCC_ENGINE_RUNNING) ? 1 : 0;
  float T_alt          = lcc_electrics_step(car, dt, e->current_rpm, engine_running);

  /* engine-side resistive torques: viscous + quadratic + engine braking + alternator */
  float omega_abs    = fabsf(omega_e);
  float T_resist_mag = e->friction * omega_abs + e->friction_quadratic * omega_abs * omega_abs + e->engine_brake_coeff * (1.0f - thr) * omega_abs;
  float T_resist     = T_resist_mag * lcc_sign(omega_e);
  T_resist += T_alt * lcc_sign(omega_e);

  /* average driven wheel angular speed -> propshaft speed (filtered) */
  float sum_w = 0.0f;
  int   cnt_w = 0;
  for(int i = 0; i < 4; ++i)
    if(drivenMask[i]) {
      sum_w += car->wheels[i].angular_velocity;
      cnt_w++;
    }
  float avg_w_omega  = (cnt_w > 0) ? sum_w / (float)cnt_w : 0.0f;
  float target_shaft = avg_w_omega * Rf;
  {
    float tau_shaft = 0.02f;
    float a         = 1.0f - expf(-dt / tau_shaft);
    car->gbx_in_omega += (target_shaft - car->gbx_in_omega) * a;
  }

  /* clutch engagement: E=1 engaged (pedal up), E=0 released (pedal down) */
  float E = 1.0f - lcc_clamp(car->clutch_input, 0.0f, 1.0f);

  /* free-run if clutch released or no drive ratio */
  if(fabsf(gr_raw) < 1e-6f || Rf < 1e-6f || E < 1e-3f) {
    float Ieng         = fmaxf(e->inertia, 1e-4f);
    float T_net_engine = T_out - T_resist; /* no clutch load */
    if(e->current_rpm >= e->redline_rpm && T_net_engine > 0.0f) T_net_engine = 0.0f;
    float domega_e = lcc_clamp((T_net_engine / Ieng) * dt, -5000.0f, 5000.0f);
    omega_e        = fmaxf(0.0f, omega_e + domega_e);
    e->current_rpm = lcc_clamp(omega_e / LCC_RAD_PER_RPM, 0.0f, e->redline_rpm * 1.2f);

    /* update engine state machine with new net torque */
    lcc_engine_state_update(car, T_net_engine);
    return;
  }

  /* clutch slip and torque transfer */
  float omega_in = car->gbx_in_omega * gr_raw; /* engine-side match rpm */
  float slip     = omega_e - omega_in;
  float s        = lcc_sign(slip);

  /* clutch capacity scales with engagement and a base capacity */
  float Tcap_base = 50.0f + 1.5f * fmaxf(100.0f, e->max_torque);
  float T_cap     = Tcap_base * (0.05f + 0.95f * E);
  float C_visc    = 1.0f + 6.0f * E; /* viscous term opposing slip */

  /* if near-synced and within capacity, lock; else transmit opposing slip torque */
  float lock_thresh   = 7.0f; /* rad/s ~ 67 rpm */
  int   lock_possible = (fabsf(slip) < lock_thresh) && (fabsf(T_out - T_resist) <= T_cap);

  float T_c = 0.0f; /* torque delivered to gearbox (engine sign convention) */
  if(lock_possible) {
    T_c     = lcc_clamp(T_out - T_resist, -T_cap, T_cap);
    omega_e = fabsf(omega_in);
  } else {
    float T_mag = C_visc * fabsf(slip) + fmaxf(0.0f, (T_out - T_resist) * s);
    T_mag       = lcc_clamp(T_mag, 0.0f, T_cap);
    T_c         = s * T_mag;
  }

  /* torque to wheels through gearbox and final drive */
  float T_wheels_total = T_c * gr_raw * eff * Rf;

  /* split based on drive type and differential model */
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
  } break;
  default: break;
  }
  car->wheels[0].drive_torque = TfL;
  car->wheels[1].drive_torque = TfR;
  car->wheels[2].drive_torque = TrL;
  car->wheels[3].drive_torque = TrR;

  /* integrate engine rpm with net torque (including clutch load) */
  float Ieng         = fmaxf(e->inertia, 1e-4f);
  float T_net_engine = T_out - T_resist - T_c;
  if(lock_possible) {
    e->current_rpm = lcc_clamp(omega_e / LCC_RAD_PER_RPM, 0.0f, e->redline_rpm * 1.2f);
  } else {
    if(e->current_rpm >= e->redline_rpm && T_net_engine > 0.0f) T_net_engine = 0.0f;
    float domega_e = lcc_clamp((T_net_engine / Ieng) * dt, -5000.0f, 5000.0f);
    omega_e        = fmaxf(0.0f, omega_e + domega_e);
    e->current_rpm = lcc_clamp(omega_e / LCC_RAD_PER_RPM, 0.0f, e->redline_rpm * 1.2f);
  }

  /* update engine state */
  lcc_engine_state_update(car, T_net_engine);
}

/*}}}*/

/* aerodynamics and loads {{{*/

/* drag and downforce in world coordinates; split downforce by aero balance */
static void lcc_aero_forces(const lcc_car_t *car, float out_Fdrag_world[2], float *out_DF_front, float *out_DF_rear) {
  float vmag2 = lcc_length2((float *)car->velocity);
  if(vmag2 < 1e-5f) {
    out_Fdrag_world[0] = out_Fdrag_world[1] = 0.0f;
    *out_DF_front = *out_DF_rear = 0.0f;
    return;
  }
  float rho      = car->air_density;
  float q        = 0.5f * rho * vmag2; /* dynamic pressure */
  float Fd       = q * car->aerodynamics.drag_coefficient * car->aerodynamics.frontal_area;
  float dfA      = car->aerodynamics.downforce_coefficient * car->aerodynamics.downforce_area;
  float DF_total = q * dfA; /* positive downward */

  /* drag opposes velocity */
  float vdir[2] = { -car->velocity[0], -car->velocity[1] };
  lcc_norm2(vdir);
  out_Fdrag_world[0] = Fd * vdir[0];
  out_Fdrag_world[1] = Fd * vdir[1];

  /* split downforce between axles */
  float fb      = lcc_clamp(car->aerodynamics.aero_balance_front, 0.0f, 1.0f);
  *out_DF_front = DF_total * fb;
  *out_DF_rear  = DF_total * (1.0f - fb);
}

/* load transfer estimation (quasi-static) with smoothing to avoid spikes */
static void lcc_compute_loads(lcc_car_t *car, float ax_body, float ay_body, float DF_front, float DF_rear) {
  /* static axle loads from cg position */
  float W         = car->mass * LCC_GRAVITY;
  float Wf_static = W * (1.0f - car->cg_position);
  float Wr_static = W * car->cg_position;

  /* longitudinal transfer: positive ax shifts to rear */
  float dF_long = (car->mass * ax_body * car->cg_height) / fmaxf(car->wheelbase, 0.1f);

  /* lateral transfer across track */
  float dF_lat_total = (car->mass * ay_body * car->cg_height) / fmaxf(car->track_width, 0.1f);

  /* split lateral transfer evenly between axles (simple model) */
  float lambda       = 0.5f;
  float dF_lat_front = dF_lat_total * lambda;
  float dF_lat_rear  = dF_lat_total * (1.0f - lambda);

  /* base per-wheel vertical loads before aero */
  float Fz_FL = 0.5f * Wf_static - 0.5f * dF_long - 0.5f * dF_lat_front;
  float Fz_FR = 0.5f * Wf_static - 0.5f * dF_long + 0.5f * dF_lat_front;
  float Fz_RL = 0.5f * Wr_static + 0.5f * dF_long - 0.5f * dF_lat_rear;
  float Fz_RR = 0.5f * Wr_static + 0.5f * dF_long + 0.5f * dF_lat_rear;

  /* add aero downforce evenly to each axle's wheels */
  Fz_FL += 0.5f * DF_front;
  Fz_FR += 0.5f * DF_front;
  Fz_RL += 0.5f * DF_rear;
  Fz_RR += 0.5f * DF_rear;

  /* smooth the loads to avoid numerical chattering */
  float Fz_target[4] = { fmaxf(0.0f, Fz_FL), fmaxf(0.0f, Fz_FR), fmaxf(0.0f, Fz_RL), fmaxf(0.0f, Fz_RR) };
  float alpha        = 1.0f - expf(-car->timestep * 20.0f);
  for(int i = 0; i < 4; ++i) {
    car->Fz_smooth[i]   = lcc_lerp(car->Fz_smooth[i], Fz_target[i], alpha);
    car->wheels[i].load = car->Fz_smooth[i];
  }
}

/*}}}*/

/* tire model {{{*/

/* tire step: compute slips, forces, rolling resistance, and spin dynamics */
static void lcc_tire_step(lcc_car_t *car, int i) {
  lcc_wheel_state_t *w  = &car->wheels[i];
  lcc_tire_params_t *tp = &car->tire_params[i];

  /* wheel position in world from body yaw */
  float ca = cosf(car->angle), sa = sinf(car->angle);
  float rWx = w->position[0] * ca - w->position[1] * sa;
  float rWy = w->position[0] * sa + w->position[1] * ca;

  /* velocity at contact: v + (omega_z x r) */
  float vWx = car->velocity[0] - car->angular_velocity * rWy;
  float vWy = car->velocity[1] + car->angular_velocity * rWx;

  /* rotate into wheel frame (includes steer angle) */
  float psi = car->angle + w->steer_angle;
  float c = cosf(psi), s = sinf(psi);
  float v_long = vWx * c + vWy * s;
  float v_lat  = -vWx * s + vWy * c;
  float v_abs  = sqrtf(v_long * v_long + v_lat * v_lat);

  /* relaxation filters for slip ratio and slip angle */
  float R         = fmaxf(tp->radius, 0.05f);
  float vref_long = fmaxf(fabsf(v_long), 0.5f);
  float kappa_tgt = (w->angular_velocity * R - v_long) / vref_long; /* normalized slip ratio */
  kappa_tgt       = lcc_clamp(kappa_tgt, -2.5f, 2.5f);
  float alpha_tgt = atanf((fabsf(v_long) > 0.2f) ? (v_lat / fabsf(v_long)) : (v_lat / 0.2f));
  alpha_tgt       = lcc_clamp(alpha_tgt, -LCC_PI * 0.5f, LCC_PI * 0.5f);

  float Lx     = fmaxf(tp->relax_length_long, 0.05f);
  float Ly     = fmaxf(tp->relax_length_lat, 0.05f);
  float rate_k = fminf(25.0f, vref_long / Lx);
  float rate_a = fminf(25.0f, v_abs / Ly);
  w->slip_ratio += (kappa_tgt - w->slip_ratio) * rate_k * car->timestep;
  w->slip_angle += (alpha_tgt - w->slip_angle) * rate_a * car->timestep;

  /* effective mu with temperature/wear and load sensitivity */
  float mu_surface = fmaxf(w->surface_friction, 0.1f) * fmaxf(car->surface_friction, 0.1f);
  float Fz         = fmaxf(w->load, 0.0f);

  /* temperature: best near 80C (simple linear degrade) */
  float temp_term = 1.0f - 0.0025f * (w->temperature - 80.0f);
  temp_term       = lcc_clamp(temp_term, 0.5f, 1.05f);
  float wear_term = lcc_clamp(1.0f - 0.6f * tp->wear, 0.4f, 1.0f);

  float mu0 = lcc_lerp(tp->slip_friction, tp->peak_friction, lcc_clamp(temp_term, 0.0f, 1.0f)) * wear_term;

  float Ls      = lcc_clamp(tp->load_sensitivity, 0.0f, 0.9f);
  float mu_load = mu0 * (1.0f - Ls * (Fz - tp->nominal_load) / fmaxf(tp->nominal_load, 1.0f));
  mu_load       = lcc_clamp(mu_load, fmaxf(0.1f, tp->mu_min), fminf(2.5f, fmaxf(mu0, tp->mu_max)));
  float mu      = mu_surface * mu_load;

  /* lightweight MF-style force computation */
  float Dy     = mu * Fz;
  float Cy     = 1.35f;
  float By     = tp->cornering_stiffness / fmaxf(Cy * Dy, 1.0f);
  float Fy_lin = -Dy * sinf(Cy * atanf(By * w->slip_angle)) - tp->camber_stiffness * w->camber_angle;

  float Dx     = mu * Fz;
  float Cx     = 1.30f;
  float Bx     = tp->stiffness / fmaxf(Cx * Dx, 1.0f);
  float Fx_lin = Dx * sinf(Cx * atanf(Bx * w->slip_ratio));

  /* combined slip scaling (friction ellipse) */
  float Fx0 = Fx_lin, Fy0 = Fy_lin;
  float denom = fmaxf(LCC_EPS, sqrtf(Fx0 * Fx0 + Fy0 * Fy0));
  float scale = fminf(1.0f, mu * Fz / denom);
  float Fx    = Fx0 * scale;
  float Fy    = Fy0 * scale;

  /* rolling resistance (direction opposite rolling) */
  float Crr    = tp->rolling_resistance;
  float Frr    = Crr * Fz * (vref_long / (vref_long + 5.0f));
  float rr_dir = (fabsf(v_long) > 1e-3f) ? -lcc_sign(v_long) : -(lcc_sign(w->angular_velocity));
  Fx += Frr * rr_dir;

  /* wheel spin dynamics: drive + contact - brake */
  float Iw        = fmaxf(w->rotational_inertia, 1e-4f);
  float omega_abs = fabsf(w->angular_velocity);

  float eps       = fmaxf(0.02f, car->timestep);
  float brake_dir = (omega_abs > 0.0f) ? (w->angular_velocity / sqrtf(omega_abs * omega_abs + eps * eps)) : 0.0f;
  float T_brake   = w->brake_torque * brake_dir;
  float T_contact = -Fx * R;
  float T_net     = w->drive_torque + T_contact - T_brake;
  float domega    = (T_net / Iw) * car->timestep;
  domega          = lcc_clamp(domega, -500.0f, 500.0f);
  w->angular_velocity += domega;

  /* lock wheel to zero spin if nearly stopped and static torques insufficient */
  if(car->brake_input > 0.1f && fabsf(w->angular_velocity) < 0.5f) {
    float static_torques = w->drive_torque + T_contact;
    if(fabsf(static_torques) < w->brake_torque) w->angular_velocity = 0.0f;
  }

  w->Fx = Fx;
  w->Fy = Fy;

  /* simple thermal and wear model driven by slip power */
  float       slip_power = fabsf(Fx) * (fabsf(w->angular_velocity * R - v_long)) + fabsf(Fy) * fabsf(v_lat);
  float       v_gate     = v_abs / (v_abs + 1.0f);
  const float c_heat     = 3.5e-4f; /* K per (N*m/s) */
  float       k_cool     = 0.04f + 0.012f * lcc_length((float *)car->velocity);
  float       dTdt       = c_heat * slip_power * v_gate - k_cool * (w->temperature - car->ambient_temp);
  w->temperature         = lcc_clamp(w->temperature + dTdt * car->timestep, car->ambient_temp, 160.0f);
  tp->wear               = lcc_clamp(tp->wear + (slip_power * car->timestep) * 1.0e-7f, 0.0f, 1.0f);
}

/*}}}*/

/* fuel consumption {{{*/

/* fuel consumption model:
   - uses shaft power and a BSFC interpolation between worst and best
   - applies minimum idle consumption
   - converts grams to liters and subtracts from tank
   - returns grams/sec (for telemetry if desired)
*/
static float lcc_fuel_consume(lcc_car_t *car, float dt, float engine_rpm, float crank_torque_nm, int combustion_enabled) {
  lcc_fuel_system_t *fu = &car->fuel;
  if(fu->fuel_level_L <= 0.0f || !fu->pump_ok || !combustion_enabled) return 0.0f;

  float omega     = engine_rpm * LCC_RAD_PER_RPM;
  float P_shaft_W = fmaxf(0.0f, omega * crank_torque_nm);
  float P_kW      = P_shaft_W * 0.001f;

  /* normalize load to approximate BSFC changes with load */
  float load_norm = lcc_clamp(P_kW / fmaxf(car->engine.max_power * 0.001f, 1.0f), 0.0f, 1.0f);
  float bsfc      = lcc_lerp(fu->bsfc_worst_gpkWh, fu->bsfc_best_gpkWh, sqrtf(load_norm)); /* g/kWh */

  /* base grams/sec and minimum idle grams/sec */
  float min_gps = 0.5f;
  float gps     = (bsfc * P_kW) / 3600.0f; /* g/s */
  gps           = fmaxf(gps, min_gps);

  /* convert to liters and subtract from tank */
  float g_per_L = 1000.0f * fu->density_kg_per_L;
  float L_used  = (gps * dt) / g_per_L;
  if(L_used > fu->fuel_level_L) L_used = fu->fuel_level_L;
  fu->fuel_level_L -= L_used;

  return gps;
}

/*}}}*/

/* vehicle integration {{{*/

/* integrates vehicle linear and angular states from total forces and moments */
static void lcc_vehicle_step(lcc_car_t *car) {
  float cx = cosf(car->angle), sx = sinf(car->angle);

  /* sum forces in world and net yaw moment in body coordinates */
  float Fw[2] = { 0.0f, 0.0f };
  float Mz    = 0.0f;
  for(int i = 0; i < 4; ++i) {
    lcc_wheel_state_t *w   = &car->wheels[i];
    float              psi = car->angle + w->steer_angle;
    float              cw = cosf(psi), sw = sinf(psi);
    /* rotate tire forces into world */
    float Fxw = w->Fx * cw - w->Fy * sw;
    float Fyw = w->Fx * sw + w->Fy * cw;
    Fw[0] += Fxw;
    Fw[1] += Fyw;

    /* compute yaw moment using body-frame arm */
    float Fb[2];
    lcc_world_to_body(cx, sx, (float[2]){ Fxw, Fyw }, Fb);
    float rb[2] = { w->position[0], w->position[1] };
    Mz += rb[0] * Fb[1] - rb[1] * Fb[0];
  }

  /* aerodynamic drag and downforce */
  float Fdrag[2], DFf = 0.0f, DFr = 0.0f;
  lcc_aero_forces(car, Fdrag, &DFf, &DFr);
  Fw[0] += Fdrag[0];
  Fw[1] += Fdrag[1];

  /* integrate linear motion */
  float ax = Fw[0] / car->mass;
  float ay = Fw[1] / car->mass;
  car->velocity[0] += ax * car->timestep;
  car->velocity[1] += ay * car->timestep;
  car->position[0] += car->velocity[0] * car->timestep;
  car->position[1] += car->velocity[1] * car->timestep;

  /* integrate yaw */
  float yaw_acc = Mz / fmaxf(car->inertia, 1e-3f);
  car->angular_velocity += yaw_acc * car->timestep;
  car->angle += car->angular_velocity * car->timestep;

  /* wrap yaw angle to [-pi, pi] */
  if(car->angle > LCC_PI) car->angle -= 2.0f * LCC_PI;
  if(car->angle < -LCC_PI) car->angle += 2.0f * LCC_PI;

  /* compute body-frame accelerations for next-step load transfer */
  float Ab[2];
  lcc_world_to_body(cx, sx, (float[2]){ ax, ay }, Ab);
  lcc_compute_loads(car, Ab[0], Ab[1], DFf, DFr);
}

/* ackermann front-steering model:
   - given steering input in [-1,1], compute inner/outer wheel angles in radians
*/
static void ackermann_steering(const lcc_car_t *car, float steering_input, float *out_angle_i, float *out_angle_o) {
  float max_sa = 40.0f * (LCC_PI / 180.0f); /* 40 degrees */

  if(fabsf(steering_input) < LCC_EPS) {
    *out_angle_i = 0.0f;
    *out_angle_o = 0.0f;
  } else {
    float L = car->wheelbase, T = car->track_width;
    float delta       = -steering_input * max_sa; /* right turn positive steering -> negative angle */
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

/*}}}*/

/* public api implementation {{{*/

/* shift up one gear if possible */
void lcc_car_shift_up(lcc_car_t *car) {
  if(car->transmission.current_gear < car->transmission.num_gears) car->transmission.current_gear++;
}

/* shift down one gear (won't go below reverse) */
void lcc_car_shift_down(lcc_car_t *car) {
  if(car->transmission.current_gear > -1) car->transmission.current_gear--;
}

/* set explicit gear (-1..num_gears) */
void lcc_car_set_gear(lcc_car_t *car, int gear) {
  if(gear >= -1 && gear <= car->transmission.num_gears) car->transmission.current_gear = gear;
}

/* driving inputs (throttle 0..1, brake 0..1, steering -1..1, clutch 0..1) */
void lcc_car_set_inputs(lcc_car_t *car, float throttle, float brake, float steering, float clutch) {
  car->throttle_input = lcc_clamp(throttle, 0.0f, 1.0f);
  car->brake_input    = lcc_clamp(brake, 0.0f, 1.0f);
  car->steering_input = lcc_clamp(steering, -1.0f, 1.0f);
  car->clutch_input   = lcc_clamp(clutch, 0.0f, 1.0f);
}

/* step simulation forward by dt seconds */
void lcc_car_update(lcc_car_t *car, float dt) {
  if(dt <= 0.0f) return;
  car->timestep = dt;
  car->simulation_time += dt;

  /* steering computation (front axle) */
  ackermann_steering(car, car->steering_input, &car->wheels[0].steer_angle, &car->wheels[1].steer_angle);

  /* throttle smoothing (simple first-order lag) */
  float tau             = fmaxf(car->engine.response_time, 1e-3f);
  float k               = 1.0f - expf(-dt / tau);
  float target_throttle = car->throttle_input;
  car->engine.throttle  = lcc_lerp(car->engine.throttle, target_throttle, k);

  /* engine torque production (combustion, idle, starter) */
  lcc_engine_physics(car);

  /* driveline and engine RPM integration (also updates electrics and engine state) */
  lcc_transmission_physics(car);

  /* tires and vehicle integration */
  for(int i = 0; i < 4; ++i) lcc_tire_step(car, i);
  lcc_vehicle_step(car);

  /* fuel consumption this step (only when running and fuel not cut) */
  int combustion_enabled = (car->engine.key_pos >= LCC_KEY_RUN) && (car->engine.state == LCC_ENGINE_RUNNING) && !car->engine.fuel_cut_active;
  (void)lcc_fuel_consume(car, dt, car->engine.current_rpm, car->engine.output_torque, combustion_enabled);
}

/* utility getters */
float lcc_car_get_speed(const lcc_car_t *car) {
  return lcc_length((float *)car->velocity) * 3.6f;
}

float lcc_car_get_engine_rpm(const lcc_car_t *car) {
  return car->engine.current_rpm;
}

const char *lcc_get_version(void) {
  return LCC_VERSION;
}

int lcc_car_engine_is_running(const lcc_car_t *car) {
  return car->engine.state == LCC_ENGINE_RUNNING;
}

/* electrics/fuel interface */
float lcc_car_get_battery_voltage(const lcc_car_t *car) {
  return car->battery.voltage;
}

float lcc_car_get_battery_soc(const lcc_car_t *car) {
  return car->battery.soc;
}

void lcc_car_set_accessory_load(lcc_car_t *car, float watts) {
  car->battery.accessory_load_watts = fmaxf(0.0f, watts);
}

float lcc_car_get_fuel_level_L(const lcc_car_t *car) {
  return car->fuel.fuel_level_L;
}

float lcc_car_get_fuel_capacity_L(const lcc_car_t *car) {
  return car->fuel.tank_capacity_L;
}

void lcc_car_refuel(lcc_car_t *car, float liters) {
  car->fuel.fuel_level_L = lcc_clamp(car->fuel.fuel_level_L + fmaxf(0.0f, liters), 0.0f, car->fuel.tank_capacity_L);
}

void lcc_car_set_fuel_level(lcc_car_t *car, float liters) {
  car->fuel.fuel_level_L = lcc_clamp(liters, 0.0f, car->fuel.tank_capacity_L);
}

void lcc_car_set_keypos(lcc_car_t *car, lcc_key_state_t key) {
  car->engine.key_pos = key;
}

void lcc_car_set_ignition(lcc_car_t *car, lcc_ignition_state_t ignition) {
  car->engine.ignition = ignition;
}

lcc_key_state_t lcc_car_get_keypos(const lcc_car_t *car) {
  return (lcc_key_state_t)car->engine.key_pos;
}

lcc_ignition_state_t lcc_car_get_ignition(const lcc_car_t *car) {
  return (lcc_ignition_state_t)car->engine.ignition;
}

/* create a car with a preset and initialize all subsystems */
lcc_car_t lcc_car_create(lcc_preset_t preset) {
  lcc_car_t car;
  memset(&car, 0, sizeof(car));

  /* baseline chassis and environment */
  car.mass             = 1500.0f;
  car.wheelbase        = 2.7f;
  car.track_width      = 1.6f;
  car.cg_height        = 0.50f;
  car.cg_position      = 0.55f;
  car.air_density      = LCC_AIR_DENSITY;
  car.ambient_temp     = 20.0f;
  car.surface_friction = 1.0f;
  car.front_brake_bias = 0.65f;
  car.max_brake_torque = 8000.0f;

  /* tires default parameters (later overridden by presets) */
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

    lcc_wheel_state_t *w  = &car.wheels[i];
    w->surface_friction   = car.surface_friction;
    w->temperature        = car.ambient_temp;
    w->rotational_inertia = 1.2f;
  }

  /* wheel placement */
  car.wheels[0].position[0] = +car.wheelbase * 0.5f;
  car.wheels[0].position[1] = +car.track_width * 0.5f; /* FL */
  car.wheels[1].position[0] = +car.wheelbase * 0.5f;
  car.wheels[1].position[1] = -car.track_width * 0.5f; /* FR */
  car.wheels[2].position[0] = -car.wheelbase * 0.5f;
  car.wheels[2].position[1] = +car.track_width * 0.5f; /* RL */
  car.wheels[3].position[0] = -car.wheelbase * 0.5f;
  car.wheels[3].position[1] = -car.track_width * 0.5f; /* RR */

  /* engine baseline */
  car.engine.max_power          = 200000.0f;
  car.engine.max_torque         = 400.0f;
  car.engine.idle_rpm           = 800.0f;
  car.engine.max_rpm            = 6000.0f;
  car.engine.redline_rpm        = 6500.0f;
  car.engine.inertia            = 0.20f;
  car.engine.friction           = 0.05f;
  car.engine.response_time      = 0.10f;
  car.engine.current_rpm        = 0.0f;
  car.engine.peak_torque_rpm    = 3500.0f;
  car.engine.peak_power_rpm     = 5800.0f;
  car.engine.engine_brake_coeff = 0.08f;
  car.engine.friction_quadratic = 5e-4f;
  car.engine.idle_torque        = 30.0f;
  car.engine.stall_rpm          = 600.0f;

  /* controls and limiter defaults */
  car.engine.decel_fuel_cut_enabled  = 1;
  car.engine.decel_fuel_cut_rpm      = 1500.0f;
  car.engine.decel_fuel_cut_throttle = 0.02f;
  car.engine.rev_limiter_hyst        = 200.0f;
  car.engine.rev_limiter_soft_zone   = 200.0f;
  car.engine.rev_limiter_cut_ratio   = 0.0f;
  car.engine.rev_limiter_mode        = LCC_REV_CUT_MIXED;

  /* starter parameters */
  car.engine.min_start_rpm       = 300.0f;
  car.engine.starter_torque      = 80.0f;
  car.engine.starter_power_watts = 1800.0f;
  car.engine.starter_efficiency  = 0.55f;

  /* idle control */
  car.engine.idle_target_rpm      = car.engine.idle_rpm;
  car.engine.idle_P               = 0.05f;
  car.engine.idle_I               = 0.02f;
  car.engine.idle_integrator      = 0.0f;
  car.engine.idle_max_torque      = 80.0f;
  car.engine.idle_throttle_window = 0.03f;

  /* stall tracking */
  car.engine.stall_time_accum = 0.0f;
  car.engine.stall_delay      = 0.15f;
  car.engine.last_net_torque  = 0.0f;

  /* start with engine running and key in RUN */
  car.engine.state   = LCC_ENGINE_RUNNING;
  car.engine.key_pos = LCC_KEY_RUN;

  /* transmission */
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
  car.transmission.current_gear   = 0; /* neutral */
  car.transmission.drive_type     = LCC_DRIVE_RWD;

  /* differential */
  car.differential.preload             = 50.0f;
  car.differential.power_factor        = 0.25f;
  car.differential.coast_factor        = 0.20f;
  car.differential.viscous_coefficient = 5.0f;
  car.differential.bias_limit          = 600.0f;

  /* aerodynamics */
  car.aerodynamics.drag_coefficient      = 0.30f;
  car.aerodynamics.frontal_area          = 2.0f;
  car.aerodynamics.downforce_coefficient = 0.10f;
  car.aerodynamics.downforce_area        = 2.0f;
  car.aerodynamics.aero_balance_front    = 0.55f;

  /* electrics */
  car.battery.capacity_Ah          = 60.0f;
  car.battery.soc                  = 0.9f;
  car.battery.internal_resistance  = 0.015f;
  car.battery.voltage              = lcc_batt_ocv(car.battery.soc);
  car.battery.min_ignition_voltage = 9.5f;
  car.battery.min_starter_voltage  = 10.0f;
  car.battery.temp_C               = 25.0f;
  car.battery.accessory_load_watts = 0.0f;
  car.battery.ecu_load_watts       = 30.0f;
  car.battery.fuel_pump_watts      = 60.0f;
  car.battery.parasitic_watts      = 0.2f;
  car.alternator.rated_power_W     = 1500.0f;
  car.alternator.efficiency        = 0.6f;
  car.alternator.cut_in_rpm        = 1500.0f;
  car.alternator.pulley_ratio      = 2.8f;
  car.alternator.target_voltage    = 14.2f;
  car.alternator.current_output_W  = 0.0f;

  /* fuel */
  car.fuel.tank_capacity_L  = 50.0f;
  car.fuel.fuel_level_L     = 40.0f;
  car.fuel.density_kg_per_L = 0.745f;
  car.fuel.bsfc_best_gpkWh  = 230.0f;
  car.fuel.bsfc_worst_gpkWh = 380.0f;
  car.fuel.pump_ok          = 1;

  /* yaw inertia from rectangular approximation */
  car.inertia = car.mass * (car.wheelbase * car.wheelbase + car.track_width * car.track_width) / 12.0f;

  /* preset-specific overrides */
  switch(preset) {
  case LCC_PRESET_ECONOMY: { /* economy car */
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
    car.engine.idle_target_rpm    = 750.0f;

    car.transmission.num_gears      = 6;
    car.transmission.gear_ratios[0] = 3.64f;
    car.transmission.gear_ratios[1] = 2.08f;
    car.transmission.gear_ratios[2] = 1.36f;
    car.transmission.gear_ratios[3] = 1.03f;
    car.transmission.gear_ratios[4] = 0.86f;
    car.transmission.gear_ratios[5] = 0.69f;
    car.transmission.final_drive    = 4.10f;
    car.transmission.reverse_ratio  = 3.58f;
    car.transmission.drive_type     = LCC_DRIVE_FWD;

    car.differential.viscous_coefficient = 2.0f;

    car.aerodynamics.drag_coefficient   = 0.27f;
    car.aerodynamics.frontal_area       = 2.20f;
    car.aerodynamics.aero_balance_front = 0.60f;

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
      tp->mu_min                       = 0.75f;
      tp->mu_max                       = 1.15f;
      car.wheels[i].rotational_inertia = 1.00f;
    }

    car.alternator.rated_power_W = 1200.0f;
    car.fuel.tank_capacity_L     = 47.0f;
    car.fuel.fuel_level_L        = 36.0f;
  } break;

  case LCC_PRESET_MIDSIZE: { /* midsize sedan */
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
    car.engine.idle_target_rpm    = 680.0f;

    car.transmission.num_gears      = 6;
    car.transmission.gear_ratios[0] = 3.54f;
    car.transmission.gear_ratios[1] = 2.05f;
    car.transmission.gear_ratios[2] = 1.39f;
    car.transmission.gear_ratios[3] = 1.00f;
    car.transmission.gear_ratios[4] = 0.73f;
    car.transmission.gear_ratios[5] = 0.59f;
    car.transmission.final_drive    = 3.36f;
    car.transmission.reverse_ratio  = 3.16f;
    car.transmission.drive_type     = LCC_DRIVE_FWD;

    car.differential.viscous_coefficient = 3.0f;

    car.aerodynamics.drag_coefficient   = 0.28f;
    car.aerodynamics.frontal_area       = 2.25f;
    car.aerodynamics.aero_balance_front = 0.58f;

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
      tp->mu_min                       = 0.75f;
      tp->mu_max                       = 1.20f;
      car.wheels[i].rotational_inertia = 1.05f;
    }

    car.alternator.rated_power_W = 1500.0f;
    car.fuel.tank_capacity_L     = 55.0f;
    car.fuel.fuel_level_L        = 45.0f;
  } break;

  case LCC_PRESET_SPORTS: { /* sport sedan/coupe */
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
    car.engine.idle_target_rpm    = 800.0f;

    car.transmission.num_gears      = 6;
    car.transmission.gear_ratios[0] = 4.11f;
    car.transmission.gear_ratios[1] = 2.32f;
    car.transmission.gear_ratios[2] = 1.54f;
    car.transmission.gear_ratios[3] = 1.18f;
    car.transmission.gear_ratios[4] = 1.00f;
    car.transmission.gear_ratios[5] = 0.85f;
    car.transmission.final_drive    = 3.46f;
    car.transmission.reverse_ratio  = 3.68f;
    car.transmission.drive_type     = LCC_DRIVE_RWD;

    car.differential.preload             = 80.0f;
    car.differential.power_factor        = 0.35f;
    car.differential.coast_factor        = 0.25f;
    car.differential.viscous_coefficient = 5.0f;
    car.differential.bias_limit          = 1200.0f;

    car.aerodynamics.drag_coefficient   = 0.34f;
    car.aerodynamics.frontal_area       = 2.20f;
    car.aerodynamics.aero_balance_front = 0.52f;

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
      tp->mu_min                       = 0.85f;
      tp->mu_max                       = 1.35f;
      car.wheels[i].rotational_inertia = 1.10f;
    }

    car.alternator.rated_power_W = 1800.0f;
    car.fuel.tank_capacity_L     = 55.0f;
    car.fuel.fuel_level_L        = 50.0f;
  } break;

  case LCC_PRESET_SUPERCAR: { /* supercar */
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
    car.engine.idle_target_rpm    = 800.0f;

    car.transmission.num_gears      = 6;
    car.transmission.gear_ratios[0] = 3.13f;
    car.transmission.gear_ratios[1] = 2.18f;
    car.transmission.gear_ratios[2] = 1.56f;
    car.transmission.gear_ratios[3] = 1.19f;
    car.transmission.gear_ratios[4] = 0.94f;
    car.transmission.gear_ratios[5] = 0.76f;
    car.transmission.final_drive    = 3.54f;
    car.transmission.reverse_ratio  = 2.90f;
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
      tp->cornering_stiffness          = 70000.0f;
      tp->camber_stiffness             = 60000.0f;
      tp->rolling_resistance           = 0.012f;
      tp->relax_length_long            = 0.35f;
      tp->relax_length_lat             = 0.55f;
      tp->mu_min                       = 0.95f;
      tp->mu_max                       = 1.55f;
      car.wheels[i].rotational_inertia = 1.15f;
    }

    car.alternator.rated_power_W = 2000.0f;
    car.fuel.tank_capacity_L     = 78.0f;
    car.fuel.fuel_level_L        = 60.0f;
  } break;

  case LCC_PRESET_HYPERCAR: { /* hypercar */
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
    car.engine.idle_target_rpm    = 800.0f;

    car.transmission.num_gears      = 6;
    car.transmission.gear_ratios[0] = 3.286f;
    car.transmission.gear_ratios[1] = 2.130f;
    car.transmission.gear_ratios[2] = 1.556f;
    car.transmission.gear_ratios[3] = 1.157f;
    car.transmission.gear_ratios[4] = 0.852f;
    car.transmission.gear_ratios[5] = 0.628f;
    car.transmission.final_drive    = 2.80f;
    car.transmission.reverse_ratio  = 2.90f;
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
      tp->cornering_stiffness          = 70000.0f;
      tp->camber_stiffness             = 65000.0f;
      tp->rolling_resistance           = 0.011f;
      tp->relax_length_long            = 0.37f;
      tp->relax_length_lat             = 0.58f;
      tp->mu_min                       = 0.90f;
      tp->mu_max                       = 1.45f;
      car.wheels[i].rotational_inertia = 1.50f;
    }

    car.alternator.rated_power_W = 2500.0f;
    car.fuel.tank_capacity_L     = 100.0f;
    car.fuel.fuel_level_L        = 85.0f;
  } break;

  default: break;
  }

  /* update yaw inertia with final geometry */
  car.inertia = car.mass * (car.wheelbase * car.wheelbase + car.track_width * car.track_width) / 12.0f;

  /* initialize load smoothing using static + aero at rest */
  float dummyFdrag[2];
  float DFf = 0.0f, DFr = 0.0f;
  lcc_aero_forces(&car, dummyFdrag, &DFf, &DFr);
  float W          = car.mass * LCC_GRAVITY;
  float Wf         = W * (1.0f - car.cg_position) + DFf;
  float Wr         = W * (car.cg_position) + DFr;
  car.Fz_smooth[0] = car.Fz_smooth[1] = 0.5f * Wf;
  car.Fz_smooth[2] = car.Fz_smooth[3] = 0.5f * Wr;

  /* battery initial voltage from SOC */
  car.battery.voltage = lcc_batt_ocv(car.battery.soc);
  car.battery_voltage = car.battery.voltage;

  return car;
}

/* destroy, not used right now*/
void lcc_car_destroy(lcc_car_t *car) {
  (void)car;
}

/*}}}*/

#endif /* LIBCCAR_IMPLEMENTATION */
#endif /* LIBCCAR_H */
