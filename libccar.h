/*
   libccar - 2d top-down car simulation api (c99 single header)
   top-level api: public functions and data structures
   prefix: lcc_
   space: 2d plane (+x forward, +y right), right-handed rotation (cw positive)
   fuck the license for now i just wanna get things to work
*/

#ifndef LIBCCAR_H
#define LIBCCAR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

/* ============================== configuration macros ============================== {{{*/

/* library version */
#define LCC_VERSION "0.2.1"

/* constants and limits */
#define LCC_MAX_WHEELS 4
#define LCC_MAX_GEARS  16

/* default math constants */
#ifndef LCC_PI
#define LCC_PI 3.14159265358979323846f
#endif
#ifndef LCC_TAU
#define LCC_TAU (2.0f * LCC_PI)
#endif

/* engine map generation */
#ifndef LCC_ENG_GEN_WOT_POINTS
#define LCC_ENG_GEN_WOT_POINTS 9
#endif
#ifndef LCC_ENG_GEN_FRICT_POINTS
#define LCC_ENG_GEN_FRICT_POINTS 6
#endif

/* electrics tuning */
#ifndef LCC_ALT_REG_VOLTAGE_V
#define LCC_ALT_REG_VOLTAGE_V 14.2f
#endif
#ifndef LCC_BATT_OCV_FULL_V
#define LCC_BATT_OCV_FULL_V 12.70f
#endif
#ifndef LCC_BATT_OCV_EMPTY_V
#define LCC_BATT_OCV_EMPTY_V 11.80f
#endif
#ifndef LCC_BATT_R_INTERNAL_OHM
#define LCC_BATT_R_INTERNAL_OHM 0.018f
#endif
#ifndef LCC_BATT_R_SOC_GAIN
#define LCC_BATT_R_SOC_GAIN 0.8f /* R rises as SOC drops */
#endif
#ifndef LCC_BATT_R_TEMP_GAIN
#define LCC_BATT_R_TEMP_GAIN 1.5f /* R rises when cold */
#endif
#ifndef LCC_BATT_CAP_TEMP_COEF_PER_K
#define LCC_BATT_CAP_TEMP_COEF_PER_K -0.0045f /* capacity change per K relative to 25C */
#endif
#ifndef LCC_BATT_CHARGE_EFF
#define LCC_BATT_CHARGE_EFF 0.95f
#endif
#ifndef LCC_STARTER_EFF
#define LCC_STARTER_EFF 0.60f /* mech->elec power ratio for starter draw */
#endif

/* engine starting/cranking tuning */
#ifndef LCC_ENGINE_CATCH_RPM_FACTOR
#define LCC_ENGINE_CATCH_RPM_FACTOR 0.80f /* fraction of idle rpm at which engine "catches" */
#endif
#ifndef LCC_ENGINE_CRANK_MIN_TIME_S
#define LCC_ENGINE_CRANK_MIN_TIME_S 0.25f /* minimum cranking time before a catch is possible */
#endif
#ifndef LCC_ENGINE_STARTER_MAX_TORQUE_NM
#define LCC_ENGINE_STARTER_MAX_TORQUE_NM 120.0f /* clamp for starter torque */
#endif
#ifndef LCC_ENGINE_STARTER_MIN_OMEGA_RADPS
#define LCC_ENGINE_STARTER_MIN_OMEGA_RADPS 10.0f /* prevents infinite torque near 0 rad/s */
#endif

/* pacejka shape and combined-slip tuning (overridable by defining before include)*/
#ifndef LCC_PACEJKA_CX
#define LCC_PACEJKA_CX 1.65f /* longitudinal shape factor */
#endif
#ifndef LCC_PACEJKA_CY
#define LCC_PACEJKA_CY 1.30f /* lateral shape factor */
#endif
#ifndef LCC_PACEJKA_EX

#define LCC_PACEJKA_EX 0.97f /* longitudinal curvature */
#endif
#ifndef LCC_PACEJKA_EY
#define LCC_PACEJKA_EY 0.97f /* lateral curvature */
#endif
#ifndef LCC_PACEJKA_KAPPA_SLIDE
#define LCC_PACEJKA_KAPPA_SLIDE 0.12f /* slip ratio threshold for kinetic mu drop */
#endif
#ifndef LCC_PACEJKA_ALPHA_SLIDE
#define LCC_PACEJKA_ALPHA_SLIDE 0.12f /* slip angle [rad] threshold for kinetic mu drop */
#endif
#ifndef LCC_PACEJKA_MU_KINETIC
#define LCC_PACEJKA_MU_KINETIC 0.90f /* mu fraction past the slide threshold */
#endif

/* c api export macro */
#undef LCC_API
#ifdef _WIN32
#define LCC_API __declspec(dllexport)
#else
#define LCC_API extern
#endif

/*}}}*/

/* ============================== enums ============================== {{{*/

/* result codes */
typedef enum lcc_result_e { LCC_OK = 0, LCC_ERR_UNKNOWN = -1, LCC_ERR_INVALID_ARG = -2, LCC_ERR_OUT_OF_MEMORY = -3, LCC_ERR_BAD_STATE = -4, LCC_ERR_UNSUPPORTED = -5, LCC_ERR_BOUNDS = -6, LCC_ERR_NOT_FOUND = -7 } lcc_result_t;

/* drivetrain layout */
typedef enum lcc_drivetrain_layout_e { LCC_LAYOUT_FWD = 0, LCC_LAYOUT_RWD = 1, LCC_LAYOUT_AWD = 2 } lcc_drivetrain_layout_t;

/* transmission type TODO: implement CVT and DCT */
typedef enum lcc_transmission_type_e { LCC_TRANS_MANUAL = 0, LCC_TRANS_AUTOMATIC = 1, LCC_TRANS_CVT = 2, LCC_TRANS_DCT = 3 } lcc_transmission_type_t;

/* predefined gear indices */
typedef enum lcc_gear_e { LCC_GEAR_REVERSE = 0, LCC_GEAR_NEUTRAL = 1 } lcc_gear_t;

/* fuel type TODO: expand on this */
typedef enum lcc_fuel_type_e { LCC_FUEL_GASOLINE = 0, LCC_FUEL_DIESEL = 1, LCC_FUEL_E85 = 2, LCC_FUEL_LPG = 3, LCC_FUEL_CNG = 4, LCC_FUEL_METHANOL = 5, LCC_FUEL_HYDROGEN = 6 } lcc_fuel_type_t;

/* forced induction */
typedef enum lcc_forced_induction_e { LCC_FI_NONE = 0, LCC_FI_TURBO = 1, LCC_FI_SUPERCHARGER = 2, LCC_FI_TWINCHARGED = 3 } lcc_forced_induction_t;

/* differential type */
typedef enum lcc_diff_type_e { LCC_DIFF_OPEN = 0, LCC_DIFF_LOCKED = 1, LCC_DIFF_LSD_CLUTCH = 2, LCC_DIFF_TORSEN = 3, LCC_DIFF_ACTIVE = 4 } lcc_diff_type_t;

/* events and callbacks */
typedef enum lcc_event_type_e { LCC_EVENT_ENGINE_START = 0, LCC_EVENT_ENGINE_STOP = 1, LCC_EVENT_GEAR_CHANGE = 2, LCC_EVENT_ENGINE_STALL = 3, LCC_EVENT_ABS_ACTIVE = 4, LCC_EVENT_TC_ACTIVE = 5, LCC_EVENT_ESC_ACTIVE = 6, LCC_EVENT_OVERHEAT = 7, LCC_EVENT_FUEL_STARVATION = 8 } lcc_event_type_t;

/* driver aids */
typedef enum lcc_abs_mode_e { LCC_ABS_OFF = 0, LCC_ABS_ON = 1 } lcc_abs_mode_t;

typedef enum lcc_tc_mode_e { LCC_TC_OFF = 0, LCC_TC_ON = 1 } lcc_tc_mode_t;

typedef enum lcc_esc_mode_e { LCC_ESC_OFF = 0, LCC_ESC_ON = 1 } lcc_esc_mode_t;

/*}}}*/

/* ============================== types ============================== {{{*/

typedef int lcc_bool_t; /* 0 = false, nonzero = true */

/* allocator hooks */
typedef void *(*lcc_alloc_fn)(size_t size, void *user);
typedef void (*lcc_free_fn)(void *ptr, void *user);

/* curve helpers */
typedef struct {
  float x; /* input, e.g. rpm */
  float y; /* output, e.g. torque nm */
} lcc_curve1d_point_t;

typedef struct {
  const lcc_curve1d_point_t *points;
  int                        count;
} lcc_curve1d_t;

typedef struct {
  float x;
  float y;
  float v;
} lcc_map2d_point_t;

typedef struct {
  const lcc_map2d_point_t *points;
  int                      count;
} lcc_map2d_t;

/* environment and surface */
typedef struct {
  float ambient_temp_c;
  float air_density;
  float wind_world[2];
  float surface_temp_c;
  float global_friction_scale;
} lcc_environment_t;

/* chassis and aero descriptors */
typedef struct {
  float mass_kg;
  float inertia_zz;
  float cg_local_x;
  float wheelbase_m;
  float track_front_m;
  float track_rear_m;
  float cg_height_m;
  float width_m;
  float length_m;
} lcc_chassis_desc_t;

typedef struct {
  float drag_coefficient;
  float frontal_area_m2;
  float lift_coefficient_front;
  float lift_coefficient_rear;
  float yaw_drag_gain;
} lcc_aero_desc_t;

/* engine and fuel system descriptors */
typedef struct {
  /* TODO: properly implement this */
  lcc_fuel_type_t fuel;
  /* TODO: properly implement this */
  lcc_forced_induction_t forced_induction;

  float idle_rpm;
  float redline_rpm;
  float stall_rpm;
  float inertia_kgm2;

  /* TODO: dynamically generate these */
  lcc_curve1d_t wot_torque_nm_vs_rpm;
  lcc_curve1d_t friction_torque_nm_vs_rpm;
  lcc_curve1d_t throttle_map;
  lcc_map2d_t   boost_pressure_kpa_vs_rpm_throttle;

  float wastegate_pressure_kpa;

  float coolant_heat_capacity_j_per_k;
  float oil_heat_capacity_j_per_k;
} lcc_engine_desc_t;

typedef struct {
  float tank_capacity_l;
  float fuel_density_kg_per_l;
  float initial_fuel_l;
} lcc_fuel_desc_t;

typedef struct {
  float radiator_ua_w_per_k;
  float fan_on_c;
} lcc_cooling_desc_t;

/* electrical system descriptors */
typedef struct {
  float capacity_ah;
  float nominal_voltage_v;
  float initial_soc;
  float internal_resistance_ohm;
  float ocv_full_v;
  float ocv_empty_v;
  float charge_efficiency;
} lcc_battery_desc_t;

typedef struct {
  float max_current_a;
  float cut_in_rpm;
  float regulator_voltage_v;
} lcc_alternator_desc_t;

typedef struct {
  float power_w;
} lcc_starter_desc_t;

/* simple engine spec (user-facing, from common datasheet values) */
typedef struct {
  float                  rated_power_kw;   /* peak power (required) */
  float                  rated_power_rpm;  /* rpm of rated power (0 -> derive from redline & FI) */
  float                  redline_rpm;      /* required */
  float                  idle_rpm;         /* 0 -> default */
  float                  stall_rpm;        /* 0 -> default */
  float                  peak_torque_nm;   /* 0 -> derive typical from power & FI */
  float                  peak_torque_rpm;  /* 0 -> derive typical from FI */
  lcc_forced_induction_t forced_induction; /* FI_NONE/FI_TURBO/FI_SUPERCHARGER/FI_TWINCHARGED */
  float                  boost_target_kpa; /* target gauge boost at WOT (turbo/super) */
} lcc_engine_simple_spec_t;

/* ecu and driver aids */
typedef struct {
  lcc_abs_mode_t abs_mode;
  lcc_tc_mode_t  tc_mode;
  lcc_esc_mode_t esc_mode;
  lcc_bool_t     auto_clutch;
  lcc_bool_t     idle_control;
  float          idle_pid_p, idle_pid_i;
} lcc_ecu_desc_t;

/* transmission and driveline descriptors */
typedef struct {
  lcc_transmission_type_t type;
  int                     gear_count;
  float                   gear_ratios[LCC_MAX_GEARS];
  float                   final_drive_ratio;
  float                   shift_time_s;
  float                   auto_upshift_rpm;
  float                   auto_downshift_rpm;
} lcc_transmission_desc_t;

typedef struct {
  lcc_diff_type_t type;
  float           preload_nm;
  float           bias_ratio;
  float           lock_coef;
} lcc_diff_desc_t;

typedef struct {
  lcc_drivetrain_layout_t layout;
  lcc_diff_desc_t         front_diff;
  lcc_diff_desc_t         rear_diff;
  float                   front_torque_split;
} lcc_driveline_desc_t;

/* wheel end descriptors */
typedef struct {
  float      position_local[2];
  float      radius_m;
  float      width_m;
  float      inertia_kgm2;
  lcc_bool_t steerable;
  lcc_bool_t driven;
  lcc_bool_t has_brake;
} lcc_wheel_desc_t;

typedef struct {
  float mu_nominal;
  float load_sensitivity;
  float rolling_resistance;
  float pressure_kpa;
  float ideal_pressure_kpa;
  float relaxation_length_long_m;
  float relaxation_length_lat_m;
  float wear_rate;
} lcc_tire_desc_t;

typedef struct {
  float max_torque_nm;
  float disc_radius_m;
  float pad_mu;
  float cooling_area_m2;
} lcc_brake_desc_t;

typedef struct {
  float front_rate_n_per_rad;
  float rear_rate_n_per_rad;
} lcc_arb_desc_t;

typedef struct {
  float max_steer_deg;
  float ackermann_factor;
} lcc_steering_desc_t;

/* full car descriptor */
typedef struct {
  lcc_chassis_desc_t chassis;
  lcc_aero_desc_t    aero;

  lcc_engine_desc_t  engine;
  lcc_fuel_desc_t    fuel;
  lcc_cooling_desc_t cooling;

  lcc_battery_desc_t    battery;
  lcc_alternator_desc_t alternator;
  lcc_starter_desc_t    starter;
  lcc_ecu_desc_t        ecu;

  lcc_transmission_desc_t transmission;
  lcc_driveline_desc_t    driveline;

  int                 wheel_count;
  lcc_wheel_desc_t    wheels[LCC_MAX_WHEELS]; /* FL;FR;RL;RR */
  lcc_tire_desc_t     tires[LCC_MAX_WHEELS];  /* FL;FR;RL;RR */
  lcc_brake_desc_t    brakes[LCC_MAX_WHEELS]; /* FL;FR;RL;RR */
  lcc_arb_desc_t      arbs;
  lcc_steering_desc_t steering;

  lcc_environment_t environment;
} lcc_car_desc_t;

/* control input */
typedef struct {
  float throttle;
  float brake;
  float clutch; /* 1 = fully disengaged */
  float steer;  /* -1..1 */
  float handbrake;

  lcc_bool_t ignition_switch;
  lcc_bool_t starter;
} lcc_controls_t;

/* states */
typedef struct {
  float omega_radps;
  float drive_torque_nm;
  float brake_torque_nm;
  float normal_force_n;
  float slip_ratio;
  float slip_angle_rad;
  float tire_force_long_n;
  float tire_force_lat_n;
  float tire_temp_c;
  float tire_wear;
} lcc_wheel_state_t;

typedef struct {
  lcc_bool_t running;
  lcc_bool_t cranking;
  float      rpm;
} lcc_engine_state_t;

typedef struct {
  int        gear_index;
  float      clutch_engagement;
  lcc_bool_t shifting;
} lcc_transmission_state_t;

typedef struct {
  float temp_c[LCC_MAX_WHEELS];
  float pad_wear[LCC_MAX_WHEELS];
} lcc_brake_state_t;

typedef struct {
  float      battery_soc;
  lcc_bool_t consumers_headlights;
  float      bus_voltage_v;  /* solved system voltage */
  float      alt_current_a;  /* alternator output current (+ to bus) */
  float      batt_current_a; /* battery current (+ discharging to bus, - charging) */
} lcc_electrics_state_t;

typedef struct {
  float fuel_l;
} lcc_fuel_state_t;

typedef struct {
  float coolant_temp_c;
} lcc_cooling_state_t;

typedef struct {
  double time_s;
  float  pos_world[2];
  float  vel_world[2];
  float  acc_world[2];
  float  yaw_rad;
  float  yaw_rate_radps;
  float  vel_body[2];
  float  acc_body[2];
  float  speed_mps;
  float  mass_kg;
} lcc_car_state_t;

typedef struct {
  float brake_health[LCC_MAX_WHEELS];
  float tire_health[LCC_MAX_WHEELS];
} lcc_damage_state_t;

typedef struct {
  lcc_event_type_t type;
  double           time_s;
  int              data_i32;
  float            data_f32;
} lcc_event_t;

typedef void (*lcc_event_cb)(const lcc_event_t *evt, void *user);

typedef struct {
  lcc_car_desc_t    desc;
  lcc_environment_t env;
  lcc_controls_t    controls;

  /* runtime states */
  lcc_car_state_t          car_state;
  lcc_engine_state_t       engine_state;
  lcc_transmission_state_t trans_state;
  lcc_brake_state_t        brake_state;
  lcc_electrics_state_t    elec_state;
  lcc_fuel_state_t         fuel_state;
  lcc_cooling_state_t      cool_state;
  lcc_wheel_state_t        wheel_states[LCC_MAX_WHEELS];
  lcc_damage_state_t       damage_state;

  int   wheel_count;
  float wheel_steer_rad[LCC_MAX_WHEELS];
  float wheel_static_load_n[LCC_MAX_WHEELS];

  float shift_timer_s;
  int   pending_gear_index;

  /* aids internal state TODO: refactor this, this is bad lol? */
  float abs_mod[LCC_MAX_WHEELS];
  float tc_cut; /* 0..1 cut factor applied to engine torque */
  float esc_extra_brake[LCC_MAX_WHEELS];
  float idle_i; /* idle control integrator (internal) */

  float engine_crank_timer_s;

  /* generated engine maps (owned by car; freed on destroy or when replaced) */
  lcc_curve1d_point_t *owned_wot_pts;
  int                  owned_wot_count;
  lcc_curve1d_point_t *owned_fric_pts;
  int                  owned_fric_count;
  lcc_curve1d_point_t *owned_thr_pts;
  int                  owned_thr_count;
  lcc_map2d_point_t   *owned_boost_pts;
  int                  owned_boost_count;

  /* tire slip dynamics states for Pacejka */
  float slip_kappa_filt[LCC_MAX_WHEELS];
  float slip_alpha_filt[LCC_MAX_WHEELS];

  lcc_event_cb evt_cb;
  void        *evt_user;

  double last_overheat_evt_time;
  double last_abs_evt_time;
  double last_tc_evt_time;
  double last_esc_evt_time;
} lcc_car_t;

/*}}}*/

/* ============================== API declarations ============================== {{{*/

/* library/system functions */
LCC_API void        lcc_set_allocators(lcc_alloc_fn alloc_fn, lcc_free_fn free_fn, void *user);
LCC_API const char *lcc_version_string(void);

/* descriptor defaults */
LCC_API void lcc_car_desc_init_defaults(lcc_car_desc_t *desc);
LCC_API void lcc_engine_desc_init_defaults(lcc_engine_desc_t *desc);
LCC_API void lcc_fuel_desc_init_defaults(lcc_fuel_desc_t *desc);
LCC_API void lcc_cooling_desc_init_defaults(lcc_cooling_desc_t *desc);
LCC_API void lcc_battery_desc_init_defaults(lcc_battery_desc_t *desc);
LCC_API void lcc_alternator_desc_init_defaults(lcc_alternator_desc_t *desc);
LCC_API void lcc_starter_desc_init_defaults(lcc_starter_desc_t *desc);
LCC_API void lcc_ecu_desc_init_defaults(lcc_ecu_desc_t *desc);
LCC_API void lcc_transmission_desc_init_defaults(lcc_transmission_desc_t *desc);
LCC_API void lcc_driveline_desc_init_defaults(lcc_driveline_desc_t *desc);
LCC_API void lcc_chassis_desc_init_defaults(lcc_chassis_desc_t *desc);
LCC_API void lcc_aero_desc_init_defaults(lcc_aero_desc_t *desc);
LCC_API void lcc_wheel_desc_init_defaults(lcc_wheel_desc_t *desc);
LCC_API void lcc_tire_desc_init_defaults(lcc_tire_desc_t *desc);
LCC_API void lcc_brake_desc_init_defaults(lcc_brake_desc_t *desc);
LCC_API void lcc_arb_desc_init_defaults(lcc_arb_desc_t *desc);
LCC_API void lcc_steering_desc_init_defaults(lcc_steering_desc_t *desc);
LCC_API void lcc_environment_init_defaults(lcc_environment_t *env);
LCC_API void lcc_engine_simple_spec_init_defaults(lcc_engine_simple_spec_t *spec);

LCC_API lcc_result_t lcc_car_generate_engine_from_simple_spec(lcc_car_t *car, const lcc_engine_simple_spec_t *spec);

/* lifecycle */
LCC_API lcc_car_t   *lcc_car_create(const lcc_car_desc_t *desc);
LCC_API void         lcc_car_destroy(lcc_car_t *car);
LCC_API lcc_result_t lcc_car_reset(lcc_car_t *car, const lcc_car_state_t *optional_state);

/* simulation stepping */
LCC_API void         lcc_car_set_controls(lcc_car_t *car, const lcc_controls_t *controls);
LCC_API void         lcc_car_get_controls(const lcc_car_t *car, lcc_controls_t *controls_out);
LCC_API lcc_result_t lcc_car_step(lcc_car_t *car, float dt_s);

/* configuration at runtime */
LCC_API void lcc_car_set_environment(lcc_car_t *car, const lcc_environment_t *env);
LCC_API void lcc_car_get_environment(const lcc_car_t *car, lcc_environment_t *env_out);

LCC_API lcc_result_t lcc_car_set_engine_map(lcc_car_t *car, const lcc_curve1d_t *wot_torque, const lcc_curve1d_t *friction);
LCC_API lcc_result_t lcc_car_set_boost_map(lcc_car_t *car, const lcc_map2d_t *boost);
LCC_API lcc_result_t lcc_car_set_gear_ratios(lcc_car_t *car, const float *gear_ratios, int gear_count, float final_drive);
LCC_API lcc_result_t lcc_car_set_diff_params(lcc_car_t *car, lcc_diff_type_t front, lcc_diff_type_t rear, float preload_nm, float bias_ratio);
LCC_API lcc_result_t lcc_car_set_tire_params(lcc_car_t *car, int wheel_index, const lcc_tire_desc_t *tire);
LCC_API lcc_result_t lcc_car_set_brake_params(lcc_car_t *car, int wheel_index, const lcc_brake_desc_t *brake);
LCC_API lcc_result_t lcc_car_set_arb_params(lcc_car_t *car, const lcc_arb_desc_t *arb);
LCC_API lcc_result_t lcc_car_set_steering_params(lcc_car_t *car, const lcc_steering_desc_t *steer);

/* powertrain controls */
LCC_API lcc_result_t lcc_car_request_gear(lcc_car_t *car, int gear_index);
LCC_API lcc_result_t lcc_car_shift_up(lcc_car_t *car);
LCC_API lcc_result_t lcc_car_shift_down(lcc_car_t *car);

/* fuel and energy management */
LCC_API lcc_result_t lcc_car_refuel(lcc_car_t *car, float liters);
LCC_API lcc_result_t lcc_car_set_fuel(lcc_car_t *car, float liters);
LCC_API lcc_result_t lcc_car_recharge_battery(lcc_car_t *car, float state_of_charge_0_to_1);

/* driver aids toggles */
LCC_API void lcc_car_set_abs(lcc_car_t *car, lcc_abs_mode_t mode);
LCC_API void lcc_car_set_tc(lcc_car_t *car, lcc_tc_mode_t mode);
LCC_API void lcc_car_set_esc(lcc_car_t *car, lcc_esc_mode_t mode);

/* pos and velocity */
LCC_API void lcc_car_set_pos(lcc_car_t *car, const float pos_world[2], float yaw_rad);
LCC_API void lcc_car_get_pos(const lcc_car_t *car, float pos_world_out[2], float *yaw_rad_out);
LCC_API void lcc_car_set_velocity(lcc_car_t *car, const float vel_world[2], float yaw_rate_radps);
LCC_API void lcc_car_get_velocity(const lcc_car_t *car, float vel_world_out[2], float *yaw_rate_radps_out);

/* utilities */
LCC_API void lcc_car_get_local_bounds(const lcc_car_t *car, float min_local_out[2], float max_local_out[2]);
LCC_API int  lcc_car_get_wheel_global_positions(const lcc_car_t *car, float out_positions[][2], int max_wheels);

/* event subscription */
LCC_API void lcc_car_set_event_callback(lcc_car_t *car, lcc_event_cb callback, void *user);

/* unit helpers */
LCC_API float lcc_deg_to_rad(float deg);
LCC_API float lcc_rad_to_deg(float rad);

/*}}}*/

#ifdef __cplusplus
}
#endif
#endif /* LIBCCAR_H */

#define LCC_IMPLEMENTATION
#ifdef LCC_IMPLEMENTATION

#include <math.h>
#include <stdlib.h>

/* ============================== helpers ============================== {{{*/

/* default engine curves so a car drives without user maps */
static const lcc_curve1d_point_t LCC__DEF_ENGINE_WOT_POINTS[]      = { { 800.0f, 100.0f }, { 1200.0f, 135.0f }, { 1800.0f, 165.0f }, { 2500.0f, 185.0f }, { 3200.0f, 200.0f }, { 4000.0f, 210.0f }, { 5000.0f, 205.0f }, { 6000.0f, 190.0f }, { 6500.0f, 175.0f } };
static const lcc_curve1d_point_t LCC__DEF_ENGINE_FRICTION_POINTS[] = { { 600.0f, 10.0f }, { 1000.0f, 13.0f }, { 2000.0f, 18.0f }, { 3000.0f, 24.0f }, { 4000.0f, 30.0f }, { 5000.0f, 36.0f }, { 6000.0f, 42.0f }, { 7000.0f, 49.0f } };
static const lcc_curve1d_point_t LCC__DEF_THROTTLE_MAP_POINTS[]    = { { 0.0f, 0.0f }, { 0.25f, 0.25f }, { 0.5f, 0.5f }, { 0.75f, 0.75f }, { 1.0f, 1.0f } };

/* allocator state */
static lcc_alloc_fn lcc__alloc      = NULL;
static lcc_free_fn  lcc__free       = NULL;
static void        *lcc__alloc_user = NULL;

/* default allocators */
static void *lcc__malloc(size_t size, void *user) {
  (void)user;
  return malloc(size);
}

static void lcc__freefn(void *ptr, void *user) {
  (void)user;
  free(ptr);
}

static inline void mem_zero(void *ptr, size_t n) {
#if defined(__has_builtin)
#if __has_builtin(__builtin_memset)
  __builtin_memset(ptr, 0, n);
#endif
#elif defined(__GNUC__) || defined(__clang__)
  __builtin_memset(ptr, 0, n);
#else
  unsigned char *p = (unsigned char *)ptr;
  while(n--) *p++ = 0;
#endif
}

#define lcc__zero(x)  mem_zero(&(x), sizeof(x))
#define lcc__pzero(x) mem_zero((x), sizeof(*x))

/* math helpers */
static float lcc__clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static float lcc__saturate(float v) {
  return lcc__clampf(v, 0.0f, 1.0f);
}

static float lcc__signf(float v) {
  return (v > 0.0f) - (v < 0.0f);
}

static float lcc__absf(float v) {
  return v < 0.0f ? -v : v;
}

static float lcc__lerp(float a, float b, float t) {
  return a + (b - a) * t;
}

static float lcc__safe_inv(float v, float eps) {
  return lcc__absf(v) > eps ? 1.0f / v : 0.0f;
}

static float lcc__deg2rad(float d) {
  return d * (LCC_PI / 180.0f);
}

static float lcc__rad2deg(float r) {
  return r * (180.0f / LCC_PI);
}

static float lcc__hypot2(float x, float y) {
  return sqrtf(x * x + y * y);
}

static float lcc__smoothstep2(float x) {
  return x * x * x * (10.0f + x * (-15.0f + 6.0f * x)); /* smootherstep */
}

/* vector helpers using float[2] */
static void lcc__v2_zero(float a[2]) {
  a[0] = 0.0f;
  a[1] = 0.0f;
}

static float lcc__v2_len(const float a[2]) {
  return sqrtf(a[0] * a[0] + a[1] * a[1]);
}

static void lcc__v2_norm(float a[2]) {
  float l = lcc__v2_len(a);
  if(l > 1e-6f) {
    a[0] /= l;
    a[1] /= l;
  }
}

static void lcc__v2_rot(float out[2], const float in[2], float ang) {
  float c = cosf(ang), s = sinf(ang);
  out[0] = c * in[0] - s * in[1];
  out[1] = s * in[0] + c * in[1];
}

/* curve/map evaluators */
static float lcc__curve1d_eval(const lcc_curve1d_t *c, float x) {
  if(!c || !c->points || c->count <= 0) return 0.0f;
  if(c->count == 1) return c->points[0].y;
  if(x <= c->points[0].x) return c->points[0].y;
  if(x >= c->points[c->count - 1].x) return c->points[c->count - 1].y;
  int lo = 0, hi = c->count - 1;
  while(hi - lo > 1) {
    int mid = (lo + hi) >> 1;
    if(x >= c->points[mid].x) lo = mid;
    else
      hi = mid;
  }
  float x0 = c->points[lo].x, y0 = c->points[lo].y;
  float x1 = c->points[hi].x, y1 = c->points[hi].y;
  float t = (x - x0) * lcc__safe_inv((x1 - x0), 1e-9f);
  return lcc__lerp(y0, y1, lcc__saturate(t));
}

static float lcc__map2d_eval(const lcc_map2d_t *m, float x, float y) {
  if(!m || !m->points || m->count <= 0) return 0.0f;
  float bestd2 = 1e30f;
  float bestv  = m->points[0].v;
  for(int i = 0; i < m->count; ++i) {
    float dx = m->points[i].x - x;
    float dy = m->points[i].y - y;
    float d2 = dx * dx + dy * dy;
    if(d2 < bestd2) {
      bestd2 = d2;
      bestv  = m->points[i].v;
    }
  }
  return bestv;
}

static float lcc__rpm_to_radps(float rpm) {
  return rpm * (LCC_TAU / 60.0f);
}

static float lcc__radps_to_rpm(float radps) {
  return radps * (60.0f / LCC_TAU);
}

/* event emit helper forward */
static void lcc__emit_event(lcc_car_t *car, lcc_event_type_t type, int data_i32, float data_f32);

/* longitudinal slip ratio */
static float lcc__slip_ratio(float Vlong, float omega, float r) {
  float wR    = omega * r;
  float denom = fmaxf(fabsf(Vlong), 0.3f);
  float s     = (wR - Vlong) / denom;
  return lcc__clampf(s, -3.0f, 3.0f);
}

/* slip angle with sign-preserving Vx softening */
static float lcc__slip_angle(float Vlong, float Vlat) {
  return atan2f(Vlat, fmaxf(fabsf(Vlong), 0.1f));
}

/*  electrics helpers */
static float lcc__batt_ocv(const lcc_battery_desc_t *bd, float soc) {
  float s = lcc__saturate(soc);
  /* simple linear OCV model across SOC */
  return bd->ocv_empty_v + (bd->ocv_full_v - bd->ocv_empty_v) * s;
}

static float lcc__batt_r_internal(const lcc_battery_desc_t *bd, float soc, float temp_c) {
  float s       = lcc__saturate(soc);
  float cold    = lcc__clampf((25.0f - temp_c) / 30.0f, 0.0f, 1.0f); /* 0 at 25C, 1 at -5C */
  float r_scale = 1.0f + LCC_BATT_R_SOC_GAIN * (1.0f - s) + LCC_BATT_R_TEMP_GAIN * cold;
  float R       = bd->internal_resistance_ohm * r_scale;
  return lcc__clampf(R, 0.005f, 0.12f);
}

static float lcc__batt_cap_temp_scale(float temp_c) {
  /* crude lead-acid capacity reduction in cold */
  float dT = 25.0f - temp_c;
  float sc = 1.0f + LCC_BATT_CAP_TEMP_COEF_PER_K * dT;
  return lcc__clampf(sc, 0.55f, 1.10f);
}

static float lcc__alt_current_cap(const lcc_alternator_desc_t *alt, float rpm, lcc_bool_t engine_running) {
  if(!engine_running) return 0.0f;
  if(rpm < alt->cut_in_rpm) return 0.0f;
  float frac = lcc__clampf((rpm - alt->cut_in_rpm) / 2000.0f, 0.0f, 1.0f);
  return alt->max_current_a * frac;
}

/* free any owned generated maps */
static void lcc__free_owned_engine_maps(lcc_car_t *car) {
  if(!car) return;
  if(car->owned_wot_pts) {
    lcc__free(car->owned_wot_pts, lcc__alloc_user);
    car->owned_wot_pts   = NULL;
    car->owned_wot_count = 0;
  }
  if(car->owned_fric_pts) {
    lcc__free(car->owned_fric_pts, lcc__alloc_user);
    car->owned_fric_pts   = NULL;
    car->owned_fric_count = 0;
  }
  if(car->owned_thr_pts) {
    lcc__free(car->owned_thr_pts, lcc__alloc_user);
    car->owned_thr_pts   = NULL;
    car->owned_thr_count = 0;
  }
  if(car->owned_boost_pts) {
    lcc__free(car->owned_boost_pts, lcc__alloc_user);
    car->owned_boost_pts   = NULL;
    car->owned_boost_count = 0;
  }
}

static void lcc__engine_shape_params(lcc_forced_induction_t fi, float redline, float *out_tpeak_mult, float *out_rpm_tpeak, float *out_rpm_pwr) {
  float tmult = 1.20f; /* Tpeak ~ k * T@P_rated */
  float rpm_t = 0.45f * redline;
  float rpm_p = 0.87f * redline;
  switch(fi) {
  default:
  case LCC_FI_NONE:
    tmult = 1.20f;
    rpm_t = 0.45f * redline;
    rpm_p = 0.87f * redline;
    break;
  case LCC_FI_TURBO:
    tmult = 1.45f;
    rpm_t = 0.35f * redline;
    rpm_p = 0.80f * redline;
    break;
  case LCC_FI_SUPERCHARGER:
    tmult = 1.35f;
    rpm_t = 0.40f * redline;
    rpm_p = 0.85f * redline;
    break;
  case LCC_FI_TWINCHARGED:
    tmult = 1.50f;
    rpm_t = 0.35f * redline;
    rpm_p = 0.82f * redline;
    break;
  }
  if(out_tpeak_mult) *out_tpeak_mult = tmult;
  if(out_rpm_tpeak) *out_rpm_tpeak = fmaxf(1000.0f, rpm_t);
  if(out_rpm_pwr) *out_rpm_pwr = fmaxf(1500.0f, rpm_p);
}

static float lcc__kpa_target_from_fi(lcc_forced_induction_t fi, float user_target) {
  if(user_target > 0.0f) return user_target;
  switch(fi) {
  default:
  case LCC_FI_NONE: return 0.0f;
  case LCC_FI_TURBO: return 100.0f;       /* ~1 bar gauge */
  case LCC_FI_SUPERCHARGER: return 60.0f; /* ~0.6 bar */
  case LCC_FI_TWINCHARGED: return 120.0f; /* 1.2 bar */
  }
}

static void lcc__make_throttle_map_points(float gamma, lcc_curve1d_point_t *pts, int *count_out) {
  static const float xs[5] = { 0.0f, 0.25f, 0.5f, 0.75f, 1.0f };
  for(int i = 0; i < 5; ++i) {
    float x  = xs[i];
    float y  = powf(lcc__saturate(x), lcc__clampf(gamma, 0.5f, 1.5f));
    pts[i].x = x;
    pts[i].y = y;
  }
  if(count_out) *count_out = 5;
}

static void lcc__make_boost_map_points(lcc_forced_induction_t fi, float redline, float target_kpa, lcc_map2d_point_t *pts, int *count_out) {
  int n = 0;
  if(fi == LCC_FI_NONE || target_kpa <= 1.0f) {
    if(count_out) *count_out = 0;
    return;
  }
  float rpm_nodes[7];
  rpm_nodes[0] = 0.0f;
  rpm_nodes[1] = 0.20f * redline;
  rpm_nodes[2] = 0.30f * redline;
  rpm_nodes[3] = 0.40f * redline;
  rpm_nodes[4] = 0.55f * redline;
  rpm_nodes[5] = 0.75f * redline;
  rpm_nodes[6] = 0.95f * redline;

  float thr_nodes[5] = { 0.2f, 0.4f, 0.6f, 0.8f, 1.0f };

  float spool_lo = (fi == LCC_FI_TURBO || fi == LCC_FI_TWINCHARGED) ? (0.28f * redline) : (0.10f * redline);
  float spool_hi = (fi == LCC_FI_TURBO || fi == LCC_FI_TWINCHARGED) ? (0.48f * redline) : (0.20f * redline);

  for(int i = 0; i < 7; ++i) {
    float rpm = rpm_nodes[i];
    float s   = 1.0f;
    if(fi == LCC_FI_TURBO || fi == LCC_FI_TWINCHARGED) {
      float t = (rpm - spool_lo) / fmaxf(200.0f, spool_hi - spool_lo);
      s       = lcc__saturate(lcc__smoothstep2(t));
    } else if(fi == LCC_FI_SUPERCHARGER) {
      s = lcc__saturate(0.5f + 0.5f * (rpm / fmaxf(1000.0f, redline))); /* slight rise with rpm */
    }
    for(int j = 0; j < 5; ++j) {
      float th = thr_nodes[j];
      float tn = powf(th, (fi == LCC_FI_TURBO || fi == LCC_FI_TWINCHARGED) ? 1.1f : 1.0f);
      float k  = target_kpa * s * tn;
      pts[n].x = rpm;
      pts[n].y = th;
      pts[n].v = k;
      n++;
    }
  }
  if(count_out) *count_out = n;
}

/*}}}*/

/* ============================== physics implementation ============================== {{{*/

static void lcc__compute_static_loads(lcc_car_t *car) {
  float m = car->car_state.mass_kg;
  if(car->wheel_count <= 0) return;

  float half_wb    = fmaxf(0.05f, car->desc.chassis.wheelbase_m * 0.5f);
  float cg_x       = car->desc.chassis.cg_local_x;
  float front_frac = (half_wb + cg_x) / car->desc.chassis.wheelbase_m;
  front_frac       = lcc__clampf(front_frac, 0.0f, 1.0f);

  int front_wheels = 0, rear_wheels = 0;
  for(int i = 0; i < car->wheel_count; ++i)
    if(car->desc.wheels[i].position_local[0] >= 0.0f) front_wheels++;
    else
      rear_wheels++;

  float front_total = m * 9.81 * front_frac;
  float rear_total  = m * 9.81 * (1.0f - front_frac);

  float per_front = (front_wheels > 0) ? (front_total / (float)front_wheels) : 0.0f;
  float per_rear  = (rear_wheels > 0) ? (rear_total / (float)rear_wheels) : 0.0f;

  for(int i = 0; i < car->wheel_count; ++i) {
    int   is_front                      = (car->desc.wheels[i].position_local[0] >= 0.0f);
    float base                          = is_front ? per_front : per_rear;
    car->wheel_static_load_n[i]         = base;
    car->wheel_states[i].normal_force_n = base;
  }
}

static void lcc__init_runtime(lcc_car_t *car) {
  lcc__zero(car->car_state);
  lcc__zero(car->engine_state);
  lcc__zero(car->trans_state);
  lcc__zero(car->brake_state);
  lcc__zero(car->elec_state);
  lcc__zero(car->fuel_state);
  lcc__zero(car->cool_state);
  lcc__zero(car->wheel_states); // note: no [0]; this zeros the whole array
  lcc__zero(car->damage_state);
  lcc__zero(car->controls);
  lcc__zero(car->abs_mod);
  lcc__zero(car->esc_extra_brake);
  lcc__zero(car->slip_kappa_filt);
  lcc__zero(car->slip_alpha_filt);

  car->tc_cut = 0.0f;

  car->wheel_count = car->desc.wheel_count;
  for(int i = 0; i < car->wheel_count; ++i) {
    car->wheel_steer_rad[i]                = 0.0f;
    car->brake_state.temp_c[i]             = car->env.ambient_temp_c;
    car->brake_state.pad_wear[i]           = 0.0f;
    car->wheel_states[i].tire_temp_c       = car->env.surface_temp_c;
    car->wheel_states[i].tire_wear         = 0.0f;
    car->abs_mod[i]                        = 1.0f;
    car->wheel_states[i].omega_radps       = 0.0f;
    car->wheel_states[i].drive_torque_nm   = 0.0f;
    car->wheel_states[i].brake_torque_nm   = 0.0f;
    car->wheel_states[i].slip_ratio        = 0.0f;
    car->wheel_states[i].slip_angle_rad    = 0.0f;
    car->wheel_states[i].tire_force_long_n = 0.0f;
    car->wheel_states[i].tire_force_lat_n  = 0.0f;
  }

  car->car_state.time_s = 0.0;
  lcc__v2_zero(car->car_state.pos_world);
  lcc__v2_zero(car->car_state.vel_world);
  lcc__v2_zero(car->car_state.acc_world);
  lcc__v2_zero(car->car_state.vel_body);
  lcc__v2_zero(car->car_state.acc_body);
  car->car_state.yaw_rad        = 0.0f;
  car->car_state.yaw_rate_radps = 0.0f;

  car->fuel_state.fuel_l               = lcc__clampf(car->desc.fuel.initial_fuel_l, 0.0f, car->desc.fuel.tank_capacity_l);
  float fuel_mass                      = car->fuel_state.fuel_l * car->desc.fuel.fuel_density_kg_per_l;
  car->elec_state.battery_soc          = lcc__saturate(car->desc.battery.initial_soc);
  car->elec_state.consumers_headlights = 0;
  /* initialize a reasonable bus voltage from battery OCV */
  {
    float soc                     = car->elec_state.battery_soc;
    float ocv                     = car->desc.battery.ocv_empty_v + (car->desc.battery.ocv_full_v - car->desc.battery.ocv_empty_v) * lcc__saturate(soc);
    car->elec_state.bus_voltage_v = ocv;
  }

  /* generated engine maps ownership init */
  car->owned_wot_pts     = NULL;
  car->owned_wot_count   = 0;
  car->owned_fric_pts    = NULL;
  car->owned_fric_count  = 0;
  car->owned_thr_pts     = NULL;
  car->owned_thr_count   = 0;
  car->owned_boost_pts   = NULL;
  car->owned_boost_count = 0;

  car->car_state.mass_kg = car->desc.chassis.mass_kg + fuel_mass;

  car->engine_state.running  = 0;
  car->engine_state.cranking = 0;
  car->engine_crank_timer_s  = 0.0f;
  car->engine_state.rpm      = 0.0f;

  car->cool_state.coolant_temp_c = car->env.ambient_temp_c;

  car->trans_state.gear_index        = 1; /* start in neutral */
  car->trans_state.clutch_engagement = 1.0f;
  car->trans_state.shifting          = 0;

  car->shift_timer_s      = 0.0f;
  car->pending_gear_index = car->trans_state.gear_index;

  for(int i = 0; i < car->wheel_count; ++i) {
    car->damage_state.brake_health[i] = 1.0f;
    car->damage_state.tire_health[i]  = 1.0f;
  }

  car->last_overheat_evt_time = -1e9;
  car->last_abs_evt_time      = -1e9;
  car->last_tc_evt_time       = -1e9;
  car->last_esc_evt_time      = -1e9;

  lcc__compute_static_loads(car);

  /* initialize wheel patch positions in world right away */
  {
    for(int i = 0; i < car->wheel_count; ++i) {
      float local[2] = { car->desc.wheels[i].position_local[0], car->desc.wheels[i].position_local[1] };
      float world[2];
      lcc__v2_rot(world, local, car->car_state.yaw_rad);
      world[0] += car->car_state.pos_world[0];
      world[1] += car->car_state.pos_world[1];
    }
  }
}

/* event helper */
static void lcc__emit_event(lcc_car_t *car, lcc_event_type_t type, int data_i32, float data_f32) {
  if(!car || !car->evt_cb) return;
  lcc_event_t evt;
  evt.type     = type;
  evt.time_s   = car->car_state.time_s;
  evt.data_i32 = data_i32;
  evt.data_f32 = data_f32;
  car->evt_cb(&evt, car->evt_user);
}

/* Ackermann on front axle if two steerable wheels are found, else uniform */
static void lcc__apply_steering(lcc_car_t *car) {
  float in    = lcc__clampf(car->controls.steer, -1.0f, 1.0f);
  float max   = lcc__deg2rad(car->desc.steering.max_steer_deg);
  float delta = in * max;

  /* find front-left & front-right among steerables */
  int fiL = -1, fiR = -1;
  for(int i = 0; i < car->wheel_count; ++i) {
    if(car->desc.wheels[i].steerable && car->desc.wheels[i].position_local[0] >= 0.0f) {
      if(car->desc.wheels[i].position_local[1] < 0.0f) fiL = i;
      else
        fiR = i;
    }
  }

  float wb    = fmaxf(0.1f, car->desc.chassis.wheelbase_m);
  float track = fmaxf(0.4f, car->desc.chassis.track_front_m);
  float af    = lcc__saturate(car->desc.steering.ackermann_factor);

  if(fiL >= 0 && fiR >= 0 && fabsf(delta) > 1e-6f) {
    float R     = wb / fmaxf(1e-3f, fabsf(tanf(delta)));
    float d_in  = atanf(wb / fmaxf(0.05f, R - 0.5f * track));
    float d_out = atanf(wb / fmaxf(0.05f, R + 0.5f * track));
    if(delta > 0.0f) {
      /* steer right: FR inner, FL outer */
      car->wheel_steer_rad[fiR] = lcc__lerp(delta, d_in, af);
      car->wheel_steer_rad[fiL] = lcc__lerp(delta, d_out, af);
    } else {
      /* steer left: FL inner, FR outer */
      car->wheel_steer_rad[fiL] = -lcc__lerp(-delta, d_in, af);
      car->wheel_steer_rad[fiR] = -lcc__lerp(-delta, d_out, af);
    }
  } else {
    /* uniform steer for all steerable wheels (or non-front configurations) */
    for(int i = 0; i < car->wheel_count; ++i) car->wheel_steer_rad[i] = car->desc.wheels[i].steerable ? delta : 0.0f;
  }
}

/* tire mu and stiffness */
static float lcc__tire_mu(const lcc_tire_desc_t *td, float fz, float global_scale, float health) {
  float mu = td->mu_nominal + td->load_sensitivity * fz;
  mu       = lcc__clampf(mu, 0.2f, 3.0f);
  mu *= lcc__clampf(global_scale, 0.1f, 2.0f);
  mu *= lcc__clampf(health, 0.1f, 1.0f);
  return mu;
}

/* tire stiffness scaled by pressure and width (normalized to ~0.22m) */
static void lcc__tire_stiffness(const lcc_tire_desc_t *td, float fz, float tire_width_m, float *Cx_out, float *Cy_out) {
  float p_ratio = (td->ideal_pressure_kpa > 1.0f) ? (td->pressure_kpa / td->ideal_pressure_kpa) : 1.0f;
  p_ratio       = lcc__clampf(p_ratio, 0.6f, 1.4f);
  float base    = fmaxf(fz, 100.0f);
  float w_scale = lcc__clampf((tire_width_m > 0.0f) ? (tire_width_m / 0.22f) : 1.0f, 0.6f, 1.6f);
  float Cx      = 12.0f * base * p_ratio * w_scale;
  float Cy      = 16.0f * base * p_ratio * w_scale;
  if(Cx_out) *Cx_out = Cx;
  if(Cy_out) *Cy_out = Cy;
}

/* Pacejka */
static float lcc__pacejka_magic(float x, float B, float C, float D, float E) {
  /* D*sin(C*atan(Bx - E(Bx - atan(Bx)))) */
  float Bx     = B * x;
  float atanBx = atanf(Bx);
  float inner  = Bx - E * (Bx - atanBx);
  return D * sinf(C * atanf(inner));
}

/* derive B,C,D,E from muFz (D) and small-slip stiffness K ( dF/dx at 0) */
static void lcc__pacejka_coeffs_from_muK(float mu, float Fz, float K, float C_def, float E_def, float *B_out, float *C_out, float *D_out, float *E_out) {
  float D     = fmaxf(0.0f, mu) * fmaxf(0.0f, Fz);
  float C     = fmaxf(0.8f, C_def);
  float E     = lcc__clampf(E_def, 0.0f, 1.2f);
  float denom = fmaxf(1e-3f, C * D);
  float B     = lcc__clampf(K / denom, 0.01f, 50.0f);
  if(B_out) *B_out = B;
  if(C_out) *C_out = C;
  if(D_out) *D_out = D;
  if(E_out) *E_out = E;
}

static float lcc__pacejka_Fx_pure(float kappa, float mu, float Fz, float Kx) {
  float B, C, D, E;
  lcc__pacejka_coeffs_from_muK(mu, Fz, Kx, LCC_PACEJKA_CX, LCC_PACEJKA_EX, &B, &C, &D, &E);
  return lcc__pacejka_magic(kappa, B, C, D, E);
}

static float lcc__pacejka_Fy_pure(float alpha, float mu, float Fz, float Ky) {
  float B, C, D, E;
  lcc__pacejka_coeffs_from_muK(mu, Fz, Ky, LCC_PACEJKA_CY, LCC_PACEJKA_EY, &B, &C, &D, &E);
  /* negative to oppose lateral slip */
  return -lcc__pacejka_magic(alpha, B, C, D, E);
}

/* slip relaxation filtering */
static float lcc__relax_filt(float prev, float target, float Vrel, float Lrel, float dt) {
  if(Lrel <= 1e-4f) return target;
  float rate = Vrel / Lrel; /* [1/s] */
  float a    = lcc__clampf(rate * dt, 0.0f, 1.0f);
  return prev + a * (target - prev);
}

/* aero */
static void lcc__aero_compute(const lcc_car_t *car, const float rel_vel_world[2], float out_drag_world[2], float *df_front, float *df_rear) {
  float rho  = car->env.air_density;
  float v    = lcc__v2_len(rel_vel_world);
  float q    = 0.5f * rho * v * v;
  float Cd   = car->desc.aero.drag_coefficient;
  float A    = car->desc.aero.frontal_area_m2;
  float beta = 0.0f;
  if(car->car_state.speed_mps > 0.1f) {
    float vel_body[2];
    lcc__v2_rot(vel_body, car->car_state.vel_world, -car->car_state.yaw_rad);
    beta = atanf(lcc__absf(vel_body[1]) / fmaxf(0.1f, lcc__absf(vel_body[0])));
  }
  float drag_mag = q * Cd * A * (1.0f + car->desc.aero.yaw_drag_gain * lcc__absf(beta));
  float dir[2]   = { rel_vel_world[0], rel_vel_world[1] };
  lcc__v2_norm(dir);
  out_drag_world[0] = -drag_mag * dir[0];
  out_drag_world[1] = -drag_mag * dir[1];

  float Clf = car->desc.aero.lift_coefficient_front;
  float Clr = car->desc.aero.lift_coefficient_rear;
  float Fdf = q * A * (-Clf);
  float Fdr = q * A * (-Clr);
  if(df_front) *df_front = Fdf;
  if(df_rear) *df_rear = Fdr;
}

/* compute relative patch velocity in tire frame */
static void lcc__wheel_vel_tire_frame(const lcc_car_t *car, int i, float out_tire_vel[2]) {
  float r_local[2]     = { car->desc.wheels[i].position_local[0], car->desc.wheels[i].position_local[1] };
  float v_body[2]      = { car->car_state.vel_body[0], car->car_state.vel_body[1] };
  float v_spin_body[2] = { -car->car_state.yaw_rate_radps * r_local[1], car->car_state.yaw_rate_radps * r_local[0] };
  float hub_body[2]    = { v_body[0] + v_spin_body[0], v_body[1] + v_spin_body[1] };

  float steer = car->wheel_steer_rad[i];
  float hub_tire[2];
  lcc__v2_rot(hub_tire, hub_body, -steer);

  float rel_tire[2] = { hub_tire[0], hub_tire[1] };

  out_tire_vel[0] = rel_tire[0];
  out_tire_vel[1] = rel_tire[1];
}

/* normal loads including longitudinal and lateral transfer */
static void lcc__compute_normal_loads(lcc_car_t *car, float df_front, float df_rear) {
  float m       = car->car_state.mass_kg;
  float wb      = fmaxf(0.1f, car->desc.chassis.wheelbase_m);
  float track_f = fmaxf(0.4f, car->desc.chassis.track_front_m);
  float track_r = fmaxf(0.4f, car->desc.chassis.track_rear_m);
  float h       = lcc__clampf(car->desc.chassis.cg_height_m, 0.0f, 1.5f);

  /* base static axle loads */
  float Fz_front_static = 0.0f, Fz_rear_static = 0.0f;
  for(int i = 0; i < car->wheel_count; ++i)
    if(car->desc.wheels[i].position_local[0] >= 0.0f) Fz_front_static += car->wheel_static_load_n[i];
    else
      Fz_rear_static += car->wheel_static_load_n[i];

  Fz_front_static += df_front;
  Fz_rear_static += df_rear;

  /* body-frame accelerations */
  float ax = car->car_state.acc_body[0];
  float ay = car->car_state.acc_body[1];

  /* longitudinal transfer: forward accel -> rear gain */
  float dF_long = m * ax * h / wb;

  /* lateral transfer per axle */
  float dF_lat_front = m * ay * h / track_f;
  float dF_lat_rear  = m * ay * h / track_r;

  float Fz_front = Fz_front_static - dF_long;
  float Fz_rear  = Fz_rear_static + dF_long;

  int front_wheels = 0, rear_wheels = 0;
  for(int i = 0; i < car->wheel_count; ++i)
    if(car->desc.wheels[i].position_local[0] >= 0.0f) front_wheels++;
    else
      rear_wheels++;
  float per_front = (front_wheels > 0) ? (Fz_front / (float)front_wheels) : 0.0f;
  float per_rear  = (rear_wheels > 0) ? (Fz_rear / (float)rear_wheels) : 0.0f;

  for(int i = 0; i < car->wheel_count; ++i) {
    int   is_front                      = (car->desc.wheels[i].position_local[0] >= 0.0f);
    float base                          = is_front ? per_front : per_rear;
    float side                          = (car->desc.wheels[i].position_local[1] < 0.0f) ? -1.0f : 1.0f; /* left<0, right>0 */
    float dlat                          = is_front ? (-side * 0.5f * dF_lat_front) : (-side * 0.5f * dF_lat_rear);
    float Fz                            = fmaxf(0.0f, base + dlat);
    car->wheel_states[i].normal_force_n = Fz;
  }
}

/* diff distributions */
typedef struct lcc_axle_torques_s {
  float left_nm;
  float right_nm;
} lcc_axle_torques_t;

static lcc_axle_torques_t lcc__diff_open(float T_in, float Tlim_L, float Tlim_R) {
  lcc_axle_torques_t r;
  float              s    = lcc__signf(T_in);
  float              Tin  = lcc__absf(T_in);
  float              TLc  = fmaxf(0.0f, Tlim_L);
  float              TRc  = fmaxf(0.0f, Tlim_R);
  float              half = 0.5f * Tin;
  float              TL   = fminf(half, TLc);
  float              TR   = fminf(half, TRc);
  r.left_nm               = s * TL;
  r.right_nm              = s * TR;
  return r;
}

static lcc_axle_torques_t lcc__diff_locked(float T_in, float Tlim_L, float Tlim_R) {
  lcc_axle_torques_t r      = (lcc_axle_torques_t){ 0 };
  float              s      = lcc__signf(T_in);
  float              Tin    = lcc__absf(T_in);
  float              TLcap  = fmaxf(0.0f, Tlim_L);
  float              TRcap  = fmaxf(0.0f, Tlim_R);
  float              TL     = fminf(0.5f * Tin, TLcap);
  float              TR     = fminf(0.5f * Tin, TRcap);
  float              rem    = fmaxf(0.0f, Tin - (TL + TR));
  float              spareL = fmaxf(0.0f, TLcap - TL);
  float              dL     = fminf(spareL, 0.5f * rem);
  TL += dL;
  float spareR = fmaxf(0.0f, TRcap - TR);
  float dR     = fminf(spareR, rem - dL);
  TR += dR;
  r.left_nm  = s * TL;
  r.right_nm = s * TR;
  return r;
}

static lcc_axle_torques_t lcc__diff_torsen(float T_in, float Tlim_L, float Tlim_R, float bias_ratio, float preload_nm) {
  lcc_axle_torques_t r            = (lcc_axle_torques_t){ 0 };
  float              s            = lcc__signf(T_in);
  float              Tin          = lcc__absf(T_in);
  float              B            = fmaxf(1.0f, bias_ratio);
  float              low          = fminf(fmaxf(0.0f, Tlim_L), fmaxf(0.0f, Tlim_R));
  float              high         = fmaxf(fmaxf(0.0f, Tlim_L), fmaxf(0.0f, Tlim_R));
  float              preload_each = 0.5f * preload_nm;
  float              capacity     = low * (1.0f + B) + preload_nm;
  float              T_use        = fminf(Tin, capacity);
  float              Tlow         = fminf(low + preload_each, T_use / (1.0f + B));
  float              Thigh        = fminf(high + preload_each, T_use - Tlow);
  if(Tlim_L <= Tlim_R) {
    r.left_nm  = s * Tlow;
    r.right_nm = s * Thigh;
  } else {
    r.left_nm  = s * Thigh;
    r.right_nm = s * Tlow;
  }
  return r;
}

static lcc_axle_torques_t lcc__diff_clutch_lsd(float T_in, float Tlim_L, float Tlim_R, float bias_ratio, float preload_nm) {
  float B = fmaxf(1.0f, 0.5f * bias_ratio + 0.5f);
  return lcc__diff_torsen(T_in, Tlim_L, Tlim_R, B, preload_nm);
}

static lcc_axle_torques_t lcc__diff_active(float T_in, float Tlim_L, float Tlim_R, float lock_coef, float preload_nm) {
  lcc_axle_torques_t ro = lcc__diff_open(T_in, Tlim_L, Tlim_R);
  lcc_axle_torques_t rl = lcc__diff_locked(T_in, Tlim_L, Tlim_R);
  lcc_axle_torques_t r;
  float              k = lcc__saturate(lock_coef);
  r.left_nm            = lcc__lerp(ro.left_nm, rl.left_nm, k);
  r.right_nm           = lcc__lerp(ro.right_nm, rl.right_nm, k);
  float add            = 0.5f * preload_nm;
  r.left_nm            = fminf(r.left_nm + add, Tlim_L);
  r.right_nm           = fminf(r.right_nm + add, Tlim_R);
  return r;
}

/* engine torque model */
static float lcc__engine_torque_compute(lcc_car_t *car, float rpm, float throttle) {
  const lcc_engine_desc_t *ed   = &car->desc.engine;
  float                    load = throttle;
  if(ed->throttle_map.points && ed->throttle_map.count > 0) load = lcc__clampf(lcc__curve1d_eval(&ed->throttle_map, throttle), 0.0f, 1.0f);

  float wot      = lcc__curve1d_eval(&ed->wot_torque_nm_vs_rpm, rpm);
  float friction = lcc__curve1d_eval(&ed->friction_torque_nm_vs_rpm, rpm);
  if(friction < 0.0f) friction = 0.0f;

  float boost_kpa = 0.0f;
  if(ed->forced_induction != LCC_FI_NONE && ed->boost_pressure_kpa_vs_rpm_throttle.points) {
    boost_kpa = lcc__map2d_eval(&ed->boost_pressure_kpa_vs_rpm_throttle, rpm, throttle);
    boost_kpa = fminf(boost_kpa, ed->wastegate_pressure_kpa);
  }
  float map_kpa = 100.0f + boost_kpa;

  float pr = map_kpa / 100.0f;
  float tq = wot * load * pr - friction * (0.5f + 0.5f * (1.0f - load));

  if(rpm >= ed->redline_rpm) tq = fminf(tq, 0.0f);
  return tq;
}

/* driveline efficiency TODO: expand this lol? */
static float lcc__driveline_efficiency(const lcc_car_t *car) {
  (void)car;
  return 0.92f;
}

/* ABS modulation with speed-adaptive target and soft rate limiting;
   disabled below crawl so brakes can hold the car */
static float lcc__abs_apply(lcc_car_t *car, int i, float T_brake_cmd, float slip, float dt) {
  if(car->desc.ecu.abs_mode == LCC_ABS_OFF) return T_brake_cmd;

  float v = car->car_state.speed_mps;
  if(v < 1.0f || car->controls.brake <= 0.01f) {
    car->abs_mod[i] = 1.0f;
    return T_brake_cmd;
  }

  float sabs     = fabsf(slip);
  float s_target = 0.20f + 0.05f * expf(-v * 0.5f); /* a bit more slip allowed at very low speed */
  float dead     = 0.02f;

  /* higher speeds -> faster response */
  float k_up   = 3.0f + 3.0f * fminf(v, 20.0f) / 20.0f;
  float k_down = 10.0f + 8.0f * fminf(v, 20.0f) / 20.0f;

  float mod = car->abs_mod[i];
  if(sabs > s_target + dead) mod -= k_down * dt * (sabs - (s_target + dead));
  else if(sabs < s_target - dead)
    mod += k_up * dt * ((s_target - dead) - sabs);

  mod             = lcc__clampf(mod, 0.25f, 1.0f);
  car->abs_mod[i] = mod;
  return T_brake_cmd * mod;
}

/* tc engine cut update from driven wheel slip */
static void lcc__tc_update(lcc_car_t *car, float max_pos_slip, float dt) {
  if(car->desc.ecu.tc_mode == LCC_TC_OFF) {
    car->tc_cut = 0.0f;
    return;
  }
  float s_target = 0.12f;
  float cut      = car->tc_cut;
  if(max_pos_slip > s_target) cut += 6.0f * dt * (max_pos_slip - s_target);
  else
    cut -= 3.0f * dt * (s_target - max_pos_slip);
  car->tc_cut = lcc__clampf(cut, 0.0f, 0.7f); /* up to 70% cut */
}

/* esc yaw control -> additional brake torque distribution */
static void lcc__esc_update(lcc_car_t *car) {
  for(int i = 0; i < car->wheel_count; ++i) car->esc_extra_brake[i] = 0.0f;
  if(car->desc.ecu.esc_mode == LCC_ESC_OFF) return;

  float v = car->car_state.speed_mps;
  if(v < 1.0f) return;

  /* desired yaw rate from simple bicycle model approx */
  float wb    = fmaxf(0.1f, car->desc.chassis.wheelbase_m);
  float delta = 0.0f;
  for(int i = 0; i < car->wheel_count; ++i)
    if(car->desc.wheels[i].steerable) {
      delta = car->wheel_steer_rad[i];
      break;
    }
  float yaw_des = v / wb * tanf(delta);
  float yaw_err = car->car_state.yaw_rate_radps - yaw_des;

  float Kp     = 800.0f; /* yaw moment per rad/s error */
  float Mz_cmd = -Kp * yaw_err;

  /* convert Mz_cmd to additional brake torque on outside wheels */
  float track_f = fmaxf(0.8f, car->desc.chassis.track_front_m);
  float track_r = fmaxf(0.8f, car->desc.chassis.track_rear_m);
  float track   = 0.5f * (track_f + track_r);
  float r_eff   = 0.3f;
  /* choose outside wheels depending on yaw sign; left= y<0 */
  int   left_is_outside = (Mz_cmd < 0.0f) ? 1 : 0;
  float T_side          = lcc__absf(Mz_cmd) / fmaxf(0.2f, track) / fmaxf(0.05f, r_eff);
  T_side                = lcc__clampf(T_side, 0.0f, 1500.0f);

  for(int i = 0; i < car->wheel_count; ++i) {
    int is_left = (car->desc.wheels[i].position_local[1] < 0.0f);
    if((left_is_outside && is_left) || (!left_is_outside && !is_left)) car->esc_extra_brake[i] = T_side;
  }
}

/* braking torque command with thermal/wear/health and stable standstill gating */
static float lcc__wheel_brake_torque_base(const lcc_car_t *car, int i) {
  const lcc_brake_desc_t *br = &car->desc.brakes[i];
  if(!car->desc.wheels[i].has_brake) return 0.0f;

  float pedal = lcc__saturate(car->controls.brake);
  if(car->controls.handbrake > 0.0f && car->desc.wheels[i].position_local[0] < 0.0f) pedal = fmaxf(pedal, lcc__saturate(car->controls.handbrake));

  float T      = ((lcc_car_t *)car)->brake_state.temp_c[i];
  float f_temp = 1.0f;
  if(T <= 350.0f) f_temp = 1.0f;
  else if(T <= 600.0f)
    f_temp = 1.0f - 0.5f * ((T - 350.0f) / 250.0f);
  else if(T <= 800.0f)
    f_temp = 0.5f - 0.3f * ((T - 600.0f) / 200.0f);
  else
    f_temp = 0.2f;

  float f_wear   = lcc__clampf(1.0f - ((lcc_car_t *)car)->brake_state.pad_wear[i], 0.2f, 1.0f);
  float f_health = lcc__clampf(((lcc_car_t *)car)->damage_state.brake_health[i], 0.2f, 1.0f);

  float mu_scale = lcc__clampf(br->pad_mu / 0.4f, 0.7f, 1.3f);
  float r_scale  = lcc__clampf(br->disc_radius_m / 0.15f, 0.8f, 1.25f);

  float T_cmd = pedal * br->max_torque_nm;
  T_cmd *= mu_scale * r_scale;
  T_cmd *= f_temp * f_wear * f_health;
  T_cmd = fmaxf(0.0f, T_cmd);

  return T_cmd;
}

/* thermal models */
static void lcc__thermal_update(lcc_car_t *car, float engine_power_kw, float dt) {
  /* brakes */
  for(int i = 0; i < car->wheel_count; ++i) {
    float w    = lcc__absf(car->wheel_states[i].omega_radps);
    float P    = lcc__absf(car->wheel_states[i].brake_torque_nm * w); /* watts */
    float mcp  = 8000.0f * car->desc.brakes[i].cooling_area_m2;       /* j/k, crude */
    float hA   = 8.0f * car->desc.brakes[i].cooling_area_m2;          /* w/k */
    float T    = car->brake_state.temp_c[i];
    float Tamb = car->env.ambient_temp_c;
    float dT   = 0.0f;
    if(mcp > 1.0f) dT = (P - hA * (T - Tamb)) / mcp;
    T += dT * dt;
    car->brake_state.temp_c[i] = lcc__clampf(T, Tamb, 800.0f);
    /* wear increases with high temp and power */
    car->brake_state.pad_wear[i] = lcc__clampf(car->brake_state.pad_wear[i] + 1e-7f * P * dt + 1e-4f * fmaxf(0.0f, T - 300.0f) * dt, 0.0f, 1.0f);
    /* degrade brake health slowly with extreme heat */
    if(T > 600.0f) car->damage_state.brake_health[i] = lcc__clampf(car->damage_state.brake_health[i] - 1e-4f * dt * (T - 600.0f), 0.2f, 1.0f);
  }

  /* tires */
  for(int i = 0; i < car->wheel_count; ++i) {
    float vtire[2];
    lcc__wheel_vel_tire_frame(car, i, vtire);
    float slip_speed = lcc__absf(vtire[0] - car->wheel_states[i].omega_radps * car->desc.wheels[i].radius_m) + lcc__absf(vtire[1]);
    float P          = lcc__absf(car->wheel_states[i].tire_force_long_n * (vtire[1])) + lcc__absf(car->wheel_states[i].tire_force_lat_n * vtire[0]);
    float mcp        = 15000.0f; /* j/k per tire crude */
    float hA         = 6.0f;     /* w/k convective */
    float T          = car->wheel_states[i].tire_temp_c;
    float Tamb       = car->env.surface_temp_c;
    float dT         = (P * 0.2f - hA * (T - Tamb)) / mcp;
    T += dT * dt;
    car->wheel_states[i].tire_temp_c = lcc__clampf(T, Tamb, 180.0f);
    /* wear grows with slip and high temp */
    float wear_rate                  = car->desc.tires[i].wear_rate;
    car->wheel_states[i].tire_wear   = lcc__clampf(car->wheel_states[i].tire_wear + wear_rate * (0.00005f * slip_speed + 0.00002f * fmaxf(0.0f, T - 90.0f)) * dt, 0.0f, 1.0f);
    car->damage_state.tire_health[i] = lcc__clampf(1.0f - car->wheel_states[i].tire_wear, 0.2f, 1.0f);
  }

  /* engine cooling */
  float Tcool = car->cool_state.coolant_temp_c;
  float Tamb  = car->env.ambient_temp_c;
  float UA    = fmaxf(1.0f, car->desc.cooling.radiator_ua_w_per_k) * (1.0f + 0.02f * car->car_state.speed_mps);
  float fan   = (Tcool > car->desc.cooling.fan_on_c);
  UA += fan * 100.0f;
  float engine_kw = fmaxf(0.0f, engine_power_kw);
  float waste_kw  = engine_kw * 0.8f;                                                         /* lots of waste to heat */
  float Cc        = fmaxf(1000.0f, car->desc.engine.coolant_heat_capacity_j_per_k) / 1000.0f; /* kJ/K */
  float dTcool    = (waste_kw - UA * (Tcool - Tamb) / 1000.0f) / Cc;
  Tcool += dTcool * dt;
  car->cool_state.coolant_temp_c = Tcool;

  /* overheat event throttle */
  if(Tcool > 115.0f && (car->car_state.time_s - car->last_overheat_evt_time) > 1.0) {
    car->last_overheat_evt_time = car->car_state.time_s;
    lcc__emit_event(car, LCC_EVENT_OVERHEAT, 0, Tcool);
  }
}

static lcc_result_t lcc__car_step(lcc_car_t *car, float dt_s) {
  if(!car) return LCC_ERR_INVALID_ARG;
  if(!(dt_s > 0.0f) || dt_s > 0.1f) return LCC_ERR_INVALID_ARG;

  car->car_state.time_s += (double)dt_s;

  /* auto gearbox simple logic */
  if(car->desc.transmission.type != LCC_TRANS_MANUAL && !car->trans_state.shifting) {
    if(car->engine_state.rpm > car->desc.transmission.auto_upshift_rpm) lcc_car_shift_up(car);
    else if(car->engine_state.rpm < car->desc.transmission.auto_downshift_rpm)
      lcc_car_shift_down(car);
  }

  /* clutch engagement: 1 = engaged */
  float clutch_cmd = car->trans_state.gear_index != LCC_GEAR_NEUTRAL ? 1.0f - car->controls.clutch : 0.0f;
  if(car->desc.ecu.auto_clutch && car->trans_state.shifting) {
    float half  = 0.5f * fmaxf(0.05f, car->desc.transmission.shift_time_s);
    float t     = car->shift_timer_s;
    float phase = (t > half) ? 0.0f : (1.0f - t / fmaxf(1e-3f, half));
    clutch_cmd  = 1.0f - 0.9f * phase;
  }
  car->trans_state.clutch_engagement = lcc__clampf(clutch_cmd, 0.0f, 1.0f);

  /* steering */
  lcc__apply_steering(car);

  /* aero */
  float rel_vel_world[2] = { car->car_state.vel_world[0] - car->env.wind_world[0], car->car_state.vel_world[1] - car->env.wind_world[1] };
  float drag_world[2];
  float df_front = 0.0f, df_rear = 0.0f;
  lcc__aero_compute(car, rel_vel_world, drag_world, &df_front, &df_rear);

  /* normal loads (uses previous acc_body; OK) */
  lcc__compute_normal_loads(car, df_front, df_rear);

  /* driveline ratios */
  float gear_ratio = 0.0f;
  if(car->trans_state.gear_index >= 0 && car->trans_state.gear_index < car->desc.transmission.gear_count) gear_ratio = car->desc.transmission.gear_ratios[car->trans_state.gear_index];
  float final_drive   = car->desc.transmission.final_drive_ratio;
  float driveline_eff = lcc__driveline_efficiency(car);

  /* average driven wheel speed -> output rpm */
  float sum_omega_driven = 0.0f;
  int   count_driven     = 0;
  for(int i = 0; i < car->wheel_count; ++i) {
    if(car->desc.wheels[i].driven) {
      sum_omega_driven += car->wheel_states[i].omega_radps;
      count_driven++;
    }
  }
  float avg_omega     = (count_driven > 0) ? (sum_omega_driven / (float)count_driven) : 0.0f;
  float shaft_out_rpm = lcc__radps_to_rpm(avg_omega) * fmaxf(0.1f, final_drive);

  /* engine model */
  const lcc_engine_desc_t *ed             = &car->desc.engine;
  float                    tq_engine_comb = 0.0f; /* combustion torque (already net of internal friction in map) */
  float                    T_starter      = 0.0f; /* starter torque */
  float                    T_fric_drag    = 0.0f; /* passive friction when not combusting */

  /* cranking request: either from API (engine_state.cranking) or driver controls */
  int has_fuel   = (car->fuel_state.fuel_l > 1e-3f);
  int want_crank = (!car->engine_state.running) && (car->engine_state.cranking || (car->controls.ignition_switch && car->controls.starter));
  if(want_crank && has_fuel && car->elec_state.battery_soc > 0.05f) car->engine_state.cranking = 1, car->engine_crank_timer_s += dt_s;
  else if(!car->controls.starter) {
    car->engine_state.cranking = 0;
    car->engine_crank_timer_s  = 0.0f;
  }

  /* running -> produce combustion torque with idle control & TC cut */
  if(car->engine_state.running && car->controls.ignition_switch) {
    float throttle_eff = lcc__saturate(car->controls.throttle);

    /* idle control */
    if(car->desc.ecu.idle_control && throttle_eff < 0.05f) {
      float rpm_target = ed->idle_rpm;
      float err_rpm    = rpm_target - car->engine_state.rpm;
      float norm_err   = err_rpm / fmaxf(500.0f, rpm_target);
      float P          = car->desc.ecu.idle_pid_p;
      float I          = car->desc.ecu.idle_pid_i;

      car->idle_i += I * norm_err * dt_s;
      car->idle_i = lcc__clampf(car->idle_i, 0.0f, 0.3f);

      int   in_neutral = (car->trans_state.gear_index == LCC_GEAR_NEUTRAL) || (fabsf(gear_ratio) < 1e-3f);
      float clutch_eng = car->trans_state.clutch_engagement;
      float idle_max   = (in_neutral || clutch_eng < 0.2f) ? 0.30f : 0.12f;
      float idle_thr   = lcc__clampf(P * norm_err + car->idle_i, 0.0f, idle_max);
      if(idle_thr > throttle_eff) throttle_eff = idle_thr;
    } else {
      car->idle_i *= 0.9f;
    }

    throttle_eff *= (1.0f - car->tc_cut);
    tq_engine_comb = lcc__engine_torque_compute(car, car->engine_state.rpm, throttle_eff);

    if(!has_fuel) {
      tq_engine_comb            = 0.0f;
      car->engine_state.running = 0;
      lcc__emit_event(car, LCC_EVENT_FUEL_STARVATION, 0, 0.0f);
    }
  }

  /* passive friction/pumping drag when not combusting */
  if(!(car->engine_state.running && car->controls.ignition_switch)) {
    float fr = lcc__curve1d_eval(&ed->friction_torque_nm_vs_rpm, car->engine_state.rpm);
    if(fr > 0.0f) T_fric_drag = fr;
  }

  /* starter torque during cranking (power-limited) */
  if(car->engine_state.cranking) {
    float omega     = lcc__rpm_to_radps(car->engine_state.rpm);
    float omega_eff = fmaxf(omega, LCC_ENGINE_STARTER_MIN_OMEGA_RADPS);
    float P         = car->desc.starter.power_w;
    float soc_scale = lcc__clampf(car->elec_state.battery_soc * 1.2f, 0.25f, 1.0f);
    T_starter       = lcc__clampf((P * soc_scale) / omega_eff, 0.0f, LCC_ENGINE_STARTER_MAX_TORQUE_NM);
  }

  /* clutch/TC converter to trans input (only combustion torque goes to gearbox) */
  float T_to_trans = 0.0f;
  float clutch_k   = lcc__saturate(car->trans_state.clutch_engagement);
  float T_cap      = 800.0f * clutch_k;
  T_to_trans       = tq_engine_comb;
  if(fabsf(T_to_trans) > T_cap) T_to_trans = lcc__signf(T_to_trans) * T_cap;

  float Tw_total = T_to_trans * gear_ratio * final_drive * driveline_eff;

  /* axle split */
  float splitF = 0.0f, splitR = 0.0f;
  int   front_has = 0, rear_has = 0;
  for(int i = 0; i < car->wheel_count; ++i)
    if(car->desc.wheels[i].driven) {
      if(car->desc.wheels[i].position_local[0] >= 0.0f) front_has = 1;
      else
        rear_has = 1;
    }
  switch(car->desc.driveline.layout) {
  case LCC_LAYOUT_FWD:
    splitF = 1.0f;
    splitR = 0.0f;
    break;
  case LCC_LAYOUT_RWD:
    splitF = 0.0f;
    splitR = 1.0f;
    break;
  default: {
    splitF = lcc__clampf(car->desc.driveline.front_torque_split, 0.0f, 1.0f);
    splitR = 1.0f - splitF;
  } break;
  }
  if(!front_has) {
    splitF = 0.0f;
    splitR = 1.0f;
  }
  if(!rear_has) {
    splitF = 1.0f;
    splitR = 0.0f;
  }

  float Tw_front = Tw_total * splitF;
  float Tw_rear  = Tw_total * splitR;

  /* Precompute tire kinematics and base lateral demand for traction limits */
  float mu_i[LCC_MAX_WHEELS];
  float Cx_i[LCC_MAX_WHEELS], Cy_i[LCC_MAX_WHEELS];
  float vt_x[LCC_MAX_WHEELS], vt_y[LCC_MAX_WHEELS];
  float s_eff[LCC_MAX_WHEELS], a_eff[LCC_MAX_WHEELS]; /* filtered for Pacejka */

  float s_i[LCC_MAX_WHEELS], a_i[LCC_MAX_WHEELS];
  float r_i[LCC_MAX_WHEELS], Fz_i[LCC_MAX_WHEELS];

  for(int i = 0; i < car->wheel_count; ++i) {
    float vtire[2];
    lcc__wheel_vel_tire_frame(car, i, vtire);
    vt_x[i] = vtire[0];
    vt_y[i] = vtire[1];
    r_i[i]  = fmaxf(0.05f, car->desc.wheels[i].radius_m);
    Fz_i[i] = fmaxf(0.0f, car->wheel_states[i].normal_force_n);

    s_i[i] = lcc__slip_ratio(vt_x[i], car->wheel_states[i].omega_radps, r_i[i]);
    a_i[i] = lcc__slip_angle(vt_x[i], vt_y[i]);

    mu_i[i] = lcc__tire_mu(&car->desc.tires[i], Fz_i[i], car->env.global_friction_scale, car->damage_state.tire_health[i]);
    lcc__tire_stiffness(&car->desc.tires[i], Fz_i[i], car->desc.wheels[i].width_m, &Cx_i[i], &Cy_i[i]);

    /* relaxation dynamics: default to ~wheel radius if not set; clamp into realistic band */
    {
      float Vrel = lcc__hypot2(vt_x[i], vt_y[i]);
      float Lx   = car->desc.tires[i].relaxation_length_long_m;
      float Ly   = car->desc.tires[i].relaxation_length_lat_m;
      /* If unset/non-positive, use wheel radius; then clamp to typical 0.12–0.45 m band */
      if(!(Lx > 0.0f)) Lx = r_i[i];
      if(!(Ly > 0.0f)) Ly = r_i[i];
      Lx                      = lcc__clampf(Lx, 0.12f, 0.45f);
      Ly                      = lcc__clampf(Ly, 0.12f, 0.45f);
      car->slip_kappa_filt[i] = lcc__relax_filt(car->slip_kappa_filt[i], s_i[i], Vrel, Lx, dt_s);
      car->slip_alpha_filt[i] = lcc__relax_filt(car->slip_alpha_filt[i], a_i[i], Vrel, Ly, dt_s);
      s_eff[i]                = car->slip_kappa_filt[i];
      a_eff[i]                = car->slip_alpha_filt[i];
    }
  }

  /* find indices per axle */
  int fiL = -1, fiR = -1, riL = -1, riR = -1;
  for(int i = 0; i < car->wheel_count; ++i) {
    if(car->desc.wheels[i].position_local[0] >= 0.0f) {
      if(fiL < 0 && car->desc.wheels[i].position_local[1] < 0.0f) fiL = i;
      if(fiR < 0 && car->desc.wheels[i].position_local[1] > 0.0f) fiR = i;
    } else {
      if(riL < 0 && car->desc.wheels[i].position_local[1] < 0.0f) riL = i;
      if(riR < 0 && car->desc.wheels[i].position_local[1] > 0.0f) riR = i;
    }
  }
  /* per-wheel longitudinal capacity from friction circle */
  float Tlim[LCC_MAX_WHEELS] = { 0 };

  for(int i = 0; i < car->wheel_count; ++i) {
    float D      = fmaxf(0.0f, mu_i[i]) * fmaxf(0.0f, Fz_i[i]);               /* friction circle radius (N) */
    float Fy_est = lcc__pacejka_Fy_pure(a_eff[i], mu_i[i], Fz_i[i], Cy_i[i]); /* lateral demand from slip angle only */
    Fy_est       = lcc__clampf(Fy_est, -D, D);
    float Fx_cap = sqrtf(fmaxf(0.0f, D * D - Fy_est * Fy_est)); /* remaining room for Fx (N) */

    Tlim[i] = Fx_cap * r_i[i]; /* convert to Nm (cap at contact patch) */
  }
  /* torque split */
  lcc_axle_torques_t axleF = (lcc_axle_torques_t){ 0 }, axleR = (lcc_axle_torques_t){ 0 };
  if(front_has && fiL >= 0 && fiR >= 0) {
    lcc_diff_desc_t d = car->desc.driveline.front_diff;
    switch(d.type) {
    case LCC_DIFF_OPEN: axleF = lcc__diff_open(Tw_front, Tlim[fiL], Tlim[fiR]); break;
    case LCC_DIFF_LOCKED: axleF = lcc__diff_locked(Tw_front, Tlim[fiL], Tlim[fiR]); break;
    case LCC_DIFF_TORSEN: axleF = lcc__diff_torsen(Tw_front, Tlim[fiL], Tlim[fiR], d.bias_ratio, d.preload_nm); break;
    case LCC_DIFF_LSD_CLUTCH: axleF = lcc__diff_clutch_lsd(Tw_front, Tlim[fiL], Tlim[fiR], d.bias_ratio, d.preload_nm); break;
    case LCC_DIFF_ACTIVE:
    default: axleF = lcc__diff_active(Tw_front, Tlim[fiL], Tlim[fiR], d.lock_coef, d.preload_nm); break;
    }
  }
  if(rear_has && riL >= 0 && riR >= 0) {
    lcc_diff_desc_t d = car->desc.driveline.rear_diff;
    switch(d.type) {
    case LCC_DIFF_OPEN: axleR = lcc__diff_open(Tw_rear, Tlim[riL], Tlim[riR]); break;
    case LCC_DIFF_LOCKED: axleR = lcc__diff_locked(Tw_rear, Tlim[riL], Tlim[riR]); break;
    case LCC_DIFF_TORSEN: axleR = lcc__diff_torsen(Tw_rear, Tlim[riL], Tlim[riR], d.bias_ratio, d.preload_nm); break;
    case LCC_DIFF_LSD_CLUTCH: axleR = lcc__diff_clutch_lsd(Tw_rear, Tlim[riL], Tlim[riR], d.bias_ratio, d.preload_nm); break;
    case LCC_DIFF_ACTIVE:
    default: axleR = lcc__diff_active(Tw_rear, Tlim[riL], Tlim[riR], d.lock_coef, d.preload_nm); break;
    }
  }
  float Tw_cmd[LCC_MAX_WHEELS] = { 0 };
  if(fiL >= 0) Tw_cmd[fiL] = axleF.left_nm;
  if(fiR >= 0) Tw_cmd[fiR] = axleF.right_nm;
  if(riL >= 0) Tw_cmd[riL] = axleR.left_nm;
  if(riR >= 0) Tw_cmd[riR] = axleR.right_nm;

  /* ESC yaw update (computes extra brake torque on outside) */
  lcc__esc_update(car);

  /* integrate wheels and accumulate body forces/moment */
  float F_body_sum[2] = { 0.0f, 0.0f };
  float Mz            = 0.0f;
  float max_pos_slip  = 0.0f;

  for(int i = 0; i < car->wheel_count; ++i) {
    const lcc_wheel_desc_t *wd = &car->desc.wheels[i];
    const lcc_tire_desc_t  *td = &car->desc.tires[i];
    lcc_wheel_state_t      *ws = &car->wheel_states[i];

    float Vx = vt_x[i], Vy = vt_y[i];
    float r       = r_i[i];
    float Fz      = Fz_i[i];
    float mu_base = mu_i[i];
    float Kx = Cx_i[i], Ky = Cy_i[i];

    float s = s_eff[i];
    float a = a_eff[i];
    if(wd->driven && s > max_pos_slip) max_pos_slip = s;

    /* Pacejka-only: compute pure forces */
    float mu_drop = 1.0f;
    float s_abs = fabsf(s), a_abs = fabsf(a);
    if(s_abs > LCC_PACEJKA_KAPPA_SLIDE || a_abs > LCC_PACEJKA_ALPHA_SLIDE) mu_drop = LCC_PACEJKA_MU_KINETIC;
    float mu_eff = mu_base * mu_drop;

    float Fx_pure = lcc__pacejka_Fx_pure(s, mu_eff, Fz, Kx);
    float Fy_pure = lcc__pacejka_Fy_pure(a, mu_eff, Fz, Ky);

    /* no additional combined-slip scaling (simple for now) */
    float Fx0 = Fx_pure, Fy0 = Fy_pure;

    /* rolling resistance (smooth sign) */
    float sgn_v = Vx / (fabsf(Vx) + 0.05f);
    float Frr   = -td->rolling_resistance * Fz * sgn_v;
    Fx0 += Frr;

    /* low-speed vector friction fallback for static behavior */
    if(car->car_state.speed_mps < 0.25f) {
      float Vx_slip = Vx - ws->omega_radps * r;
      float Vy_slip = Vy;
      float vmag    = sqrtf(Vx_slip * Vx_slip + Vy_slip * Vy_slip);
      if(vmag > 1e-6f) {
        float Fcap = mu_base * Fz;
        float nx = Vx_slip / vmag, ny = Vy_slip / vmag;
        float Fx_ls = -Fcap * nx;
        float Fy_ls = -Fcap * ny;
        float b     = lcc__saturate((0.25f - car->car_state.speed_mps) / 0.25f);
        Fx0         = lcc__lerp(Fx0, Fx_ls, b);
        Fy0         = lcc__lerp(Fy0, Fy_ls, b);
      }
    }

    /* brake torque + ESC */
    float Tbr_base = lcc__wheel_brake_torque_base(car, i);
    float Tbr      = Tbr_base + car->esc_extra_brake[i];
    Tbr            = lcc__abs_apply(car, i, Tbr, s, dt_s);

    float Tbr_sign = lcc__signf(ws->omega_radps);

    /* contact patch torque from longitudinal */
    float T_contact = Fx0 * r;

    /* drive torque if driven */
    float Tdrive = car->desc.wheels[i].driven ? Tw_cmd[i] : 0.0f;

    /* integrate wheel  (guard against crossing rolling speed in one step) */
    {
      float Iw   = fmaxf(0.01f, wd->inertia_kgm2);
      float netT = Tdrive - Tbr_sign * Tbr - T_contact;

      int   sub = (car->controls.brake > 0.2f || fabsf(s) > 0.8f || fabsf(netT) > 300.0f) ? 4 : 1;
      float dtw = dt_s / (float)sub;
      for(int k = 0; k < sub; ++k) {
        float w      = ws->omega_radps;
        float w_tgt  = (fabsf(r) > 1e-4f) ? (Vx / r) : 0.0f;
        float w_next = w + (netT / Iw) * dtw;
        if((w - w_tgt) * (w_next - w_tgt) < 0.0f) w_next = w_tgt; /* don't cross in a substep */
        ws->omega_radps = w_next;
      }
    }

    /* rotate tire force to body */
    float steer = car->wheel_steer_rad[i];
    float Fb[2];
    float Ftire[2] = { Fx0, Fy0 };
    lcc__v2_rot(Fb, Ftire, steer);

    F_body_sum[0] += Fb[0];
    F_body_sum[1] += Fb[1];

    float rx = car->desc.wheels[i].position_local[0];
    float ry = car->desc.wheels[i].position_local[1];
    Mz += rx * Fb[1] - ry * Fb[0];

    /* telemetry */
    {
      float denom_te = fmaxf(0.3f, fmaxf(fabsf(Vx), fabsf(ws->omega_radps * r)));
      float s_te     = (ws->omega_radps * r - Vx) / denom_te;

      ws->slip_ratio        = lcc__clampf(s_te, -3.0f, 3.0f);
      ws->slip_angle_rad    = a;
      ws->tire_force_long_n = Fx0;
      ws->tire_force_lat_n  = Fy0;
      ws->brake_torque_nm   = Tbr_sign * Tbr;
      ws->drive_torque_nm   = Tdrive;
    }
  }

  /* ABS event */
  int any_abs = 0;
  for(int i = 0; i < car->wheel_count; ++i)
    if(car->abs_mod[i] < 0.999f) {
      any_abs = 1;
      break;
    }
  if(car->desc.ecu.abs_mode == LCC_ABS_ON && any_abs && (car->car_state.time_s - car->last_abs_evt_time) > 0.2) {
    car->last_abs_evt_time = car->car_state.time_s;
    lcc__emit_event(car, LCC_EVENT_ABS_ACTIVE, 0, 0.0f);
  }

  /* TC update and event */
  lcc__tc_update(car, max_pos_slip, dt_s);
  if(car->desc.ecu.tc_mode == LCC_TC_ON && car->tc_cut > 0.01f && (car->car_state.time_s - car->last_tc_evt_time) > 0.2) {
    car->last_tc_evt_time = car->car_state.time_s;
    lcc__emit_event(car, LCC_EVENT_TC_ACTIVE, 0, car->tc_cut);
  }

  /* aero -> body */
  float drag_body[2];
  lcc__v2_rot(drag_body, drag_world, -car->car_state.yaw_rad);
  F_body_sum[0] += drag_body[0];
  F_body_sum[1] += drag_body[1];

  /* integrate rigid body */
  float m   = fmaxf(1.0f, car->car_state.mass_kg);
  float Izz = fmaxf(1.0f, car->desc.chassis.inertia_zz);

  /* yaw dynamics from Mz */
  float yaw_acc = Mz / Izz;
  car->car_state.yaw_rate_radps += yaw_acc * dt_s;
  car->car_state.yaw_rad += car->car_state.yaw_rate_radps * dt_s;

  /* linear dynamics in world frame */
  float F_world[2];
  lcc__v2_rot(F_world, F_body_sum, car->car_state.yaw_rad); /* body -> world */

  car->car_state.acc_world[0] = F_world[0] / m;
  car->car_state.acc_world[1] = F_world[1] / m;

  car->car_state.vel_world[0] += car->car_state.acc_world[0] * dt_s;
  car->car_state.vel_world[1] += car->car_state.acc_world[1] * dt_s;

  car->car_state.pos_world[0] += car->car_state.vel_world[0] * dt_s;
  car->car_state.pos_world[1] += car->car_state.vel_world[1] * dt_s;

  /* derive body-frame kinematics for tires/sloads */
  lcc__v2_rot(car->car_state.vel_body, car->car_state.vel_world, -car->car_state.yaw_rad);
  lcc__v2_rot(car->car_state.acc_body, car->car_state.acc_world, -car->car_state.yaw_rad);

  car->car_state.speed_mps = lcc__v2_len(car->car_state.vel_world);

  /* engine speed dynamics */
  float eng_omega    = lcc__rpm_to_radps(car->engine_state.rpm);
  float load_omega   = lcc__rpm_to_radps(gear_ratio * shaft_out_rpm);
  float Ieng         = fmaxf(0.02f, car->desc.engine.inertia_kgm2);
  float T_net        = T_starter + tq_engine_comb - T_to_trans - T_fric_drag;
  float domega_free  = T_net / Ieng;
  float domega_track = (load_omega - eng_omega) * clutch_k;

  eng_omega += (domega_free * (1.0f - clutch_k) + domega_track) * dt_s;
  car->engine_state.rpm = fmaxf(0.0f, lcc__radps_to_rpm(eng_omega));

  /* catch during cranking */
  if(!car->engine_state.running && car->engine_state.cranking && has_fuel && car->controls.ignition_switch) {
    float catch_rpm = ed->idle_rpm * LCC_ENGINE_CATCH_RPM_FACTOR;
    if(car->engine_crank_timer_s > LCC_ENGINE_CRANK_MIN_TIME_S && car->engine_state.rpm > catch_rpm) {
      car->engine_state.running  = 1;
      car->engine_state.cranking = 0;
      car->engine_crank_timer_s  = 0.0f;
      lcc__emit_event(car, LCC_EVENT_ENGINE_START, 0, 0.0f);
    }
  }

  /* stall detection: more permissive in neutral vs loaded in gear */
  if(car->engine_state.running) {
    int   in_gear        = (car->trans_state.gear_index != LCC_GEAR_NEUTRAL) && (fabsf(gear_ratio) > 1e-3f);
    int   clutch_engaged = (car->trans_state.clutch_engagement > 0.7f);
    int   loaded         = (in_gear && clutch_engaged);
    float stall_thresh   = ed->stall_rpm * (loaded ? 1.0f : 0.6f);
    if(car->engine_state.rpm < stall_thresh) {
      car->engine_state.running  = 0;
      car->engine_state.cranking = 0;
      lcc__emit_event(car, LCC_EVENT_ENGINE_STALL, 0, 0.0f);
    }
  }

  /* shift timing */
  if(car->trans_state.shifting) {
    car->shift_timer_s -= dt_s;
    if(car->shift_timer_s <= 0.0f) {
      car->trans_state.gear_index = car->pending_gear_index;
      car->trans_state.shifting   = 0;
      lcc__emit_event(car, LCC_EVENT_GEAR_CHANGE, car->trans_state.gear_index, 0.0f);
    }
  }

  /* update patch world pos */
  for(int i = 0; i < car->wheel_count; ++i) {
    float local[2] = { car->desc.wheels[i].position_local[0], car->desc.wheels[i].position_local[1] };
    float world[2];
    lcc__v2_rot(world, local, car->car_state.yaw_rad);
    world[0] += car->car_state.pos_world[0];
    world[1] += car->car_state.pos_world[1];
  }

  /* electrics: solve bus voltage with battery (OCV+R) and alternator (regulated with current limit) */
  {
    /* total constant-power loads on the bus */
    float P_load_w = 150.0f; /* base consumers */
    if(car->elec_state.consumers_headlights) P_load_w += 110.0f;
    if(car->engine_state.cranking && !car->engine_state.running) {
      /* electrical power draw higher than mechanical starter power by efficiency */
      P_load_w += car->desc.starter.power_w / fmaxf(LCC_STARTER_EFF, 0.15f);
    }

    /* initial guess for bus voltage */
    float V_guess = car->elec_state.bus_voltage_v > 1.0f ? car->elec_state.bus_voltage_v : (car->engine_state.running ? car->desc.alternator.regulator_voltage_v : lcc__batt_ocv(&car->desc.battery, car->elec_state.battery_soc));

    float Tamb     = car->env.ambient_temp_c;
    float soc      = car->elec_state.battery_soc;
    float OCV      = lcc__batt_ocv(&car->desc.battery, soc);
    float Rint     = lcc__batt_r_internal(&car->desc.battery, soc, Tamb);
    float Vreg     = car->desc.alternator.regulator_voltage_v;
    float I_altcap = lcc__alt_current_cap(&car->desc.alternator, car->engine_state.rpm, car->engine_state.running);

    float V_bus  = V_guess;
    float I_load = 0.0f, I_alt = 0.0f, I_batt = 0.0f;

    for(int it = 0; it < 2; ++it) {
      V_bus  = fmaxf(V_bus, 6.0f);
      I_load = P_load_w / V_bus;

      if(I_altcap > 0.0f) {
        /* try regulated bus at Vreg */
        float I_batt_req = (OCV - Vreg) / Rint; /* + means battery discharging */
        float I_alt_req  = I_load - I_batt_req; /* alternator supplies remainder */
        if(I_alt_req <= I_altcap) {
          /* regulator holds */
          V_bus  = Vreg;
          I_alt  = fmaxf(0.0f, I_alt_req);
          I_batt = I_load - I_alt;
        } else {
          /* alternator saturated, bus falls to battery terminal voltage */
          I_alt  = I_altcap;
          I_batt = I_load - I_alt;
          V_bus  = OCV - I_batt * Rint;
        }
      } else {
        /* alternator offline */
        I_alt  = 0.0f;
        I_batt = I_load;
        V_bus  = OCV - I_batt * Rint;
      }
    }

    /* SOC integration with temp-adjusted capacity */
    float cap_as_eff = car->desc.battery.capacity_ah * 3600.0f * lcc__batt_cap_temp_scale(Tamb);
    if(cap_as_eff > 1.0f) {
      if(I_batt >= 0.0f) soc -= (I_batt * dt_s) / cap_as_eff;
      else
        soc += (-I_batt * dt_s) / cap_as_eff * lcc__clampf(car->desc.battery.charge_efficiency, 0.5f, 1.0f);
      car->elec_state.battery_soc = lcc__saturate(soc);
    }

    car->elec_state.bus_voltage_v  = lcc__clampf(V_bus, 6.0f, 15.5f);
    car->elec_state.alt_current_a  = I_alt;
    car->elec_state.batt_current_a = I_batt;
  }

  /* fuel consumption */
  float power_kw         = fmaxf(0.0f, tq_engine_comb) * eng_omega / 1000.0f;
  float bsfc_l_per_kwh   = 0.32f;
  float fuel_lps         = (power_kw * bsfc_l_per_kwh) / 3600.0f;
  float idle_lps         = (car->engine_state.running && car->controls.throttle < 0.02f) ? 0.00015f : 0.0f;
  float fuel_use         = (fuel_lps + idle_lps) * dt_s;
  car->fuel_state.fuel_l = fmaxf(0.0f, car->fuel_state.fuel_l - fuel_use);

  /* thermal updates */
  lcc__thermal_update(car, power_kw, dt_s);

  /* ESC event */
  int any_esc = 0;
  for(int i = 0; i < car->wheel_count; ++i)
    if(car->esc_extra_brake[i] > 1.0f) {
      any_esc = 1;
      break;
    }
  if(car->desc.ecu.esc_mode == LCC_ESC_ON && any_esc && (car->car_state.time_s - car->last_esc_evt_time) > 0.3) {
    car->last_esc_evt_time = car->car_state.time_s;
    lcc__emit_event(car, LCC_EVENT_ESC_ACTIVE, 0, 0.0f);
  }

  return LCC_OK;
}

/*}}}*/

/* ============================== api implementation ============================== {{{*/

/* allocators */
void lcc_set_allocators(lcc_alloc_fn alloc_fn, lcc_free_fn free_fn, void *user) {
  lcc__alloc      = alloc_fn ? alloc_fn : lcc__malloc;
  lcc__free       = free_fn ? free_fn : lcc__freefn;
  lcc__alloc_user = user;
}

/* version */
const char *lcc_version_string(void) {
  return LCC_VERSION;
}

/* defaults */
void lcc_engine_desc_init_defaults(lcc_engine_desc_t *desc) {
  if(!desc) return;
  lcc__pzero(desc);
  desc->fuel                          = LCC_FUEL_GASOLINE;
  desc->forced_induction              = LCC_FI_NONE;
  desc->idle_rpm                      = 800.0f;
  desc->redline_rpm                   = 6500.0f;
  desc->stall_rpm                     = 400.0f;
  desc->inertia_kgm2                  = 0.2f;
  desc->wastegate_pressure_kpa        = 110.0f;
  desc->coolant_heat_capacity_j_per_k = 60000.0f;
  desc->oil_heat_capacity_j_per_k     = 40000.0f;

  /* sane default maps */
  desc->wot_torque_nm_vs_rpm.points      = LCC__DEF_ENGINE_WOT_POINTS;
  desc->wot_torque_nm_vs_rpm.count       = (int)(sizeof(LCC__DEF_ENGINE_WOT_POINTS) / sizeof(LCC__DEF_ENGINE_WOT_POINTS[0]));
  desc->friction_torque_nm_vs_rpm.points = LCC__DEF_ENGINE_FRICTION_POINTS;
  desc->friction_torque_nm_vs_rpm.count  = (int)(sizeof(LCC__DEF_ENGINE_FRICTION_POINTS) / sizeof(LCC__DEF_ENGINE_FRICTION_POINTS[0]));
  desc->throttle_map.points              = LCC__DEF_THROTTLE_MAP_POINTS;
  desc->throttle_map.count               = (int)(sizeof(LCC__DEF_THROTTLE_MAP_POINTS) / sizeof(LCC__DEF_THROTTLE_MAP_POINTS[0]));
}

void lcc_fuel_desc_init_defaults(lcc_fuel_desc_t *desc) {
  if(!desc) return;
  lcc__pzero(desc);
  desc->tank_capacity_l       = 50.0f;
  desc->fuel_density_kg_per_l = 0.745f;
  desc->initial_fuel_l        = 20.0f;
}

void lcc_cooling_desc_init_defaults(lcc_cooling_desc_t *desc) {
  if(!desc) return;
  lcc__pzero(desc);
  desc->radiator_ua_w_per_k = 500.0f;
  desc->fan_on_c            = 100.0f;
}

void lcc_battery_desc_init_defaults(lcc_battery_desc_t *desc) {
  if(!desc) return;
  lcc__pzero(desc);
  desc->capacity_ah             = 60.0f;
  desc->nominal_voltage_v       = 12.6f;
  desc->initial_soc             = 0.9f;
  desc->internal_resistance_ohm = LCC_BATT_R_INTERNAL_OHM;
  desc->ocv_full_v              = LCC_BATT_OCV_FULL_V;
  desc->ocv_empty_v             = LCC_BATT_OCV_EMPTY_V;
  desc->charge_efficiency       = LCC_BATT_CHARGE_EFF;
}

void lcc_alternator_desc_init_defaults(lcc_alternator_desc_t *desc) {
  if(!desc) return;
  lcc__pzero(desc);
  desc->max_current_a       = 90.0f;
  desc->cut_in_rpm          = 1200.0f;
  desc->regulator_voltage_v = LCC_ALT_REG_VOLTAGE_V;
}

void lcc_starter_desc_init_defaults(lcc_starter_desc_t *desc) {
  if(!desc) return;
  lcc__pzero(desc);
  desc->power_w = 1000.0f;
}

void lcc_ecu_desc_init_defaults(lcc_ecu_desc_t *desc) {
  if(!desc) return;
  lcc__pzero(desc);

  /* TODO: one of these is fucked up */
  desc->abs_mode = LCC_ABS_ON;
  desc->tc_mode  = LCC_TC_ON;
  desc->esc_mode = LCC_ESC_ON;

  /* TODO: this only in automatic transmission? could be completely removed */
  desc->auto_clutch = 0;

  desc->idle_control = 1;
  desc->idle_pid_p   = 0.5f;
  desc->idle_pid_i   = 0.1f;
}

void lcc_transmission_desc_init_defaults(lcc_transmission_desc_t *desc) {
  if(!desc) return;
  lcc__pzero(desc);
  desc->type = LCC_TRANS_MANUAL;
  /* gear_ratios: [0]=reverse, [1]=neutral */
  desc->gear_count = 8;
  float gr[8]      = {
    -3.2f,
    0.0f,
    3.1f,
    2.1f,
    1.5f,
    1.2f,
    1.0f,
    0.84f,
  };
  for(int i = 0; i < desc->gear_count; ++i) desc->gear_ratios[i] = gr[i];
  desc->final_drive_ratio  = 3.9f;
  desc->shift_time_s       = 0.30f;
  desc->auto_upshift_rpm   = 6500.0f;
  desc->auto_downshift_rpm = 1200.0f;
}

void lcc_driveline_desc_init_defaults(lcc_driveline_desc_t *desc) {
  if(!desc) return;
  lcc__pzero(desc);
  desc->layout                = LCC_LAYOUT_RWD;
  desc->front_diff.type       = LCC_DIFF_LOCKED;
  desc->rear_diff.type        = LCC_DIFF_LOCKED;
  desc->front_diff.preload_nm = 20.0f;
  desc->rear_diff.preload_nm  = 20.0f;
  desc->front_diff.bias_ratio = 2.5f;
  desc->rear_diff.bias_ratio  = 2.5f;
  desc->front_torque_split    = 0.9f;
}

void lcc_chassis_desc_init_defaults(lcc_chassis_desc_t *desc) {
  if(!desc) return;
  lcc__pzero(desc);
  desc->mass_kg       = 1200.0f;
  desc->inertia_zz    = 1200.0f;
  desc->cg_local_x    = 0.0f;
  desc->wheelbase_m   = 2.6f;
  desc->track_front_m = 1.55f;
  desc->track_rear_m  = 1.55f;
  desc->cg_height_m   = 0.50f;
  desc->width_m       = 1.8f;
  desc->length_m      = 4.2f;
}

void lcc_aero_desc_init_defaults(lcc_aero_desc_t *desc) {
  if(!desc) return;
  lcc__pzero(desc);
  desc->drag_coefficient       = 0.31f;
  desc->frontal_area_m2        = 2.2f;
  desc->lift_coefficient_front = -0.05f;
  desc->lift_coefficient_rear  = -0.10f;
  desc->yaw_drag_gain          = 0.05f;
}

void lcc_wheel_desc_init_defaults(lcc_wheel_desc_t *desc) {
  if(!desc) return;
  lcc__pzero(desc);
  desc->radius_m     = 0.31f;
  desc->width_m      = 0.22f;
  desc->inertia_kgm2 = 1.2f;
  desc->steerable    = 0;
  desc->driven       = 1;
  desc->has_brake    = 1;
}

void lcc_tire_desc_init_defaults(lcc_tire_desc_t *desc) {
  if(!desc) return;
  lcc__pzero(desc);
  desc->mu_nominal               = 1.0f;
  desc->load_sensitivity         = -0.0002f;
  desc->rolling_resistance       = 0.015f;
  desc->pressure_kpa             = 220.0f;
  desc->ideal_pressure_kpa       = 240.0f;
  desc->relaxation_length_long_m = 0.3f;
  desc->relaxation_length_lat_m  = 0.2f;
  desc->wear_rate                = 1.0f;
}

void lcc_brake_desc_init_defaults(lcc_brake_desc_t *desc) {
  if(!desc) return;
  lcc__pzero(desc);
  desc->max_torque_nm   = 3000.0f;
  desc->disc_radius_m   = 0.15f;
  desc->pad_mu          = 0.4f;
  desc->cooling_area_m2 = 0.05f;
}

void lcc_arb_desc_init_defaults(lcc_arb_desc_t *desc) {
  if(!desc) return;
  lcc__pzero(desc);
  desc->front_rate_n_per_rad = 2000.0f;
  desc->rear_rate_n_per_rad  = 1800.0f;
}

void lcc_steering_desc_init_defaults(lcc_steering_desc_t *desc) {
  if(!desc) return;
  lcc__pzero(desc);
  desc->max_steer_deg    = 35.0f;
  desc->ackermann_factor = 0.9f;
}

void lcc_environment_init_defaults(lcc_environment_t *env) {
  if(!env) return;
  lcc__pzero(env);
  env->ambient_temp_c        = 20.0f;
  env->air_density           = 1.225f;
  env->wind_world[0]         = 0.0f;
  env->wind_world[1]         = 0.0f;
  env->surface_temp_c        = 20.0f;
  env->global_friction_scale = 1.0f;
}

void lcc_car_desc_init_defaults(lcc_car_desc_t *desc) {
  if(!desc) return;
  lcc__pzero(desc);

  lcc_chassis_desc_init_defaults(&desc->chassis);
  lcc_aero_desc_init_defaults(&desc->aero);

  lcc_engine_desc_init_defaults(&desc->engine);
  lcc_fuel_desc_init_defaults(&desc->fuel);
  lcc_cooling_desc_init_defaults(&desc->cooling);

  lcc_battery_desc_init_defaults(&desc->battery);
  lcc_alternator_desc_init_defaults(&desc->alternator);
  lcc_starter_desc_init_defaults(&desc->starter);
  lcc_ecu_desc_init_defaults(&desc->ecu);

  lcc_transmission_desc_init_defaults(&desc->transmission);
  lcc_driveline_desc_init_defaults(&desc->driveline);

  desc->wheel_count = 4;

  float half_wb = desc->chassis.wheelbase_m * 0.5f;
  float hf      = desc->chassis.track_front_m * 0.5f;
  float hr      = desc->chassis.track_rear_m * 0.5f;

  /* FL;FR;RL;RR */
  lcc_wheel_desc_init_defaults(&desc->wheels[0]);
  desc->wheels[0].steerable         = 1;
  desc->wheels[0].driven            = (desc->driveline.layout != LCC_LAYOUT_RWD);
  desc->wheels[0].position_local[0] = half_wb;
  desc->wheels[0].position_local[1] = -hf;
  lcc_wheel_desc_init_defaults(&desc->wheels[1]);
  desc->wheels[1].steerable         = 1;
  desc->wheels[1].driven            = (desc->driveline.layout != LCC_LAYOUT_RWD);
  desc->wheels[1].position_local[0] = half_wb;
  desc->wheels[1].position_local[1] = hf;
  lcc_wheel_desc_init_defaults(&desc->wheels[2]);
  desc->wheels[2].steerable         = 0;
  desc->wheels[2].driven            = (desc->driveline.layout != LCC_LAYOUT_FWD);
  desc->wheels[2].position_local[0] = -half_wb;
  desc->wheels[2].position_local[1] = -hr;
  lcc_wheel_desc_init_defaults(&desc->wheels[3]);
  desc->wheels[3].steerable         = 0;
  desc->wheels[3].driven            = (desc->driveline.layout != LCC_LAYOUT_FWD);
  desc->wheels[3].position_local[0] = -half_wb;
  desc->wheels[3].position_local[1] = hr;

  for(int i = 0; i < desc->wheel_count; ++i) {
    lcc_tire_desc_init_defaults(&desc->tires[i]);
    lcc_brake_desc_init_defaults(&desc->brakes[i]);
  }

  lcc_arb_desc_init_defaults(&desc->arbs);
  lcc_steering_desc_init_defaults(&desc->steering);
  lcc_environment_init_defaults(&desc->environment);
}

/* main step */
lcc_car_t *lcc_car_create(const lcc_car_desc_t *desc) {
  if(!lcc__alloc) {
    lcc__alloc      = lcc__malloc;
    lcc__free       = lcc__freefn;
    lcc__alloc_user = NULL;
  }
  if(!desc) return NULL;

  lcc_car_t *car = (lcc_car_t *)lcc__alloc(sizeof(lcc_car_t), lcc__alloc_user);
  if(!car) return NULL;
  lcc__pzero(car);

  car->desc        = *desc;
  car->wheel_count = car->desc.wheel_count;
  car->env         = desc->environment;

  lcc__init_runtime(car);
  return car;
}

void lcc_car_destroy(lcc_car_t *car) {
  if(!car) return;
  lcc__free_owned_engine_maps(car);
  lcc__free(car, lcc__alloc_user);
}

lcc_result_t lcc_car_reset(lcc_car_t *car, const lcc_car_state_t *optional_state) {
  if(!car) return LCC_ERR_INVALID_ARG;
  lcc__init_runtime(car);
  if(optional_state) car->car_state = *optional_state;
  lcc__compute_static_loads(car);
  return LCC_OK;
}

/* configuration */
void lcc_car_set_environment(lcc_car_t *car, const lcc_environment_t *env) {
  if(!car || !env) return;
  car->env = *env;
}

void lcc_car_get_environment(const lcc_car_t *car, lcc_environment_t *env_out) {
  if(!car || !env_out) return;
  *env_out = car->env;
}

lcc_result_t lcc_car_set_engine_map(lcc_car_t *car, const lcc_curve1d_t *wot_torque, const lcc_curve1d_t *friction) {
  if(!car) return LCC_ERR_INVALID_ARG;
  /* if we previously generated maps, free them before overriding */
  if(wot_torque) {
    if(car->owned_wot_pts) {
      lcc__free(car->owned_wot_pts, lcc__alloc_user);
      car->owned_wot_pts   = NULL;
      car->owned_wot_count = 0;
    }
  }
  if(friction) {
    if(car->owned_fric_pts) {
      lcc__free(car->owned_fric_pts, lcc__alloc_user);
      car->owned_fric_pts   = NULL;
      car->owned_fric_count = 0;
    }
  }

  if(wot_torque) car->desc.engine.wot_torque_nm_vs_rpm = *wot_torque;
  if(friction) car->desc.engine.friction_torque_nm_vs_rpm = *friction;
  return LCC_OK;
}

lcc_result_t lcc_car_set_boost_map(lcc_car_t *car, const lcc_map2d_t *boost) {
  if(!car || !boost) return LCC_ERR_INVALID_ARG;
  /* free previously generated boost map if any */
  if(car->owned_boost_pts) {
    lcc__free(car->owned_boost_pts, lcc__alloc_user);
    car->owned_boost_pts   = NULL;
    car->owned_boost_count = 0;
  }

  car->desc.engine.boost_pressure_kpa_vs_rpm_throttle = *boost;
  return LCC_OK;
}

lcc_result_t lcc_car_set_gear_ratios(lcc_car_t *car, const float *gear_ratios, int gear_count, float final_drive) {
  if(!car || !gear_ratios || gear_count <= 0 || gear_count > LCC_MAX_GEARS) return LCC_ERR_INVALID_ARG;
  for(int i = 0; i < gear_count; ++i) car->desc.transmission.gear_ratios[i] = gear_ratios[i];
  car->desc.transmission.gear_count        = gear_count;
  car->desc.transmission.final_drive_ratio = final_drive;
  return LCC_OK;
}

lcc_result_t lcc_car_set_diff_params(lcc_car_t *car, lcc_diff_type_t front, lcc_diff_type_t rear, float preload_nm, float bias_ratio) {
  if(!car) return LCC_ERR_INVALID_ARG;
  car->desc.driveline.front_diff.type       = front;
  car->desc.driveline.rear_diff.type        = rear;
  car->desc.driveline.front_diff.preload_nm = preload_nm;
  car->desc.driveline.rear_diff.preload_nm  = preload_nm;
  car->desc.driveline.front_diff.bias_ratio = bias_ratio;
  car->desc.driveline.rear_diff.bias_ratio  = bias_ratio;
  return LCC_OK;
}

lcc_result_t lcc_car_set_tire_params(lcc_car_t *car, int wheel_index, const lcc_tire_desc_t *tire) {
  if(!car || !tire) return LCC_ERR_INVALID_ARG;
  if(wheel_index < 0 || wheel_index >= car->wheel_count) return LCC_ERR_BOUNDS;
  car->desc.tires[wheel_index] = *tire;
  return LCC_OK;
}

lcc_result_t lcc_car_set_brake_params(lcc_car_t *car, int wheel_index, const lcc_brake_desc_t *brake) {
  if(!car || !brake) return LCC_ERR_INVALID_ARG;
  if(wheel_index < 0 || wheel_index >= car->wheel_count) return LCC_ERR_BOUNDS;
  car->desc.brakes[wheel_index] = *brake;
  return LCC_OK;
}

lcc_result_t lcc_car_set_arb_params(lcc_car_t *car, const lcc_arb_desc_t *arb) {
  if(!car || !arb) return LCC_ERR_INVALID_ARG;
  car->desc.arbs = *arb;
  return LCC_OK;
}

lcc_result_t lcc_car_set_steering_params(lcc_car_t *car, const lcc_steering_desc_t *steer) {
  if(!car || !steer) return LCC_ERR_INVALID_ARG;
  car->desc.steering = *steer;
  return LCC_OK;
}

lcc_result_t lcc_car_request_gear(lcc_car_t *car, int gear_index) {
  if(!car) return LCC_ERR_INVALID_ARG;
  if(gear_index < 0 || gear_index >= car->desc.transmission.gear_count) return LCC_ERR_BOUNDS;
  car->pending_gear_index   = gear_index;
  car->trans_state.shifting = 1;
  car->shift_timer_s        = lcc__clampf(car->desc.transmission.shift_time_s, 0.05f, 1.0f);
  return LCC_OK;
}

lcc_result_t lcc_car_shift_up(lcc_car_t *car) {
  if(!car) return LCC_ERR_INVALID_ARG;
  int next = car->trans_state.gear_index + 1;
  if(next >= car->desc.transmission.gear_count) next = car->desc.transmission.gear_count - 1;
  return lcc_car_request_gear(car, next);
}

lcc_result_t lcc_car_shift_down(lcc_car_t *car) {
  if(!car) return LCC_ERR_INVALID_ARG;
  int next     = car->trans_state.gear_index - 1;
  int min_gear = car->desc.transmission.type != LCC_TRANS_MANUAL ? 2 : 0;
  if(next < min_gear) next = min_gear;
  return lcc_car_request_gear(car, next);
}

/* fuel and energy */
lcc_result_t lcc_car_refuel(lcc_car_t *car, float liters) {
  if(!car) return LCC_ERR_INVALID_ARG;
  if(liters < 0.0f) return LCC_ERR_INVALID_ARG;
  float new_l            = car->fuel_state.fuel_l + liters;
  float cap              = car->desc.fuel.tank_capacity_l;
  car->fuel_state.fuel_l = new_l > cap ? cap : new_l;
  float fuel_mass        = car->fuel_state.fuel_l * car->desc.fuel.fuel_density_kg_per_l;
  car->car_state.mass_kg = car->desc.chassis.mass_kg + fuel_mass;
  lcc__compute_static_loads(car);
  return LCC_OK;
}

lcc_result_t lcc_car_set_fuel(lcc_car_t *car, float liters) {
  if(!car) return LCC_ERR_INVALID_ARG;
  float cap              = car->desc.fuel.tank_capacity_l;
  liters                 = lcc__clampf(liters, 0.0f, cap);
  car->fuel_state.fuel_l = liters;
  float fuel_mass        = car->fuel_state.fuel_l * car->desc.fuel.fuel_density_kg_per_l;
  car->car_state.mass_kg = car->desc.chassis.mass_kg + fuel_mass;
  lcc__compute_static_loads(car);
  return LCC_OK;
}

lcc_result_t lcc_car_recharge_battery(lcc_car_t *car, float state_of_charge_0_to_1) {
  if(!car) return LCC_ERR_INVALID_ARG;
  car->elec_state.battery_soc = lcc__saturate(state_of_charge_0_to_1);
  return LCC_OK;
}

/* aids toggles */
void lcc_car_set_abs(lcc_car_t *car, lcc_abs_mode_t mode) {
  if(!car) return;
  car->desc.ecu.abs_mode = mode;
}

void lcc_car_set_tc(lcc_car_t *car, lcc_tc_mode_t mode) {
  if(!car) return;
  car->desc.ecu.tc_mode = mode;
}

void lcc_car_set_esc(lcc_car_t *car, lcc_esc_mode_t mode) {
  if(!car) return;
  car->desc.ecu.esc_mode = mode;
}

/* pos and velocity */
void lcc_car_set_pos(lcc_car_t *car, const float pos_world[2], float yaw_rad) {
  if(!car || !pos_world) return;
  car->car_state.pos_world[0] = pos_world[0];
  car->car_state.pos_world[1] = pos_world[1];
  car->car_state.yaw_rad      = yaw_rad;
}

void lcc_car_get_pos(const lcc_car_t *car, float pos_world_out[2], float *yaw_rad_out) {
  if(!car) return;
  if(pos_world_out) {
    pos_world_out[0] = car->car_state.pos_world[0];
    pos_world_out[1] = car->car_state.pos_world[1];
  }
  if(yaw_rad_out) *yaw_rad_out = car->car_state.yaw_rad;
}

void lcc_car_set_velocity(lcc_car_t *car, const float vel_world[2], float yaw_rate_radps) {
  if(!car || !vel_world) return;
  car->car_state.vel_world[0]   = vel_world[0];
  car->car_state.vel_world[1]   = vel_world[1];
  car->car_state.yaw_rate_radps = yaw_rate_radps;
}

void lcc_car_get_velocity(const lcc_car_t *car, float vel_world_out[2], float *yaw_rate_radps_out) {
  if(!car) return;
  if(vel_world_out) {
    vel_world_out[0] = car->car_state.vel_world[0];
    vel_world_out[1] = car->car_state.vel_world[1];
  }
  if(yaw_rate_radps_out) *yaw_rate_radps_out = car->car_state.yaw_rate_radps;
}

/* controls */
void lcc_car_set_controls(lcc_car_t *car, const lcc_controls_t *controls) {
  if(!car || !controls) return;
  car->controls           = *controls;
  car->controls.throttle  = lcc__saturate(car->controls.throttle);
  car->controls.brake     = lcc__saturate(car->controls.brake);
  car->controls.clutch    = lcc__saturate(car->controls.clutch);
  car->controls.steer     = lcc__clampf(-car->controls.steer, -1.0f, 1.0f);
  car->controls.handbrake = lcc__saturate(car->controls.handbrake);
}

void lcc_car_get_controls(const lcc_car_t *car, lcc_controls_t *controls_out) {
  if(!car || !controls_out) return;
  *controls_out = car->controls;
}

lcc_result_t lcc_car_step(lcc_car_t *car, float dt_s) {
  return lcc__car_step(car, dt_s);
}

/* units */
float lcc_deg_to_rad(float deg) {
  return lcc__deg2rad(deg);
}

float lcc_rad_to_deg(float rad) {
  return lcc__rad2deg(rad);
}

/* event subscription */
void lcc_car_set_event_callback(lcc_car_t *car, lcc_event_cb callback, void *user) {
  if(!car) return;
  car->evt_cb   = callback;
  car->evt_user = user;
}

/* ============================== engine map generation ============================== {{{*/
void lcc_engine_simple_spec_init_defaults(lcc_engine_simple_spec_t *spec) {
  if(!spec) return;
  lcc__pzero(spec);
  spec->rated_power_kw   = 150.0f;
  spec->rated_power_rpm  = 6000.0f;
  spec->redline_rpm      = 6500.0f;
  spec->idle_rpm         = 800.0f;
  spec->stall_rpm        = 0.0f; /* use engine desc default if 0 */
  spec->peak_torque_nm   = 0.0f; /* derive */
  spec->peak_torque_rpm  = 0.0f; /* derive */
  spec->forced_induction = LCC_FI_NONE;
  spec->boost_target_kpa = 0.0f;
}

/* generate WOT torque and friction curves + throttle & boost maps, attach to car and own the memory */
lcc_result_t lcc_car_generate_engine_from_simple_spec(lcc_car_t *car, const lcc_engine_simple_spec_t *spec) {
  if(!car || !spec) return LCC_ERR_INVALID_ARG;
  if(!(spec->rated_power_kw > 1.0f) || !(spec->redline_rpm > 1000.0f)) return LCC_ERR_INVALID_ARG;

  const float rl         = spec->redline_rpm;
  const float idle       = (spec->idle_rpm > 0.0f) ? spec->idle_rpm : 800.0f;
  float       rpm_pwr    = (spec->rated_power_rpm > 0.0f) ? spec->rated_power_rpm : 0.0f;
  float       tpeak_mult = 1.20f, rpm_tpeak = 0.45f * rl, rpm_power_def = 0.87f * rl;
  lcc__engine_shape_params(spec->forced_induction, rl, &tpeak_mult, &rpm_tpeak, &rpm_power_def);
  if(!(rpm_pwr > 0.0f)) rpm_pwr = rpm_power_def;
  if(spec->peak_torque_rpm > 0.0f) rpm_tpeak = spec->peak_torque_rpm;
  rpm_tpeak = lcc__clampf(rpm_tpeak, idle + 100.0f, rl - 500.0f);
  rpm_pwr   = lcc__clampf(rpm_pwr, rpm_tpeak + 200.0f, rl - 200.0f);

  float omega_pwr = lcc__rpm_to_radps(rpm_pwr);
  float T_at_pwr  = (spec->rated_power_kw * 1000.0f) / fmaxf(1.0f, omega_pwr);
  float T_peak    = (spec->peak_torque_nm > 0.0f) ? spec->peak_torque_nm : (tpeak_mult * T_at_pwr);

  /* target torques at some key rpms (shape varies by FI) */
  float T_idle = T_peak * ((spec->forced_induction == LCC_FI_TURBO || spec->forced_induction == LCC_FI_TWINCHARGED) ? 0.40f : (spec->forced_induction == LCC_FI_SUPERCHARGER ? 0.50f : 0.55f));
  float T_pre  = T_peak * 0.95f;                                                       /* before peak */
  float T_mid  = T_peak * ((spec->forced_induction == LCC_FI_NONE) ? 0.93f : 0.98f);   /* between Tpeak and Ppeak */
  float T_pwr  = T_at_pwr;                                                             /* at power peak */
  float T_92   = T_at_pwr * ((spec->forced_induction == LCC_FI_NONE) ? 0.85f : 0.90f); /* near redline */
  float T_rl   = T_at_pwr * ((spec->forced_induction == LCC_FI_NONE) ? 0.70f : 0.80f); /* redline */

  /* Build WOT torque points */
  int                  nwot = LCC_ENG_GEN_WOT_POINTS;
  lcc_curve1d_point_t *wot  = (lcc_curve1d_point_t *)lcc__alloc(sizeof(lcc_curve1d_point_t) * (size_t)nwot, lcc__alloc_user);
  if(!wot) return LCC_ERR_OUT_OF_MEMORY;
  float r0                         = idle;
  float r1                         = lcc__lerp(idle, rl, 0.25f);
  float r2                         = lcc__lerp(idle, rpm_tpeak, 0.75f);
  float r3                         = rpm_tpeak;
  float r4                         = lcc__lerp(rpm_tpeak, rpm_pwr, 0.5f);
  float r5                         = rpm_pwr;
  float r6                         = lcc__lerp(rpm_pwr, rl, 0.92f);
  float r7                         = rl;
  float r8                         = rl + 0.01f * rl; /* tiny overshoot to ensure clamp */
  float rs[LCC_ENG_GEN_WOT_POINTS] = { r0, r1, r2, r3, r4, r5, r6, r7, r8 };
  float ts[LCC_ENG_GEN_WOT_POINTS] = { T_idle, lcc__lerp(T_idle, T_pre, 0.6f), T_pre, T_peak, T_mid, T_pwr, T_92, T_rl, T_rl * 0.95f };
  for(int i = 0; i < nwot; ++i) {
    wot[i].x = rs[i];
    wot[i].y = fmaxf(0.0f, ts[i]);
  }

  /* friction torque vs rpm: ~10% of rated power at P_peak, tapering to ~10 Nm at idle */
  int                  nfr = LCC_ENG_GEN_FRICT_POINTS;
  lcc_curve1d_point_t *frc = (lcc_curve1d_point_t *)lcc__alloc(sizeof(lcc_curve1d_point_t) * (size_t)nfr, lcc__alloc_user);
  if(!frc) {
    lcc__free(wot, lcc__alloc_user);
    return LCC_ERR_OUT_OF_MEMORY;
  }
  float Pfrac                        = 0.10f;
  float Tf_pwr                       = (spec->rated_power_kw * 1000.0f * Pfrac) / fmaxf(1.0f, omega_pwr);
  float Tf_idle                      = 10.0f;
  float Tf_rl                        = Tf_pwr * 1.10f;
  float rF[LCC_ENG_GEN_FRICT_POINTS] = { idle, lcc__lerp(idle, rpm_tpeak, 0.5f), rpm_pwr, lcc__lerp(rpm_pwr, rl, 0.5f), rl, rl + 0.01f * rl };
  float tF[LCC_ENG_GEN_FRICT_POINTS] = { Tf_idle, lcc__lerp(Tf_idle, Tf_pwr, 0.7f), Tf_pwr, lcc__lerp(Tf_pwr, Tf_rl, 0.7f), Tf_rl, Tf_rl * 1.05f };
  for(int i = 0; i < nfr; ++i) {
    frc[i].x = rF[i];
    frc[i].y = fmaxf(0.0f, tF[i]);
  }

  /* slight shaping based on FI */
  int                  nth = 5;
  lcc_curve1d_point_t *thr = (lcc_curve1d_point_t *)lcc__alloc(sizeof(lcc_curve1d_point_t) * (size_t)nth, lcc__alloc_user);
  if(!thr) {
    lcc__free(wot, lcc__alloc_user);
    lcc__free(frc, lcc__alloc_user);
    return LCC_ERR_OUT_OF_MEMORY;
  }
  float gamma = (spec->forced_induction == LCC_FI_TURBO || spec->forced_induction == LCC_FI_TWINCHARGED) ? 1.10f : 0.90f;
  lcc__make_throttle_map_points(gamma, thr, &nth);

  /* Boost map if FI */
  int                nboost       = 0;
  lcc_map2d_point_t *boost_pts    = NULL;
  float              boost_target = lcc__kpa_target_from_fi(spec->forced_induction, spec->boost_target_kpa);
  if(spec->forced_induction != LCC_FI_NONE && boost_target > 1.0f) {
    nboost    = 7 * 5;
    boost_pts = (lcc_map2d_point_t *)lcc__alloc(sizeof(lcc_map2d_point_t) * (size_t)nboost, lcc__alloc_user);
    if(!boost_pts) {
      lcc__free(wot, lcc__alloc_user);
      lcc__free(frc, lcc__alloc_user);
      lcc__free(thr, lcc__alloc_user);
      return LCC_ERR_OUT_OF_MEMORY;
    }
    lcc__make_boost_map_points(spec->forced_induction, rl, boost_target, boost_pts, &nboost);
  }

  /* free any existing generated maps, then attach new */
  lcc__free_owned_engine_maps(car);
  car->desc.engine.wot_torque_nm_vs_rpm.points      = wot;
  car->desc.engine.wot_torque_nm_vs_rpm.count       = LCC_ENG_GEN_WOT_POINTS;
  car->desc.engine.friction_torque_nm_vs_rpm.points = frc;
  car->desc.engine.friction_torque_nm_vs_rpm.count  = LCC_ENG_GEN_FRICT_POINTS;
  car->desc.engine.throttle_map.points              = thr;
  car->desc.engine.throttle_map.count               = 5;
  if(boost_pts && nboost > 0) {
    car->desc.engine.boost_pressure_kpa_vs_rpm_throttle.points = boost_pts;
    car->desc.engine.boost_pressure_kpa_vs_rpm_throttle.count  = nboost;
    car->desc.engine.wastegate_pressure_kpa                    = boost_target;
  } else {
    car->desc.engine.boost_pressure_kpa_vs_rpm_throttle.points = NULL;
    car->desc.engine.boost_pressure_kpa_vs_rpm_throttle.count  = 0;
  }
  /* engine basic params */
  car->desc.engine.idle_rpm = idle;
  if(spec->stall_rpm > 0.0f) car->desc.engine.stall_rpm = spec->stall_rpm;
  car->desc.engine.redline_rpm      = rl;
  car->desc.engine.forced_induction = spec->forced_induction;

  /* record ownership so we can free on destroy or replace */
  car->owned_wot_pts     = wot;
  car->owned_wot_count   = LCC_ENG_GEN_WOT_POINTS;
  car->owned_fric_pts    = frc;
  car->owned_fric_count  = LCC_ENG_GEN_FRICT_POINTS;
  car->owned_thr_pts     = thr;
  car->owned_thr_count   = 5;
  car->owned_boost_pts   = boost_pts;
  car->owned_boost_count = nboost;

  return LCC_OK;
}

/*}}}*/

/* utilities */
void lcc_car_get_local_bounds(const lcc_car_t *car, float min_local_out[2], float max_local_out[2]) {
  if(!car) return;
  float half_w = 0.5f * fmaxf(0.1f, car->desc.chassis.width_m);
  float half_l = 0.5f * fmaxf(0.1f, car->desc.chassis.length_m);
  if(min_local_out) {
    min_local_out[0] = -half_l;
    min_local_out[1] = -half_w;
  }
  if(max_local_out) {
    max_local_out[0] = half_l;
    max_local_out[1] = half_w;
  }
}

/* TODO: test this function */
int lcc_car_get_wheel_global_positions(const lcc_car_t *car, float out_positions[][2], int max_wheels) {
  if(!car || !out_positions || max_wheels <= 0) return 0;
  int count = car->wheel_count;
  if(count > max_wheels) count = max_wheels;
  float cos_r = cosf(car->car_state.yaw_rad);
  float sin_r = sinf(car->car_state.yaw_rad);
  for(int i = 0; i < count; ++i) {
    float lx            = car->desc.wheels[i].position_local[0];
    float ly            = car->desc.wheels[i].position_local[1];
    float gx            = cos_r * lx - sin_r * ly;
    float gy            = sin_r * lx + cos_r * ly;
    out_positions[i][0] = car->car_state.pos_world[0] + gx;
    out_positions[i][1] = car->car_state.pos_world[1] + gy;
  }
  return count;
}

/*}}}*/

#undef lcc__zero
#undef lcc__pzero
#endif /* LCC_IMPLEMENTATION */
