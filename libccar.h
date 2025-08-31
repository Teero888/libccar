/*
   libccar - 2d top-down car simulation api (c99 single header)
   top-level api: public functions and data structures
   prefix: lcc_
   space: 2d plane (+x forward, +y right), right-handed rotation (cw positive)
   fuck the license for now i just wanna get things to work
*/

#ifndef LIBCCAR_H
#define LIBCCAR_H

#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

/* configuration macros */

/* library version */
#define LCC_VERSION_MAJOR 0
#define LCC_VERSION_MINOR 2
#define LCC_VERSION_PATCH 0

/* constants and limits */
#define LCC_MAX_WHEELS        8
#define LCC_MAX_GEARS         16
#define LCC_MAX_AXLES         4
#define LCC_MAX_NAME_CHARS    64
#define LCC_MAX_TORQUE_POINTS 64
#define LCC_MAX_MAP_POINTS_2D 64
#define LCC_MAX_EVENTS        16

/* default math constants */
#ifndef LCC_PI
#define LCC_PI 3.14159265358979323846f
#endif
#ifndef LCC_TAU
#define LCC_TAU (2.0f * LCC_PI)
#endif

/* c api export macro */
#ifndef LCC_API
#define LCC_API extern
#endif

/* basic types */
typedef int lcc_bool_t; /* 0 = false, nonzero = true */

/* result codes */
typedef enum lcc_result_e {
  LCC_OK                = 0,
  LCC_ERR_UNKNOWN       = -1,
  LCC_ERR_INVALID_ARG   = -2,
  LCC_ERR_OUT_OF_MEMORY = -3,
  LCC_ERR_BAD_STATE     = -4,
  LCC_ERR_UNSUPPORTED   = -5,
  LCC_ERR_BOUNDS        = -6,
  LCC_ERR_NOT_FOUND     = -7
} lcc_result_t;

/* car topology and subsystem enums */

/* drivetrain layout */
typedef enum lcc_drivetrain_layout_e { LCC_LAYOUT_FWD = 0, LCC_LAYOUT_RWD = 1, LCC_LAYOUT_AWD = 2, LCC_LAYOUT_4X4 = 3 } lcc_drivetrain_layout_t;

/* transmission type */
typedef enum lcc_transmission_type_e { LCC_TRANS_MANUAL = 0, LCC_TRANS_AUTOMATIC = 1, LCC_TRANS_CVT = 2, LCC_TRANS_DCT = 3 } lcc_transmission_type_t;

/* predefined gear indices */
typedef enum lcc_gear_e { LCC_GEAR_REVERSE = 0, LCC_GEAR_NEUTRAL = 1 } lcc_gear_t;

/* clutch / converter type */
typedef enum lcc_clutch_type_e {
  LCC_CLUTCH_DRY_SINGLE       = 0,
  LCC_CLUTCH_MULTI_PLATE      = 1,
  LCC_CLUTCH_WET              = 2,
  LCC_CLUTCH_TORQUE_CONVERTER = 3
} lcc_clutch_type_t;

/* engine type */
typedef enum lcc_engine_type_e { LCC_ENGINE_ICE_SPARK = 0, LCC_ENGINE_ICE_DIESEL = 1, LCC_ENGINE_DISABLED = 2 } lcc_engine_type_t;

/* fuel type */
typedef enum lcc_fuel_type_e {
  LCC_FUEL_GASOLINE = 0,
  LCC_FUEL_DIESEL   = 1,
  LCC_FUEL_E85      = 2,
  LCC_FUEL_LPG      = 3,
  LCC_FUEL_CNG      = 4,
  LCC_FUEL_METHANOL = 5,
  LCC_FUEL_HYDROGEN = 6
} lcc_fuel_type_t;

/* forced induction */
typedef enum lcc_forced_induction_e { LCC_FI_NONE = 0, LCC_FI_TURBO = 1, LCC_FI_SUPERCHARGER = 2, LCC_FI_TWINCHARGED = 3 } lcc_forced_induction_t;

/* differential type */
typedef enum lcc_diff_type_e {
  LCC_DIFF_OPEN       = 0,
  LCC_DIFF_LOCKED     = 1,
  LCC_DIFF_LSD_CLUTCH = 2,
  LCC_DIFF_TORSEN     = 3,
  LCC_DIFF_ACTIVE     = 4
} lcc_diff_type_t;

/* brake type */
typedef enum lcc_brake_type_e { LCC_BRAKE_DISC = 0, LCC_BRAKE_DRUM = 1 } lcc_brake_type_t;

/* tire model */
typedef enum lcc_tire_model_e { LCC_TIRE_BRUSH = 0, LCC_TIRE_PACEJKA = 1, LCC_TIRE_FRICTION_ELLIPSE = 2 } lcc_tire_model_t;

/* suspension type */
typedef enum lcc_suspension_type_e {
  LCC_SUS_DWISHBONE    = 0,
  LCC_SUS_MACPHERSON   = 1,
  LCC_SUS_TRAILING_ARM = 2,
  LCC_SUS_SOLID_AXLE   = 3,
  LCC_SUS_MULTI_LINK   = 4
} lcc_suspension_type_t;

/* steering assist */
typedef enum lcc_steer_assist_e { LCC_STEER_ASSIST_NONE = 0, LCC_STEER_ASSIST_HYDRAULIC = 1, LCC_STEER_ASSIST_ELECTRIC = 2 } lcc_steer_assist_t;

/* driver aids */
typedef enum lcc_abs_mode_e { LCC_ABS_OFF = 0, LCC_ABS_ON = 1 } lcc_abs_mode_t;

typedef enum lcc_tc_mode_e { LCC_TC_OFF = 0, LCC_TC_ON = 1 } lcc_tc_mode_t;

typedef enum lcc_esc_mode_e { LCC_ESC_OFF = 0, LCC_ESC_ON = 1 } lcc_esc_mode_t;

/* standard wheel location hints */
typedef enum lcc_wheel_loc_e {
  LCC_WHEEL_FL     = 0,
  LCC_WHEEL_FR     = 1,
  LCC_WHEEL_RL     = 2,
  LCC_WHEEL_RR     = 3,
  LCC_WHEEL_EXTRA0 = 4,
  LCC_WHEEL_EXTRA1 = 5,
  LCC_WHEEL_EXTRA2 = 6,
  LCC_WHEEL_EXTRA3 = 7
} lcc_wheel_loc_t;

/* allocator hooks */
typedef void *(*lcc_alloc_fn)(size_t size, void *user);
typedef void (*lcc_free_fn)(void *ptr, void *user);

/* curve helpers */
typedef struct lcc_curve1d_point_s {
  float x; /* input, e.g. rpm */
  float y; /* output, e.g. torque nm */
} lcc_curve1d_point_t;

typedef struct lcc_curve1d_s {
  const lcc_curve1d_point_t *points;
  int                        count;
} lcc_curve1d_t;

typedef struct lcc_map2d_point_s {
  float x;
  float y;
  float v;
} lcc_map2d_point_t;

typedef struct lcc_map2d_s {
  const lcc_map2d_point_t *points;
  int                      count;
} lcc_map2d_t;

/* environment and surface */
typedef struct lcc_environment_s {
  float ambient_temp_c;
  float air_density;
  float wind_world[2];
  float surface_temp_c;
  float humidity;
  float altitude_m;
  float global_friction_scale;
  float water_depth_m;
} lcc_environment_t;

typedef struct lcc_contact_patch_s {
  lcc_bool_t override_enabled;
  float      mu_dynamic;
  float      mu_static;
  float      rolling_resistance;
  float      surface_vel_world[2];
  float      slip_scale_long;
  float      slip_scale_lat;
} lcc_contact_patch_t;

/* chassis and aero descriptors */
typedef struct lcc_chassis_desc_s {
  char  name[LCC_MAX_NAME_CHARS];
  float mass_kg;
  float inertia_zz;
  float cg_local[2];
  float wheelbase_m;
  float track_front_m;
  float track_rear_m;
  float cg_height_m;
  float width_m;
  float length_m;
} lcc_chassis_desc_t;

typedef struct lcc_aero_desc_s {
  float drag_coefficient;
  float frontal_area_m2;
  float lift_coefficient_front;
  float lift_coefficient_rear;
  float aero_balance;
  float yaw_drag_gain;
} lcc_aero_desc_t;

/* engine and fuel system descriptors */
typedef struct lcc_engine_desc_s {
  lcc_engine_type_t      type;
  lcc_fuel_type_t        fuel;
  lcc_forced_induction_t forced_induction;

  float displacement_l;
  int   cylinders;
  float idle_rpm;
  float redline_rpm;
  float stall_rpm;
  float inertia_kgm2;

  lcc_curve1d_t wot_torque_nm_vs_rpm;
  lcc_curve1d_t friction_torque_nm_vs_rpm;

  float         throttle_response;
  lcc_curve1d_t throttle_map;

  lcc_map2d_t boost_pressure_kpa_vs_rpm_throttle;
  float       wastegate_pressure_kpa;
  float       intercooler_efficiency;

  lcc_bool_t fuel_cut_on_redline;
  lcc_bool_t spark_cut_on_redline;

  float coolant_heat_capacity_j_per_k;
  float oil_heat_capacity_j_per_k;
  float egt_nominal_c;
} lcc_engine_desc_t;

typedef struct lcc_ignition_desc_s {
  lcc_map2d_t spark_advance_deg_vs_rpm_load;
  float       knock_sensitivity;
  float       base_dwell_ms;
} lcc_ignition_desc_t;

typedef struct lcc_fuel_desc_s {
  float tank_capacity_l;
  float fuel_density_kg_per_l;
  float initial_fuel_l;
  float pump_pressure_kpa;
  float injector_flow_cc_per_min;
  float stoich_afr;
} lcc_fuel_desc_t;

typedef struct lcc_exhaust_desc_s {
  float backpressure_kpa;
  float cat_efficiency;
  float muffler_loss_kpa;
} lcc_exhaust_desc_t;

typedef struct lcc_cooling_desc_s {
  float coolant_mass_kg;
  float radiator_area_m2;
  float radiator_ua_w_per_k;
  float water_pump_flow_l_per_min;
  float thermostat_open_c;
  float fan_on_c;
  float fan_power_w;
} lcc_cooling_desc_t;

typedef struct lcc_oil_desc_s {
  float oil_mass_kg;
  float viscosity_index;
  float pump_pressure_kpa;
} lcc_oil_desc_t;

/* electrical system descriptors */
typedef struct lcc_battery_desc_s {
  float capacity_ah;
  float nominal_voltage_v;
  float internal_resistance_ohm;
  float initial_soc;
} lcc_battery_desc_t;

typedef struct lcc_alternator_desc_s {
  float max_current_a;
  float cut_in_rpm;
  float efficiency;
} lcc_alternator_desc_t;

typedef struct lcc_starter_desc_s {
  float power_w;
  float max_torque_nm;
  float draw_current_a;
} lcc_starter_desc_t;

/* ecu and driver aids */
typedef struct lcc_ecu_desc_s {
  lcc_abs_mode_t abs_mode;
  lcc_tc_mode_t  tc_mode;
  lcc_esc_mode_t esc_mode;
  lcc_bool_t     auto_clutch;
  lcc_bool_t     idle_control;
  float          idle_pid_p, idle_pid_i, idle_pid_d;
  float          launch_control_rpm;
} lcc_ecu_desc_t;

/* transmission and driveline descriptors */
typedef struct lcc_transmission_desc_s {
  lcc_transmission_type_t type;
  lcc_clutch_type_t       clutch;
  int                     gear_count;
  float                   gear_ratios[LCC_MAX_GEARS];
  float                   final_drive_ratio;
  float                   shift_time_s;
  float                   auto_upshift_rpm;
  float                   auto_downshift_rpm;
  float                   converter_stall_rpm;
  float                   converter_k_factor;
} lcc_transmission_desc_t;

typedef struct lcc_diff_desc_s {
  lcc_diff_type_t type;
  float           preload_nm;
  float           bias_ratio;
  float           lock_coef;
} lcc_diff_desc_t;

typedef struct lcc_driveline_desc_s {
  lcc_drivetrain_layout_t layout;
  lcc_diff_desc_t         center_diff;
  lcc_diff_desc_t         front_diff;
  lcc_diff_desc_t         rear_diff;
  float                   front_torque_split;
} lcc_driveline_desc_t;

/* wheel end descriptors */
typedef struct lcc_wheel_desc_s {
  lcc_wheel_loc_t location;
  float           position_local[2];
  float           radius_m;
  float           width_m;
  float           mass_kg;
  float           inertia_kgm2;
  lcc_bool_t      steerable;
  lcc_bool_t      driven;
  lcc_bool_t      has_brake;
} lcc_wheel_desc_t;

typedef struct lcc_tire_desc_s {
  lcc_tire_model_t model;
  float            mu_nominal;
  float            load_sensitivity;
  float            rolling_resistance;
  float            pressure_kpa;
  float            ideal_pressure_kpa;
  float            relaxation_length_long_m;
  float            relaxation_length_lat_m;
  float            wear_rate;
} lcc_tire_desc_t;

typedef struct lcc_brake_desc_s {
  lcc_brake_type_t type;
  float            max_torque_nm;
  float            disc_radius_m;
  float            pad_mu;
  float            cooling_area_m2;
} lcc_brake_desc_t;

typedef struct lcc_suspension_desc_s {
  lcc_suspension_type_t type;
  float                 spring_rate_n_per_m;
  float                 damper_c_compression_n_per_mps;
  float                 damper_c_rebound_n_per_mps;
  float                 travel_m;
  float                 bump_stop_rate_n_per_m;
  float                 motion_ratio;
} lcc_suspension_desc_t;

typedef struct lcc_arb_desc_s {
  float front_rate_n_per_rad;
  float rear_rate_n_per_rad;
} lcc_arb_desc_t;

typedef struct lcc_steering_desc_s {
  lcc_steer_assist_t assist;
  float              max_steer_deg;
  float              rack_ratio;
  float              ackermann_factor;
  float              assist_gain;
} lcc_steering_desc_t;

/* damage/tolerance model description */
typedef struct lcc_damage_desc_s {
  float engine_health;
  float transmission_health;
  float cooling_health;
  float brake_health[LCC_MAX_WHEELS];
  float tire_health[LCC_MAX_WHEELS];
  float suspension_health[LCC_MAX_WHEELS];
} lcc_damage_desc_t;

/* full car descriptor */
typedef struct lcc_car_desc_s {
  lcc_chassis_desc_t chassis;
  lcc_aero_desc_t    aero;

  lcc_engine_desc_t   engine;
  lcc_ignition_desc_t ignition;
  lcc_fuel_desc_t     fuel;
  lcc_exhaust_desc_t  exhaust;
  lcc_cooling_desc_t  cooling;
  lcc_oil_desc_t      oil;

  lcc_battery_desc_t    battery;
  lcc_alternator_desc_t alternator;
  lcc_starter_desc_t    starter;
  lcc_ecu_desc_t        ecu;

  lcc_transmission_desc_t transmission;
  lcc_driveline_desc_t    driveline;

  int                   wheel_count;
  lcc_wheel_desc_t      wheels[LCC_MAX_WHEELS];
  lcc_tire_desc_t       tires[LCC_MAX_WHEELS];
  lcc_brake_desc_t      brakes[LCC_MAX_WHEELS];
  lcc_suspension_desc_t suspension[LCC_MAX_WHEELS];
  lcc_arb_desc_t        arbs;
  lcc_steering_desc_t   steering;

  lcc_environment_t environment;
} lcc_car_desc_t;

/* control input */
typedef struct lcc_controls_s {
  float throttle;
  float brake;
  float clutch; /* 1 = fully disengaged */
  float steer;  /* -1..1 */
  float handbrake;

  lcc_bool_t ignition_switch;
  lcc_bool_t starter;

  lcc_bool_t abs_enable_override;
  lcc_bool_t tc_enable_override;
  lcc_bool_t esc_enable_override;
} lcc_controls_t;

/* states */
typedef struct lcc_wheel_state_s {
  float               omega_radps;
  float               drive_torque_nm;
  float               brake_torque_nm;
  float               normal_force_n;
  float               slip_ratio;
  float               slip_angle_rad;
  float               camber_rad;
  float               tire_force_long_n;
  float               tire_force_lat_n;
  float               patch_world_pos[2];
  float               tire_temp_c;
  float               tire_wear;
  float               pressure_kpa;
  lcc_contact_patch_t contact;
} lcc_wheel_state_t;

typedef struct lcc_engine_state_s {
  lcc_bool_t running;
  lcc_bool_t stalled;
  float      rpm;
  float      throttle_effective;
  float      torque_nm;
  float      manifold_pressure_kpa;
  float      coolant_temp_c;
  float      oil_temp_c;
  float      egt_c;
  float      fuel_rate_lph;
} lcc_engine_state_t;

typedef struct lcc_ignition_state_s {
  float      spark_advance_deg;
  lcc_bool_t knock_detected;
  float      misfire_rate;
} lcc_ignition_state_t;

typedef struct lcc_transmission_state_s {
  int        gear_index;
  float      clutch_engagement;
  float      input_rpm;
  float      output_rpm;
  float      converter_slip;
  lcc_bool_t shifting;
} lcc_transmission_state_t;

typedef struct lcc_diff_state_s {
  float left_shaft_torque_nm;
  float right_shaft_torque_nm;
  float lock_fraction;
} lcc_diff_state_t;

typedef struct lcc_brake_state_s {
  float          line_pressure_kpa;
  float          temp_c[LCC_MAX_WHEELS];
  float          pad_wear[LCC_MAX_WHEELS];
  lcc_abs_mode_t abs_active;
} lcc_brake_state_t;

typedef struct lcc_suspension_state_s {
  float travel_m[LCC_MAX_WHEELS];
  float spring_force_n[LCC_MAX_WHEELS];
  float damper_force_n[LCC_MAX_WHEELS];
  float arb_torque_front_nm;
  float arb_torque_rear_nm;
  float roll_angle_rad;
} lcc_suspension_state_t;

typedef struct lcc_electrics_state_s {
  float      battery_soc;
  float      battery_voltage_v;
  float      battery_current_a;
  float      alternator_current_a;
  lcc_bool_t consumers_headlights;
} lcc_electrics_state_t;

typedef struct lcc_fuel_state_s {
  float fuel_l;
  float rail_pressure_kpa;
} lcc_fuel_state_t;

typedef struct lcc_cooling_state_s {
  float      coolant_temp_c;
  float      radiator_heat_w;
  lcc_bool_t fan_on;
  float      oil_temp_c;
} lcc_cooling_state_t;

typedef struct lcc_exhaust_state_s {
  float backpressure_kpa;
  float cat_temp_c;
} lcc_exhaust_state_t;

typedef struct lcc_aero_state_s {
  float drag_force_world[2];
  float downforce_n_front;
  float downforce_n_rear;
} lcc_aero_state_t;

typedef struct lcc_car_state_s {
  double time_s;
  float  dt_s;
  float  pos_world[2];
  float  vel_world[2];
  float  acc_world[2];
  float  yaw_rad;
  float  yaw_rate_radps;
  float  vel_body[2];
  float  acc_body[2];
  float  speed_mps;
  float  beta_rad;
  float  mass_kg;
} lcc_car_state_t;

typedef struct lcc_telemetry_s {
  lcc_car_state_t          car;
  lcc_engine_state_t       engine;
  lcc_transmission_state_t transmission;
  lcc_brake_state_t        brakes;
  lcc_suspension_state_t   suspension;
  lcc_electrics_state_t    electrics;
  lcc_fuel_state_t         fuel;
  lcc_cooling_state_t      cooling;
  lcc_exhaust_state_t      exhaust;
  lcc_aero_state_t         aero;

  lcc_wheel_state_t wheels[LCC_MAX_WHEELS];
  int               wheel_count;
} lcc_telemetry_t;

typedef struct lcc_damage_state_s {
  float engine_health;
  float transmission_health;
  float cooling_health;
  float brake_health[LCC_MAX_WHEELS];
  float tire_health[LCC_MAX_WHEELS];
  float suspension_health[LCC_MAX_WHEELS];
} lcc_damage_state_t;

/* public opaque car handle */
typedef struct lcc_car_s lcc_car_t;

/* events and callbacks */
typedef enum lcc_event_type_e {
  LCC_EVENT_ENGINE_START    = 0,
  LCC_EVENT_ENGINE_STOP     = 1,
  LCC_EVENT_GEAR_CHANGE     = 2,
  LCC_EVENT_ENGINE_STALL    = 3,
  LCC_EVENT_ABS_ACTIVE      = 4,
  LCC_EVENT_TC_ACTIVE       = 5,
  LCC_EVENT_ESC_ACTIVE      = 6,
  LCC_EVENT_OVERHEAT        = 7,
  LCC_EVENT_FUEL_STARVATION = 8
} lcc_event_type_t;

typedef struct lcc_event_s {
  lcc_event_type_t type;
  double           time_s;
  int              data_i32;
  float            data_f32;
} lcc_event_t;

typedef void (*lcc_event_cb)(const lcc_event_t *evt, void *user);

/* library/system functions */
LCC_API void        lcc_set_allocators(lcc_alloc_fn alloc_fn, lcc_free_fn free_fn, void *user);
LCC_API void        lcc_get_version(int *major_out, int *minor_out, int *patch_out);
LCC_API const char *lcc_version_string(void);

/* descriptor defaults */
LCC_API void lcc_car_desc_init_defaults(lcc_car_desc_t *desc);
LCC_API void lcc_engine_desc_init_defaults(lcc_engine_desc_t *desc);
LCC_API void lcc_ignition_desc_init_defaults(lcc_ignition_desc_t *desc);
LCC_API void lcc_fuel_desc_init_defaults(lcc_fuel_desc_t *desc);
LCC_API void lcc_exhaust_desc_init_defaults(lcc_exhaust_desc_t *desc);
LCC_API void lcc_cooling_desc_init_defaults(lcc_cooling_desc_t *desc);
LCC_API void lcc_oil_desc_init_defaults(lcc_oil_desc_t *desc);
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
LCC_API void lcc_suspension_desc_init_defaults(lcc_suspension_desc_t *desc);
LCC_API void lcc_arb_desc_init_defaults(lcc_arb_desc_t *desc);
LCC_API void lcc_steering_desc_init_defaults(lcc_steering_desc_t *desc);
LCC_API void lcc_environment_init_defaults(lcc_environment_t *env);

/* lifecycle */
LCC_API lcc_car_t   *lcc_car_create(const lcc_car_desc_t *desc);
LCC_API void         lcc_car_destroy(lcc_car_t *car);
LCC_API lcc_result_t lcc_car_reset(lcc_car_t *car, const lcc_car_state_t *optional_state);

/* configuration at runtime */
LCC_API void lcc_car_set_environment(lcc_car_t *car, const lcc_environment_t *env);
LCC_API void lcc_car_get_environment(const lcc_car_t *car, lcc_environment_t *env_out);

LCC_API lcc_result_t lcc_car_set_wheel_contact(lcc_car_t *car, int wheel_index, const lcc_contact_patch_t *contact);
LCC_API lcc_result_t lcc_car_get_wheel_contact(const lcc_car_t *car, int wheel_index, lcc_contact_patch_t *contact_out);

LCC_API lcc_result_t lcc_car_set_engine_map(lcc_car_t *car, const lcc_curve1d_t *wot_torque, const lcc_curve1d_t *friction);
LCC_API lcc_result_t lcc_car_set_ignition_map(lcc_car_t *car, const lcc_map2d_t *spark);
LCC_API lcc_result_t lcc_car_set_boost_map(lcc_car_t *car, const lcc_map2d_t *boost);
LCC_API lcc_result_t lcc_car_set_gear_ratios(lcc_car_t *car, const float *gear_ratios, int gear_count, float final_drive);
LCC_API lcc_result_t lcc_car_set_diff_params(lcc_car_t *car, lcc_diff_type_t front, lcc_diff_type_t rear, lcc_diff_type_t center, float preload_nm,
  float bias_ratio);
LCC_API lcc_result_t lcc_car_set_tire_params(lcc_car_t *car, int wheel_index, const lcc_tire_desc_t *tire);
LCC_API lcc_result_t lcc_car_set_brake_params(lcc_car_t *car, int wheel_index, const lcc_brake_desc_t *brake);
LCC_API lcc_result_t lcc_car_set_suspension_params(lcc_car_t *car, int wheel_index, const lcc_suspension_desc_t *sus);
LCC_API lcc_result_t lcc_car_set_arb_params(lcc_car_t *car, const lcc_arb_desc_t *arb);
LCC_API lcc_result_t lcc_car_set_steering_params(lcc_car_t *car, const lcc_steering_desc_t *steer);

/* powertrain controls */
LCC_API lcc_result_t lcc_car_engine_start(lcc_car_t *car);
LCC_API lcc_result_t lcc_car_engine_stop(lcc_car_t *car);
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

/* pose and velocity */
LCC_API void lcc_car_set_pose(lcc_car_t *car, const float pos_world[2], float yaw_rad);
LCC_API void lcc_car_get_pose(const lcc_car_t *car, float pos_world_out[2], float *yaw_rad_out);
LCC_API void lcc_car_set_velocity(lcc_car_t *car, const float vel_world[2], float yaw_rate_radps);
LCC_API void lcc_car_get_velocity(const lcc_car_t *car, float vel_world_out[2], float *yaw_rate_radps_out);

/* simulation stepping */
LCC_API void         lcc_car_set_controls(lcc_car_t *car, const lcc_controls_t *controls);
LCC_API void         lcc_car_get_controls(const lcc_car_t *car, lcc_controls_t *controls_out);
LCC_API lcc_result_t lcc_car_step(lcc_car_t *car, float dt_s);

/* fetching states */
LCC_API void lcc_car_get_telemetry(const lcc_car_t *car, lcc_telemetry_t *out);
LCC_API void lcc_car_get_state(const lcc_car_t *car, lcc_car_state_t *out);
LCC_API void lcc_car_get_engine_state(const lcc_car_t *car, lcc_engine_state_t *out);
LCC_API void lcc_car_get_ignition_state(const lcc_car_t *car, lcc_ignition_state_t *out);
LCC_API void lcc_car_get_transmission_state(const lcc_car_t *car, lcc_transmission_state_t *out);
LCC_API void lcc_car_get_brake_state(const lcc_car_t *car, lcc_brake_state_t *out);
LCC_API void lcc_car_get_suspension_state(const lcc_car_t *car, lcc_suspension_state_t *out);
LCC_API void lcc_car_get_electrics_state(const lcc_car_t *car, lcc_electrics_state_t *out);
LCC_API void lcc_car_get_fuel_state(const lcc_car_t *car, lcc_fuel_state_t *out);
LCC_API void lcc_car_get_cooling_state(const lcc_car_t *car, lcc_cooling_state_t *out);
LCC_API void lcc_car_get_exhaust_state(const lcc_car_t *car, lcc_exhaust_state_t *out);
LCC_API void lcc_car_get_aero_state(const lcc_car_t *car, lcc_aero_state_t *out);
LCC_API void lcc_car_get_wheel_state(const lcc_car_t *car, int wheel_index, lcc_wheel_state_t *out);

/* damage and failures */
LCC_API void lcc_car_get_damage_state(const lcc_car_t *car, lcc_damage_state_t *out);
LCC_API void lcc_car_set_damage_state(lcc_car_t *car, const lcc_damage_state_t *in);
LCC_API void lcc_car_apply_damage_factor(lcc_car_t *car, const char *subsystem, float factor_0_to_1);

/* utilities */
LCC_API void lcc_car_get_local_bounds(const lcc_car_t *car, float min_local_out[2], float max_local_out[2]);
LCC_API int  lcc_car_get_wheel_local_positions(const lcc_car_t *car, float out_positions[][2], int max_wheels);

/* event subscription */
LCC_API void lcc_car_set_event_callback(lcc_car_t *car, lcc_event_cb callback, void *user);

/* deterministic controls recording/playback */
typedef struct lcc_control_frame_s {
  double         time_s;
  lcc_controls_t controls;
} lcc_control_frame_t;

LCC_API lcc_result_t lcc_car_playback_controls(lcc_car_t *car, const lcc_control_frame_t *frames, int frame_count);

/* debug helpers */
LCC_API int lcc_format_engine_summary(const lcc_engine_state_t *es, char *out_str, int maxlen);
LCC_API int lcc_format_transmission_summary(const lcc_transmission_state_t *ts, char *out_str, int maxlen);
LCC_API int lcc_format_wheel_summary(const lcc_wheel_state_t *ws, char *out_str, int maxlen);

/* unit helpers */
LCC_API float lcc_deg_to_rad(float deg);
LCC_API float lcc_rad_to_deg(float rad);

#ifdef __cplusplus
}
#endif

#endif /* LIBCCAR_H */

/* ========================================================================================= */
/*                                        implementation                                     */
/* ========================================================================================= */
#define LCC_IMPLEMENTATION
#ifdef LCC_IMPLEMENTATION

#ifdef LCC_API
#undef LCC_API
#define LCC_API
#endif

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* internal helpers */

/* =========================================================================================
   Defaults and constants
   ========================================================================================= */
/* Default engine curves so a car drives without user maps */
static const lcc_curve1d_point_t LCC__DEF_ENGINE_WOT_POINTS[] = { { 800.0f, 100.0f }, { 1200.0f, 135.0f }, { 1800.0f, 165.0f }, { 2500.0f, 185.0f },
  { 3200.0f, 200.0f }, { 4000.0f, 210.0f }, { 5000.0f, 205.0f }, { 6000.0f, 190.0f }, { 6500.0f, 175.0f } };
static const lcc_curve1d_point_t LCC__DEF_ENGINE_FRICTION_POINTS[] = { { 600.0f, 10.0f }, { 1000.0f, 13.0f }, { 2000.0f, 18.0f }, { 3000.0f, 24.0f },
  { 4000.0f, 30.0f }, { 5000.0f, 36.0f }, { 6000.0f, 42.0f }, { 7000.0f, 49.0f } };
static const lcc_curve1d_point_t LCC__DEF_THROTTLE_MAP_POINTS[]    = { { 0.0f, 0.0f }, { 0.25f, 0.25f }, { 0.5f, 0.5f }, { 0.75f, 0.75f },
     { 1.0f, 1.0f } };

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

/* math helpers */
static float lcc__clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static int lcc__mini(int a, int b) {
  return a < b ? a : b;
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

/* rpm/rad/s */
static float lcc__rpm_to_radps(float rpm) {
  return rpm * (LCC_TAU / 60.0f);
}

static float lcc__radps_to_rpm(float radps) {
  return radps * (60.0f / LCC_TAU);
}

/* simple low-pass filter */
static float lcc__lp(float prev, float target, float cutoff_hz, float dt) {
  float a = 1.0f - expf(-2.0f * LCC_PI * lcc__clampf(cutoff_hz, 0.0f, 1000.0f) * lcc__clampf(dt, 0.0f, 1.0f));
  return prev + a * (target - prev);
}

/* event emit helper forward */
static void lcc__emit_event(struct lcc_car_s *car, lcc_event_type_t type, int data_i32, float data_f32);

/* longitudinal slip ratio */
static float lcc__slip_ratio(float Vlong, float omega, float r) {
  float wR    = omega * r;
  float v_ref = 0.30f; /* m/s reference to avoid exploding at crawl */
  float denom = fmaxf(fabsf(Vlong), fabsf(wR));
  denom       = fmaxf(denom, v_ref);
  float s     = (wR - Vlong) / denom;
  return lcc__clampf(s, -3.0f, 3.0f);
}

/* slip angle with sign-preserving Vx softening */
static float lcc__slip_angle(float Vlong, float Vlat) {
  float Vx = (fabsf(Vlong) > 0.05f) ? Vlong : (Vlong >= 0.0f ? 0.05f : -0.05f);
  return atanf(Vlat / Vx);
}

/* simple friction circle projection (can be changed to ellipse if needed) */
static void lcc__friction_project(float *Fx, float *Fy, float Fcap) {
  float fx = *Fx, fy = *Fy;
  float mag = sqrtf(fx * fx + fy * fy);
  float cap = fmaxf(0.0f, Fcap);
  if(mag > cap && mag > 1e-9f) {
    float s = cap / mag;
    *Fx     = fx * s;
    *Fy     = fy * s;
  }
}

/* per-wheel torque limit from friction circle given precomputed arrays */
static float lcc__wheel_torque_limit_from_circle_idx(int idx, const float mu_i[static LCC_MAX_WHEELS], const float Fz_i[static LCC_MAX_WHEELS],
  const float Fy0_abs[static LCC_MAX_WHEELS], const float r_i[static LCC_MAX_WHEELS]) {
  if(idx < 0) return 0.0f;
  float Fcap     = mu_i[idx] * Fz_i[idx];
  float Fy_d     = Fy0_abs[idx];
  float Fx_allow = sqrtf(fmaxf(0.0f, Fcap * Fcap - Fy_d * Fy_d));
  return Fx_allow * r_i[idx];
}

/* ----------------------------------------------------------------------------------------- */
/*                                       car internals                                       */
/* ----------------------------------------------------------------------------------------- */

struct lcc_car_s {
  lcc_car_desc_t    desc;
  lcc_environment_t env;
  lcc_controls_t    controls;

  /* runtime states */
  lcc_car_state_t          car_state;
  lcc_engine_state_t       engine_state;
  lcc_ignition_state_t     ign_state;
  lcc_transmission_state_t trans_state;
  lcc_diff_state_t         front_diff_state;
  lcc_diff_state_t         rear_diff_state;
  lcc_diff_state_t         center_diff_state;
  lcc_brake_state_t        brake_state;
  lcc_suspension_state_t   susp_state;
  lcc_electrics_state_t    elec_state;
  lcc_fuel_state_t         fuel_state;
  lcc_cooling_state_t      cool_state;
  lcc_exhaust_state_t      exh_state;
  lcc_aero_state_t         aero_state;
  lcc_wheel_state_t        wheel_states[LCC_MAX_WHEELS];
  lcc_damage_state_t       damage_state;

  int   wheel_count;
  float wheel_steer_rad[LCC_MAX_WHEELS];
  float wheel_static_load_n[LCC_MAX_WHEELS];
  float total_mass_kg;

  float shift_timer_s;
  int   pending_gear_index;

  /* aids internal state TODO: refactor this, this is bad lol? */
  float abs_mod[LCC_MAX_WHEELS];
  float tc_cut; /* 0..1 cut factor applied to engine torque */
  float esc_extra_brake[LCC_MAX_WHEELS];
  float idle_i; /* idle control integrator (internal) */

  /* suspension internals */
  float susp_travel[LCC_MAX_WHEELS];
  float susp_vel[LCC_MAX_WHEELS];

  lcc_event_cb evt_cb;
  void        *evt_user;

  double last_overheat_evt_time;
  double last_abs_evt_time;
  double last_tc_evt_time;
  double last_esc_evt_time;

  char version_str[32];
};

/* ----------------------------------------------------------------------------------------- */
/*                                    api implementations                                    */
/* ----------------------------------------------------------------------------------------- */

/* allocators */
void lcc_set_allocators(lcc_alloc_fn alloc_fn, lcc_free_fn free_fn, void *user) {
  lcc__alloc      = alloc_fn ? alloc_fn : lcc__malloc;
  lcc__free       = free_fn ? free_fn : lcc__freefn;
  lcc__alloc_user = user;
}

/* version */
void lcc_get_version(int *major_out, int *minor_out, int *patch_out) {
  if(major_out) *major_out = LCC_VERSION_MAJOR;
  if(minor_out) *minor_out = LCC_VERSION_MINOR;
  if(patch_out) *patch_out = LCC_VERSION_PATCH;
}

const char *lcc_version_string(void) {
  static char ver[32];
  static int  inited = 0;
  if(!inited) {
    snprintf(ver, sizeof(ver), "%d.%d.%d", LCC_VERSION_MAJOR, LCC_VERSION_MINOR, LCC_VERSION_PATCH);
    inited = 1;
  }
  return ver;
}

/* defaults */
void lcc_engine_desc_init_defaults(lcc_engine_desc_t *desc) {
  if(!desc) return;
  memset(desc, 0, sizeof(*desc));
  desc->type                          = LCC_ENGINE_ICE_SPARK;
  desc->fuel                          = LCC_FUEL_GASOLINE;
  desc->forced_induction              = LCC_FI_NONE;
  desc->displacement_l                = 2.0f;
  desc->cylinders                     = 4;
  desc->idle_rpm                      = 800.0f;
  desc->redline_rpm                   = 6500.0f;
  desc->stall_rpm                     = 400.0f;
  desc->inertia_kgm2                  = 0.2f;
  desc->throttle_response             = 0.5f;
  desc->wastegate_pressure_kpa        = 110.0f;
  desc->intercooler_efficiency        = 0.7f;
  desc->fuel_cut_on_redline           = 1;
  desc->spark_cut_on_redline          = 0;
  desc->coolant_heat_capacity_j_per_k = 60000.0f;
  desc->oil_heat_capacity_j_per_k     = 40000.0f;
  desc->egt_nominal_c                 = 750.0f;

  /* sane default maps */
  desc->wot_torque_nm_vs_rpm.points      = LCC__DEF_ENGINE_WOT_POINTS;
  desc->wot_torque_nm_vs_rpm.count       = (int)(sizeof(LCC__DEF_ENGINE_WOT_POINTS) / sizeof(LCC__DEF_ENGINE_WOT_POINTS[0]));
  desc->friction_torque_nm_vs_rpm.points = LCC__DEF_ENGINE_FRICTION_POINTS;
  desc->friction_torque_nm_vs_rpm.count  = (int)(sizeof(LCC__DEF_ENGINE_FRICTION_POINTS) / sizeof(LCC__DEF_ENGINE_FRICTION_POINTS[0]));
  desc->throttle_map.points              = LCC__DEF_THROTTLE_MAP_POINTS;
  desc->throttle_map.count               = (int)(sizeof(LCC__DEF_THROTTLE_MAP_POINTS) / sizeof(LCC__DEF_THROTTLE_MAP_POINTS[0]));
}

void lcc_ignition_desc_init_defaults(lcc_ignition_desc_t *desc) {
  if(!desc) return;
  memset(desc, 0, sizeof(*desc));
  desc->knock_sensitivity = 0.5f;
  desc->base_dwell_ms     = 2.5f;
}

void lcc_fuel_desc_init_defaults(lcc_fuel_desc_t *desc) {
  if(!desc) return;
  memset(desc, 0, sizeof(*desc));
  desc->tank_capacity_l          = 50.0f;
  desc->fuel_density_kg_per_l    = 0.745f;
  desc->initial_fuel_l           = 20.0f;
  desc->pump_pressure_kpa        = 300.0f;
  desc->injector_flow_cc_per_min = 1500.0f;
  desc->stoich_afr               = 14.7f;
}

void lcc_exhaust_desc_init_defaults(lcc_exhaust_desc_t *desc) {
  if(!desc) return;
  memset(desc, 0, sizeof(*desc));
  desc->backpressure_kpa = 5.0f;
  desc->cat_efficiency   = 0.9f;
  desc->muffler_loss_kpa = 1.0f;
}

void lcc_cooling_desc_init_defaults(lcc_cooling_desc_t *desc) {
  if(!desc) return;
  memset(desc, 0, sizeof(*desc));
  desc->coolant_mass_kg           = 6.0f;
  desc->radiator_area_m2          = 0.5f;
  desc->radiator_ua_w_per_k       = 500.0f;
  desc->water_pump_flow_l_per_min = 60.0f;
  desc->thermostat_open_c         = 90.0f;
  desc->fan_on_c                  = 100.0f;
  desc->fan_power_w               = 150.0f;
}

void lcc_oil_desc_init_defaults(lcc_oil_desc_t *desc) {
  if(!desc) return;
  memset(desc, 0, sizeof(*desc));
  desc->oil_mass_kg       = 4.0f;
  desc->viscosity_index   = 1.0f;
  desc->pump_pressure_kpa = 400.0f;
}

void lcc_battery_desc_init_defaults(lcc_battery_desc_t *desc) {
  if(!desc) return;
  memset(desc, 0, sizeof(*desc));
  desc->capacity_ah             = 60.0f;
  desc->nominal_voltage_v       = 12.6f;
  desc->internal_resistance_ohm = 0.02f;
  desc->initial_soc             = 0.9f;
}

void lcc_alternator_desc_init_defaults(lcc_alternator_desc_t *desc) {
  if(!desc) return;
  memset(desc, 0, sizeof(*desc));
  desc->max_current_a = 90.0f;
  desc->cut_in_rpm    = 1200.0f;
  desc->efficiency    = 0.6f;
}

void lcc_starter_desc_init_defaults(lcc_starter_desc_t *desc) {
  if(!desc) return;
  memset(desc, 0, sizeof(*desc));
  desc->power_w        = 1000.0f;
  desc->max_torque_nm  = 50.0f;
  desc->draw_current_a = 200.0f;
}

void lcc_ecu_desc_init_defaults(lcc_ecu_desc_t *desc) {
  if(!desc) return;
  memset(desc, 0, sizeof(*desc));

  /* TODO: one of these is fucked up */
  desc->abs_mode = LCC_ABS_OFF;
  desc->tc_mode  = LCC_TC_OFF;
  desc->esc_mode = LCC_ESC_OFF;

  /* TODO: this only in automatic transmission? could be completely removed */
  desc->auto_clutch = 0;

  desc->idle_control       = 1;
  desc->idle_pid_p         = 0.5f;
  desc->idle_pid_i         = 0.1f;
  desc->idle_pid_d         = 0.0f;
  desc->launch_control_rpm = 3500.0f;
}

void lcc_transmission_desc_init_defaults(lcc_transmission_desc_t *desc) {
  if(!desc) return;
  memset(desc, 0, sizeof(*desc));
  desc->type   = LCC_TRANS_MANUAL;
  desc->clutch = LCC_CLUTCH_DRY_SINGLE;
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
  desc->final_drive_ratio   = 3.9f;
  desc->shift_time_s        = 0.30f;
  desc->auto_upshift_rpm    = 6500.0f;
  desc->auto_downshift_rpm  = 1200.0f;
  desc->converter_stall_rpm = 1800.0f;
  desc->converter_k_factor  = 150.0f;
}

void lcc_driveline_desc_init_defaults(lcc_driveline_desc_t *desc) {
  if(!desc) return;
  memset(desc, 0, sizeof(*desc));
  desc->layout                 = LCC_LAYOUT_FWD;
  desc->front_diff.type        = LCC_DIFF_OPEN;
  desc->rear_diff.type         = LCC_DIFF_OPEN;
  desc->center_diff.type       = LCC_DIFF_OPEN;
  desc->front_diff.preload_nm  = 20.0f;
  desc->rear_diff.preload_nm   = 20.0f;
  desc->center_diff.preload_nm = 10.0f;
  desc->front_diff.bias_ratio  = 2.5f;
  desc->rear_diff.bias_ratio   = 2.5f;
  desc->center_diff.bias_ratio = 1.0f;
  desc->front_torque_split     = 0.9f;
}

void lcc_chassis_desc_init_defaults(lcc_chassis_desc_t *desc) {
  if(!desc) return;
  memset(desc, 0, sizeof(*desc));
  strncpy(desc->name, "lcc car", sizeof(desc->name) - 1);
  desc->mass_kg       = 1200.0f;
  desc->inertia_zz    = 1200.0f;
  desc->cg_local[0]   = 0.0f;
  desc->cg_local[1]   = 0.1f;
  desc->wheelbase_m   = 2.6f;
  desc->track_front_m = 1.55f;
  desc->track_rear_m  = 1.55f;
  desc->cg_height_m   = 0.50f;
  desc->width_m       = 1.8f;
  desc->length_m      = 4.2f;
}

void lcc_aero_desc_init_defaults(lcc_aero_desc_t *desc) {
  if(!desc) return;
  memset(desc, 0, sizeof(*desc));
  desc->drag_coefficient       = 0.31f;
  desc->frontal_area_m2        = 2.2f;
  desc->lift_coefficient_front = -0.05f;
  desc->lift_coefficient_rear  = -0.10f;
  desc->aero_balance           = 0.5f;
  desc->yaw_drag_gain          = 0.05f;
}

void lcc_wheel_desc_init_defaults(lcc_wheel_desc_t *desc) {
  if(!desc) return;
  memset(desc, 0, sizeof(*desc));
  desc->radius_m     = 0.31f;
  desc->width_m      = 0.22f;
  desc->mass_kg      = 20.0f;
  desc->inertia_kgm2 = 1.2f;
  desc->steerable    = 0;
  desc->driven       = 1;
  desc->has_brake    = 1;
}

void lcc_tire_desc_init_defaults(lcc_tire_desc_t *desc) {
  if(!desc) return;
  memset(desc, 0, sizeof(*desc));
  desc->model                    = LCC_TIRE_FRICTION_ELLIPSE;
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
  memset(desc, 0, sizeof(*desc));
  desc->type            = LCC_BRAKE_DISC;
  desc->max_torque_nm   = 3000.0f;
  desc->disc_radius_m   = 0.15f;
  desc->pad_mu          = 0.4f;
  desc->cooling_area_m2 = 0.05f;
}

void lcc_suspension_desc_init_defaults(lcc_suspension_desc_t *desc) {
  if(!desc) return;
  memset(desc, 0, sizeof(*desc));
  desc->type                           = LCC_SUS_MACPHERSON;
  desc->spring_rate_n_per_m            = 30000.0f;
  desc->damper_c_compression_n_per_mps = 2500.0f;
  desc->damper_c_rebound_n_per_mps     = 3000.0f;
  desc->travel_m                       = 0.20f;
  desc->bump_stop_rate_n_per_m         = 100000.0f;
  desc->motion_ratio                   = 1.0f;
}

void lcc_arb_desc_init_defaults(lcc_arb_desc_t *desc) {
  if(!desc) return;
  memset(desc, 0, sizeof(*desc));
  desc->front_rate_n_per_rad = 2000.0f;
  desc->rear_rate_n_per_rad  = 1800.0f;
}

void lcc_steering_desc_init_defaults(lcc_steering_desc_t *desc) {
  if(!desc) return;
  memset(desc, 0, sizeof(*desc));
  desc->assist           = LCC_STEER_ASSIST_ELECTRIC;
  desc->max_steer_deg    = 35.0f;
  desc->rack_ratio       = 14.0f;
  desc->ackermann_factor = 0.9f;
  desc->assist_gain      = 1.0f;
}

void lcc_environment_init_defaults(lcc_environment_t *env) {
  if(!env) return;
  memset(env, 0, sizeof(*env));
  env->ambient_temp_c        = 20.0f;
  env->air_density           = 1.225f;
  env->wind_world[0]         = 0.0f;
  env->wind_world[1]         = 0.0f;
  env->surface_temp_c        = 20.0f;
  env->humidity              = 0.5f;
  env->altitude_m            = 0.0f;
  env->global_friction_scale = 1.0f;
  env->water_depth_m         = 0.0f;
}

void lcc_car_desc_init_defaults(lcc_car_desc_t *desc) {
  if(!desc) return;
  memset(desc, 0, sizeof(*desc));

  lcc_chassis_desc_init_defaults(&desc->chassis);
  lcc_aero_desc_init_defaults(&desc->aero);

  lcc_engine_desc_init_defaults(&desc->engine);
  lcc_ignition_desc_init_defaults(&desc->ignition);
  lcc_fuel_desc_init_defaults(&desc->fuel);
  lcc_exhaust_desc_init_defaults(&desc->exhaust);
  lcc_cooling_desc_init_defaults(&desc->cooling);
  lcc_oil_desc_init_defaults(&desc->oil);

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

  lcc_wheel_desc_init_defaults(&desc->wheels[0]);
  desc->wheels[0].location          = LCC_WHEEL_FL;
  desc->wheels[0].steerable         = 1;
  desc->wheels[0].driven            = (desc->driveline.layout != LCC_LAYOUT_RWD);
  desc->wheels[0].position_local[0] = half_wb;
  desc->wheels[0].position_local[1] = -hf;
  lcc_wheel_desc_init_defaults(&desc->wheels[1]);
  desc->wheels[1].location          = LCC_WHEEL_FR;
  desc->wheels[1].steerable         = 1;
  desc->wheels[1].driven            = (desc->driveline.layout != LCC_LAYOUT_RWD);
  desc->wheels[1].position_local[0] = half_wb;
  desc->wheels[1].position_local[1] = hf;
  lcc_wheel_desc_init_defaults(&desc->wheels[2]);
  desc->wheels[2].location          = LCC_WHEEL_RL;
  desc->wheels[2].steerable         = 0;
  desc->wheels[2].driven            = (desc->driveline.layout != LCC_LAYOUT_FWD);
  desc->wheels[2].position_local[0] = -half_wb;
  desc->wheels[2].position_local[1] = -hr;
  lcc_wheel_desc_init_defaults(&desc->wheels[3]);
  desc->wheels[3].location          = LCC_WHEEL_RR;
  desc->wheels[3].steerable         = 0;
  desc->wheels[3].driven            = (desc->driveline.layout != LCC_LAYOUT_FWD);
  desc->wheels[3].position_local[0] = -half_wb;
  desc->wheels[3].position_local[1] = hr;

  for(int i = 0; i < desc->wheel_count; ++i) {
    lcc_tire_desc_init_defaults(&desc->tires[i]);
    lcc_brake_desc_init_defaults(&desc->brakes[i]);
    lcc_suspension_desc_init_defaults(&desc->suspension[i]);
  }

  lcc_arb_desc_init_defaults(&desc->arbs);
  lcc_steering_desc_init_defaults(&desc->steering);
  lcc_environment_init_defaults(&desc->environment);
}

/* lifecycle and internals */

static void lcc__compute_static_loads(lcc_car_t *car) {
  float m = car->car_state.mass_kg;
  if(car->wheel_count <= 0) return;

  float half_wb    = fmaxf(0.05f, car->desc.chassis.wheelbase_m * 0.5f);
  float cg_x       = car->desc.chassis.cg_local[0];
  float front_frac = (half_wb + cg_x) / (2.0f * half_wb);
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
  memset(&car->car_state, 0, sizeof(car->car_state));
  memset(&car->engine_state, 0, sizeof(car->engine_state));
  memset(&car->ign_state, 0, sizeof(car->ign_state));
  memset(&car->trans_state, 0, sizeof(car->trans_state));
  memset(&car->front_diff_state, 0, sizeof(car->front_diff_state));
  memset(&car->rear_diff_state, 0, sizeof(car->rear_diff_state));
  memset(&car->center_diff_state, 0, sizeof(car->center_diff_state));
  memset(&car->brake_state, 0, sizeof(car->brake_state));
  memset(&car->susp_state, 0, sizeof(car->susp_state));
  memset(&car->elec_state, 0, sizeof(car->elec_state));
  memset(&car->fuel_state, 0, sizeof(car->fuel_state));
  memset(&car->cool_state, 0, sizeof(car->cool_state));
  memset(&car->exh_state, 0, sizeof(car->exh_state));
  memset(&car->aero_state, 0, sizeof(car->aero_state));
  memset(&car->wheel_states[0], 0, sizeof(car->wheel_states));
  memset(&car->damage_state, 0, sizeof(car->damage_state));
  memset(&car->controls, 0, sizeof(car->controls));
  memset(car->abs_mod, 0, sizeof(car->abs_mod));
  memset(car->esc_extra_brake, 0, sizeof(car->esc_extra_brake));
  memset(car->susp_travel, 0, sizeof(car->susp_travel));
  memset(car->susp_vel, 0, sizeof(car->susp_vel));

  car->tc_cut = 0.0f;

  car->wheel_count = car->desc.wheel_count;
  for(int i = 0; i < car->wheel_count; ++i) {
    car->wheel_states[i].pressure_kpa             = car->desc.tires[i].pressure_kpa;
    car->wheel_states[i].contact.override_enabled = 0;
    lcc__v2_zero(car->wheel_states[i].patch_world_pos);
    car->wheel_steer_rad[i]          = 0.0f;
    car->brake_state.temp_c[i]       = car->env.ambient_temp_c;
    car->brake_state.pad_wear[i]     = 0.0f;
    car->wheel_states[i].tire_temp_c = car->env.surface_temp_c;
    car->wheel_states[i].tire_wear   = 0.0f;
    car->abs_mod[i]                  = 1.0f;
  }

  car->car_state.time_s = 0.0;
  car->car_state.dt_s   = 0.0f;
  lcc__v2_zero(car->car_state.pos_world);
  lcc__v2_zero(car->car_state.vel_world);
  lcc__v2_zero(car->car_state.acc_world);
  lcc__v2_zero(car->car_state.vel_body);
  lcc__v2_zero(car->car_state.acc_body);
  car->car_state.yaw_rad        = 0.0f;
  car->car_state.yaw_rate_radps = 0.0f;
  car->car_state.beta_rad       = 0.0f;

  car->fuel_state.fuel_l               = lcc__clampf(car->desc.fuel.initial_fuel_l, 0.0f, car->desc.fuel.tank_capacity_l);
  float fuel_mass                      = car->fuel_state.fuel_l * car->desc.fuel.fuel_density_kg_per_l;
  car->elec_state.battery_soc          = lcc__saturate(car->desc.battery.initial_soc);
  car->elec_state.battery_voltage_v    = car->desc.battery.nominal_voltage_v;
  car->elec_state.battery_current_a    = 0.0f;
  car->elec_state.alternator_current_a = 0.0f;
  car->elec_state.consumers_headlights = 0;

  car->car_state.mass_kg = car->desc.chassis.mass_kg + fuel_mass;
  car->total_mass_kg     = car->car_state.mass_kg;

  car->engine_state.rpm                   = 0.0f;
  car->engine_state.coolant_temp_c        = car->env.ambient_temp_c;
  car->engine_state.oil_temp_c            = car->env.ambient_temp_c;
  car->engine_state.egt_c                 = car->desc.engine.egt_nominal_c;
  car->engine_state.manifold_pressure_kpa = 100.0f;

  car->cool_state.coolant_temp_c  = car->env.ambient_temp_c;
  car->cool_state.oil_temp_c      = car->env.ambient_temp_c;
  car->exh_state.backpressure_kpa = car->desc.exhaust.backpressure_kpa;
  car->exh_state.cat_temp_c       = car->env.ambient_temp_c;

  car->trans_state.gear_index        = 1; /* start in neutral */
  car->trans_state.clutch_engagement = 1.0f;
  car->trans_state.input_rpm         = 0.0f;
  car->trans_state.output_rpm        = 0.0f;
  car->trans_state.converter_slip    = 0.0f;
  car->trans_state.shifting          = 0;

  car->shift_timer_s      = 0.0f;
  car->pending_gear_index = car->trans_state.gear_index;

  car->damage_state.engine_health       = 1.0f;
  car->damage_state.transmission_health = 1.0f;
  car->damage_state.cooling_health      = 1.0f;
  for(int i = 0; i < car->wheel_count; ++i) {
    car->damage_state.brake_health[i]      = 1.0f;
    car->damage_state.tire_health[i]       = 1.0f;
    car->damage_state.suspension_health[i] = 1.0f;
  }

  car->last_overheat_evt_time = -1e9;
  car->last_abs_evt_time      = -1e9;
  car->last_tc_evt_time       = -1e9;
  car->last_esc_evt_time      = -1e9;

  lcc__compute_static_loads(car);
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

static void lcc__tire_stiffness(const lcc_tire_desc_t *td, float fz, float *Cx_out, float *Cy_out) {
  float p_ratio = (td->ideal_pressure_kpa > 1.0f) ? (td->pressure_kpa / td->ideal_pressure_kpa) : 1.0f;
  p_ratio       = lcc__clampf(p_ratio, 0.6f, 1.4f);
  float base    = fmaxf(fz, 100.0f);
  float Cx      = 12.0f * base * p_ratio;
  float Cy      = 16.0f * base * p_ratio;
  if(Cx_out) *Cx_out = Cx;
  if(Cy_out) *Cy_out = Cy;
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

/* Compute relative patch velocity in tire frame:
   - start with hub velocity in body frame
   - add yaw-spin term
   - rotate to tire frame (-steer)
   - subtract surface velocity if provided (converted to tire frame)
   - apply slip scales if provided via contact */
static void lcc__wheel_vel_tire_frame(const lcc_car_t *car, int i, float out_tire_vel[2]) {
  float r_local[2]     = { car->desc.wheels[i].position_local[0], car->desc.wheels[i].position_local[1] };
  float v_body[2]      = { car->car_state.vel_body[0], car->car_state.vel_body[1] };
  float v_spin_body[2] = { -car->car_state.yaw_rate_radps * r_local[1], car->car_state.yaw_rate_radps * r_local[0] };
  float hub_body[2]    = { v_body[0] + v_spin_body[0], v_body[1] + v_spin_body[1] };

  float steer = car->wheel_steer_rad[i];
  float hub_tire[2];
  lcc__v2_rot(hub_tire, hub_body, -steer);

  float rel_tire[2] = { hub_tire[0], hub_tire[1] };

  const lcc_contact_patch_t *cp = &car->wheel_states[i].contact;
  if(cp && cp->override_enabled) {
    float surf_world[2] = { cp->surface_vel_world[0], cp->surface_vel_world[1] };
    float surf_body[2];
    lcc__v2_rot(surf_body, surf_world, -car->car_state.yaw_rad);
    float surf_tire[2];
    lcc__v2_rot(surf_tire, surf_body, -steer);
    rel_tire[0] -= surf_tire[0];
    rel_tire[1] -= surf_tire[1];

    float scl_long = (cp->slip_scale_long > 0.0f) ? cp->slip_scale_long : 1.0f;
    float scl_lat  = (cp->slip_scale_lat > 0.0f) ? cp->slip_scale_lat : 1.0f;
    rel_tire[0] *= scl_long;
    rel_tire[1] *= scl_lat;
  }

  out_tire_vel[0] = rel_tire[0];
  out_tire_vel[1] = rel_tire[1];
}

/* normal loads including longitudinal and lateral transfer + aero */
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
  float delivered_nm;
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
  r.delivered_nm          = s * (TL + TR);
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
  r.left_nm      = s * TL;
  r.right_nm     = s * TR;
  r.delivered_nm = s * (TL + TR);
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
  r.delivered_nm = s * (lcc__absf(r.left_nm) + lcc__absf(r.right_nm));
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
  r.delivered_nm       = r.left_nm + r.right_nm;
  return r;
}

/* engine torque model */
static float lcc__engine_torque_compute(lcc_car_t *car, float rpm, float throttle, float *out_manifold_kpa) {
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
  if(out_manifold_kpa) *out_manifold_kpa = map_kpa;

  float pr = map_kpa / 100.0f;
  float tq = wot * load * pr - friction * (0.5f + 0.5f * (1.0f - load));
  tq *= lcc__clampf(car->damage_state.engine_health, 0.1f, 1.0f);

  if(ed->fuel_cut_on_redline && rpm >= ed->redline_rpm) tq = fminf(tq, 0.0f);
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
  float s_target = 0.15f + 0.05f * expf(-v * 0.5f); /* a bit more slip allowed at very low speed */
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
static void lcc__esc_update(lcc_car_t *car, float dt) {
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
  memset(car, 0, sizeof(*car));

  car->desc        = *desc;
  car->wheel_count = car->desc.wheel_count;
  car->env         = desc->environment;

  snprintf(car->version_str, sizeof(car->version_str), "%d.%d.%d", LCC_VERSION_MAJOR, LCC_VERSION_MINOR, LCC_VERSION_PATCH);

  lcc__init_runtime(car);
  return car;
}

void lcc_car_destroy(lcc_car_t *car) {
  if(!car) return;
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

lcc_result_t lcc_car_set_wheel_contact(lcc_car_t *car, int wheel_index, const lcc_contact_patch_t *contact) {
  if(!car || !contact) return LCC_ERR_INVALID_ARG;
  if(wheel_index < 0 || wheel_index >= car->wheel_count) return LCC_ERR_BOUNDS;
  car->wheel_states[wheel_index].contact = *contact;
  return LCC_OK;
}

lcc_result_t lcc_car_get_wheel_contact(const lcc_car_t *car, int wheel_index, lcc_contact_patch_t *contact_out) {
  if(!car || !contact_out) return LCC_ERR_INVALID_ARG;
  if(wheel_index < 0 || wheel_index >= car->wheel_count) return LCC_ERR_BOUNDS;
  *contact_out = car->wheel_states[wheel_index].contact;
  return LCC_OK;
}

lcc_result_t lcc_car_set_engine_map(lcc_car_t *car, const lcc_curve1d_t *wot_torque, const lcc_curve1d_t *friction) {
  if(!car) return LCC_ERR_INVALID_ARG;
  if(wot_torque) car->desc.engine.wot_torque_nm_vs_rpm = *wot_torque;
  if(friction) car->desc.engine.friction_torque_nm_vs_rpm = *friction;
  return LCC_OK;
}

lcc_result_t lcc_car_set_ignition_map(lcc_car_t *car, const lcc_map2d_t *spark) {
  if(!car || !spark) return LCC_ERR_INVALID_ARG;
  car->desc.ignition.spark_advance_deg_vs_rpm_load = *spark;
  return LCC_OK;
}

lcc_result_t lcc_car_set_boost_map(lcc_car_t *car, const lcc_map2d_t *boost) {
  if(!car || !boost) return LCC_ERR_INVALID_ARG;
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

lcc_result_t lcc_car_set_diff_params(lcc_car_t *car, lcc_diff_type_t front, lcc_diff_type_t rear, lcc_diff_type_t center, float preload_nm,
  float bias_ratio) {
  if(!car) return LCC_ERR_INVALID_ARG;
  car->desc.driveline.front_diff.type        = front;
  car->desc.driveline.rear_diff.type         = rear;
  car->desc.driveline.center_diff.type       = center;
  car->desc.driveline.front_diff.preload_nm  = preload_nm;
  car->desc.driveline.rear_diff.preload_nm   = preload_nm;
  car->desc.driveline.center_diff.preload_nm = preload_nm * 0.5f;
  car->desc.driveline.front_diff.bias_ratio  = bias_ratio;
  car->desc.driveline.rear_diff.bias_ratio   = bias_ratio;
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

lcc_result_t lcc_car_set_suspension_params(lcc_car_t *car, int wheel_index, const lcc_suspension_desc_t *sus) {
  if(!car || !sus) return LCC_ERR_INVALID_ARG;
  if(wheel_index < 0 || wheel_index >= car->wheel_count) return LCC_ERR_BOUNDS;
  car->desc.suspension[wheel_index] = *sus;
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

/* powertrain controls */
lcc_result_t lcc_car_engine_start(lcc_car_t *car) {
  if(!car) return LCC_ERR_INVALID_ARG;
  if(car->engine_state.running) return LCC_OK;
  if(car->elec_state.battery_soc <= 0.05f) return LCC_ERR_BAD_STATE;
  if(car->fuel_state.fuel_l <= 0.01f) return LCC_ERR_BAD_STATE;

  car->engine_state.running = 1;
  car->engine_state.stalled = 0;
  if(car->engine_state.rpm < car->desc.engine.idle_rpm * 0.8f) car->engine_state.rpm = car->desc.engine.idle_rpm * 0.8f;
  lcc__emit_event(car, LCC_EVENT_ENGINE_START, 0, 0.0f);
  return LCC_OK;
}

lcc_result_t lcc_car_engine_stop(lcc_car_t *car) {
  if(!car) return LCC_ERR_INVALID_ARG;
  if(!car->engine_state.running) return LCC_OK;
  car->engine_state.running       = 0;
  car->engine_state.torque_nm     = 0.0f;
  car->engine_state.fuel_rate_lph = 0.0f;
  lcc__emit_event(car, LCC_EVENT_ENGINE_STOP, 0, 0.0f);
  return LCC_OK;
}

lcc_result_t lcc_car_request_gear(lcc_car_t *car, int gear_index) {
  if(!car) return LCC_ERR_INVALID_ARG;
  if(gear_index < 0 || gear_index >= car->desc.transmission.gear_count) return LCC_ERR_BOUNDS;
  car->pending_gear_index   = gear_index;
  car->trans_state.shifting = 1;
  car->shift_timer_s        = lcc__clampf(car->desc.transmission.shift_time_s, 0.05f, 1.0f);
  printf("Requesting gear %d, shift_timer_s: %.2f\n", gear_index, car->shift_timer_s);
  return LCC_OK;
}

lcc_result_t lcc_car_shift_up(lcc_car_t *car) {
  if(!car) return LCC_ERR_INVALID_ARG;
  int next = car->trans_state.gear_index + 1;
  if(next >= car->desc.transmission.gear_count) next = car->desc.transmission.gear_count - 1;
  printf("Shifting up to %d\n", next);
  return lcc_car_request_gear(car, next);
}

lcc_result_t lcc_car_shift_down(lcc_car_t *car) {
  if(!car) return LCC_ERR_INVALID_ARG;
  int next     = car->trans_state.gear_index - 1;
  int min_gear = car->desc.transmission.type != LCC_TRANS_MANUAL ? 2 : 0;
  if(next < min_gear) next = min_gear;
  printf("Shifting down to %d\n", next);
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
  car->total_mass_kg     = car->car_state.mass_kg;
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
  car->total_mass_kg     = car->car_state.mass_kg;
  lcc__compute_static_loads(car);
  return LCC_OK;
}

lcc_result_t lcc_car_recharge_battery(lcc_car_t *car, float state_of_charge_0_to_1) {
  if(!car) return LCC_ERR_INVALID_ARG;
  car->elec_state.battery_soc       = lcc__saturate(state_of_charge_0_to_1);
  car->elec_state.battery_voltage_v = car->desc.battery.nominal_voltage_v;
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

/* pose and velocity */
void lcc_car_set_pose(lcc_car_t *car, const float pos_world[2], float yaw_rad) {
  if(!car || !pos_world) return;
  car->car_state.pos_world[0] = pos_world[0];
  car->car_state.pos_world[1] = pos_world[1];
  car->car_state.yaw_rad      = yaw_rad;
}

void lcc_car_get_pose(const lcc_car_t *car, float pos_world_out[2], float *yaw_rad_out) {
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
  car->controls.steer     = lcc__clampf(car->controls.steer, -1.0f, 1.0f);
  car->controls.handbrake = lcc__saturate(car->controls.handbrake);
}

void lcc_car_get_controls(const lcc_car_t *car, lcc_controls_t *controls_out) {
  if(!car || !controls_out) return;
  *controls_out = car->controls;
}

/* braking torque command with thermal/wear/health and stable standstill gating */
static float lcc__wheel_brake_torque_base(const lcc_car_t *car, int i) {
  const lcc_brake_desc_t *br = &car->desc.brakes[i];
  if(!car->desc.wheels[i].has_brake) return 0.0f;

  float pedal = lcc__saturate(car->controls.brake);
  if(car->controls.handbrake > 0.0f && car->desc.wheels[i].position_local[0] < 0.0f) pedal = fmaxf(pedal, lcc__saturate(car->controls.handbrake));

  const float Pmax_kpa                              = 12000.0f;
  float       pressure_kpa                          = pedal * Pmax_kpa;
  ((lcc_car_t *)car)->brake_state.line_pressure_kpa = pressure_kpa;

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

  /* Slip-speed-driven activation to avoid oscillations at standstill */
  float r = fmaxf(0.05f, car->desc.wheels[i].radius_m);
  float vtire[2];
  lcc__wheel_vel_tire_frame(car, i, vtire);
  float Vx     = vtire[0];
  float omega  = car->wheel_states[i].omega_radps;
  float slip_v = fabsf(omega * r - Vx);

  const float v_on   = 0.02f; /* ~2 cm/s */
  const float v_full = 0.20f; /* ~20 cm/s */
  float       act    = lcc__saturate((slip_v - v_on) * lcc__safe_inv((v_full - v_on), 1e-6f));

  /* in neutral near stop: greatly limit brake torque to avoid excitation */
  int gi = ((const lcc_car_t *)car)->trans_state.gear_index;
  int in_neutral =
    (gi == LCC_GEAR_NEUTRAL) || (gi >= 0 && gi < car->desc.transmission.gear_count && fabsf(car->desc.transmission.gear_ratios[gi]) < 1e-3f);

  if(in_neutral && car->car_state.speed_mps < 0.30f && fabsf(omega * r) < 0.30f) act = fminf(act, 0.10f);

  T_cmd *= act;
  return T_cmd;
}

/* suspension outputs for telemetry (quasi-static target tracking) */
static void lcc__suspension_update(lcc_car_t *car, float dt) {
  for(int i = 0; i < car->wheel_count; ++i) {
    const lcc_suspension_desc_t *sd         = &car->desc.suspension[i];
    float                        k          = fmaxf(1000.0f, sd->spring_rate_n_per_m);
    float                        c_comp     = fmaxf(0.0f, sd->damper_c_compression_n_per_mps);
    float                        c_reb      = fmaxf(0.0f, sd->damper_c_rebound_n_per_mps);
    float                        travel_max = fmaxf(0.05f, sd->travel_m);
    float                        static_F   = car->wheel_static_load_n[i];
    float                        Fz         = car->wheel_states[i].normal_force_n;
    float                        x_target   = lcc__clampf((Fz - static_F) / k, -0.5f * travel_max, 0.5f * travel_max);
    float                        x          = car->susp_travel[i];
    float                        v          = car->susp_vel[i];
    float                        rate       = 10.0f; /* how fast it tracks */
    float                        dx         = (x_target - x) * rate;
    v += dx * dt;
    x += v * dt;
    x                                 = lcc__clampf(x, -0.5f * travel_max, 0.5f * travel_max);
    float c                           = (v >= 0.0f) ? c_comp : c_reb;
    float Fspring                     = k * x * sd->motion_ratio;
    float Fdamper                     = c * v * sd->motion_ratio;
    car->susp_travel[i]               = x;
    car->susp_vel[i]                  = v;
    car->susp_state.travel_m[i]       = x;
    car->susp_state.spring_force_n[i] = Fspring;
    car->susp_state.damper_force_n[i] = Fdamper;
  }
  /* approximate roll from lateral load difference */
  float FzL = 0.0f, FzR = 0.0f;
  for(int i = 0; i < car->wheel_count; ++i)
    if(car->desc.wheels[i].position_local[1] < 0.0f) FzL += car->wheel_states[i].normal_force_n;
    else
      FzR += car->wheel_states[i].normal_force_n;
  float dF                            = FzR - FzL;
  float roll_stiff                    = car->desc.arbs.front_rate_n_per_rad + car->desc.arbs.rear_rate_n_per_rad + 5000.0f;
  car->susp_state.roll_angle_rad      = dF / fmaxf(1000.0f, roll_stiff);
  car->susp_state.arb_torque_front_nm = car->desc.arbs.front_rate_n_per_rad * car->susp_state.roll_angle_rad;
  car->susp_state.arb_torque_rear_nm  = car->desc.arbs.rear_rate_n_per_rad * car->susp_state.roll_angle_rad;
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
    float wear_rate = car->desc.tires[i].wear_rate;
    car->wheel_states[i].tire_wear =
      lcc__clampf(car->wheel_states[i].tire_wear + wear_rate * (0.00005f * slip_speed + 0.00002f * fmaxf(0.0f, T - 90.0f)) * dt, 0.0f, 1.0f);
    car->damage_state.tire_health[i] = lcc__clampf(1.0f - car->wheel_states[i].tire_wear, 0.2f, 1.0f);
  }

  /* engine cooling */
  float Tcool = car->cool_state.coolant_temp_c;
  float Toil  = car->cool_state.oil_temp_c;
  float Tamb  = car->env.ambient_temp_c;
  float UA    = fmaxf(1.0f, car->desc.cooling.radiator_ua_w_per_k) * (1.0f + 0.02f * car->car_state.speed_mps);
  float fan   = (Tcool > car->desc.cooling.fan_on_c) ? 1.0f : 0.0f;
  UA += fan * 100.0f;

  float engine_kw = fmaxf(0.0f, engine_power_kw);
  float waste_kw  = engine_kw * 0.8f; /* lots of waste to heat */

  float Cc = fmaxf(1000.0f, car->desc.engine.coolant_heat_capacity_j_per_k) / 1000.0f; /* kJ/K */
  float Co = fmaxf(1000.0f, car->desc.engine.oil_heat_capacity_j_per_k) / 1000.0f;     /* kJ/K */

  float dTcool = (waste_kw - UA * (Tcool - Tamb) / 1000.0f) / Cc;
  float dToil  = (0.5f * waste_kw - 0.3f * UA * (Toil - Tamb) / 1000.0f) / Co;

  Tcool += dTcool * dt;
  Toil += dToil * dt;

  car->cool_state.coolant_temp_c  = Tcool;
  car->cool_state.oil_temp_c      = Toil;
  car->cool_state.radiator_heat_w = UA * (Tcool - Tamb);
  car->cool_state.fan_on          = fan > 0.0f;

  /* exhaust catalyst */
  float catT  = car->exh_state.cat_temp_c;
  float dTcat = (engine_kw * 0.3f - 30.0f * (catT - Tamb)) / 100.0f;
  catT += dTcat * dt;
  car->exh_state.cat_temp_c = lcc__clampf(catT, Tamb, 950.0f);

  /* overheat event throttle */
  if(Tcool > 115.0f && (car->car_state.time_s - car->last_overheat_evt_time) > 1.0) {
    car->last_overheat_evt_time = car->car_state.time_s;
    lcc__emit_event(car, LCC_EVENT_OVERHEAT, 0, Tcool);
  }
}

lcc_result_t lcc_car_step(lcc_car_t *car, float dt_s) {
  if(!car) return LCC_ERR_INVALID_ARG;
  if(!(dt_s > 0.0f) || dt_s > 0.1f) return LCC_ERR_INVALID_ARG;

  car->car_state.dt_s = dt_s;
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
  if(car->trans_state.gear_index >= 0 && car->trans_state.gear_index < car->desc.transmission.gear_count)
    gear_ratio = car->desc.transmission.gear_ratios[car->trans_state.gear_index];
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
  float avg_omega             = (count_driven > 0) ? (sum_omega_driven / (float)count_driven) : 0.0f;
  float shaft_out_rpm         = lcc__radps_to_rpm(avg_omega) * fmaxf(0.1f, final_drive);
  car->trans_state.output_rpm = shaft_out_rpm;

  /* engine model */
  float tq_engine = 0.0f;
  if(!car->engine_state.running) {
    if(car->controls.ignition_switch && car->controls.starter && car->elec_state.battery_soc > 0.05f && car->fuel_state.fuel_l > 0.01f)
      car->engine_state.rpm = lcc__lp(car->engine_state.rpm, car->desc.engine.idle_rpm, 4.0f, dt_s);
  } else {
    float throttle_eff = lcc__saturate(car->controls.throttle);

    /* idle control */
    if(car->desc.ecu.idle_control && throttle_eff < 0.05f) {
      float rpm_target = car->desc.engine.idle_rpm;
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
    car->engine_state.throttle_effective = throttle_eff;

    tq_engine                   = lcc__engine_torque_compute(car, car->engine_state.rpm, throttle_eff, &car->engine_state.manifold_pressure_kpa);
    car->engine_state.torque_nm = tq_engine;

    if(car->fuel_state.fuel_l <= 1e-3f) {
      tq_engine                 = 0.0f;
      car->engine_state.running = 0;
      lcc__emit_event(car, LCC_EVENT_FUEL_STARVATION, 0, 0.0f);
    }
  }

  /* clutch/TC converter to trans input */
  float eng_rpm    = car->engine_state.rpm;
  float T_to_trans = 0.0f;
  if(car->desc.transmission.clutch == LCC_CLUTCH_TORQUE_CONVERTER) {
    float target_input_rpm          = (fabsf(gear_ratio) > 1e-3f) ? gear_ratio * shaft_out_rpm : 0.0f;
    float SR                        = (eng_rpm > 1.0f) ? lcc__clampf(target_input_rpm / fmaxf(1.0f, eng_rpm), 0.0f, 1.0f) : 0.0f;
    float stall_ratio               = 2.2f;
    float mult                      = 1.0f + (stall_ratio - 1.0f) * (1.0f - SR);
    float K                         = fmaxf(10.0f, car->desc.transmission.converter_k_factor);
    float T_cap                     = (eng_rpm / K);
    T_cap                           = T_cap * T_cap;
    T_to_trans                      = fminf(tq_engine * mult, T_cap);
    car->trans_state.converter_slip = 1.0f - SR;
  } else {
    float clutch_k = lcc__saturate(car->trans_state.clutch_engagement);
    float T_cap    = 800.0f * clutch_k * car->damage_state.transmission_health;
    T_to_trans     = tq_engine;
    if(fabsf(T_to_trans) > T_cap) T_to_trans = lcc__signf(T_to_trans) * T_cap;
    car->trans_state.converter_slip = 0.0f;
  }

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
  float Fy0_abs[LCC_MAX_WHEELS];
  float vt_x[LCC_MAX_WHEELS], vt_y[LCC_MAX_WHEELS];
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
    lcc__tire_stiffness(&car->desc.tires[i], Fz_i[i], &Cx_i[i], &Cy_i[i]);

    float Fy0  = -Cy_i[i] * tanf(a_i[i]);
    Fy0_abs[i] = fabsf(Fy0);
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

  lcc_axle_torques_t axleF = (lcc_axle_torques_t){ 0 }, axleR = (lcc_axle_torques_t){ 0 };
  /* front axle */
  if(front_has && fiL >= 0 && fiR >= 0) {
    float           TlimL = lcc__wheel_torque_limit_from_circle_idx(fiL, mu_i, Fz_i, Fy0_abs, r_i);
    float           TlimR = lcc__wheel_torque_limit_from_circle_idx(fiR, mu_i, Fz_i, Fy0_abs, r_i);
    lcc_diff_desc_t d     = car->desc.driveline.front_diff;
    switch(d.type) {
    case LCC_DIFF_OPEN: axleF = lcc__diff_open(Tw_front, TlimL, TlimR); break;
    case LCC_DIFF_LOCKED: axleF = lcc__diff_locked(Tw_front, TlimL, TlimR); break;
    case LCC_DIFF_TORSEN: axleF = lcc__diff_torsen(Tw_front, TlimL, TlimR, d.bias_ratio, d.preload_nm); break;
    case LCC_DIFF_LSD_CLUTCH: axleF = lcc__diff_clutch_lsd(Tw_front, TlimL, TlimR, d.bias_ratio, d.preload_nm); break;
    case LCC_DIFF_ACTIVE:
    default: axleF = lcc__diff_active(Tw_front, TlimL, TlimR, d.lock_coef, d.preload_nm); break;
    }
  }

  /* rear axle */
  if(rear_has && riL >= 0 && riR >= 0) {
    float           TlimL = lcc__wheel_torque_limit_from_circle_idx(riL, mu_i, Fz_i, Fy0_abs, r_i);
    float           TlimR = lcc__wheel_torque_limit_from_circle_idx(riR, mu_i, Fz_i, Fy0_abs, r_i);
    lcc_diff_desc_t d     = car->desc.driveline.rear_diff;
    switch(d.type) {
    case LCC_DIFF_OPEN: axleR = lcc__diff_open(Tw_rear, TlimL, TlimR); break;
    case LCC_DIFF_LOCKED: axleR = lcc__diff_locked(Tw_rear, TlimL, TlimR); break;
    case LCC_DIFF_TORSEN: axleR = lcc__diff_torsen(Tw_rear, TlimL, TlimR, d.bias_ratio, d.preload_nm); break;
    case LCC_DIFF_LSD_CLUTCH: axleR = lcc__diff_clutch_lsd(Tw_rear, TlimL, TlimR, d.bias_ratio, d.preload_nm); break;
    case LCC_DIFF_ACTIVE:
    default: axleR = lcc__diff_active(Tw_rear, TlimL, TlimR, d.lock_coef, d.preload_nm); break;
    }
  }
  float Tw_cmd[LCC_MAX_WHEELS] = { 0 };
  if(fiL >= 0) Tw_cmd[fiL] = axleF.left_nm;
  if(fiR >= 0) Tw_cmd[fiR] = axleF.right_nm;
  if(riL >= 0) Tw_cmd[riL] = axleR.left_nm;
  if(riR >= 0) Tw_cmd[riR] = axleR.right_nm;

  /* ESC yaw update (computes extra brake torque on outside) */
  lcc__esc_update(car, dt_s);

  /* integrate wheels and accumulate body forces/moment */
  float F_body_sum[2] = { 0.0f, 0.0f };
  float Mz            = 0.0f;
  float max_pos_slip  = 0.0f;

  for(int i = 0; i < car->wheel_count; ++i) {
    const lcc_wheel_desc_t *wd = &car->desc.wheels[i];
    const lcc_tire_desc_t  *td = &car->desc.tires[i];
    lcc_wheel_state_t      *ws = &car->wheel_states[i];

    float Vx = vt_x[i], Vy = vt_y[i];
    float r  = r_i[i];
    float Fz = Fz_i[i];
    float mu = mu_i[i];
    float Cx = Cx_i[i], Cy = Cy_i[i];

    float s = s_i[i];
    float a = a_i[i];
    if(wd->driven && s > max_pos_slip) max_pos_slip = s;

    /* base forces */
    float Fx0 = Cx * s;
    float Fy0 = -Cy * tanf(a);

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
        float Fcap = mu * Fz;
        float nx = Vx_slip / vmag, ny = Vy_slip / vmag;
        float Fx_ls = -Fcap * nx;
        float Fy_ls = -Fcap * ny;
        float b     = lcc__saturate((0.25f - car->car_state.speed_mps) / 0.25f);
        Fx0         = lcc__lerp(Fx0, Fx_ls, b);
        Fy0         = lcc__lerp(Fy0, Fy_ls, b);
      }
    }

    /* friction circle cap */
    float Fcap = mu * Fz;
    lcc__friction_project(&Fx0, &Fy0, Fcap);

    /* brake torque + ESC */
    float Tbr_base = lcc__wheel_brake_torque_base(car, i);
    float Tbr      = Tbr_base + car->esc_extra_brake[i];
    Tbr            = lcc__abs_apply(car, i, Tbr, s, dt_s);

    /* choose brake sign to oppose rolling slip */
    float slip_w   = ws->omega_radps * r - Vx;
    float Tbr_sign = (fabsf(slip_w) > 0.05f) ?
      lcc__signf(slip_w) :
      ((fabsf(ws->omega_radps) > 0.5f) ? lcc__signf(ws->omega_radps) : (ws->brake_torque_nm >= 0.0f ? 1.0f : -1.0f));

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
      if(car->car_state.speed_mps < 0.2f && fabsf(Vx) < 0.2f && fabsf(ws->omega_radps * r) < 0.2f) s_te = 0.0f;

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
  float ax      = F_body_sum[0] / fmaxf(1.0f, car->car_state.mass_kg);
  float ay      = F_body_sum[1] / fmaxf(1.0f, car->car_state.mass_kg);
  float Izz     = fmaxf(1.0f, car->desc.chassis.inertia_zz);
  float yaw_acc = Mz / Izz;

  car->car_state.vel_body[0] += ax * dt_s;
  car->car_state.vel_body[1] += ay * dt_s;

  float vel_world_new[2];
  lcc__v2_rot(vel_world_new, car->car_state.vel_body, car->car_state.yaw_rad);
  car->car_state.vel_world[0] = vel_world_new[0];
  car->car_state.vel_world[1] = vel_world_new[1];

  car->car_state.pos_world[0] += car->car_state.vel_world[0] * dt_s;
  car->car_state.pos_world[1] += car->car_state.vel_world[1] * dt_s;

  car->car_state.yaw_rate_radps += yaw_acc * dt_s;
  car->car_state.yaw_rad += car->car_state.yaw_rate_radps * dt_s;

  car->car_state.speed_mps   = lcc__v2_len(car->car_state.vel_world);
  car->car_state.acc_body[0] = ax;
  car->car_state.acc_body[1] = ay;

  float acc_world_new[2];
  lcc__v2_rot(acc_world_new, car->car_state.acc_body, car->car_state.yaw_rad);
  car->car_state.acc_world[0] = acc_world_new[0];
  car->car_state.acc_world[1] = acc_world_new[1];

  car->car_state.beta_rad = atanf(car->car_state.vel_body[1] / fmaxf(0.001f, car->car_state.vel_body[0]));

  car->trans_state.input_rpm  = car->engine_state.rpm;
  car->trans_state.output_rpm = shaft_out_rpm;

  /* engine speed dynamics */
  float eng_omega    = lcc__rpm_to_radps(car->engine_state.rpm);
  float load_omega   = lcc__rpm_to_radps(fabsf(gear_ratio * shaft_out_rpm));
  float clutch_k     = (car->desc.transmission.clutch == LCC_CLUTCH_TORQUE_CONVERTER) ? 0.5f : lcc__saturate(car->trans_state.clutch_engagement);
  float Ieng         = fmaxf(0.02f, car->desc.engine.inertia_kgm2);
  float domega_free  = (tq_engine - T_to_trans) / Ieng;
  float domega_track = (load_omega - eng_omega) * (6.0f * clutch_k);

  eng_omega += (domega_free * (1.0f - clutch_k) + domega_track) * dt_s;
  car->engine_state.rpm = fmaxf(0.0f, lcc__radps_to_rpm(eng_omega));

  /* stall detection */
  if(car->engine_state.running) {
    int in_gear    = (car->trans_state.gear_index != LCC_GEAR_NEUTRAL) && (fabsf(gear_ratio) > 1e-3f);
    int clutch_eng = (car->trans_state.clutch_engagement > 0.7f);
    if(car->engine_state.rpm < car->desc.engine.stall_rpm && (car->engine_state.throttle_effective < 0.05f) && (in_gear && clutch_eng)) {
      car->engine_state.running   = 0;
      car->engine_state.stalled   = 1;
      car->engine_state.torque_nm = 0.0f;
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
    car->wheel_states[i].patch_world_pos[0] = world[0];
    car->wheel_states[i].patch_world_pos[1] = world[1];
  }

  /* aero state */
  car->aero_state.drag_force_world[0] = drag_world[0];
  car->aero_state.drag_force_world[1] = drag_world[1];
  car->aero_state.downforce_n_front   = df_front;
  car->aero_state.downforce_n_rear    = df_rear;

  /* suspension telemetry */
  lcc__suspension_update(car, dt_s);

  /* electrics */
  float base_consumers_w = 150.0f;
  if(car->controls.starter && !car->engine_state.running) base_consumers_w += car->desc.starter.power_w;
  if(car->elec_state.consumers_headlights) base_consumers_w += 110.0f;
  float V      = car->desc.battery.nominal_voltage_v;
  float I_load = base_consumers_w / fmaxf(V, 1.0f);
  float I_alt  = 0.0f;
  if(car->engine_state.rpm >= car->desc.alternator.cut_in_rpm && car->engine_state.running) {
    float frac = lcc__clampf((car->engine_state.rpm - car->desc.alternator.cut_in_rpm) / 2000.0f, 0.0f, 1.0f);
    I_alt      = car->desc.alternator.max_current_a * frac;
  }
  float I_net                          = I_alt - I_load;
  car->elec_state.alternator_current_a = I_alt;
  car->elec_state.battery_current_a    = -I_net;
  float capacity_as                    = car->desc.battery.capacity_ah * 3600.0f;
  if(capacity_as > 1.0f) car->elec_state.battery_soc = lcc__clampf(car->elec_state.battery_soc + (I_net * dt_s) / capacity_as, 0.0f, 1.0f);

  /* fuel consumption */
  float power_kw                  = fmaxf(0.0f, tq_engine) * eng_omega / 1000.0f;
  float bsfc_l_per_kwh            = 0.32f;
  float fuel_lps                  = (power_kw * bsfc_l_per_kwh) / 3600.0f;
  float idle_lps                  = (car->engine_state.running && car->controls.throttle < 0.02f) ? 0.00015f : 0.0f;
  float fuel_use                  = (fuel_lps + idle_lps) * dt_s;
  car->engine_state.fuel_rate_lph = (fuel_lps + idle_lps) * 3600.0f;
  car->fuel_state.fuel_l          = fmaxf(0.0f, car->fuel_state.fuel_l - fuel_use);

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

  /* parking hold */
  if(car->controls.brake > 0.6f && car->car_state.speed_mps < 0.03f) {
    car->car_state.vel_body[0] = car->car_state.vel_body[1] = 0.0f;
    car->car_state.vel_world[0] = car->car_state.vel_world[1] = 0.0f;
    car->car_state.yaw_rate_radps                             = 0.0f;
    for(int i = 0; i < car->wheel_count; ++i) car->wheel_states[i].omega_radps = 0.0f;
  }

  return LCC_OK;
}

/* telemetry and states getters */
void lcc_car_get_telemetry(const lcc_car_t *car, lcc_telemetry_t *out) {
  if(!car || !out) return;
  out->car          = car->car_state;
  out->engine       = car->engine_state;
  out->transmission = car->trans_state;
  out->brakes       = car->brake_state;
  out->suspension   = car->susp_state;
  out->electrics    = car->elec_state;
  out->fuel         = car->fuel_state;
  out->cooling      = car->cool_state;
  out->exhaust      = car->exh_state;
  out->aero         = car->aero_state;
  out->wheel_count  = car->wheel_count;
  for(int i = 0; i < car->wheel_count; ++i) out->wheels[i] = car->wheel_states[i];
}

void lcc_car_get_state(const lcc_car_t *car, lcc_car_state_t *out) {
  if(!car || !out) return;
  *out = car->car_state;
}

void lcc_car_get_engine_state(const lcc_car_t *car, lcc_engine_state_t *out) {
  if(!car || !out) return;
  *out = car->engine_state;
}

void lcc_car_get_ignition_state(const lcc_car_t *car, lcc_ignition_state_t *out) {
  if(!car || !out) return;
  *out = car->ign_state;
}

void lcc_car_get_transmission_state(const lcc_car_t *car, lcc_transmission_state_t *out) {
  if(!car || !out) return;
  *out = car->trans_state;
}

void lcc_car_get_brake_state(const lcc_car_t *car, lcc_brake_state_t *out) {
  if(!car || !out) return;
  *out = car->brake_state;
}

void lcc_car_get_suspension_state(const lcc_car_t *car, lcc_suspension_state_t *out) {
  if(!car || !out) return;
  *out = car->susp_state;
}

void lcc_car_get_electrics_state(const lcc_car_t *car, lcc_electrics_state_t *out) {
  if(!car || !out) return;
  *out = car->elec_state;
}

void lcc_car_get_fuel_state(const lcc_car_t *car, lcc_fuel_state_t *out) {
  if(!car || !out) return;
  *out = car->fuel_state;
}

void lcc_car_get_cooling_state(const lcc_car_t *car, lcc_cooling_state_t *out) {
  if(!car || !out) return;
  *out = car->cool_state;
}

void lcc_car_get_exhaust_state(const lcc_car_t *car, lcc_exhaust_state_t *out) {
  if(!car || !out) return;
  *out = car->exh_state;
}

void lcc_car_get_aero_state(const lcc_car_t *car, lcc_aero_state_t *out) {
  if(!car || !out) return;
  *out = car->aero_state;
}

void lcc_car_get_wheel_state(const lcc_car_t *car, int wheel_index, lcc_wheel_state_t *out) {
  if(!car || !out || wheel_index < 0 || wheel_index >= car->wheel_count) return;
  *out = car->wheel_states[wheel_index];
}

void lcc_car_get_damage_state(const lcc_car_t *car, lcc_damage_state_t *out) {
  if(!car || !out) return;
  *out = car->damage_state;
}

void lcc_car_set_damage_state(lcc_car_t *car, const lcc_damage_state_t *in) {
  if(!car || !in) return;
  car->damage_state = *in;
}

/* ========================= remaining API implementations ========================= */

#include <ctype.h> /* tolower, isdigit */

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

/* damage helpers */

static int lcc__find_wheel_by_loc(const lcc_car_t *car, lcc_wheel_loc_t loc) {
  if(!car) return -1;
  for(int i = 0; i < car->wheel_count; ++i)
    if(car->desc.wheels[i].location == loc) return i;
  return -1;
}

static void lcc__apply_to_wheels(lcc_car_t *car, const char *scope, void (*fn)(lcc_car_t *, int, float), float factor) {
  if(!car || !scope || !fn) return;

  /* lowercase copy */
  char   s[64] = { 0 };
  size_t n     = strlen(scope);
  if(n >= sizeof(s)) n = sizeof(s) - 1;
  for(size_t i = 0; i < n; ++i) s[i] = (char)tolower((unsigned char)scope[i]);

  /* try explicit index first */
  int idx_from_digit = -1;
  for(size_t i = 0; i < n; ++i) {
    if(isdigit((unsigned char)s[i])) {
      idx_from_digit = atoi(&s[i]);
      break;
    }
  }

  if(idx_from_digit >= 0 && idx_from_digit < car->wheel_count) {
    fn(car, idx_from_digit, factor);
    return;
  }

  /* named positions */
  int idx = -1;
  if(strstr(s, "fl")) idx = lcc__find_wheel_by_loc(car, LCC_WHEEL_FL);
  else if(strstr(s, "fr"))
    idx = lcc__find_wheel_by_loc(car, LCC_WHEEL_FR);
  else if(strstr(s, "rl"))
    idx = lcc__find_wheel_by_loc(car, LCC_WHEEL_RL);
  else if(strstr(s, "rr"))
    idx = lcc__find_wheel_by_loc(car, LCC_WHEEL_RR);

  if(idx >= 0) {
    fn(car, idx, factor);
    return;
  }

  /* broader scopes: front/rear/left/right */
  int apply_front = strstr(s, "front") != NULL;
  int apply_rear  = strstr(s, "rear") != NULL;
  int apply_left  = strstr(s, "left") != NULL;
  int apply_right = strstr(s, "right") != NULL;

  int matched = 0;
  for(int i = 0; i < car->wheel_count; ++i) {
    int is_front = (car->desc.wheels[i].position_local[0] >= 0.0f);
    int is_left  = (car->desc.wheels[i].position_local[1] < 0.0f);

    if((apply_front || apply_rear || apply_left || apply_right)) {
      if((apply_front && !is_front) || (apply_rear && is_front)) continue;
      if((apply_left && !is_left) || (apply_right && is_left)) continue;
      fn(car, i, factor);
      matched = 1;
    }
  }

  if(!matched) {
    /* apply to all wheels */
    for(int i = 0; i < car->wheel_count; ++i) fn(car, i, factor);
  }
}

static void lcc__apply_brake(lcc_car_t *car, int i, float f) {
  car->damage_state.brake_health[i] = lcc__clampf(car->damage_state.brake_health[i] * lcc__saturate(f), 0.0f, 1.0f);
}

static void lcc__apply_tire(lcc_car_t *car, int i, float f) {
  car->damage_state.tire_health[i] = lcc__clampf(car->damage_state.tire_health[i] * lcc__saturate(f), 0.0f, 1.0f);
}

static void lcc__apply_susp(lcc_car_t *car, int i, float f) {
  car->damage_state.suspension_health[i] = lcc__clampf(car->damage_state.suspension_health[i] * lcc__saturate(f), 0.0f, 1.0f);
}

void lcc_car_apply_damage_factor(lcc_car_t *car, const char *subsystem, float factor_0_to_1) {
  if(!car || !subsystem) return;
  float f = lcc__saturate(factor_0_to_1);

  /* lowercase copy */
  char   s[64] = { 0 };
  size_t n     = strlen(subsystem);
  if(n >= sizeof(s)) n = sizeof(s) - 1;
  for(size_t i = 0; i < n; ++i) s[i] = (char)tolower((unsigned char)subsystem[i]);

  if(strstr(s, "engine")) {
    car->damage_state.engine_health = lcc__clampf(car->damage_state.engine_health * f, 0.0f, 1.0f);
    return;
  }
  if(strstr(s, "transmission") || strstr(s, "gearbox")) {
    car->damage_state.transmission_health = lcc__clampf(car->damage_state.transmission_health * f, 0.0f, 1.0f);
    return;
  }
  if(strstr(s, "cooling") || strstr(s, "radiator")) {
    car->damage_state.cooling_health = lcc__clampf(car->damage_state.cooling_health * f, 0.0f, 1.0f);
    return;
  }

  if(strstr(s, "brake")) {
    lcc__apply_to_wheels(car, s, lcc__apply_brake, f);
    return;
  }
  if(strstr(s, "tire") || strstr(s, "tyre")) {
    lcc__apply_to_wheels(car, s, lcc__apply_tire, f);
    return;
  }
  if(strstr(s, "susp")) {
    lcc__apply_to_wheels(car, s, lcc__apply_susp, f);
    return;
  }

  /* fallback: if a bare wheel spec is provided, apply to all three systems */
  if(strstr(s, "front") || strstr(s, "rear") || strstr(s, "left") || strstr(s, "right") || strstr(s, "fl") || strstr(s, "fr") || strstr(s, "rl") ||
    strstr(s, "rr")) {
    lcc__apply_to_wheels(car, s, lcc__apply_brake, f);
    lcc__apply_to_wheels(car, s, lcc__apply_tire, f);
    lcc__apply_to_wheels(car, s, lcc__apply_susp, f);
  }
}

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

int lcc_car_get_wheel_local_positions(const lcc_car_t *car, float out_positions[][2], int max_wheels) {
  if(!car || !out_positions || max_wheels <= 0) return 0;
  int n = lcc__mini(car->wheel_count, max_wheels);
  for(int i = 0; i < n; ++i) {
    out_positions[i][0] = car->desc.wheels[i].position_local[0];
    out_positions[i][1] = car->desc.wheels[i].position_local[1];
  }
  return n;
}

/* playback */
lcc_result_t lcc_car_playback_controls(lcc_car_t *car, const lcc_control_frame_t *frames, int frame_count) {
  if(!car || !frames || frame_count <= 0) return LCC_ERR_INVALID_ARG;

  for(int i = 1; i < frame_count; ++i) {
    double t0 = frames[i - 1].time_s;
    double t1 = frames[i].time_s;
    double dt = t1 - t0;
    if(!(dt > 0.0)) return LCC_ERR_INVALID_ARG;

    /* piecewise-constant hold of previous frame's controls */
    lcc_car_set_controls(car, &frames[i - 1].controls);

    /* Step in sub-steps no larger than 0.05s, and within engine limits (<=0.1s) */
    int steps = (int)ceil(dt / 0.05);
    if(steps < 1) steps = 1;
    float dt_step = (float)(dt / (double)steps);
    if(dt_step > 0.1f) {
      steps   = (int)ceil(dt / 0.1);
      dt_step = (float)(dt / (double)steps);
    }

    for(int s = 0; s < steps; ++s) {
      lcc_result_t r = lcc_car_step(car, dt_step);
      if(r != LCC_OK) return r;
    }
  }

  /* Set final controls to the last frame */
  lcc_car_set_controls(car, &frames[frame_count - 1].controls);
  return LCC_OK;
}

/* debug formatters */
int lcc_format_engine_summary(const lcc_engine_state_t *es, char *out_str, int maxlen) {
  if(!out_str || maxlen <= 0) return 0;
  if(!es) return snprintf(out_str, (size_t)maxlen, "engine: <null>");
  return snprintf(out_str, (size_t)maxlen, "engine: rpm=%.0f thr=%.0f%% tq=%.1fNm MAP=%.0fkPa cool=%.1fC oil=%.1fC egt=%.0fC fuel=%.2f L/h", es->rpm,
    100.0f * es->throttle_effective, es->torque_nm, es->manifold_pressure_kpa, es->coolant_temp_c, es->oil_temp_c, es->egt_c, es->fuel_rate_lph);
}

int lcc_format_transmission_summary(const lcc_transmission_state_t *ts, char *out_str, int maxlen) {
  if(!out_str || maxlen <= 0) return 0;
  if(!ts) return snprintf(out_str, (size_t)maxlen, "trans: <null>");
  return snprintf(out_str, (size_t)maxlen, "trans: gear=%d clutch=%.0f%% in=%.0frpm out=%.0frpm conv_slip=%.2f shifting=%s", ts->gear_index,
    100.0f * ts->clutch_engagement, ts->input_rpm, ts->output_rpm, ts->converter_slip, ts->shifting ? "yes" : "no");
}

int lcc_format_wheel_summary(const lcc_wheel_state_t *ws, char *out_str, int maxlen) {
  if(!out_str || maxlen <= 0) return 0;
  if(!ws) return snprintf(out_str, (size_t)maxlen, "wheel: <null>");
  return snprintf(out_str, (size_t)maxlen,
    "wheel: omega=%.2frad/s slip=%.1f%% alpha=%.1fdeg Fx=%.0fN Fy=%.0fN Fz=%.0fN Tdrv=%.0fNm Tbr=%.0fNm T=%.1fC wear=%.2f", ws->omega_radps,
    100.0f * ws->slip_ratio, lcc__rad2deg(ws->slip_angle_rad), ws->tire_force_long_n, ws->tire_force_lat_n, ws->normal_force_n, ws->drive_torque_nm,
    ws->brake_torque_nm, ws->tire_temp_c, ws->tire_wear);
}

#endif
