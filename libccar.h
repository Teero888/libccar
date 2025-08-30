/*
   libccar - 2d top-down car simulation api (c99 single header)
   top-level api: public functions and data structures
   prefix: lcc_
   space: 2d plane (x forward, y right), right-handed rotation (cw positive)
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
  float gravity_world[2];
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
  lcc_bool_t     auto_blip;
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
  int                     gear_index_neutral;
  int                     gear_index_reverse;
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

  lcc_bool_t request_gear_up;
  lcc_bool_t request_gear_down;
  int        request_gear_index;

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

/* serialization helpers (binary blob) */
LCC_API lcc_result_t lcc_car_serialize(const lcc_car_t *car, void *buffer, size_t buffer_size, size_t *bytes_written);
LCC_API lcc_result_t lcc_car_deserialize(lcc_car_t **car_out, const void *buffer, size_t buffer_size);

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

static int lcc__maxi(int a, int b) {
  return a > b ? a : b;
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

static void lcc__v2_set(float a[2], float x, float y) {
  a[0] = x;
  a[1] = y;
}

static void lcc__v2_copy(float a[2], const float b[2]) {
  a[0] = b[0];
  a[1] = b[1];
}

static void lcc__v2_add(float a[2], const float b[2]) {
  a[0] += b[0];
  a[1] += b[1];
}

static void lcc__v2_sub(float a[2], const float b[2]) {
  a[0] -= b[0];
  a[1] -= b[1];
}

static void lcc__v2_scale(float a[2], float s) {
  a[0] *= s;
  a[1] *= s;
}

static float lcc__v2_dot(const float a[2], const float b[2]) {
  return a[0] * b[0] + a[1] * b[1];
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

/* string util */
static int lcc__stricmp(const char *a, const char *b) {
  if(!a || !b) return (a == b) ? 0 : (a ? 1 : -1);
  while(*a && *b) {
    char ca = *a, cb = *b;
    if(ca >= 'A' && ca <= 'Z') ca = (char)(ca - 'A' + 'a');
    if(cb >= 'A' && cb <= 'Z') cb = (char)(cb - 'A' + 'a');
    if(ca != cb) return (int)((unsigned char)ca) - (int)((unsigned char)cb);
    ++a;
    ++b;
  }
  return (int)((unsigned char)*a) - (int)((unsigned char)*b);
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

  /* aids internal state */
  float abs_mod[LCC_MAX_WHEELS];
  float tc_cut; /* 0..1 cut factor applied to engine torque */
  float esc_extra_brake[LCC_MAX_WHEELS];

  /* suspension internals */
  float susp_travel[LCC_MAX_WHEELS];
  float susp_vel[LCC_MAX_WHEELS];

  /* ownership for curve/map memory when loaded via deserialize */
  lcc_curve1d_point_t *own_wot_pts;
  int                  own_wot_cnt;
  lcc_curve1d_point_t *own_fric_pts;
  int                  own_fric_cnt;
  lcc_curve1d_point_t *own_thrt_pts;
  int                  own_thrt_cnt;
  lcc_map2d_point_t   *own_boost_pts;
  int                  own_boost_cnt;
  lcc_map2d_point_t   *own_spark_pts;
  int                  own_spark_cnt;

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
  desc->abs_mode           = LCC_ABS_ON;
  desc->tc_mode            = LCC_TC_ON;
  desc->esc_mode           = LCC_ESC_ON;
  desc->auto_blip          = 1;
  desc->auto_clutch        = 1;
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
  /* gear_ratios: [0]=reverse, [1]=1st */
  desc->gear_count = 7;
  float gr[7]      = { -3.2f, 3.1f, 2.1f, 1.5f, 1.2f, 1.0f, 0.84f };
  for(int i = 0; i < desc->gear_count; ++i) desc->gear_ratios[i] = gr[i];
  desc->gear_index_neutral  = -1;
  desc->gear_index_reverse  = 0;
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
  env->gravity_world[0]      = 0.0f;
  env->gravity_world[1]      = 0.0f;
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
  float g = 9.81f;
  if(car->wheel_count <= 0) return;

  float half_wb    = car->desc.chassis.wheelbase_m * 0.5f;
  float cg_y       = car->desc.chassis.cg_local[1];
  float front_frac = (half_wb + cg_y) / (2.0f * half_wb);
  front_frac       = lcc__clampf(front_frac, 0.0f, 1.0f);

  int front_wheels = 0, rear_wheels = 0;
  for(int i = 0; i < car->wheel_count; ++i)
    if(car->desc.wheels[i].position_local[1] >= 0.0f) front_wheels++;
    else
      rear_wheels++;

  float front_total = m * g * front_frac;
  float rear_total  = m * g * (1.0f - front_frac);

  for(int i = 0; i < car->wheel_count; ++i) {
    float axle_total            = (car->desc.wheels[i].position_local[1] >= 0.0f) ? (front_wheels ? front_total / (float)front_wheels : 0.0f) :
                                                                                    (rear_wheels ? rear_total / (float)rear_wheels : 0.0f);
    car->wheel_static_load_n[i] = axle_total;
    car->wheel_states[i].normal_force_n = axle_total;
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

  car->trans_state.gear_index        = (car->desc.transmission.gear_index_neutral >= 0) ? car->desc.transmission.gear_index_neutral : 1;
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

/* steering geometry with ackermann */
/* TODO: FIX THE BOTTOM PART */
static void lcc__apply_steering(lcc_car_t *car) {
  float in    = lcc__clampf(car->controls.steer, -1.0f, 1.0f);
  float max   = lcc__deg2rad(car->desc.steering.max_steer_deg);
  float delta = in * max;

  /* find front left/right indices */
  int   fiL = -1, fiR = -1;
  float wb    = fmaxf(0.1f, car->desc.chassis.wheelbase_m);
  float track = fmaxf(0.8f, car->desc.chassis.track_front_m);
  for(int i = 0; i < car->wheel_count; ++i) {
    if(car->desc.wheels[i].steerable) {
      if(car->desc.wheels[i].position_local[1] < 0.0f) fiL = i;
      else
        fiR = i;
    }
  }
  float inner = delta, outer = delta;
  float af = lcc__saturate(car->desc.steering.ackermann_factor);
  if(fabsf(delta) > 1e-6f && fiL >= 0 && fiR >= 0) {
    float R     = wb / fabsf(tanf(delta));
    float d_in  = atanf(wb / fmaxf(0.1f, R - track * 0.5f));
    float d_out = atanf(wb / fmaxf(0.1f, R + track * 0.5f));
    if(delta > 0.0f) { /* steer right: fr inner, fl outer */
      inner                     = lcc__lerp(delta, d_in, af);
      outer                     = lcc__lerp(delta, d_out, af);
      car->wheel_steer_rad[fiR] = inner;
      car->wheel_steer_rad[fiL] = outer;
    } else {
      inner                     = -lcc__lerp(-delta, d_in, af);
      outer                     = -lcc__lerp(-delta, d_out, af);
      car->wheel_steer_rad[fiL] = inner;
      car->wheel_steer_rad[fiR] = outer;
    }
  } else {
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
    beta = atanf(lcc__absf(vel_body[0]) / fmaxf(0.1f, lcc__absf(vel_body[1])));
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

/* compute body-frame hub velocity in tire frame */
/* TODO: critical function make sure this is correct */
static void lcc__wheel_vel_tire_frame(const lcc_car_t *car, int i, float out_tire_vel[2]) {
  float yaw        = car->car_state.yaw_rad;
  float invyaw     = -yaw;
  float r_local[2] = { car->desc.wheels[i].position_local[0], car->desc.wheels[i].position_local[1] };
  float r_world[2];
  lcc__v2_rot(r_world, r_local, yaw);
  float v_world[2]      = { car->car_state.vel_world[0], car->car_state.vel_world[1] };
  float v_spin_world[2] = { -car->car_state.yaw_rate_radps * r_world[1], car->car_state.yaw_rate_radps * r_world[0] };
  float hub_world[2]    = { v_world[0] + v_spin_world[0], v_world[1] + v_spin_world[1] };
  float hub_body[2];
  lcc__v2_rot(hub_body, hub_world, invyaw);
  float steer = car->wheel_steer_rad[i];
  float hub_tire[2];
  lcc__v2_rot(hub_tire, hub_body, -steer);
  out_tire_vel[0] = hub_tire[0];
  out_tire_vel[1] = hub_tire[1];
}

/* normal load with transfer and aero */
static void lcc__compute_normal_loads(lcc_car_t *car, float df_front, float df_rear) {
  float m       = car->car_state.mass_kg;
  float wb      = fmaxf(0.1f, car->desc.chassis.wheelbase_m);
  float track_f = fmaxf(0.8f, car->desc.chassis.track_front_m);
  float track_r = fmaxf(0.8f, car->desc.chassis.track_rear_m);
  float h       = lcc__clampf(car->desc.chassis.cg_height_m, 0.0f, 1.5f);

  float Fz_front_static = 0.0f, Fz_rear_static = 0.0f;
  for(int i = 0; i < car->wheel_count; ++i)
    if(car->desc.wheels[i].position_local[1] >= 0.0f) Fz_front_static += car->wheel_static_load_n[i];
    else
      Fz_rear_static += car->wheel_static_load_n[i];

  Fz_front_static += df_front;
  Fz_rear_static += df_rear;

  float ax = car->car_state.acc_body[1];
  float ay = car->car_state.acc_body[0];

  float dF_long      = m * ax * h / wb;
  float dF_lat_front = m * ay * h / track_f;
  float dF_lat_rear  = m * ay * h / track_r;

  float Fz_front = Fz_front_static - dF_long;
  float Fz_rear  = Fz_rear_static + dF_long;

  int front_wheels = 0, rear_wheels = 0;
  for(int i = 0; i < car->wheel_count; ++i) (car->desc.wheels[i].position_local[1] >= 0.0f) ? front_wheels++ : rear_wheels++;

  float Fz_front_per = (front_wheels > 0) ? (Fz_front / (float)front_wheels) : 0.0f;
  float Fz_rear_per  = (rear_wheels > 0) ? (Fz_rear / (float)rear_wheels) : 0.0f;

  for(int i = 0; i < car->wheel_count; ++i) {
    float base = (car->desc.wheels[i].position_local[1] >= 0.0f) ? Fz_front_per : Fz_rear_per;
    float dlat = (car->desc.wheels[i].position_local[1] >= 0.0f) ? (-lcc__signf(car->desc.wheels[i].position_local[0]) * 0.5f * dF_lat_front) :
                                                                   (-lcc__signf(car->desc.wheels[i].position_local[0]) * 0.5f * dF_lat_rear);
    float Fz   = base + dlat;
    car->wheel_states[i].normal_force_n = fmaxf(0.0f, Fz);
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
  float              half = 0.5f * T_in;
  r.left_nm               = fminf(half, Tlim_L);
  r.right_nm              = fminf(half, Tlim_R);
  r.delivered_nm          = r.left_nm + r.right_nm;
  return r;
}

static lcc_axle_torques_t lcc__diff_locked(float T_in, float Tlim_L, float Tlim_R) {
  lcc_axle_torques_t r      = { 0 };
  float              half   = 0.5f * T_in;
  float              TL     = fminf(half, Tlim_L);
  float              TR     = fminf(half, Tlim_R);
  float              rem    = T_in - (TL + TR);
  float              spareL = fmaxf(0.0f, Tlim_L - TL);
  float              dL     = fminf(spareL, 0.5f * rem);
  TL += fmaxf(0.0f, dL);
  float spareR = fmaxf(0.0f, Tlim_R - TR);
  float dR     = fminf(spareR, rem - dL);
  TR += fmaxf(0.0f, dR);
  r.left_nm      = TL;
  r.right_nm     = TR;
  r.delivered_nm = TL + TR;
  return r;
}

static lcc_axle_torques_t lcc__diff_torsen(float T_in, float Tlim_L, float Tlim_R, float bias_ratio, float preload_nm) {
  lcc_axle_torques_t r            = { 0 };
  float              B            = fmaxf(1.0f, bias_ratio);
  float              low          = fminf(Tlim_L, Tlim_R);
  float              high         = fmaxf(Tlim_L, Tlim_R);
  float              preload_each = 0.5f * preload_nm;
  float              capacity     = low * (1.0f + B) + preload_nm;
  float              T_use        = fminf(T_in, capacity);
  float              Tlow         = fminf(low + preload_each, T_use / (1.0f + B));
  float              Thigh        = fminf(high + preload_each, T_use - Tlow);
  if(Tlim_L <= Tlim_R) {
    r.left_nm  = Tlow;
    r.right_nm = Thigh;
  } else {
    r.left_nm  = Thigh;
    r.right_nm = Tlow;
  }
  r.delivered_nm = r.left_nm + r.right_nm;
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

/* abs controller per wheel */
static float lcc__abs_apply(lcc_car_t *car, int i, float T_brake_cmd, float slip, float dt) {
  if(car->desc.ecu.abs_mode == LCC_ABS_OFF) return T_brake_cmd;
  float s_target = 0.2f;
  float k_up = 4.0f, k_down = 12.0f;
  if(car->controls.brake <= 0.01f) {
    car->abs_mod[i] = 1.0f;
    return T_brake_cmd;
  }
  float mod  = car->abs_mod[i];
  float sabs = fabsf(slip);
  if(sabs > s_target) mod -= k_down * dt * (sabs - s_target);
  else
    mod += k_up * dt * (s_target - sabs);
  mod             = lcc__clampf(mod, 0.2f, 1.0f);
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

  /* choose outside wheels depending on yaw sign */
  int   left_is_outside = (Mz_cmd < 0.0f) ? 1 : 0; /* negative moment -> brake left to reduce pos yaw */
  float T_side          = lcc__absf(Mz_cmd) / fmaxf(0.2f, track) / fmaxf(0.05f, r_eff);
  T_side                = lcc__clampf(T_side, 0.0f, 1500.0f);

  for(int i = 0; i < car->wheel_count; ++i) {
    int is_left = (car->desc.wheels[i].position_local[0] < 0.0f);
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

  car->own_wot_pts   = NULL;
  car->own_wot_cnt   = 0;
  car->own_fric_pts  = NULL;
  car->own_fric_cnt  = 0;
  car->own_thrt_pts  = NULL;
  car->own_thrt_cnt  = 0;
  car->own_boost_pts = NULL;
  car->own_boost_cnt = 0;
  car->own_spark_pts = NULL;
  car->own_spark_cnt = 0;

  lcc__init_runtime(car);
  return car;
}

void lcc_car_destroy(lcc_car_t *car) {
  if(!car) return;
  /* free any owned curve/map memory */
  if(car->own_wot_pts) lcc__free(car->own_wot_pts, lcc__alloc_user);
  if(car->own_fric_pts) lcc__free(car->own_fric_pts, lcc__alloc_user);
  if(car->own_thrt_pts) lcc__free(car->own_thrt_pts, lcc__alloc_user);
  if(car->own_boost_pts) lcc__free(car->own_boost_pts, lcc__alloc_user);
  if(car->own_spark_pts) lcc__free(car->own_spark_pts, lcc__alloc_user);
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
  int next = car->trans_state.gear_index - 1;
  if(car->desc.transmission.type != LCC_TRANS_MANUAL) {
    int rev     = car->desc.transmission.gear_index_reverse;
    int neu     = car->desc.transmission.gear_index_neutral;
    int min_fwd = 0;
    for(int i = 0; i < car->desc.transmission.gear_count; ++i) {
      if(i != rev && i != neu) {
        min_fwd = i;
        break;
      }
    }
    if(next < min_fwd) next = min_fwd; /* don't auto-shift to N/R */
  } else {
    if(next < 0) next = 0;
  }
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

  if(car->controls.request_gear_index >= 0) lcc_car_request_gear(car, car->controls.request_gear_index);
  else if(car->controls.request_gear_up)
    lcc_car_shift_up(car);
  else if(car->controls.request_gear_down)
    lcc_car_shift_down(car);
}

void lcc_car_get_controls(const lcc_car_t *car, lcc_controls_t *controls_out) {
  if(!car || !controls_out) return;
  *controls_out = car->controls;
}

/* braking torque command */
static float lcc__wheel_brake_torque_base(const lcc_car_t *car, int i) {
  const lcc_brake_desc_t *br = &car->desc.brakes[i];
  if(!car->desc.wheels[i].has_brake) return 0.0f;
  float cmd = lcc__saturate(car->controls.brake);
  if(car->controls.handbrake > 0.0f && car->desc.wheels[i].position_local[1] < 0.0f) cmd = fmaxf(cmd, lcc__saturate(car->controls.handbrake));
  float T = cmd * br->max_torque_nm;
  return fmaxf(0.0f, T);
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
    if(car->desc.wheels[i].position_local[0] < 0.0f) FzL += car->wheel_states[i].normal_force_n;
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

/* torque limits per wheel from friction ellipse */
static float wheel_torque_limit(lcc_car_t *car, float mu_i[static LCC_MAX_WHEELS], float Fy0_abs[static LCC_MAX_WHEELS], int idx) {
  if(idx < 0) return 0.0f;
  float Fz       = fmaxf(0.0f, car->wheel_states[idx].normal_force_n);
  float r        = fmaxf(0.05f, car->desc.wheels[idx].radius_m);
  float Fcap     = fmaxf(0.0f, mu_i[idx] * Fz);
  float Fx_allow = fmaxf(0.0f, sqrtf(fmaxf(0.0f, Fcap * Fcap - Fy0_abs[idx] * Fy0_abs[idx])));
  return Fx_allow * r;
};

/* step integration */
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

  /* clutch engagement from control or auto-clutch during shift */
  float clutch_cmd = 1.0f - car->controls.clutch; /* driver clutch: 1 engaged */
  if(car->desc.ecu.auto_clutch && car->trans_state.shifting) {
    float half  = 0.5f * car->desc.transmission.shift_time_s;
    float t     = car->shift_timer_s;
    float phase = (t > half) ? 0.0f : (1.0f - t / fmaxf(1e-3f, half));
    clutch_cmd  = 1.0f - 0.9f * phase; /* nearly open at mid shift */
  }
  car->trans_state.clutch_engagement = lcc__clampf(clutch_cmd, 0.0f, 1.0f);

  /* steering */
  lcc__apply_steering(car);

  /* environment relative velocity for aero */
  float rel_vel_world[2] = { car->car_state.vel_world[0] - car->env.wind_world[0], car->car_state.vel_world[1] - car->env.wind_world[1] };
  float drag_world[2];
  float df_front = 0.0f, df_rear = 0.0f;
  lcc__aero_compute(car, rel_vel_world, drag_world, &df_front, &df_rear);

  /* update normal loads with transfer and aero */
  lcc__compute_normal_loads(car, df_front, df_rear);

  /* driveline ratios */
  float gear_ratio = 0.0f;
  if(car->trans_state.gear_index >= 0 && car->trans_state.gear_index < car->desc.transmission.gear_count)
    gear_ratio = car->desc.transmission.gear_ratios[car->trans_state.gear_index];
  float final_drive   = car->desc.transmission.final_drive_ratio;
  float driveline_eff = lcc__driveline_efficiency(car);

  /* wheel angular speeds -> gearbox output */
  float sum_omega_driven = 0.0f;
  int   count_driven     = 0;
  for(int i = 0; i < car->wheel_count; ++i)
    if(car->desc.wheels[i].driven) {
      sum_omega_driven += car->wheel_states[i].omega_radps;
      count_driven++;
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
    float throttle_eff = car->controls.throttle;
    /* tc engine cut */
    throttle_eff *= (1.0f - car->tc_cut);
    car->engine_state.throttle_effective = throttle_eff;

    tq_engine = lcc__engine_torque_compute(car, car->engine_state.rpm, throttle_eff, &car->engine_state.manifold_pressure_kpa);

    if(car->fuel_state.fuel_l <= 0.001f) {
      tq_engine                 = 0.0f;
      car->engine_state.running = 0;
      lcc__emit_event(car, LCC_EVENT_FUEL_STARVATION, 0, 0.0f);
    }
  }

  /* torque to trans input with clutch or converter */
  float eng_rpm    = car->engine_state.rpm;
  float T_to_trans = 0.0f;
  if(car->desc.transmission.clutch == LCC_CLUTCH_TORQUE_CONVERTER) {
    float target_input_rpm          = lcc__absf(gear_ratio) > 1e-3f ? lcc__absf(gear_ratio) * shaft_out_rpm : 0.0f;
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
    if(lcc__absf(T_to_trans) > T_cap) T_to_trans = lcc__signf(T_to_trans) * T_cap; /* TODO: what?? */
    car->trans_state.converter_slip = 0.0f;
  }

  /* driveline torque to wheels */
  float Tw_total = T_to_trans * gear_ratio * final_drive * driveline_eff;

  /* axle split */
  /* TODO: this logic is stupid, the thing just overrides, maybe remove the LCC_LAYOUT and just keep the driven wheels or the other way around */
  float splitF = 0.0f, splitR = 0.0f;
  int   front_has = 0, rear_has = 0;
  for(int i = 0; i < car->wheel_count; ++i) {
    if(car->desc.wheels[i].driven) {
      if(car->desc.wheels[i].position_local[0] >= 0.0f) front_has = 1;
      else
        rear_has = 1;
    }
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
  default:
    splitF = lcc__clampf(car->desc.driveline.front_torque_split, 0.0f, 1.0f);
    splitR = 1.0f - splitF;
    break;
  }
  if(!front_has) {
    splitR = 1.0f;
    splitF = 0.0f;
  }
  if(!rear_has) {
    splitF = 1.0f;
    splitR = 0.0f;
  }

  float Tw_front = Tw_total * splitF;
  float Tw_rear  = Tw_total * splitR;

  /* precompute lateral demand for traction limits */
  float mu_i[LCC_MAX_WHEELS];
  float Fy0_abs[LCC_MAX_WHEELS];
  for(int i = 0; i < car->wheel_count; ++i) {
    float vtire[2];
    lcc__wheel_vel_tire_frame(car, i, vtire);
    float Vlat  = vtire[0];
    float Vlong = vtire[1];
    float Fz    = fmaxf(0.0f, car->wheel_states[i].normal_force_n);
    mu_i[i]     = lcc__tire_mu(&car->desc.tires[i], Fz, car->env.global_friction_scale, car->damage_state.tire_health[i]);
    float Cx, Cy;
    lcc__tire_stiffness(&car->desc.tires[i], Fz, &Cx, &Cy);
    float alpha = atan2f(Vlat, (lcc__absf(Vlong) > 0.05f) ? lcc__absf(Vlong) : 0.05f);
    float Fy0   = -Cy * tanf(alpha);
    Fy0_abs[i]  = lcc__absf(Fy0);
  }

  /* find front/rear left/right indices cuz we can have a variable amount of wheels TODO: THIS IS BAD REALLY BAD */
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

  lcc_axle_torques_t axleF = { 0 }, axleR = { 0 };
  if(front_has && fiL >= 0 && fiR >= 0) {
    float           TlimL = wheel_torque_limit(car, mu_i, Fy0_abs, fiL);
    float           TlimR = wheel_torque_limit(car, mu_i, Fy0_abs, fiR);
    lcc_diff_desc_t d     = car->desc.driveline.front_diff; /* AAAAAAA IDK MAN TODO: */
    switch(d.type) {
    case LCC_DIFF_OPEN: axleF = lcc__diff_open(Tw_front, TlimL, TlimR); break;
    case LCC_DIFF_LOCKED: axleF = lcc__diff_locked(Tw_front, TlimL, TlimR); break;
    case LCC_DIFF_TORSEN: axleF = lcc__diff_torsen(Tw_front, TlimL, TlimR, d.bias_ratio, d.preload_nm); break;
    case LCC_DIFF_LSD_CLUTCH: axleF = lcc__diff_clutch_lsd(Tw_front, TlimL, TlimR, d.bias_ratio, d.preload_nm); break;
    case LCC_DIFF_ACTIVE:
    default: axleF = lcc__diff_active(Tw_front, TlimL, TlimR, d.lock_coef, d.preload_nm); break;
    }
  }
  if(rear_has && riL >= 0 && riR >= 0) {
    float           TlimL = wheel_torque_limit(car, mu_i, Fy0_abs, riL);
    float           TlimR = wheel_torque_limit(car, mu_i, Fy0_abs, riR);
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

  /* esc yaw braking update using current dynamics */
  lcc__esc_update(car, dt_s);

  /* integrate wheels and accumulate forces */
  float F_body_sum[2] = { 0.0f, 0.0f };
  float Mz            = 0.0f;
  float max_pos_slip  = 0.0f;

  for(int i = 0; i < car->wheel_count; ++i) {
    const lcc_wheel_desc_t *wd = &car->desc.wheels[i];
    const lcc_tire_desc_t  *td = &car->desc.tires[i];
    lcc_wheel_state_t      *ws = &car->wheel_states[i];

    float vtire[2];
    lcc__wheel_vel_tire_frame(car, i, vtire);
    float Vlong = vtire[0], Vlat = vtire[1];
    float r     = fmaxf(0.05f, wd->radius_m);
    float omega = ws->omega_radps;

    float denom    = fmaxf(0.5f, lcc__absf(Vlong));
    float slip     = (omega * r - Vlong) / denom;
    ws->slip_ratio = slip;

    if(wd->driven && slip > max_pos_slip) max_pos_slip = slip;

    float base_Tbr = lcc__wheel_brake_torque_base(car, i) + car->esc_extra_brake[i];
    float Tbr      = lcc__absf(slip) > 0.01f ? lcc__absf(lcc__absf(base_Tbr)) : base_Tbr;
    Tbr            = lcc__absf(Tbr);

    /* abs modulation */
    Tbr = lcc__abs_apply(car, i, Tbr, slip, dt_s);

    /* tc event emission */
  }

  /* emit abs/tc events if active */
  int any_abs = 0;
  int any_tc  = 0;
  for(int i = 0; i < car->wheel_count; ++i)
    if(car->abs_mod[i] < 0.999f) {
      any_abs = 1;
      break;
    }
  if(car->desc.ecu.abs_mode == LCC_ABS_ON && any_abs && (car->car_state.time_s - car->last_abs_evt_time) > 0.2) {
    car->last_abs_evt_time = car->car_state.time_s;
    lcc__emit_event(car, LCC_EVENT_ABS_ACTIVE, 0, 0.0f);
  }

  /* tc cut update and event */
  lcc__tc_update(car, max_pos_slip, dt_s);
  if(car->desc.ecu.tc_mode == LCC_TC_ON && car->tc_cut > 0.01f && (car->car_state.time_s - car->last_tc_evt_time) > 0.2) {
    car->last_tc_evt_time = car->car_state.time_s;
    lcc__emit_event(car, LCC_EVENT_TC_ACTIVE, 0, car->tc_cut);
  }

  /* re-loop wheels for forces with final brake torque */
  F_body_sum[0] = F_body_sum[1] = 0.0f;
  Mz                            = 0.0f;
  for(int i = 0; i < car->wheel_count; ++i) {
    const lcc_wheel_desc_t *wd = &car->desc.wheels[i];
    const lcc_tire_desc_t  *td = &car->desc.tires[i];
    lcc_wheel_state_t      *ws = &car->wheel_states[i];

    float vtire[2];
    lcc__wheel_vel_tire_frame(car, i, vtire);
    float Vlong = vtire[0], Vlat = vtire[1];

    float r     = fmaxf(0.05f, wd->radius_m);
    float omega = ws->omega_radps;

    float denom = fmaxf(0.5f, lcc__absf(Vlong));
    float s     = (omega * r - Vlong) / denom;
    s           = lcc__clampf(s, -3.0f, 3.0f);
    float alpha = atan2f(Vlat, (lcc__absf(Vlong) > 0.05f) ? lcc__absf(Vlong) : 0.05f);
    float Fz    = fmaxf(0.0f, ws->normal_force_n);
    float mu    = lcc__tire_mu(td, Fz, car->env.global_friction_scale, car->damage_state.tire_health[i]);
    float Cx, Cy;
    lcc__tire_stiffness(td, Fz, &Cx, &Cy);

    float Fx0 = Cx * s;
    float Fy0 = -Cy * tanf(alpha);
    float Frr = -td->rolling_resistance * Fz * lcc__signf(Vlong);
    Fx0 += Frr;

    float Fcap  = mu * Fz;
    float mag0  = sqrtf(Fx0 * Fx0 + Fy0 * Fy0) + 1e-6f;
    float scale = fminf(1.0f, Fcap / mag0);
    float Fx    = Fx0 * scale;
    float Fy    = Fy0 * scale;

    float T_contact = Fx * r;
    float Tdrive    = car->desc.wheels[i].driven ? Tw_cmd[i] : 0.0f;
    float Tbr       = lcc__wheel_brake_torque_base(car, i) + car->esc_extra_brake[i];
    Tbr             = lcc__abs_apply(car, i, Tbr, s, dt_s);

    /* integrate wheel */
    float Iw     = fmaxf(0.01f, wd->inertia_kgm2);
    float netT   = Tdrive - Tbr - T_contact;
    float domega = netT / Iw;
    omega += domega * dt_s;

    ws->omega_radps       = omega;
    ws->slip_ratio        = s;
    ws->slip_angle_rad    = alpha;
    ws->tire_force_long_n = Fx;
    ws->tire_force_lat_n  = Fy;
    ws->brake_torque_nm   = Tbr;
    ws->drive_torque_nm   = Tdrive;

    /* rotate tire force to body */
    float steer = car->wheel_steer_rad[i];
    float Fb[2];
    float Ftire[2] = { Fx, Fy };
    lcc__v2_rot(Fb, Ftire, steer);

    F_body_sum[0] += Fb[0];
    F_body_sum[1] += Fb[1];

    float rx = car->desc.wheels[i].position_local[0];
    float ry = car->desc.wheels[i].position_local[1];
    Mz += rx * Fb[1] - ry * Fb[0];
  }

  /* aero and gravity */
  float drag_body[2];
  lcc__v2_rot(drag_body, drag_world, -car->car_state.yaw_rad);
  F_body_sum[0] += drag_body[0];
  F_body_sum[1] += drag_body[1];

  float g_body[2];
  lcc__v2_rot(g_body, car->env.gravity_world, -car->car_state.yaw_rad);
  F_body_sum[0] += car->car_state.mass_kg * g_body[0];
  F_body_sum[1] += car->car_state.mass_kg * g_body[1];

  /* integrate rigid body */
  float ax      = F_body_sum[1] / fmaxf(1.0f, car->car_state.mass_kg);
  float ay      = F_body_sum[0] / fmaxf(1.0f, car->car_state.mass_kg);
  float Izz     = fmaxf(1.0f, car->desc.chassis.inertia_zz);
  float yaw_acc = Mz / Izz;

  car->car_state.vel_body[0] += ay * dt_s;
  car->car_state.vel_body[1] += ax * dt_s;

  float vel_world_new[2];
  lcc__v2_rot(vel_world_new, car->car_state.vel_body, car->car_state.yaw_rad);
  car->car_state.vel_world[0] = vel_world_new[0];
  car->car_state.vel_world[1] = vel_world_new[1];

  car->car_state.pos_world[0] += car->car_state.vel_world[0] * dt_s;
  car->car_state.pos_world[1] += car->car_state.vel_world[1] * dt_s;

  car->car_state.yaw_rate_radps += yaw_acc * dt_s;
  car->car_state.yaw_rad += car->car_state.yaw_rate_radps * dt_s;

  car->car_state.speed_mps   = lcc__v2_len(car->car_state.vel_world);
  car->car_state.acc_body[0] = ay;
  car->car_state.acc_body[1] = ax;
  float acc_world_new[2];
  lcc__v2_rot(acc_world_new, car->car_state.acc_body, car->car_state.yaw_rad);
  car->car_state.acc_world[0] = acc_world_new[0];
  car->car_state.acc_world[1] = acc_world_new[1];

  car->car_state.beta_rad = atanf((lcc__absf(car->car_state.vel_body[1]) > 0.1f) ? (car->car_state.vel_body[0] / car->car_state.vel_body[1]) : 0.0f);

  car->trans_state.input_rpm  = car->engine_state.rpm;
  car->trans_state.output_rpm = shaft_out_rpm;

  /* engine speed dynamics */
  float eng_omega       = lcc__rpm_to_radps(car->engine_state.rpm);
  float load_omega      = lcc__rpm_to_radps(lcc__absf(gear_ratio) * shaft_out_rpm);
  float clutch_k        = (car->desc.transmission.clutch == LCC_CLUTCH_TORQUE_CONVERTER) ? 0.5f : lcc__saturate(car->trans_state.clutch_engagement);
  float Ieng            = fmaxf(0.02f, car->desc.engine.inertia_kgm2);
  float domega_eng_free = (tq_engine - T_to_trans) / Ieng;
  float domega_track    = (load_omega - eng_omega) * (6.0f * clutch_k);
  eng_omega += (domega_eng_free * (1.0f - clutch_k) + domega_track) * dt_s;

  /* idle control support */
  if(car->desc.ecu.idle_control && car->controls.throttle < 0.05f && car->engine_state.running) {
    float w_idle = lcc__rpm_to_radps(car->desc.engine.idle_rpm);
    eng_omega    = lcc__lp(eng_omega, w_idle, 2.0f, dt_s);
  }

  car->engine_state.rpm = fmaxf(0.0f, lcc__radps_to_rpm(eng_omega));

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

  /* update aero state */
  car->aero_state.drag_force_world[0] = drag_world[0];
  car->aero_state.drag_force_world[1] = drag_world[1];
  car->aero_state.downforce_n_front   = df_front;
  car->aero_state.downforce_n_rear    = df_rear;

  /* suspension telemetry */
  lcc__suspension_update(car, dt_s);

  /* electrical loads */
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

  /* abs/esc events */
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
    int is_front = (car->desc.wheels[i].position_local[1] >= 0.0f);
    int is_left  = (car->desc.wheels[i].position_local[0] < 0.0f);

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

/* =============================== serialization ================================= */

#define LCC__SER_MAGIC 0x4C434353u /* 'L''C''C''S' */

typedef struct lcc__ser_hdr_s {
  uint32_t magic;
  uint16_t vmaj, vmin, vpatch;
  uint16_t flags;
  uint16_t reserved;
  uint32_t payload_size;
} lcc__ser_hdr_t;

static size_t lcc__ser_size_curve1d(const lcc_curve1d_t *c) {
  int n = (c && c->points && c->count > 0) ? c->count : 0;
  return sizeof(int32_t) + (size_t)n * sizeof(lcc_curve1d_point_t);
}

static size_t lcc__ser_size_map2d(const lcc_map2d_t *m) {
  int n = (m && m->points && m->count > 0) ? m->count : 0;
  return sizeof(int32_t) + (size_t)n * sizeof(lcc_map2d_point_t);
}

static int lcc__write_bytes(uint8_t **p, size_t *cap, const void *src, size_t n) {
  if(*cap < n) return 0;
  memcpy(*p, src, n);
  *p += n;
  *cap -= n;
  return 1;
}

static int lcc__read_bytes(const uint8_t **p, size_t *cap, void *dst, size_t n) {
  if(*cap < n) return 0;
  memcpy(dst, *p, n);
  *p += n;
  *cap -= n;
  return 1;
}

static int lcc__write_curve1d(uint8_t **p, size_t *cap, const lcc_curve1d_t *c) {
  int32_t n = (c && c->points && c->count > 0) ? c->count : 0;
  if(!lcc__write_bytes(p, cap, &n, sizeof(n))) return 0;
  if(n > 0)
    if(!lcc__write_bytes(p, cap, c->points, (size_t)n * sizeof(lcc_curve1d_point_t))) return 0;
  return 1;
}

static int lcc__write_map2d(uint8_t **p, size_t *cap, const lcc_map2d_t *m) {
  int32_t n = (m && m->points && m->count > 0) ? m->count : 0;
  if(!lcc__write_bytes(p, cap, &n, sizeof(n))) return 0;
  if(n > 0)
    if(!lcc__write_bytes(p, cap, m->points, (size_t)n * sizeof(lcc_map2d_point_t))) return 0;
  return 1;
}

static int lcc__read_curve1d(const uint8_t **p, size_t *cap, lcc_curve1d_point_t **out_pts, int *out_cnt) {
  int32_t n = 0;
  if(!lcc__read_bytes(p, cap, &n, sizeof(n))) return 0;
  if(n < 0 || n > LCC_MAX_TORQUE_POINTS) return 0;
  if(n == 0) {
    *out_pts = NULL;
    *out_cnt = 0;
    return 1;
  }
  size_t               bytes = (size_t)n * sizeof(lcc_curve1d_point_t);
  lcc_curve1d_point_t *pts   = (lcc_curve1d_point_t *)lcc__alloc(bytes, lcc__alloc_user);
  if(!pts) return 0;
  if(!lcc__read_bytes(p, cap, pts, bytes)) {
    lcc__free(pts, lcc__alloc_user);
    return 0;
  }
  *out_pts = pts;
  *out_cnt = n;
  return 1;
}

static int lcc__read_map2d(const uint8_t **p, size_t *cap, lcc_map2d_point_t **out_pts, int *out_cnt) {
  int32_t n = 0;
  if(!lcc__read_bytes(p, cap, &n, sizeof(n))) return 0;
  if(n < 0 || n > LCC_MAX_MAP_POINTS_2D) return 0;
  if(n == 0) {
    *out_pts = NULL;
    *out_cnt = 0;
    return 1;
  }
  size_t             bytes = (size_t)n * sizeof(lcc_map2d_point_t);
  lcc_map2d_point_t *pts   = (lcc_map2d_point_t *)lcc__alloc(bytes, lcc__alloc_user);
  if(!pts) return 0;
  if(!lcc__read_bytes(p, cap, pts, bytes)) {
    lcc__free(pts, lcc__alloc_user);
    return 0;
  }
  *out_pts = pts;
  *out_cnt = n;
  return 1;
}

lcc_result_t lcc_car_serialize(const lcc_car_t *car, void *buffer, size_t buffer_size, size_t *bytes_written) {
  if(!car) return LCC_ERR_INVALID_ARG;

  /* size computation */
  size_t size = 0;
  size += sizeof(lcc__ser_hdr_t);

  /* descriptor copy with pointers nulled */
  size += sizeof(lcc_car_desc_t);

  /* current environment and controls and states */
  size += sizeof(lcc_environment_t);
  size += sizeof(lcc_controls_t);
  size += sizeof(lcc_car_state_t);
  size += sizeof(lcc_engine_state_t);
  size += sizeof(lcc_ignition_state_t);
  size += sizeof(lcc_transmission_state_t);
  size += sizeof(lcc_diff_state_t) * 3;
  size += sizeof(lcc_brake_state_t);
  size += sizeof(lcc_suspension_state_t);
  size += sizeof(lcc_electrics_state_t);
  size += sizeof(lcc_fuel_state_t);
  size += sizeof(lcc_cooling_state_t);
  size += sizeof(lcc_exhaust_state_t);
  size += sizeof(lcc_aero_state_t);
  size += sizeof(int32_t); /* wheel states count */
  size += (size_t)car->wheel_count * sizeof(lcc_wheel_state_t);
  size += sizeof(lcc_damage_state_t);

  /* dynamic curves/maps */
  size += lcc__ser_size_curve1d(&car->desc.engine.wot_torque_nm_vs_rpm);
  size += lcc__ser_size_curve1d(&car->desc.engine.friction_torque_nm_vs_rpm);
  size += lcc__ser_size_curve1d(&car->desc.engine.throttle_map);
  size += lcc__ser_size_map2d(&car->desc.engine.boost_pressure_kpa_vs_rpm_throttle);
  size += lcc__ser_size_map2d(&car->desc.ignition.spark_advance_deg_vs_rpm_load);

  if(bytes_written) *bytes_written = size;
  if(!buffer) return LCC_OK;
  if(buffer_size < size) return LCC_ERR_BOUNDS;

  /* write */
  uint8_t *p   = (uint8_t *)buffer;
  size_t   cap = buffer_size;

  lcc__ser_hdr_t hdr;
  hdr.magic        = LCC__SER_MAGIC;
  hdr.vmaj         = (uint16_t)LCC_VERSION_MAJOR;
  hdr.vmin         = (uint16_t)LCC_VERSION_MINOR;
  hdr.vpatch       = (uint16_t)LCC_VERSION_PATCH;
  hdr.flags        = 0;
  hdr.reserved     = 0;
  hdr.payload_size = (uint32_t)(size - sizeof(lcc__ser_hdr_t));
  if(!lcc__write_bytes(&p, &cap, &hdr, sizeof(hdr))) return LCC_ERR_BOUNDS;

  /* descriptor copy with pointers cleared */
  lcc_car_desc_t d                                   = car->desc;
  d.engine.wot_torque_nm_vs_rpm.points               = NULL;
  d.engine.friction_torque_nm_vs_rpm.points          = NULL;
  d.engine.throttle_map.points                       = NULL;
  d.engine.boost_pressure_kpa_vs_rpm_throttle.points = NULL;
  d.ignition.spark_advance_deg_vs_rpm_load.points    = NULL;

  if(!lcc__write_bytes(&p, &cap, &d, sizeof(d))) return LCC_ERR_BOUNDS;

  /* runtime env and controls and states */
  if(!lcc__write_bytes(&p, &cap, &car->env, sizeof(car->env))) return LCC_ERR_BOUNDS;
  if(!lcc__write_bytes(&p, &cap, &car->controls, sizeof(car->controls))) return LCC_ERR_BOUNDS;
  if(!lcc__write_bytes(&p, &cap, &car->car_state, sizeof(car->car_state))) return LCC_ERR_BOUNDS;
  if(!lcc__write_bytes(&p, &cap, &car->engine_state, sizeof(car->engine_state))) return LCC_ERR_BOUNDS;
  if(!lcc__write_bytes(&p, &cap, &car->ign_state, sizeof(car->ign_state))) return LCC_ERR_BOUNDS;
  if(!lcc__write_bytes(&p, &cap, &car->trans_state, sizeof(car->trans_state))) return LCC_ERR_BOUNDS;
  if(!lcc__write_bytes(&p, &cap, &car->front_diff_state, sizeof(car->front_diff_state))) return LCC_ERR_BOUNDS;
  if(!lcc__write_bytes(&p, &cap, &car->rear_diff_state, sizeof(car->rear_diff_state))) return LCC_ERR_BOUNDS;
  if(!lcc__write_bytes(&p, &cap, &car->center_diff_state, sizeof(car->center_diff_state))) return LCC_ERR_BOUNDS;
  if(!lcc__write_bytes(&p, &cap, &car->brake_state, sizeof(car->brake_state))) return LCC_ERR_BOUNDS;
  if(!lcc__write_bytes(&p, &cap, &car->susp_state, sizeof(car->susp_state))) return LCC_ERR_BOUNDS;
  if(!lcc__write_bytes(&p, &cap, &car->elec_state, sizeof(car->elec_state))) return LCC_ERR_BOUNDS;
  if(!lcc__write_bytes(&p, &cap, &car->fuel_state, sizeof(car->fuel_state))) return LCC_ERR_BOUNDS;
  if(!lcc__write_bytes(&p, &cap, &car->cool_state, sizeof(car->cool_state))) return LCC_ERR_BOUNDS;
  if(!lcc__write_bytes(&p, &cap, &car->exh_state, sizeof(car->exh_state))) return LCC_ERR_BOUNDS;
  if(!lcc__write_bytes(&p, &cap, &car->aero_state, sizeof(car->aero_state))) return LCC_ERR_BOUNDS;

  int32_t wc = car->wheel_count;
  if(!lcc__write_bytes(&p, &cap, &wc, sizeof(wc))) return LCC_ERR_BOUNDS;
  if(wc > 0)
    if(!lcc__write_bytes(&p, &cap, car->wheel_states, (size_t)wc * sizeof(lcc_wheel_state_t))) return LCC_ERR_BOUNDS;

  if(!lcc__write_bytes(&p, &cap, &car->damage_state, sizeof(car->damage_state))) return LCC_ERR_BOUNDS;

  /* variable data: curves/maps */
  if(!lcc__write_curve1d(&p, &cap, &car->desc.engine.wot_torque_nm_vs_rpm)) return LCC_ERR_BOUNDS;
  if(!lcc__write_curve1d(&p, &cap, &car->desc.engine.friction_torque_nm_vs_rpm)) return LCC_ERR_BOUNDS;
  if(!lcc__write_curve1d(&p, &cap, &car->desc.engine.throttle_map)) return LCC_ERR_BOUNDS;
  if(!lcc__write_map2d(&p, &cap, &car->desc.engine.boost_pressure_kpa_vs_rpm_throttle)) return LCC_ERR_BOUNDS;
  if(!lcc__write_map2d(&p, &cap, &car->desc.ignition.spark_advance_deg_vs_rpm_load)) return LCC_ERR_BOUNDS;

  return LCC_OK;
}

lcc_result_t lcc_car_deserialize(lcc_car_t **car_out, const void *buffer, size_t buffer_size) {
  if(!car_out || !buffer) return LCC_ERR_INVALID_ARG;
  *car_out = NULL;

  const uint8_t *p   = (const uint8_t *)buffer;
  size_t         cap = buffer_size;

  lcc__ser_hdr_t hdr;
  if(!lcc__read_bytes(&p, &cap, &hdr, sizeof(hdr))) return LCC_ERR_BOUNDS;
  if(hdr.magic != LCC__SER_MAGIC) return LCC_ERR_UNSUPPORTED;
  if(hdr.vmaj != LCC_VERSION_MAJOR) return LCC_ERR_UNSUPPORTED; /* strict major match */

  lcc_car_desc_t desc;
  if(!lcc__read_bytes(&p, &cap, &desc, sizeof(desc))) return LCC_ERR_BOUNDS;

  /* read runtime env & states */
  lcc_environment_t        env;
  lcc_controls_t           controls;
  lcc_car_state_t          car_state;
  lcc_engine_state_t       engine_state;
  lcc_ignition_state_t     ign_state;
  lcc_transmission_state_t trans_state;
  lcc_diff_state_t         front_diff_state, rear_diff_state, center_diff_state;
  lcc_brake_state_t        brake_state;
  lcc_suspension_state_t   susp_state;
  lcc_electrics_state_t    elec_state;
  lcc_fuel_state_t         fuel_state;
  lcc_cooling_state_t      cool_state;
  lcc_exhaust_state_t      exh_state;
  lcc_aero_state_t         aero_state;
  int32_t                  wc = 0;

  if(!lcc__read_bytes(&p, &cap, &env, sizeof(env))) return LCC_ERR_BOUNDS;
  if(!lcc__read_bytes(&p, &cap, &controls, sizeof(controls))) return LCC_ERR_BOUNDS;
  if(!lcc__read_bytes(&p, &cap, &car_state, sizeof(car_state))) return LCC_ERR_BOUNDS;
  if(!lcc__read_bytes(&p, &cap, &engine_state, sizeof(engine_state))) return LCC_ERR_BOUNDS;
  if(!lcc__read_bytes(&p, &cap, &ign_state, sizeof(ign_state))) return LCC_ERR_BOUNDS;
  if(!lcc__read_bytes(&p, &cap, &trans_state, sizeof(trans_state))) return LCC_ERR_BOUNDS;
  if(!lcc__read_bytes(&p, &cap, &front_diff_state, sizeof(front_diff_state))) return LCC_ERR_BOUNDS;
  if(!lcc__read_bytes(&p, &cap, &rear_diff_state, sizeof(rear_diff_state))) return LCC_ERR_BOUNDS;
  if(!lcc__read_bytes(&p, &cap, &center_diff_state, sizeof(center_diff_state))) return LCC_ERR_BOUNDS;
  if(!lcc__read_bytes(&p, &cap, &brake_state, sizeof(brake_state))) return LCC_ERR_BOUNDS;
  if(!lcc__read_bytes(&p, &cap, &susp_state, sizeof(susp_state))) return LCC_ERR_BOUNDS;
  if(!lcc__read_bytes(&p, &cap, &elec_state, sizeof(elec_state))) return LCC_ERR_BOUNDS;
  if(!lcc__read_bytes(&p, &cap, &fuel_state, sizeof(fuel_state))) return LCC_ERR_BOUNDS;
  if(!lcc__read_bytes(&p, &cap, &cool_state, sizeof(cool_state))) return LCC_ERR_BOUNDS;
  if(!lcc__read_bytes(&p, &cap, &exh_state, sizeof(exh_state))) return LCC_ERR_BOUNDS;
  if(!lcc__read_bytes(&p, &cap, &aero_state, sizeof(aero_state))) return LCC_ERR_BOUNDS;

  if(!lcc__read_bytes(&p, &cap, &wc, sizeof(wc))) return LCC_ERR_BOUNDS;
  if(wc < 0 || wc > LCC_MAX_WHEELS) return LCC_ERR_BOUNDS;

  lcc_wheel_state_t tmp_wheels[LCC_MAX_WHEELS];
  if(wc > 0) {
    size_t bytes = (size_t)wc * sizeof(lcc_wheel_state_t);
    if(!lcc__read_bytes(&p, &cap, tmp_wheels, bytes)) return LCC_ERR_BOUNDS;
  }

  lcc_damage_state_t damage_state;
  if(!lcc__read_bytes(&p, &cap, &damage_state, sizeof(damage_state))) return LCC_ERR_BOUNDS;

  /* curves/maps */
  lcc_curve1d_point_t *wot_pts = NULL, *fric_pts = NULL, *thrt_pts = NULL;
  int                  wot_cnt = 0, fric_cnt = 0, thrt_cnt = 0;
  lcc_map2d_point_t   *boost_pts = NULL, *spark_pts = NULL;
  int                  boost_cnt = 0, spark_cnt = 0;

  if(!lcc__read_curve1d(&p, &cap, &wot_pts, &wot_cnt)) goto fail_read;
  if(!lcc__read_curve1d(&p, &cap, &fric_pts, &fric_cnt)) goto fail_read;
  if(!lcc__read_curve1d(&p, &cap, &thrt_pts, &thrt_cnt)) goto fail_read;
  if(!lcc__read_map2d(&p, &cap, &boost_pts, &boost_cnt)) goto fail_read;
  if(!lcc__read_map2d(&p, &cap, &spark_pts, &spark_cnt)) goto fail_read;

  /* attach to descriptor before create */
  desc.engine.wot_torque_nm_vs_rpm.points               = wot_pts;
  desc.engine.wot_torque_nm_vs_rpm.count                = wot_cnt;
  desc.engine.friction_torque_nm_vs_rpm.points          = fric_pts;
  desc.engine.friction_torque_nm_vs_rpm.count           = fric_cnt;
  desc.engine.throttle_map.points                       = thrt_pts;
  desc.engine.throttle_map.count                        = thrt_cnt;
  desc.engine.boost_pressure_kpa_vs_rpm_throttle.points = boost_pts;
  desc.engine.boost_pressure_kpa_vs_rpm_throttle.count  = boost_cnt;
  desc.ignition.spark_advance_deg_vs_rpm_load.points    = spark_pts;
  desc.ignition.spark_advance_deg_vs_rpm_load.count     = spark_cnt;

  /* create car */
  lcc_car_t *car = lcc_car_create(&desc);
  if(!car) goto fail_create;

  /* mark ownership of dynamic data so destroy() frees them */
  car->own_wot_pts   = wot_pts;
  car->own_wot_cnt   = wot_cnt;
  car->own_fric_pts  = fric_pts;
  car->own_fric_cnt  = fric_cnt;
  car->own_thrt_pts  = thrt_pts;
  car->own_thrt_cnt  = thrt_cnt;
  car->own_boost_pts = boost_pts;
  car->own_boost_cnt = boost_cnt;
  car->own_spark_pts = spark_pts;
  car->own_spark_cnt = spark_cnt;

  /* restore runtime env, controls and states */
  car->env      = env;
  car->controls = controls;

  car->car_state         = car_state;
  car->engine_state      = engine_state;
  car->ign_state         = ign_state;
  car->trans_state       = trans_state;
  car->front_diff_state  = front_diff_state;
  car->rear_diff_state   = rear_diff_state;
  car->center_diff_state = center_diff_state;
  car->brake_state       = brake_state;
  car->susp_state        = susp_state;
  car->elec_state        = elec_state;
  car->fuel_state        = fuel_state;
  car->cool_state        = cool_state;
  car->exh_state         = exh_state;
  car->aero_state        = aero_state;

  car->wheel_count = desc.wheel_count;
  int ncopy        = lcc__mini(wc, car->wheel_count);
  for(int i = 0; i < ncopy; ++i) car->wheel_states[i] = tmp_wheels[i];

  car->damage_state = damage_state;

  /* recompute derived bits */
  car->total_mass_kg      = car->car_state.mass_kg;
  car->pending_gear_index = car->trans_state.gear_index;
  car->shift_timer_s      = 0.0f;
  car->tc_cut             = 0.0f;
  memset(car->abs_mod, 0, sizeof(car->abs_mod));
  for(int i = 0; i < car->wheel_count; ++i) car->abs_mod[i] = 1.0f;
  memset(car->esc_extra_brake, 0, sizeof(car->esc_extra_brake));

  lcc__compute_static_loads(car);

  *car_out = car;
  return LCC_OK;

fail_create:
  /* free on failure (car not created) */
  if(wot_pts) lcc__free(wot_pts, lcc__alloc_user);
  if(fric_pts) lcc__free(fric_pts, lcc__alloc_user);
  if(thrt_pts) lcc__free(thrt_pts, lcc__alloc_user);
  if(boost_pts) lcc__free(boost_pts, lcc__alloc_user);
  if(spark_pts) lcc__free(spark_pts, lcc__alloc_user);
  return LCC_ERR_OUT_OF_MEMORY;

fail_read:
  if(wot_pts) lcc__free(wot_pts, lcc__alloc_user);
  if(fric_pts) lcc__free(fric_pts, lcc__alloc_user);
  if(thrt_pts) lcc__free(thrt_pts, lcc__alloc_user);
  if(boost_pts) lcc__free(boost_pts, lcc__alloc_user);
  if(spark_pts) lcc__free(spark_pts, lcc__alloc_user);
  return LCC_ERR_BOUNDS;
}

#endif
