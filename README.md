# libccar: A 2D Top-Down Car Simulation Library

libccar is a lightweight, semi-realistic 2D top-down car simulation library provided as a single C99 header. It models an engine, transmission, clutch, differentials (open/locked/LSD/torsen/active), tires (Pacejka-style), aero, electrical system (battery/alternator/starter), cooling, fuel, and basic driver aids (ABS/TC/ESC). It's aimed at fast prototyping and experimentation in games and simulators.

- Space: 2D plane (+x forward, +y right)
- Rotation: right-handed (clockwise positive)

## Table of Contents
- [Repository Structure](#repository-structure)
- [Building and Using libccar](#building-and-using-libccar)
  - [Direct Compilation](#direct-compilation)
  - [Shared Library](#shared-library)
- [Quick Start](#quick-start)
- [API Overview](#api-overview)
  - [System and Memory](#system-and-memory)
  - [Descriptor Defaults](#descriptor-defaults)
  - [Lifecycle](#lifecycle)
  - [Simulation and Controls](#simulation-and-controls)
  - [Runtime Configuration](#runtime-configuration)
  - [Powertrain Controls](#powertrain-controls)
  - [Fuel and Energy](#fuel-and-energy)
  - [Driver Aids](#driver-aids)
  - [Position and Velocity](#position-and-velocity)
  - [Utilities](#utilities)
  - [Events](#events)
  - [Engine Map Generation](#engine-map-generation)
- [Units and Best Practices](#units-and-best-practices)
- [Optional Rust GUI Demo](#optional-rust-gui-demo)

## Repository Structure

```
.
├── build_libccar.c      # Translation unit for building a shared/static library
├── libccar.h            # Single-header library (define LCC_IMPLEMENTATION in one TU)
├── tests/
│   ├── example.c        # Minimal usage example
│   └── benchmark.c      # Performance benchmark
├── car_demo/            # Optional Rust GUI demo (via FFI)
├── LICENSE
└── README.md
```

## Building and Using libccar

### Direct Compilation

Include libccar directly in your C project.

1) In exactly one .c file:
```c
#define LCC_IMPLEMENTATION
#include "libccar.h"
```

2) In all other .c files:
```c
#include "libccar.h"
```

Compile and link with math:
- gcc/clang: add -std=c99 and -lm

Example:
```
clang -std=c99 main.c -lm
```

### Shared Library

Use the provided build_libccar.c (defines LCC_IMPLEMENTATION):

Linux:
```
clang -O2 -std=c99 -fPIC -shared build_libccar.c -o libccar.so -lm
```

Then link your app against the produced library and include libccar.h.

## Quick Start
See [example.c](https://github.com/Teero888/libccar/blob/master/tests/example.c) for a minimal example showing defaults, create, step, controls, and reading state.

Build:
```
clang -O2 -std=c99 example.c -lm -o example
./example
```

## API Overview

### System and Memory

| Function | Signature | Notes |
|---|---|---|
| `lcc_set_allocators` | `void lcc_set_allocators(lcc_alloc_fn alloc_fn, lcc_free_fn free_fn, void* user)` | Set custom memory allocators (optional). |
| `lcc_version_string` | `const char* lcc_version_string(void)` | Returns the library version string. |

### Descriptor Defaults

| Function | Signature | Notes |
|---|---|---|
| `lcc_car_desc_init_defaults` | `void lcc_car_desc_init_defaults(lcc_car_desc_t* desc)` | Initialize car descriptor defaults. |
| `lcc_engine_desc_init_defaults` | `void lcc_engine_desc_init_defaults(lcc_engine_desc_t* desc)` | Initialize engine descriptor defaults. |
| `lcc_fuel_desc_init_defaults` | `void lcc_fuel_desc_init_defaults(lcc_fuel_desc_t* desc)` | Initialize fuel system defaults. |
| `lcc_cooling_desc_init_defaults` | `void lcc_cooling_desc_init_defaults(lcc_cooling_desc_t* desc)` | Initialize cooling system defaults. |
| `lcc_battery_desc_init_defaults` | `void lcc_battery_desc_init_defaults(lcc_battery_desc_t* desc)` | Initialize battery defaults. |
| `lcc_alternator_desc_init_defaults` | `void lcc_alternator_desc_init_defaults(lcc_alternator_desc_t* desc)` | Initialize alternator defaults. |
| `lcc_starter_desc_init_defaults` | `void lcc_starter_desc_init_defaults(lcc_starter_desc_t* desc)` | Initialize starter defaults. |
| `lcc_ecu_desc_init_defaults` | `void lcc_ecu_desc_init_defaults(lcc_ecu_desc_t* desc)` | Initialize ECU defaults. |
| `lcc_transmission_desc_init_defaults` | `void lcc_transmission_desc_init_defaults(lcc_transmission_desc_t* desc)` | Initialize transmission defaults. |
| `lcc_driveline_desc_init_defaults` | `void lcc_driveline_desc_init_defaults(lcc_driveline_desc_t* desc)` | Initialize driveline defaults. |
| `lcc_chassis_desc_init_defaults` | `void lcc_chassis_desc_init_defaults(lcc_chassis_desc_t* desc)` | Initialize chassis defaults. |
| `lcc_aero_desc_init_defaults` | `void lcc_aero_desc_init_defaults(lcc_aero_desc_t* desc)` | Initialize aero defaults. |
| `lcc_wheel_desc_init_defaults` | `void lcc_wheel_desc_init_defaults(lcc_wheel_desc_t* desc)` | Initialize wheel defaults. |
| `lcc_tire_desc_init_defaults` | `void lcc_tire_desc_init_defaults(lcc_tire_desc_t* desc)` | Initialize tire defaults. |
| `lcc_brake_desc_init_defaults` | `void lcc_brake_desc_init_defaults(lcc_brake_desc_t* desc)` | Initialize brake defaults. |
| `lcc_arb_desc_init_defaults` | `void lcc_arb_desc_init_defaults(lcc_arb_desc_t* desc)` | Initialize anti-roll bar defaults. |
| `lcc_steering_desc_init_defaults` | `void lcc_steering_desc_init_defaults(lcc_steering_desc_t* desc)` | Initialize steering defaults. |
| `lcc_environment_init_defaults` | `void lcc_environment_init_defaults(lcc_environment_t* env)` | Initialize environment defaults. |
| `lcc_engine_simple_spec_init_defaults` | `void lcc_engine_simple_spec_init_defaults(lcc_engine_simple_spec_t* spec)` | Initialize simple engine spec defaults. |

### Lifecycle

| Function | Signature | Notes |
|---|---|---|
| `lcc_car_create` | `lcc_car_t* lcc_car_create(const lcc_car_desc_t* desc)` | Create a car from a descriptor. |
| `lcc_car_destroy` | `void lcc_car_destroy(lcc_car_t* car)` | Destroy and free a car. |
| `lcc_car_reset` | `lcc_result_t lcc_car_reset(lcc_car_t* car, const lcc_car_state_t* optional_state)` | Reset to defaults or to `optional_state` if provided. |

### Simulation and Controls

| Function | Signature | Notes |
|---|---|---|
| `lcc_car_step` | `lcc_result_t lcc_car_step(lcc_car_t* car, float dt_s)` | Step simulation.  0.0 < `dt_s` <= 0.1 |
| `lcc_car_set_controls` | `void lcc_car_set_controls(lcc_car_t* car, const lcc_controls_t* controls)` | Apply controls for next step. |
| `lcc_car_get_controls` | `void lcc_car_get_controls(const lcc_car_t* car, lcc_controls_t* controls_out)` | Read current controls. |
| `lcc_car_set_environment` | `void lcc_car_set_environment(lcc_car_t* car, const lcc_environment_t* env)` | Set environment state. |
| `lcc_car_get_environment` | `void lcc_car_get_environment(const lcc_car_t* car, lcc_environment_t* env_out)` | Get environment state. |

Controls struct (`lcc_controls_t`)

| Field | Range/Type | Notes |
|---|---|---|
| `throttle` | 0..1 | Accelerator pedal. |
| `brake` | 0..1 | Service brake. |
| `handbrake` | 0..1 | Parking/hand brake. |
| `clutch` | 0..1 | 1 = fully disengaged (pedal down). |
| `steer` | -1..1 | Left negative, right positive. |
| `ignition_switch` | bool | Ignition ON/OFF. |
| `starter` | bool | Engage starter motor. |

### Runtime Configuration

| Function | Signature | Notes |
|---|---|---|
| `lcc_car_set_engine_map` | `lcc_result_t lcc_car_set_engine_map(lcc_car_t* car, const lcc_curve1d_t* wot_torque, const lcc_curve1d_t* friction)` | Replace engine maps (owned externally unless generated). |
| `lcc_car_set_boost_map` | `lcc_result_t lcc_car_set_boost_map(lcc_car_t* car, const lcc_map2d_t* boost)` | Set boost map for forced induction. |
| `lcc_car_set_gear_ratios` | `lcc_result_t lcc_car_set_gear_ratios(lcc_car_t* car, const float* gear_ratios, int gear_count, float final_drive)` | Update gearbox ratios and final drive. |
| `lcc_car_set_diff_params` | `lcc_result_t lcc_car_set_diff_params(lcc_car_t* car, lcc_diff_type_t front, lcc_diff_type_t rear, float preload_nm, float bias_ratio)` | Configure front/rear diffs; bias used for LSD/Torsen. |
| `lcc_car_set_tire_params` | `lcc_result_t lcc_car_set_tire_params(lcc_car_t* car, int wheel_index, const lcc_tire_desc_t* tire)` | Update a tire's parameters. |
| `lcc_car_set_brake_params` | `lcc_result_t lcc_car_set_brake_params(lcc_car_t* car, int wheel_index, const lcc_brake_desc_t* brake)` | Update a brake's parameters. |
| `lcc_car_set_arb_params` | `lcc_result_t lcc_car_set_arb_params(lcc_car_t* car, const lcc_arb_desc_t* arb)` | Update anti-roll bar parameters. |
| `lcc_car_set_steering_params` | `lcc_result_t lcc_car_set_steering_params(lcc_car_t* car, const lcc_steering_desc_t* steer)` | Update steering parameters. |

### Powertrain Controls

| Function | Signature | Notes |
|---|---|---|
| `lcc_car_request_gear` | `lcc_result_t lcc_car_request_gear(lcc_car_t* car, int gear_index)` | 0 = Reverse, 1 = Neutral, 2.. = forward gears. |
| `lcc_car_shift_up` | `lcc_result_t lcc_car_shift_up(lcc_car_t* car)` | Request shift up (if applicable). |
| `lcc_car_shift_down` | `lcc_result_t lcc_car_shift_down(lcc_car_t* car)` | Request shift down (if applicable). |

### Fuel and Energy

| Function | Signature | Notes |
|---|---|---|
| `lcc_car_refuel` | `lcc_result_t lcc_car_refuel(lcc_car_t* car, float liters)` | Add fuel. |
| `lcc_car_set_fuel` | `lcc_result_t lcc_car_set_fuel(lcc_car_t* car, float liters)` | Set absolute fuel amount. |
| `lcc_car_recharge_battery` | `lcc_result_t lcc_car_recharge_battery(lcc_car_t* car, float state_of_charge_0_to_1)` | Set battery SOC. |

### Driver Aids

| Function | Signature | Notes |
|---|---|---|
| `lcc_car_set_abs` | `void lcc_car_set_abs(lcc_car_t* car, lcc_abs_mode_t mode)` | Set ABS mode. |
| `lcc_car_set_tc` | `void lcc_car_set_tc(lcc_car_t* car, lcc_tc_mode_t mode)` | Set traction control mode. |
| `lcc_car_set_esc` | `void lcc_car_set_esc(lcc_car_t* car, lcc_esc_mode_t mode)` | Set stability control mode. |

### Position and Velocity

| Function | Signature | Notes |
|---|---|---|
| `lcc_car_set_pos` | `void lcc_car_set_pos(lcc_car_t* car, const float pos_world[2], float yaw_rad)` | Set world position and yaw. |
| `lcc_car_get_pos` | `void lcc_car_get_pos(const lcc_car_t* car, float pos_world_out[2], float* yaw_rad_out)` | Get world position and yaw. |
| `lcc_car_set_velocity` | `void lcc_car_set_velocity(lcc_car_t* car, const float vel_world[2], float yaw_rate_radps)` | Set linear velocity and yaw rate. |
| `lcc_car_get_velocity` | `void lcc_car_get_velocity(const lcc_car_t* car, float vel_world_out[2], float* yaw_rate_radps_out)` | Get linear velocity and yaw rate. |

### Utilities

| Function | Signature | Notes |
|---|---|---|
| `lcc_car_get_local_bounds` | `void lcc_car_get_local_bounds(const lcc_car_t* car, float min_local_out[2], float max_local_out[2])` | Local AABB in car space. |
| `lcc_car_get_wheel_global_positions` | `int lcc_car_get_wheel_global_positions(const lcc_car_t* car, float out_positions[][2], int max_wheels)` | Write wheel positions; returns count written. |
| `lcc_car_get_speed_kmh` | `float lcc_car_get_speed_kmh(const lcc_car_t* car)` | Convenience speed in km/h. |
| `lcc_deg_to_rad` | `float lcc_deg_to_rad(float deg)` | Degrees to radians. |
| `lcc_rad_to_deg` | `float lcc_rad_to_deg(float rad)` | Radians to degrees. |

### Events

| Function | Signature | Notes |
|---|---|---|
| `lcc_car_set_event_callback` | `void lcc_car_set_event_callback(lcc_car_t* car, lcc_event_cb callback, void* user)` | Register event callback (user pointer passed through). |

Event types (`lcc_event_type_t`)

| Type | When/Meaning |
|---|---|
| `LCC_EVENT_ENGINE_START` | Engine started (combustion running). |
| `LCC_EVENT_ENGINE_STOP` | Engine stopped (shut off). |
| `LCC_EVENT_GEAR_CHANGE` | Gear index changed. |
| `LCC_EVENT_ENGINE_STALL` | Engine stalled unexpectedly. |
| `LCC_EVENT_OVERHEAT` | Coolant or brakes overheated. |
| `LCC_EVENT_FUEL_STARVATION` | Fuel supply insufficient under demand. |

Event payload (`lcc_event_t`)

| Field | Type | Description |
|---|---|---|
| `type` | `lcc_event_type_t` | Event type. |
| `time_s` | `float` | Simulation time at event. |
| `data_i32` | `int32_t` | Event-specific integer payload. |
| `data_f32` | `float` | Event-specific float payload. |

### Engine Map Generation

| Function | Signature | Notes |
|---|---|---|
| `lcc_engine_simple_spec_init_defaults` | `void lcc_engine_simple_spec_init_defaults(lcc_engine_simple_spec_t* spec)` | Init spec (rated power, redline, etc.). |
| `lcc_car_generate_engine_from_simple_spec` | `lcc_result_t lcc_car_generate_engine_from_simple_spec(lcc_car_t* car, const lcc_engine_simple_spec_t* spec)` | Auto-generate WOT torque, friction, throttle, and boost maps; car owns generated maps. |

Use this to auto-generate WOT torque, friction, throttle, and (if FI) boost maps from typical datasheet values (rated power/redline/etc). The car will own and free these generated maps.

## Units and Best Practices

- Mass: kilograms (kg)
- Distance: meters (m)
- Angles: radians internally; deg<->rad helpers provided
- Speed: m/s internally; use `lcc_car_get_speed_kmh` for km/h
- Forces: Newtons (N)
- Torques: Newton-meters (Nm)
- Time step: use 1/100 s or smaller (e.g. 1/120). lcc_car_step rejects dt_s > 0.1
- Coordinate system: body +x forward, +y right; world axes same; yaw is CW-positive

You can override some tuning via preprocessor defines (Pacejka shape/curvature, battery/alt constants, etc.) before including libccar.h if you want.

## Optional Rust GUI Demo

The car_demo/ directory contains an optional Rust GUI visualizer using egui, linked via FFI.

```
cd car_demo
cargo run --release
```
