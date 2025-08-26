# libccar: A 2D Top-Down Car Simulation Library (WIP)

**libccar** is a lightweight, semi-realistic 2D top-down car simulation library provided as a single C99 header file. It models a car with components like an engine, transmission, clutch, limited-slip differential, tires, aerodynamics, electrical system, and fuel consumption. Ideal for quick prototyping and experimentation in games or simulations.

## Table of Contents
- [Repository Structure](#repository-structure)
- [Building and Using libccar](#building-and-using-libccar)
  - [Direct Compilation](#direct-compilation)
  - [Shared Library](#shared-library)
- [Minimal Example](#minimal-example)
- [API Reference](#api-reference)
  - [Creation and Simulation](#creation-and-simulation)
  - [Input Controls](#input-controls)
  - [Gearing and Metrics](#gearing-and-metrics)
  - [Electrical and Fuel Systems](#electrical-and-fuel-systems)
- [Units and Best Practices](#units-and-best-practices)
- [Optional Rust GUI Demo](#optional-rust-gui-demo)

## Repository Structure

- **`libccar.h`**: The core library (single C99 header file).
- **`build_libccar.c`**: Helper translation unit to build a shared library.
- **`car_demo/`**: Optional Rust-based GUI demo using the C library via FFI.

## Building and Using libccar

### Direct Compilation

To include libccar directly in your C program:

1. In **exactly one** `.c` file, define the implementation before including the header:
```c
#define LIBCCAR_IMPLEMENTATION
#include "libccar.h"
```

2. In all other `.c` files that use the API, include the header without the macro:
```c
#include "libccar.h"
```

3. For C++ projects, wrap the include in `extern "C"` to ensure compatibility:
```cpp
extern "C" {
#include "libccar.h"
}
```

### Shared Library

To build libccar as a shared library, use `build_libccar.c`, which contains:
```c
#define LIBCCAR_IMPLEMENTATION
#include "libccar.h"
```

#### Build Commands
- **Linux**:
```bash
gcc -O2 -std=c99 -fPIC -shared build_libccar.c -o libccar.so -lm
```

- **macOS**:
```bash
clang -O2 -std=c99 -dynamiclib build_libccar.c -o libccar.dylib -lm
```

- **Windows (MSVC)**:
```bat
cl /O2 /LD build_libccar.c /Fe:ccar.dll
```

In your application, include `libccar.h` and link against the shared library (`-lccar` or the library file directly). On Unix systems, link with `-lm` for math functions.

## Minimal Example

This example creates a sports car, starts the engine, and drives in first gear:

```c
#include <stdio.h>
#define LIBCCAR_IMPLEMENTATION
#include "libccar.h"

int main(void) {
  lcc_car_t car = lcc_car_create(LCC_PRESET_SPORTS);

  /* Turn key to RUN and crank engine */
  lcc_car_set_keypos(&car, LCC_KEY_RUN);
  lcc_car_set_ignition(&car, LCC_IGNITION_ON);
  for(int i = 0; i < 120; ++i) lcc_car_update(&car, 1.0f / 120.0f); /* simulate 1 second at 120Hz to start the engine */
  lcc_car_set_ignition(&car, LCC_IGNITION_OFF);

  /* rev the engine for half a second to build up RPM so we don't stall */
  for(int i = 0; i < 60; ++i) {
    lcc_car_set_inputs(&car, 1.0f, 0.0f, 0.0f, 0.0f); /* full throttle, no brake/steering/clutch */
    lcc_car_update(&car, 1.0f / 120.0f);
  }

  /* drive in 1st gear with full throttle */
  lcc_car_set_gear(&car, 1);
  for(int i = 0; i < 600; ++i) {
    lcc_car_set_inputs(&car, 1.0f, 0.0f, 0.0f, 0.0f);
    lcc_car_update(&car, 1.0f / 120.0f);
  }

  /* output speed and RPM */
  printf("Speed: %.1f km/h, RPM: %.0f\n", lcc_car_get_speed(&car), lcc_car_get_engine_rpm(&car));

  /* cleanup */
  lcc_car_destroy(&car);
  return 0;
}
```

### Build and Run (Linux)
```bash
gcc -std=c99 demo.c -lm -o demo
./demo
```

## API Reference

### Creation and Simulation
- `lcc_car_t lcc_car_create(lcc_preset_t preset)`: Creates a car with a preset configuration (e.g., `LCC_PRESET_SPORTS`).
- `void lcc_car_update(lcc_car_t* car, float dt)`: Updates the car simulation with a time step `dt` (in seconds).
- `void lcc_car_destroy(lcc_car_t* car)`: Frees resources associated with the car.

### Input Controls
- `void lcc_car_set_inputs(lcc_car_t* car, float throttle, float brake, float steering, float clutch)`:
  - `throttle`, `brake`, `clutch`: Range [0..1] (0 = off, 1 = full).
  - `steering`: Range [-1..1] (-1 = full left, 0 = straight, 1 = full right).

### Gearing and Metrics
- `void lcc_car_set_gear(lcc_car_t* car, int gear)`: Sets gear (-1 = reverse, 0 = neutral, 1..N = forward gears).
- `void lcc_car_shift_up(lcc_car_t* car)`: Shifts to the next gear.
- `void lcc_car_shift_down(lcc_car_t* car)`: Shifts to the previous gear.
- `float lcc_car_get_speed(const lcc_car_t* car)`: Returns speed in km/h.
- `float lcc_car_get_engine_rpm(const lcc_car_t* car)`: Returns engine RPM.
- `const char* lcc_get_version(void)`: Returns the library version string.

### Electrical and Fuel Systems
- `void lcc_car_set_keypos(lcc_car_t* car, lcc_key_state_t key)`: Sets key position (`LCC_KEY_OFF`, `LCC_KEY_RUN`).
- `void lcc_car_set_ignition(lcc_car_t* car, lcc_ignition_state_t ignition)`: Sets ignition state (hold `LCC_IGNITION_ON` to crank).
- `float lcc_car_get_battery_voltage(const lcc_car_t* car)`: Returns battery voltage.
- `void lcc_car_set_accessory_load(lcc_car_t* car, float watts)`: Sets electrical load from accessories (in watts).
- `float lcc_car_get_fuel_level_L(const lcc_car_t* car)`: Returns fuel level in liters.
- `void lcc_car_refuel(lcc_car_t* car, float liters)`: Adds fuel to the tank.

## Units and Best Practices
- **Units**:
  - Mass: kilograms (kg)
  - Distance: meters (m)
  - Speed: meters per second (m/s) internally, km/h for `lcc_car_get_speed`
  - Torque: Newton-meters (Nm)
  - Force: Newtons (N)
- **Time Step**: Use a `dt` of at least 1/100 seconds (e.g., 1/120 or 1/100) for stable simulation.

## Optional Rust GUI Demo
The `car_demo/` directory includes an optional Rust-based GUI visualizer using `egui`, which links to a shared `libccar` library in `car_demo/lib/`.

### Running the Demo
```bash
cd car_demo
cargo run --release
```

**Requirements**: Install `clang` for `bindgen` (used for Rust-C FFI bindings).
