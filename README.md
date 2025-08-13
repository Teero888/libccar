# WORK IN PROGRESS
# WORK IN PROGRESS
# WORK IN PROGRESS
# WORK IN PROGRESS
# WORK IN PROGRESS
# DONT USE

# libccar

**libccar** is a self-contained **C99 single-header** library for a simple 2D top-down semi-realistic car simulation.
It’s lightweight, easy to integrate, and includes an optional Rust demo for visualization.

## Features
* Rear-wheel drive with throttle, brake, and steering
* Linear lateral tire model
* Single-header C99 design
* Optional Rust FFI demo
* Cross-platform (should be, haven't tried anything except linux)

![screenshot of the demo](https://files.catbox.moe/2cy74h.png "Screenshot of the rust demo")

## Usage (C)
```c
#define LIBCCAR_IMPLEMENTATION
#include "libccar.h"
```
Compile:
```sh
gcc -std=c99 my_program.c -o my_program
```
## Building the rust demo

```sh
cd libccar
# Linux: gcc --std=c99 -shared -fPIC -o car_demo/libccar.so build_libccar.c
# macOS: gcc --std=c99 -dynamiclib -o car_demo/libccar.dylib build_libccar.c
# Windows: gcc --std=c99 -shared -o car_demo/libccar.dll build_libccar.c
cd car_demo
cargo run --release
```

# Notes
This actually came in to existance because i was searching for a simple way to simulate a top down car like this but couldn't find anything that would fit my use case without bloat.
I haven't tried this on windows or macOS yet so it might not work at all or need some tweaks.
Driving in the demo might look a little bit wonky because you can only do 0 or 100% throttle with a keyboard, same thing with the steering. So you basically slip instantly unless your motor is super weak since there are no gears.
I might extend this in the future but for now regulating your inputs is your problem when using the library and not mine so it's fine.
