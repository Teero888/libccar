# WORK IN PROGRESS
# WORK IN PROGRESS
# WORK IN PROGRESS
# WORK IN PROGRESS
# WORK IN PROGRESS
# DONT READ OR USE

# libccar

**libccar** is a self-contained **C99 single-header** library for a simple 2D top-down semi-realistic car simulation.
It’s lightweight, easy to integrate, and includes an optional Rust demo for visualization.

## Features
* Single-header C99 design
* Optional Rust FFI demo
* Cross-platform (should be, haven't tried anything except linux)

## Usage (C)
```c
#define LIBCCAR_IMPLEMENTATION
#include "libccar.h"
```
Compile:
```sh
cc -std=c99 my_program.c -lm -o my_program
```
## Building the rust demo

```sh
cd libccar/car_demo
# Linux: cc --std=c99 -shared -fPIC -o lib/libccar.so ../build_libccar.c
# macOS: cc --std=c99 -dynamiclib -o lib/libccar.so ../build_libccar.c
# Windows: cc --std=c99 -shared -o lib/libccar.so ../build_libccar.c
cargo run --release
```

# Notes
This actually came in to existance because i was searching for a simple way to simulate a top down car like this but couldn't find anything that would fit my use case without bloat.
I haven't tried this on windows or macOS yet so it might not work at all or need some tweaks.