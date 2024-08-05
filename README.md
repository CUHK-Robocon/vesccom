# vesccom

> C++ library for communicating with custom VESC motor control firmware

## Requirements

- CMake
- A CMake generator e.g. Make, Ninja
- C++17 compiler
- Boost

## Build

```sh
mkdir build
cmake -B build
cmake --build build
```

## Use with CMake

Only a static library target `vesccom` is added.

In `CMakeLists.txt`, add:
```cmake
add_subdirectory(vesccom)
```
before adding your target and add
```cmake
target_link_libraries(<target> vesccom)
```
after adding your target to link against `vesccom`.
