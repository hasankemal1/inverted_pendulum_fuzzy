# Inverted Pendulum Fuzzy

Nonlinear inverted pendulum control simulation using a Mamdani-type fuzzy PID controller, implemented in C++17 with SFML, ImGui, ImPlot, and FuzzyLite.

## Features

- Real-time simulation of a nonlinear inverted pendulum on a cart.
- Mamdani-style fuzzy PID control for dynamic tuning of Kp, Ki, and Kd.
- Interactive GUI for adjusting force, speed, and friction, and toggling control.
- Live phase-diagram and error-signal plots via ImPlot.
- Error data output to `output.txt` for offline analysis.

## Dependencies

- C++17 compiler (MSVC, GCC, or Clang)
- CMake 3.10 or later
- vcpkg (optional), or manually installed dependencies:
  - SFML (Network, Graphics, Window, Audio, System)
  - ImGui
  - ImGui-SFML
  - ImPlot
  - FuzzyLite (included as a submodule in `thirdparty/fuzzylite`)

## Project Structure

```text
├── CMakeLists.txt
├── src/
│   └── main.cpp
├── include/
│   ├── pendulum.hpp
│   └── fuzzy.hpp
├── thirdparty/
│   └── fuzzylite/
├── resources/
│   └── ... assets ...
├── fEngine.fll         # FuzzyLite rule definition file
├── output.txt          # Runtime error log
└── README.md
```

## Build Instructions

Clone the repository with its submodules:

```bash
git clone --recursive https://github.com/hasankemal1/inverted_pendulum_fuzzy.git
cd inverted_pendulum_fuzzy
mkdir build
cd build
```

If you are using vcpkg, configure the project with its toolchain file:

```bash
cmake .. -DCMAKE_TOOLCHAIN_FILE=<path-to-vcpkg>/scripts/buildsystems/vcpkg.cmake
cmake --build . --config Release
```

If you are not using vcpkg, omit the `-DCMAKE_TOOLCHAIN_FILE` option and ensure all dependencies are installed system-wide:

```bash
cmake ..
cmake --build . --config Release
```
