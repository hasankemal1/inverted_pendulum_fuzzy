Nonlinear inverted pendulum control simulation using a Mamdani-type fuzzy PID controller, implemented in C++17 with SFML, ImGui, ImPlot, and FuzzyLite.

Features

Real-time simulation of a nonlinear inverted pendulum on a cart.

Mamdani-style fuzzy PID control for dynamic tuning of Kp, Ki, Kd.

Interactive GUI for adjusting force, speed, friction, and toggling control.

Live plots: phase diagram and error signal via ImPlot.

Output of error data to output.txt for offline analysis.

Dependencies

C++17 compiler (MSVC or GCC/Clang)

CMake ≥ 3.10

vcpkg (optional) or manually installed dependencies:

SFML (Network, Graphics, Window, Audio, System)

ImGui

ImGui-SFML

ImPlot

FuzzyLite (as a submodule in thirdparty/fuzzylite)

Project Structure

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
├── output.txt          # runtime error log
└── README.md           # this file

Build Instructions

git clone --recursive https://github.com/hasankemal1/inverted_pendulum_fuzzy.git

cd fuzy
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build . --config Release

If not using vcpkg, omit the -DCMAKE_TOOLCHAIN_FILE flag and ensure all dependencies are installed system-wide.

Running the Application

cd build/bin
./fuzy

Use the ImGui controls window to:

Reset simulation state

Speed Up / Slow Down simulation frame rate

Pause / Resume physics and fuzzy control

Start/Stop Fuzzy controller

Toggle Phase Diagram and Error Plot windows

Adjust Force and Friction Force sliders

Logged error signals are appended to output.txt for analysis.

Author

hasan kemal karaman

