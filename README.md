# Introduction

teensy-cmake is a template for Teensy projects that uses CMake to build your sketches.
This lets you develop for Teensy using your favorite text editor and terminal.
It can easily accommodate executables built from several source and header files.

This branch includes the needed VS Code configuration files to develop and build with Ninja. It is organized more as an independent project, rather than for use with multiple isolated sketches.

It has currently only been tested on Linux with Teensy 3.1 and 4.x.

# Requirements

* VS Code
* CMake
* Ninja
* Git
* A cross-compiler toolchain for ARM ('arm-none-eabi')
* Teensyduino for sketches that use libraries (e.g. Bounce)

# Setup

Clone this repository from GitHub:

```bash
git clone --recurse-submodules https://github.com/warnerjon12/teensy-cmake.git
cd teensy-cmake
git checkout vscode
```

This branch includes the Teensy 'cores' repository as a submodule, so no additional setup is required. Running 'Compile Project' in VS Code should do everything that is needed, but a terminal build can also be done:

```bash
cmake -G Ninja -B build
ninja -C build
```

This last step might fail with errors such as 'compiler not found'. In this case, run the CMake GUI:

```bash
cmake-gui ..
```

To compile for different boards, set 'TEENSY_BOARD' (e.g. `cmake -D TEENSY_BOARD=teensy40 ...`). For VS Code, this is set in `cmake-kits.json`.

'TEENSY_CORES_ROOT' points to the 'cores' directory cloned as a submodule, but this can be overridden if using a submodule is not desired or a different cores library is preferred.

To build sketches that use libraries, make sure that 'ARDUINO_LIB_ROOT' points to the Arduino library directory (e.g. /usr/share/arduino/libraries).

# Flashing sketches to the Teensy

It is easiest to use `teensy_loader_cli` for this. Flash and reboot tasks are included for use with VS Code.

# Creating your project

Place source files in the `src` folder and define target executables in `src/CMakeLists.txt`:

```
add_teensy_executable(MyProject
    MyProject.cpp
    MyProject_sensors.cpp
    MyProject_interrupts.cpp)
```

## Custom configuration

For some sketches, the Teensy needs to run in a different 'USB mode'. You can set this in the CMakeLists.txt file as well:

```
set(TEENSY_USB_MODE MIDI)

add_teensy_executable(...)
```

You can set the 'default' mode in the CMake GUI ('TEENSY_USB_MODE' variable).

# Importing libraries

Here is a simple example of how to import a library:

```
import_arduino_library(Bounce)

add_teensy_executable(...)
```

Make sure that the 'ARDUINO_LIB_ROOT' variable is set up correctly in CMake.

## External libraries

This template can be adjusted to include other libraries, but an example is not currently included.
