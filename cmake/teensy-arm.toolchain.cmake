# Copyright (c) 2015, Pierre-Andre Saulais <pasaulais@free.fr>
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer. 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

set(TRIPLE "arm-none-eabi")
if(NOT DEFINED TOOLCHAIN_ROOT)
    set(TOOLCHAIN_ROOT "/usr")
endif()
if(NOT DEFINED TEENSY_BOARD)
    set(TEENSY_BOARD "teensy31")
endif()
if(NOT DEFINED TEENSY_CORES_ROOT)
    set(TEENSY_CORES_ROOT "/usr/share/arduino/hardware/teensy/cores" CACHE PATH "Path to the Teensy 'cores' repository")
endif()

if(TEENSY_BOARD STREQUAL "teensy40" OR TEENSY_BOARD STREQUAL "teensy41")
    set(TEENSY_ROOT "${TEENSY_CORES_ROOT}/teensy4")
else()
    set(TEENSY_ROOT "${TEENSY_CORES_ROOT}/teensy3")
endif()

if(NOT DEFINED ARDUINO_LIB_ROOT)
    set(ARDUINO_LIB_ROOT "/usr/share/arduino/libraries" CACHE PATH "Path to the Arduino library directory")
endif()
if(NOT DEFINED ARDUINO_VERSION)
    set(ARDUINO_VERSION "106" CACHE STRING "Version of the Arduino SDK")
endif()
if(NOT DEFINED TEENSYDUINO_VERSION)
    set(TEENSYDUINO_VERSION "120" CACHE STRING "Version of the Teensyduino SDK")
endif()

if(TEENSY_BOARD STREQUAL "teensy40" OR TEENSY_BOARD STREQUAL "teensy41")
    set(TEENSY_MODEL "IMXRT1062")
    if(NOT DEFINED TEENSY_FREQUENCY)
        set(TEENSY_FREQUENCY "600" CACHE STRING "Frequency of the Teensy MCU (MHz)")
    endif()
else()
    # set(TEENSY_MODEL "MK20DX256" CACHE STRING "Model of the Teensy MCU")
    set(TEENSY_MODEL "MK20DX256") # XXX Add Teensy 3.0 support.
    if(NOT DEFINED TEENSY_FREQUENCY)
        set(TEENSY_FREQUENCY "96" CACHE STRING "Frequency of the Teensy MCU (MHz)")
    endif()
endif()

set_property(CACHE TEENSY_FREQUENCY PROPERTY STRINGS 1008 960 912 816 720 600 528 450 396 150 96 72 48 24 16 8 4 2)

if(NOT DEFINED TEENSY_USB_MODE)
    set(TEENSY_USB_MODE "SERIAL" CACHE STRING "What kind of USB device the Teensy should emulate")
endif()
set_property(CACHE TEENSY_USB_MODE PROPERTY STRINGS
             SERIAL DUAL_SERIAL TRIPLE_SERIAL
             KEYBOARD TOUCHSCREEN
             HID SERIAL_HID HID_TOUCH
             MIDI MIDI4 MIDI16
             MIDI_SERIAL MIDI4_SERIAL MIDI16_SERIAL
             AUDIO MIDI_AUDIO_SERIAL MIDI16_AUDIO_SERIAL
             MTPDISK MTPDISK_SERIAL
             RAWHID FLIGHTSIM FLIGHTSIM_JOYSTICK)

if(WIN32)
    set(TOOL_OS_SUFFIX .exe)
else(WIN32)
    set(TOOL_OS_SUFFIX )
endif(WIN32)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_CROSSCOMPILING 1)

set(CMAKE_C_COMPILER "${TOOLCHAIN_ROOT}/bin/${TRIPLE}-gcc${TOOL_OS_SUFFIX}" CACHE PATH "gcc" FORCE)
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_ROOT}/bin/${TRIPLE}-g++${TOOL_OS_SUFFIX}" CACHE PATH "g++" FORCE)
set(CMAKE_AR "${TOOLCHAIN_ROOT}/bin/${TRIPLE}-ar${TOOL_OS_SUFFIX}" CACHE PATH "archive" FORCE)
set(CMAKE_LINKER "${TOOLCHAIN_ROOT}/bin/${TRIPLE}-ld${TOOL_OS_SUFFIX}" CACHE PATH "linker" FORCE)
set(CMAKE_NM "${TOOLCHAIN_ROOT}/bin/${TRIPLE}-nm${TOOL_OS_SUFFIX}" CACHE PATH "nm" FORCE)
set(CMAKE_OBJCOPY "${TOOLCHAIN_ROOT}/bin/${TRIPLE}-objcopy${TOOL_OS_SUFFIX}" CACHE PATH "objcopy" FORCE)
set(CMAKE_OBJDUMP "${TOOLCHAIN_ROOT}/bin/${TRIPLE}-objdump${TOOL_OS_SUFFIX}" CACHE PATH "objdump" FORCE)
set(CMAKE_STRIP "${TOOLCHAIN_ROOT}/bin/${TRIPLE}-strip${TOOL_OS_SUFFIX}" CACHE PATH "strip" FORCE)
set(CMAKE_RANLIB "${TOOLCHAIN_ROOT}/bin/${TRIPLE}-ranlib${TOOL_OS_SUFFIX}" CACHE PATH "ranlib" FORCE)

include_directories("${TEENSY_ROOT}")

if(TEENSY_BOARD STREQUAL "teensy40" OR TEENSY_BOARD STREQUAL "teensy41")
    set(TARGET_FLAGS "-mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16")
    set(BASE_FLAGS "-g -Wall -ffunction-sections -fdata-sections -nostdlib ${TARGET_FLAGS}")

    set(CMAKE_CXX_FLAGS "${BASE_FLAGS} -fno-exceptions -fpermissive -fno-rtti -fno-threadsafe-statics -felide-constructors -std=gnu++0x -Wno-error=narrowing" CACHE STRING "c++ flags")

    if(TEENSY_BOARD STREQUAL "teensy41")
        set(TEENSY_LD "imxrt1062_t41.ld")
        add_definitions(-DARDUINO_TEENSY41)
    else()
        set(TEENSY_LD "imxrt1062.ld")
        add_definitions(-DARDUINO_TEENSY40)
    endif()

    set(LINKER_LIBS "-larm_cortexM7lfsp_math -lm -lstdc++" )
else()
    set(TARGET_FLAGS "-mcpu=cortex-m4 -mthumb")
    set(BASE_FLAGS "-Os -Wall -nostdlib -ffunction-sections -fdata-sections ${TARGET_FLAGS}")

    set(CMAKE_CXX_FLAGS "${BASE_FLAGS} -fno-exceptions -fno-rtti -felide-constructors -std=gnu++0x" CACHE STRING "c++ flags")

    set(TEENSY_LD "mk20dx256.ld" )
    set(LINKER_LIBS "-larm_cortexM4l_math -lm" )
endif()

set(CMAKE_C_FLAGS "${BASE_FLAGS} -DTIME_T=1421620748" CACHE STRING "c flags") # XXX Generate TIME_T dynamically.
set(LINKER_FLAGS "-Os -Wl,--gc-sections,--relax ${TARGET_FLAGS} -T${TEENSY_ROOT}/${TEENSY_LD}" )

set(CMAKE_SHARED_LINKER_FLAGS "${LINKER_FLAGS}" CACHE STRING "linker flags" FORCE)
set(CMAKE_MODULE_LINKER_FLAGS "${LINKER_FLAGS}" CACHE STRING "linker flags" FORCE)
set(CMAKE_EXE_LINKER_FLAGS "${LINKER_FLAGS}" CACHE STRING "linker flags" FORCE)

# Do not pass flags like '-ffunction-sections -fdata-sections' to the linker.
# This causes undefined symbol errors when linking.
set(CMAKE_CXX_LINK_EXECUTABLE "<CMAKE_C_COMPILER> <CMAKE_CXX_LINK_FLAGS> <LINK_FLAGS> -o <TARGET>  <OBJECTS> <LINK_LIBRARIES> ${LINKER_LIBS}" CACHE STRING "Linker command line" FORCE)

add_definitions("-DARDUINO=${ARDUINO_VERSION}")
add_definitions("-DTEENSYDUINO=${TEENSYDUINO_VERSION}")
add_definitions("-D__${TEENSY_MODEL}__")
add_definitions(-DLAYOUT_US_ENGLISH)
add_definitions(-DUSB_VID=null)
add_definitions(-DUSB_PID=null)
add_definitions(-MMD)
