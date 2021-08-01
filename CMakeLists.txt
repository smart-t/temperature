cmake_minimum_required(VERSION 3.13)

# initialize pico-sdk from GIT
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_FETCH_FROM_GIT on)

# pico_sdk_import.cmake is a single file copied from this SDK
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(temperature)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project
add_executable(temperature
    temperature.c
    bigval.c
)

pico_set_program_name(temperature "temperature")
pico_set_program_version(temperature "0.1")

pico_enable_stdio_uart(temperature 1)
pico_enable_stdio_usb(temperature 1)

# Add the standard library to the build
target_link_libraries(temperature
	pico_stdlib
	hardware_i2c
)

pico_add_extra_outputs(temperature)
