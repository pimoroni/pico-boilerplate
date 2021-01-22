cmake_minimum_required(VERSION 3.12)

# Make sure you symlink this file into the project directory
# eg: ln -s ../pico-sdk/external/pico_sdk_import.cmake .
include(pico_sdk_import.cmake)

# Change your executable name to something creative!
set(EXECUTABLE_NAME pico-boilerplate) # <-- Name your project/executable here!

# Gooey boilerplate
project(pico-boilerplate C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# Add your source files
add_executable(pico-boilerplate
    main.cpp # <-- Add source files here!
)

# Include required libraries
# This assumes `pimoroni-pico` is stored alongside your project
include(../pimoroni-pico/libraries/pico_explorer/pico_explorer.cmake)
# include(../pimoroni-pico/libraries/pico_display/pico_display.cmake)

# Don't forget to link the libraries you need!
target_link_libraries(pico-boilerplate
    pico_explorer # <-- List libraries here!
)