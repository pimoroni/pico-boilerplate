cmake_minimum_required(VERSION 3.12)

# Change your executable name to something creative!
set(NAME loopo-firmware) # <-- Name your project/executable here!

include(pimoroni_pico_import.cmake)
include(pico_sdk_import.cmake)

# Gooey boilerplate
project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# Add your source files
add_executable(${NAME}
    main.cpp
)

# Include required libraries
# This assumes `pimoroni-pico` is stored alongside your project
#include(common/pimoroni_i2c)
include(libraries/motor2040/motor2040)
include(drivers/pid/pid)
include(drivers/button/button)
include(drivers/motor/motor)
include(drivers/motor/motor_cluster)
include(drivers/pwm/pwm_cluster)
include(drivers/pwm/pwm)
include(drivers/plasma/plasma)
include(drivers/encoder/encoder)
include(drivers/analog/analog)
# include(libraries/pico_display/pico_display)

# Don't forget to link the libraries you need!
target_link_libraries(${NAME}
    #pimoroni_i2c
    motor2040
    button
    analog
    pid 
    plasma
    pwm_cluster
    pwm
    motor_cluster
    motor
    extension# <-- List libraries here!
)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# Set up files for the release packages
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.uf2
    ${CMAKE_CURRENT_LIST_DIR}/README.md
    DESTINATION .
)

set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_GENERATOR "ZIP" "TGZ")
include(CPack)
add_subdirectory(extension)
