add_library(extension INTERFACE)

target_include_directories(extension INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(extension INTERFACE motor2040 button pid plasma pwm_cluster pwm motor_cluster motor)