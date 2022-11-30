add_library(twist INTERFACE)

target_sources(twist INTERFACE ${CMAKE_CURRENT_LIST_DIR}/twist.cpp)

target_include_directories(twist INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(twist INTERFACE motor2040 button pid plasma pwm_cluster pwm motor_cluster motor)