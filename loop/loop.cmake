add_library(loop INTERFACE)

target_sources(loop INTERFACE ${CMAKE_CURRENT_LIST_DIR}/loop.cpp)

target_include_directories(loop INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(loop INTERFACE motor2040 button analog pid plasma pwm_cluster pwm motor_cluster motor)