add_library(kitronik_pico_robotics STATIC
    src/kitronik_pico_robotics.cpp
)

target_include_directories(kitronik_pico_robotics PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(kitronik_pico_robotics
    pico_stdlib
    hardware_i2c
)
