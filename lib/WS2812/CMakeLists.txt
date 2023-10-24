add_library(WS2812 INTERFACE)
target_sources(WS2812 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/WS2812.cpp
)

pico_generate_pio_header(WS2812 ${CMAKE_CURRENT_LIST_DIR}/src/WS2812.pio)

target_include_directories(WS2812 INTERFACE
    include
)

target_link_libraries(WS2812 INTERFACE
    pico_stdlib
    hardware_pio
)