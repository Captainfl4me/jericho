# Tell CMake where to find the executable source file
add_executable(${PROJECT_NAME} 
    main.cpp
    MPU6050.cpp
    HW611.cpp
    hw_config.cpp
    logger.cpp
)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

# Link to pico_stdlib (gpio, time, etc. functions)
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib
)

# Add any user requested libraries
target_link_libraries(${PROJECT_NAME}
    hardware_pio
    hardware_i2c
    pico_multicore
    FatFs_SPI
    WS2812
)