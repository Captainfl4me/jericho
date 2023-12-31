# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Add Debug flag if build type is Debug
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG)
endif()

# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C   standards
project(Jericho C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)
set(PICO_CXX_ENABLE_RTTI 1)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

add_subdirectory(lib/FatFs_SPI build_fastfs_spi)
add_subdirectory(lib/WS2812 build_ws2812)

# Tell CMake where to find the executable source file
add_executable(${PROJECT_NAME} 
src/main.cpp
src/MPU6050.cpp
src/BMP280.cpp
src/hw_config.cpp
src/logger.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE include)

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

# Enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
