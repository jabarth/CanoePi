cmake_minimum_required(VERSION 3.13)

# Initialize PICO SDK first
# The PICO_SDK_PATH is set by the ContainerFile environment
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(pico_firmware C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Add pico_extras
pico_add_extra_outputs(${PROJECT_NAME})

# Add Micro-ROS library
# The ContainerFile clones the micro_ros_raspberrypi_pico_sdk into /micro_ros_ws
add_subdirectory(/micro_ros_ws)

# Add the executable
add_executable(${PROJECT_NAME}
    src/main.c
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_adc
    hardware_pwm
    hardware_pio
    microros
)

# Enable USB and UART output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 1)

# Create the .uf2 file
pico_add_uf2_output(${PROJECT_NAME})
