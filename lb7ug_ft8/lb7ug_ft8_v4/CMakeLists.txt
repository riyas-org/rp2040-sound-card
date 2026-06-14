cmake_minimum_required(VERSION 3.13)

if (NOT PICO_SDK_PATH)
    set(PICO_SDK_PATH "$ENV{HOME}/pico-sdk")
endif()

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)



project(si4732_test C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(si4732_test
    main.c
    hal/i2c_bus.c
    hal/oled.c
    hal/si5351.c
    hal/si4732.c
)

target_include_directories(si4732_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/hal
    ${CMAKE_CURRENT_LIST_DIR}/radio
)

# Pull in common SDK libraries
target_link_libraries(si4732_test
    pico_stdlib
    hardware_i2c
    hardware_gpio
)

pico_add_extra_outputs(si4732_test)

# Enable USB stdio (optional)
pico_enable_stdio_usb(si4732_test 1)
pico_enable_stdio_uart(si4732_test 0)
