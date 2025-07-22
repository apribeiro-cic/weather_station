
cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico_w CACHE STRING "Board type")
include(pico_sdk_import.cmake)
project(weather_station C CXX ASM)
pico_sdk_init()


include_directories( ${CMAKE_SOURCE_DIR}/lib ) # Inclui os files .h na pasta lib

add_executable(${PROJECT_NAME}  
        weather_station.c 
        lib/aht20.c 
        lib/bmp280.c 
        lib/ssd1306.c
        )

pico_generate_pio_header(weather_station ${CMAKE_CURRENT_LIST_DIR}/weather_station.pio)
target_sources(weather_station PRIVATE weather_station.c)

target_include_directories(weather_station PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/lib
)

target_link_libraries(${PROJECT_NAME} 
        pico_stdlib 
        hardware_i2c
        hardware_gpio
        hardware_pio
        hardware_pwm
        pico_cyw43_arch_lwip_threadsafe_background
        )

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 1)

pico_add_extra_outputs(${PROJECT_NAME})
