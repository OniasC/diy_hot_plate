cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME stm32_hot_plate)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME} C CXX ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# CubeMX rewrites cmake/stm32cubemx/CMakeLists.txt on every regeneration and always
# lists Core/Src/main.c, the empty stub. Drop it here so the C++ main below wins.
get_target_property(MX_SOURCES ${CMAKE_PROJECT_NAME} SOURCES)
list(FILTER MX_SOURCES EXCLUDE REGEX "Core/Src/main\\.c$")
set_property(TARGET ${CMAKE_PROJECT_NAME} PROPERTY SOURCES ${MX_SOURCES})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# u8g2 ships ~88 translation units; --gc-sections drops the unused display drivers
file(GLOB U8G2_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/third_party/u8g2/*.c)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/main.cpp
    Core/Src/fault_handler.c
    api/Error_Report.c
    platform/interupts_callbacks.c
    platform/api_hal/api_hal.c
    platform/api_hal/api_hal_adc.c
    platform/api_hal/api_hal_pwm.c
    platform/buzzer/buzzer.c
    platform/gpio/gpio.c
    platform/led/led.c
    third_party/NTC_Thermistor_hpp/NTC_Thermistor.cpp
    app/hot_plate/hot_place.cpp
    ui/menu/graph_lib/graph.c
    ui/menu/graph_lib/graph_chart.c
    ui/menu/graph_lib/graph_list.c
    ui/menu/graph_lib/graph_text_bar.c
    board/bsp.c
    ${U8G2_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/app          # lets graph.h resolve its "../api/u8g2/u8g2.h"
    ${CMAKE_SOURCE_DIR}/api
    ${CMAKE_SOURCE_DIR}/api/u8g2
    ${CMAKE_SOURCE_DIR}/bsp
    ${CMAKE_SOURCE_DIR}/app/hot_plate
    ${CMAKE_SOURCE_DIR}/app/menu/graph_lib
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# newlib-nano omits float conversion from printf unless _printf_float is forced in
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
)

# 'flash' target: cmake --build --preset Debug --target flash
file(GLOB CUBECLT_DIRS "C:/ST/STM32CubeCLT_*/STM32CubeProgrammer/bin")
find_program(STM32_PROGRAMMER_CLI
    NAMES STM32_Programmer_CLI
    PATHS ${CUBECLT_DIRS}
          "$ENV{ProgramFiles}/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin"
    DOC "STM32CubeProgrammer CLI used by the 'flash' target"
)

if(STM32_PROGRAMMER_CLI)
    add_custom_target(flash
        COMMAND ${CMAKE_COMMAND}
                -DPROG=${STM32_PROGRAMMER_CLI}
                -DELF=$<TARGET_FILE:${CMAKE_PROJECT_NAME}>
                -P ${CMAKE_SOURCE_DIR}/cmake/flash.cmake
        USES_TERMINAL
        COMMENT "Flashing ${CMAKE_PROJECT_NAME} over SWD"
    )
    add_dependencies(flash ${CMAKE_PROJECT_NAME})
else()
    message(STATUS "STM32_Programmer_CLI not found - 'flash' target unavailable")
endif()
