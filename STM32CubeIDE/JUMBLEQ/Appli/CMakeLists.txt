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

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Compiler options
set(STM32_MCU_FLAGS  "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard " )

#Linker options
set(STM32_LINKER_SCRIPT STM32H7S3Z8TX_ROMxspi1.ld)
set(STM32_LINKER_OPTION  )

# Set the project name
set(CMAKE_PROJECT_NAME JUMBLEQ_Appli)
project(${CMAKE_PROJECT_NAME})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Include mx-generated
include("mx-generated.cmake")


# In order to use CMake for cross-compiling
set(CMAKE_CXX_COMPILER_FORCED "true")

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    $<$<CONFIG:Debug>:TCPP0203_SUPPORT>
)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb/src
    ${CMAKE_CURRENT_SOURCE_DIR}/stm32-ssd1306/ssd1306
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/SEGGER_RTT.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/SEGGER_RTT_printf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/SigmaStudioFW.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/adau1466.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/ak4619.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/audio_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/eeprom.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/led_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/oled_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/tcpp0203.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/tcpp0203_reg.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/ui_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/usb_descriptors.c
    ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb/src/tusb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb/src/common/tusb_fifo.c
    ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb/src/device/usbd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb/src/device/usbd_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb/src/class/audio/audio_device.c
    ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb/src/class/midi/midi_device.c
    ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb/src/portable/synopsys/dwc2/dcd_dwc2.c
    ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb/src/portable/synopsys/dwc2/dwc2_common.c
    ${CMAKE_CURRENT_SOURCE_DIR}/stm32-ssd1306/ssd1306/main_oled.c
    ${CMAKE_CURRENT_SOURCE_DIR}/stm32-ssd1306/ssd1306/sub_oled.c
    ${CMAKE_CURRENT_SOURCE_DIR}/stm32-ssd1306/ssd1306/ssd1306_fonts.c
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined libraries
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()







