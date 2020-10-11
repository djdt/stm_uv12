cmake_minimum_required(VERSION 3.6)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_EXPORT_COMPILE_COMMANDS Yes)

set(MCU_ARCH cortex-m0)
set(MCU_CHIP STM32F042x6)
set(MCU_FAMILY F0)

set(MCU_FPU none)
set(MCU_FLOAT_ABI soft)

set(LINKER_SCRIPT STM32F042K6Tx_FLASH.ld)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/STM32Toolchain.cmake)

enable_language(ASM)
project(uv12)

#######################################################################
#                           End user config                           #
#######################################################################

add_definitions(-D${MCU_CHIP})
add_definitions(-DUSE_HAL_LIBRARY)

string(TOLOWER ${MCU_CHIP} MCU_CHIP)

# Project setup
file(GLOB_RECURSE PROJECT_SOURCES Core/Src/*.c)
file(GLOB_RECURSE HAL_SOURCES Drivers/STM32${MCU_FAMILY}xx_HAL_Driver/Src/*.c)
file(GLOB_RECURSE MDL_SOURCES Middlewares/ST/STM32_USB_Device_Library/*.c)
set(STARTUP_SOURCE startup_${MCU_CHIP}.s)

include_directories(
    Core/Inc
    Drivers/STM32${MCU_FAMILY}xx_HAL_Driver/Inc
    Drivers/STM32${MCU_FAMILY}xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32${MCU_FAMILY}xx/Include
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc
)

set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES
    "${PROJECT_NAME}.hex;${PROJECT_NAME}.bin;${PROJECT_NAME}.map")

add_executable(${PROJECT_NAME}.elf ${PROJECT_SOURCES} ${HAL_SOURCES} ${MDL_SOURCES} ${STARTUP_SOURCE} ${LINKER_SCRIPT})
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf)

# target_link_libraries(${PROJECT_NAME}.elf -lc -lm -lnosys)

add_custom_target(hex DEPENDS ${PROJECT_NAME}.elf COMMAND ${CMAKE_OBJCOPY} -Oihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex)
add_custom_target(bin DEPENDS ${PROJECT_NAME}.elf COMMAND ${CMAKE_OBJCOPY} -Obinary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin)
add_custom_target(size DEPENDS ${PROJECT_NAME}.elf COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf)
add_custom_target(flash DEPENDS hex COMMAND st-flash --format ihex write ${PROJECT_NAME}.hex)
