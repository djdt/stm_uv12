# Set the compiler
set(CMAKE_SYSTEM_NAME  Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER   arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc -x assembler-with-cpp)

set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_SIZE arm-none-eabi-size)

# Flags
set(MCU "-mthumb -mcpu=${MCU_ARCH}")

set(COMMON_FLAGS "${MCU} -Wall -fdata-sections -ffunction-sections")

set(CMAKE_ASM_FLAGS "${MCU} -x assembler-with-cpp")
set(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu99")
set(CMAKE_C_FLAGS_DEBUG "-Os -g -gdwarf-2")
set(CMAKE_C_FLAGS_RELEASE "-Os")
set(CMAKE_EXE_LINKER_FLAGS "${MCU} --specs=nano.specs -T${CMAKE_SOURCE_DIR}/${LINKER_SCRIPT} -lc -lm -lnosys -Wl,-Map=${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map,--cref -Wl,--gc-sections")
