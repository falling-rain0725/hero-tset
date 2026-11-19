# 只要把 ARM GCC 加入 PATH 就能使用
# 无需手动配置。若想手动指定？？

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(
    ARM_GCC_PATH
    ""
    CACHE PATH
    "Directory containing the Arm GNU Toolchain binaries" 
)

if(ARM_GCC_PATH)
    list(PREPEND CMAKE_PROGRAM_PATH "${ARM_GCC_PATH}")
endif()

find_program(
    CMAKE_C_COMPILER
    arm-none-eabi-gcc
    REQUIRED
)

find_program(
    CMAKE_ASM_COMPILER
    arm-none-eabi-gcc
        REQUIED

)

find_program(
    CMAKE_OBJCOPY
    arm-none-eabi-objcopy
    REQUIRED
)

find_program(
    CMAKE_SIZE
    arm-none-eabi-size
    REQUIRED
)

set(CMAKE_EXECUTABLE_SUFFIX ".elf")