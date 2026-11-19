cmake_minimum_required(VERSION 3.22)

project(basic_framework LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 加载CubeMX代码和项目代码清单
include(cmake/cubemx-generated.cmake)
include(cmake/project-sources.cmake)

set(TARGET_NAME basic_framework)

# 先使用Makefile目前使用的链接脚本，保证两套构建一致
set(
    LINKER_SCRIPT
    "${CMAKE_CURRENT_SOURCE_DIR}/STM32F407XX_FLASH.ld"
)

add_executable(
    ${TARGET_NAME}
    ${CUBEMX_SOURCES}
    ${PROJECT_SOURCES}
)

set_target_properties(
    ${TARGET_NAME}
    PROPERTIES
    SUFFIX ".elf"
)

# 两个小写.s文件都需要经过C预处理器
set_source_files_properties(
    startup_stm32f407xx.s
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_ASM_ARMv7M.s
    PROPERTIES
    COMPILE_OPTIONS "-x;assembler-with-cpp"
)

target_include_directories(
    ${TARGET_NAME}
    PRIVATE
    ${CUBEMX_INCLUDE_DIRS}
    ${PROJECT_INCLUDE_DIRS}
)

target_compile_definitions(
    ${TARGET_NAME}
    PRIVATE
    ${CUBEMX_DEFINES}
)

# STM32F407：Cortex-M4F，单精度硬件浮点
set(MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

target_compile_options(
    ${TARGET_NAME}
    PRIVATE
    ${MCU_FLAGS}

    -Wall
    -fmessage-length=0
    -fdata-sections
    -ffunction-sections
    -fno-common

    $<$<CONFIG:Debug>:-Og>
    $<$<CONFIG:Debug>:-g>
    $<$<CONFIG:Debug>:-gdwarf-2>

    $<$<CONFIG:Release>:-O2>
)

target_link_directories(
    ${TARGET_NAME}
    PRIVATE
    Middlewares/ST/ARM/DSP/Lib
)

target_link_options(
    ${TARGET_NAME}
    PRIVATE
    ${MCU_FLAGS}

    --specs=nano.specs
    -T${LINKER_SCRIPT}

    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.map,--cref
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)

target_link_libraries(
    ${TARGET_NAME}
    PRIVATE
    c
    m
    nosys

    # 对应Makefile中的-l:libCMSISDSP.a
    "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib/libCMSISDSP.a"
)

add_custom_command(
    TARGET ${TARGET_NAME}
    POST_BUILD

    COMMAND
        ${CMAKE_SIZE}
        $<TARGET_FILE:${TARGET_NAME}>

    COMMAND
        ${CMAKE_OBJCOPY}
        -O ihex
        $<TARGET_FILE:${TARGET_NAME}>
        ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.hex

    COMMAND
        ${CMAKE_OBJCOPY}
        -O binary
        -S
        $<TARGET_FILE:${TARGET_NAME}>
        ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.bin

    COMMENT
        "Generating size report, HEX and BIN images"
)