# EC-26 哨兵舵轮云台工程

本工程是 EC-26 哨兵舵轮机器人的云台板固件，目标芯片为 STM32F407，包含云台、发射、遥控、视觉通信、裁判系统通信及电机驱动等功能。

## 开发环境

- VS Code
- CMake 3.24 或更高版本
- Ninja
- GNU Arm Embedded Toolchain（`arm-none-eabi-gcc`）
- clangd（用于代码补全和静态诊断）
- J-Link（用于烧录和调试）

请确保 `cmake`、`ninja` 和 ARM GCC 已加入系统 `PATH`，或根据本机安装位置配置工具链。

## 编译方法

在工程根目录执行 Debug 构建：

```powershell
cmake --preset debug
cmake --build --preset debug
```

执行 Release 构建：

```powershell
cmake --preset release
cmake --build --preset release
```

Debug 固件默认生成在：

```text
build-cmake/debug/basic_framework.elf
```

首次打开工程时，需要先完成一次 CMake 配置和编译，clangd 才能读取生成的 `compile_commands.json`。

## 工程结构

```text
application/    机器人应用层代码，包括云台、发射和命令处理
bsp/            板级外设封装
cmake/          CMake 工具链与源码清单
Drivers/        STM32 HAL 和 CMSIS 驱动
Inc/、Src/      CubeMX 生成的外设初始化代码
Middlewares/    FreeRTOS、SEGGER RTT 等中间件
modules/        电机、IMU、遥控、通信和算法模块
scripts/        烧录等辅助脚本
```
