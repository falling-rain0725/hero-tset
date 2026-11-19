# 云台板工程协作指南

本文件适用于当前仓库。修改代码前先确认实际编译的板型、控制链路和接口调用方；已有文档可作导航，具体行为以当前源码为准。

## 项目概况

- 本工程是 EC-26 哨兵舵轮机器人的 STM32F407 云台板固件，使用 C11、STM32 HAL、FreeRTOS 和 CMSIS-DSP。
- 主要功能包括遥控与模式切换、云台与发射机构、电机控制、IMU 姿态解算、双板 CAN、USB VCP 视觉通信、裁判系统和 UI。
- 当前 `application/robot_def.h` 选择 `GIMBAL_BOARD` 和 `VISION_USE_VCP`。仓库仍保留 `ONE_BOARD`、`CHASSIS_BOARD` 相关代码；修改这些路径时应明确目标板型，不能把当前宏配置当成所有板型的行为。
- 实际底盘固件在同级目录 `../agv_chassis_mc02/`。本仓库的 `application/chassis/` 是框架保留的底盘应用代码，不代表当前底盘板运行的固件。

## 目录结构

```text
application/       机器人应用：CMD、云台、发射、底盘及共享数据定义
bsp/               CAN、USB、串口、SPI、GPIO、DWT、日志等板级封装
modules/           电机、IMU、遥控、通信、消息中心、算法、裁判系统等模块
Src/、Inc/         CubeMX 工程入口、外设初始化、USB CDC 和中断代码
Drivers/           STM32 HAL、CMSIS 等厂商代码
Middlewares/       FreeRTOS、ST USB、SEGGER RTT、CMSIS-DSP 等
cmake/             工具链、CubeMX 源码清单和项目源码清单
.vscode/           VS Code 构建、调试配置
basic_framework.ioc CubeMX 工程配置
Makefile            原有 Make 构建入口
CMakePresets.json   Debug、Release 的 CMake/Ninja 预设
```

## 按任务定位代码

| 任务 | 主要位置 | 需要一并核对 |
| --- | --- | --- |
| 板型、机械参数、控制消息结构 | `application/robot_def.h` | 编译宏、单位、枚举值、打包结构体及底盘固件对应定义 |
| 启动、应用初始化、任务周期 | `Src/main.c`、`application/robot.c`、`application/robot_task.h` | `Src/freertos.c` 中的默认任务与 USB 初始化 |
| 遥控、急停、控制源切换、底盘命令 | `application/cmd/robot_cmd.c` | `modules/remote/`、`application/robot_def.h`、底盘固件 |
| 云台电机、PID、角度反馈和限位 | `application/gimbal/gimbal.c` | `modules/motor/DMmotor/`、`modules/motor/motor_def.h`、`modules/imu/` |
| 发射机构、摩擦轮、拨弹 | `application/shoot/shoot.c` | 电机模块、CMD 发射命令及反馈 |
| 电机调度、CAN 电机报文 | `modules/motor/motor_task.c`、`modules/motor/` | `bsp/can/`、电机型号和实际 CAN ID |
| IMU 初始化、标定、姿态解算 | `modules/imu/`、`modules/BMI088/`、`modules/ist8310/` | `application/robot_task.h` 的 INS 任务、云台反馈单位 |
| 视觉收发和帧解析 | `modules/master_machine/` | `application/cmd/robot_cmd.c`、`Src/usbd_cdc_if.c`、`bsp/usb/`、`vision_integration.md` |
| 双板通信、CAN 队列和底盘反馈 | `modules/can_comm/`、`bsp/can/` | `application/robot_def.h`、`application/cmd/robot_cmd.c`、`../agv_chassis_mc02/` |
| 应用之间的数据传递、掉线检测 | `modules/message_center/`、`modules/daemon/` | 对应应用的发布/订阅与超时处理 |
| 裁判系统、UI、日志与蜂鸣器 | `modules/referee/`、`bsp/log/`、`modules/alarm/` | 对应任务是否创建、数据来源是否实际接入 |
| 外设引脚、时钟、中断、DMA | `basic_framework.ioc`、`Src/`、`Inc/`、`bsp/` | CubeMX 生成区域和两套构建的源码清单 |
| 编译选项、链接与调试 | `CMakeLists.txt`、`cmake/`、`Makefile`、`.vscode/` | `STM32F407XX_FLASH.ld`、`STM32F407.svd` |

## 实际运行链路

```text
复位入口 → Src/main.c：HAL 与外设初始化
         → RobotInit()：BSPInit、CMD/Gimbal/Shoot 初始化、OSTaskInit
         → MX_FREERTOS_Init() → osKernelStart()
         → INS 任务：姿态解算、视觉发送（约 1 kHz）
         → 电机任务：MotorControlTask（约 1 kHz）
         → Daemon 任务：掉线检测、蜂鸣器（约 100 Hz）
         → 机器人任务：RobotCMDTask → GimbalTask → ShootTask（约 200 Hz）
         → UI 任务：裁判系统 UI
```

任务周期是当前 `application/robot_task.h` 中的 `osDelay` 设定，不是硬实时保证。定位某项行为时沿“输入/反馈 → CMD → 应用命令 → 电机或 CAN → 反馈”追踪，不只检查单个函数。

## 开发约定

### 架构和控制

- 应用层负责模式、安全状态和目标值；模块层负责电机、传感器和通信能力；BSP 负责外设访问。已有应用间数据通过 `modules/message_center/` 发布与订阅，修改命令或反馈时检查两端。
- 控制源切换、遥控离线、底盘 CAN 离线及急停逻辑集中在 `application/cmd/robot_cmd.c`。改遥控、视觉或底盘模式时检查停机行为、切换时目标同步、限位和发射默认状态。
- 云台 Yaw 当前以 IMU 多圈角作为 DM 电机角度环反馈，单位为度，使用 `DM_ANGLE_NEAREST_DEG`；Pitch 控制目标为弧度并受 `PITCH_MIN`、`PITCH_MAX` 限制。改反馈来源、PID 或就近转位算法时，逐项核对目标、反馈、速度、方向和单位；不要仅凭变量名推断。
- 电机 ID、CAN 总线、PID、机械零位和方向均与实物配置有关。修改这些参数时区分“编译通过”和“实机方向、幅度、稳定性已验证”。

### 接口与兼容

- `application/robot_def.h` 中的共享结构体使用紧凑布局，双板 CAN 按这些结构体的字节发送。修改字段顺序、类型、长度、枚举或角度单位前，核对 `../agv_chassis_mc02/` 的收发实现和实际固件版本；必要时同步修改两端。
- 视觉通信由 `modules/master_machine/` 解析与发送，CMD 消费目标并回传姿态；当前协议说明见 `modules/master_machine/vision_integration.md`。涉及帧头、长度、CRC、flags、float 顺序、坐标符号或超时的改动，应同时检查实现、文档和上位机约定。
- `vision_integration.md` 可能与后续代码产生偏差。发现不一致时先以源码确认当前实际行为，再决定是修实现还是更新文档，不要把文档中的预期当成已验证事实。
- 不能把云台板本地 `application/chassis/` 的实现直接当作实际底盘板协议依据；双板问题必须检查两个仓库。

### 代码和生成文件

- 维持周边 C 代码的命名、缩进和文件换行风格；优先做范围明确的修改，避免顺手格式化整个文件。仓库没有统一的 `.clang-format` 或 Clang-Tidy 强制规则，不要套用其他工程的规则。
- `Src/`、`Inc/` 含 CubeMX 生成代码，优先使用 `USER CODE` 区域或现有扩展点；确需修改 USB CDC、回调或初始化等生成区域时可以修改，但应说明再生成的覆盖风险并避免无关生成差异。
- `Drivers/` 和 `Middlewares/` 主要是第三方代码。优先在 `bsp/`、`modules/`、`application/` 扩展；确有底层缺陷需要改厂商代码时，保持补丁集中且说明原因。
- 新增或移除项目源码时检查 `cmake/project-sources.cmake` 与 `Makefile`；涉及 CubeMX 生成源码时再核对 `cmake/cubemx-generated.cmake`。不要假定两套构建会自动同步。
- 不提交 `build/`、`build-cmake/`、本地日志、Ozone/VS Code 个人状态等产物。不要为了一个小改动引入无关测试桩、图片资源或重新组织根目录。

## 构建与调试

在工程根目录运行，确保 CMake 3.24+、Ninja 和 `arm-none-eabi-gcc` 可用：

```powershell
cmake --preset debug
cmake --build --preset debug

cmake --preset release
cmake --build --preset release
```

- CMake Debug 输出在 `build-cmake/debug/`，包括 `basic_framework.elf`、`.hex`、`.bin` 和 `compile_commands.json`；`.clangd` 读取该目录的编译数据库。
- 原有 Make 构建命令是 `mingw32-make -j12`，输出在 `build/`。`.vscode/tasks.json` 的默认构建及 `.vscode/launch.json` 的 DAPLink、ST-Link、J-Link 调试配置使用这一套构建产物。
- 若只改文档，无须编译固件；若改 C 源码、编译宏、头文件或构建配置，至少运行受影响构建。两套构建都受影响时，分别验证。提交前运行 `git diff --check` 并查看改动范围。
- 当前仓库没有可依赖的自动 CI 或完整硬件仿真测试。编译成功只能说明构建通过；电机方向、IMU 标定、CAN 双板、USB 视觉和发射动作仍需按改动范围做台架或实机验证，并在交付时标明未验证项。

## 本工程常见误区

- 把 Yaw 的度与 Pitch 的弧度混用，或把电机单圈位置、IMU 多圈角和视觉展开角视为同一种反馈。
- 改了云台板的 CAN 结构体或字段语义，却没有核对实际底盘固件；只改应用层，遗漏 CAN 分包、超时或电机底层。
- 把生成目录当源码，或只更新 CMake/Makefile 其中一套源码清单。
- 根据旧文档、注释或未启用的板型分支推断当前行为，而未沿当前编译宏和任务调用链核对。
- 把“固件能编译”写成“实机已验证”，尤其是电机、急停、发射、IMU 和通信改动。

## 交付说明

完成修改时说明改了哪些控制路径或接口、为何改动、运行了哪些构建或检查，以及尚未完成的硬件验证。除非任务明确要求，不替用户擅自切换板型、重写分支历史或推送远端。
