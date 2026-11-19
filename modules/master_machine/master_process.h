#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include <stdint.h>

#include "bsp_usart.h"

#define VISION_RECV_SIZE 42u
#define VISION_SEND_SIZE 54u

typedef enum
{
    CENTER_UNOCCUPIED = 0,      // 未被占领 (Protobuf 要求必须有一个 0 值)
    CENTER_OCCUPIED_ALLY = 1,   // 被己方占领
    CENTER_OCCUPIED_ENEMY = 2,  // 被对方占领
    CENTER_OCCUPIED_BOTH = 3,   // 被双方占领
}Center_Ctrl_e;
typedef enum
{
    NO_FIRE = 0,
    AUTO_FIRE = 1,
    AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
    NO_TARGET = 0,
    TARGET_CONVERGING = 1,
    READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
    NO_TARGET_NUM = 0,
    HERO1 = 1,
    ENGINEER2 = 2,
    INFANTRY3 = 3,
    INFANTRY4 = 4,
    INFANTRY5 = 5,
    OUTPOST = 6,
    SENTRY = 7,
    BASE = 8
} Target_Type_e;

typedef enum
{
    ATTACK = 0,
    DEFEND = 1,
    MOVE = 2,
} Gesture_Type_e;

typedef enum
{
    NONE = 0,
    BLUE = 1,
    RED = 2,
} Enemy_Color_e;

typedef enum
{
    COLOR_NONE = 0,
    COLOR_BLUE = 2,
    COLOR_RED = 1,
} Color_e;

typedef enum
{
    VISION_MODE_AIM = 0,
    VISION_MODE_SMALL_BUFF = 1,
    VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

typedef enum
{
    SMALL_AMU_30 = 30,
} Bullet_Speed_e;

typedef struct
{
    Target_State_e target_state; // 瞄准的目标状态 READY_TO_FIRE才允许发射
    float pitch; // yaw 电机 ref 值
    float yaw; // yaw 电机 ref 值
    float delta_pitch; // 没用
    float delta_yaw; // 没用
    float target_distance; // 没用
    float vel_x; // 导航发送的 vx 转发给底盘
    float vel_y; // 同上
    float angle_yaw; // 没用（给后续大小yaw使用的）
} Vision_Recv_s;

typedef struct
{
    Color_e self_color; // 自己的颜色，裁判系统获得（目前视觉没用）
    Work_Mode_e work_mode; // 选择工作模式，用于判断正常自瞄还是打符（目前视觉没用）
    Bullet_Speed_e bullet_speed; // 弹速，裁判系统获得。注意必须是uint8_t类型，这里的枚举只是随便给了个值（目前视觉没用）

	// 下列角度单位均为度
    float yaw; // yaw 角度环使用的反馈（imu or 电机反馈，看你自己用的什么反馈）
    float pitch; // pitch 角度环使用的反馈
    float roll; // （目前视觉没用）
    float yaw_vel; // （目前视觉没用）
    float pitch_vel; // （目前视觉没用）
    float game_progress; // 当前比赛时间 裁判系统获得（目前视觉没用）
    float current_hp; // 当前血量（目前视觉没用）
    float current_base_hp; // 基地血量（目前视觉没用）
    float current_enemy_base_hp; // 地方基地血量（目前视觉没用）
    float current_outpost_hp; //前哨战血量
    float current_enemy_outpost_hp; //敌方前哨战血量
    float remain_time; //游戏剩余时间？
    float allow_fire_amount; //允许发弹量
    Center_Ctrl_e center_ctrl;
} Vision_Send_s;

/**
 * @brief 初始化视觉通信模块。
 * @param handle UART 模式使用的 UART 句柄；USB VCP 模式传入 NULL。
 */
Vision_Recv_s *VisionInit(void);

/**
 * @brief 将视觉发送数据封装成完整数据帧，并提交给底层发送接口。
 */
void VisionSend(const Vision_Send_s *data);

/**
 * @brief 判断视觉是否离线
 * @return 1 表示视觉在线；0 表示尚未收到有效帧或已经离线。
 */
uint8_t VisionIsOline(void);

#endif
