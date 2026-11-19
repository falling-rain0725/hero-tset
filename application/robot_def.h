/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"
#include "ch_imu.h"

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
// #define ONE_BOARD // 单板控制整车
// #define CHASSIS_BOARD //底盘板
#define GIMBAL_BOARD  //云台板


#define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 2711  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096 0 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD 3412      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
// #define PITCH_MAX_ANGLE 0           // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
// #define PITCH_MIN_ANGLE 0           // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
// 下面是极端保守的限位数据 
// #define PITCH_MAX 0.33f // 云台限位 rad
// #define PITCH_MIN -0.4f
// #define PITCH_MAX 0.43f // 云台限位 rad
// #define PITCH_MIN -0.34f
#define PITCH_MAX 0.35f // 云台限位 rad
#define PITCH_MIN -0.3f
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 80.0f    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出 测量值为84.17 理论值为80
#define REDUCTION_RATIO_LOADER 19.0f // 2006拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE 10            // 拨盘一圈的装载量
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 297              // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 297             // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL 76             // 轮子半径
#define REDUCTION_RATIO_WHEEL 19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

#define GYRO2GIMBAL_DIR_YAW 1   // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1  // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

// 添加在文件顶部其他#include之后
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
    CHASSIS_RE_ROTATE, //小陀螺反转，过滑环检测
} chassis_mode_e;

// 云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_FREE_MODE,      // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
    GIMBAL_AIM_MODE,       // 扫描状态
} gimbal_mode_e;

// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_3_BULLET,  // 三发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

// 功率限制,从裁判系统获取,是否有必要保留?
typedef struct
{ // 功率控制
    float chassis_power_mx;
} Chassis_Power_Data_s;

typedef enum
{
    UI_KEEP,
    UI_REFRESH,
} ui_mode_e;

typedef enum
{
    UNLOCK = 0,     // 未开启自瞄或视觉未锁定
    CONVERGE = 1,       // 视觉锁定过程中，正在校准
    LOCK = 2,           // 视觉锁定, 可以开火
}vision_mode_e;

typedef enum
{
    TUNNEL_ON=1,
    TUNNEL_OFF=0,
} tunnel_mode_e;

typedef enum
{
    TIM_ACTIVE_5=2,
    TIM_ACTIVE_15=1,
    TIM_INACTIVE=0,
} time_mode_e;

typedef enum
{
    NORMAL = 0,     // 正常模式
    VIOLENT,        // 狂暴模式, 提高单发射频
}attack_mode_e;

typedef enum
{
    HOLD = 0,
    FIGHT = 1,
}Heat_mode_e;

//ui data
typedef struct
{
    // uint32_t UI_ReInit;
    gimbal_mode_e gimbal_mode;				 // 云台模式
    chassis_mode_e chassis_mode;             // 底盘模式
	shoot_mode_e shoot_mode;				 // 发射模式设置
	friction_mode_e friction_mode;			 // 摩擦轮关闭
	Heat_mode_e heat_control;					 // 弹舱盖打开
	Chassis_Power_Data_s Chassis_Power_Data; // 功率控制
    loader_mode_e loader_mode;               // 拨盘电机信息
    ui_mode_e ui_mode;                       // ui
    int rest_bullet;
    float yaw;
    float pitch;


}UI_Cmd_Data_s;

typedef enum
{
    AIM_ON = 1,
    AIM_OFF = 0,
    AIM_FAST = 2,
} aim_mode_e;


/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角
    uint8_t shoot_rate;
    float init_totalangle;
    float totalangle;
    chassis_mode_e chassis_mode;             // 底盘模式
    float pitch_angle;
    float yaw_angle;
    int8_t rotate_control;//控制小陀螺方向
    
} Chassis_Ctrl_Cmd_s;



// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 云台角度控制
    float yaw;
    float pitch;
    float last_yaw;
    float feedforword;
    float yaw_omega_feedforword;
    float yaw_current_feedforword;
    float pitch_omega_feedforword;
    float pitch_current_feedforword;
    gimbal_mode_e gimbal_mode;
    aim_mode_e aim_mode;         // 是否开启自瞄
    // bool motor_online;           // 电机是否在线
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    float bullet_speed; // 弹速枚举
    uint8_t rest_heat;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
    // attitude_t chassis_imu_data;
#endif
    // 后续增加底盘的真实速度
    // float real_vx;
    // float real_vy;
    // float real_wz;

    uint8_t rest_heat;           // 剩余枪口热量
    Bullet_Speed_e bullet_speed; // 弹速限制
    Enemy_Color_e enemy_color;   // 0 for blue, 1 for red

} Chassis_Upload_Data_s;
// 控制相关的实时信息（需要高频率传输）
typedef struct
{

    uint8_t over_heat_flag;      // 枪管超热量标志
    float yaw_angle_increment;   // 云台应该旋转的角度增量
    uint8_t new_angle_increment_flag;
    uint8_t isyawonline;         // 云台是否在线
    float pitch_angle;           // 底盘的pitch角度
} Chassis_Control_Data_s;

// 裁判系统数据（10Hz频率读取，20Hz发送）
typedef struct
{
    uint16_t rest_heat_l;        // 剩余枪口1热量
    // uint16_t rest_heat_r;        // 剩余枪口2热量
    float bullet_speed;           
    Color_e self_color;          // 己方颜色
    uint16_t current_HP;            // 当前己方哨兵血量
    uint8_t game_progress;         // 游戏进度
    uint16_t current_base_hp;    // 己方基地血量
    uint16_t current_enemy_base_hp;     // 敌方基地血量
    uint16_t current_enemy_sentry_hp;   // 敌方哨兵血量
    uint16_t allow_fire_amount;         // 允许发射的子弹数量
    uint16_t current_outpost_hp;        // 当前前哨站血量
    uint16_t current_enemy_outpost_hp;  // 当前敌方前哨站血量
    float remain_time;                // 英雄X坐标
    float center_control;                // 英雄Y坐标
    float cmd_x;                 // 云台手控制X坐标
    float cmd_y;                 // 云台手控制Y坐标
} Referee_System_Data_s;


typedef struct
{
    CH_IMU_Data_s gimbal_imu_data_ch;
    //uint16_t yaw_motor_single_round_angle;
    float yaw_angle;
    float pitch_angle;
} Gimbal_Upload_Data_s;

typedef struct
{
    float init_totalangle;
    float totalangle;
} Shoot_Upload_Data_s;


//速度设置结构体
typedef struct
{
    float vt_set;
    float vt_out;
    float t;
    uint32_t dwt_cnt;
    float del_t;
    float t_set;
} Vt_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H
