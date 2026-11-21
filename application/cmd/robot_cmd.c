// 应用层
#include "robot_def.h"
#include "robot_cmd.h"
// 模块层
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
#include "user_lib.h"
#include "buzzer.h"
// 板级支持层
#include "bsp_dwt.h"
#include "bsp_log.h"

#include "stdbool.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE  -1.141 // 这个为弧度制// 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360


// 遥控器通道映射宏
#define RC_CHANNEL_MAX      784.0f
#define CHASSIS_V_MAX       2.0f  // m/s
#define CHASSIS_WZ_MAX      3.0f  // rad/s
#define RC_TO_V_GAIN  (CHASSIS_V_MAX  / RC_CHANNEL_MAX)
#define RC_TO_WZ_GAIN (CHASSIS_WZ_MAX / RC_CHANNEL_MAX)
#define YAW_REMOTE_MAX_LEAD_DEG 90.0f // 遥控目标最多领先 IMU 半圈以内，避免就近转位反向
// 遥控器死区设置
#define RC_DEADZONE 15
#define RC_APPLY_DEADZONE(value) \
    ((abs(value) <= RC_DEADZONE) ? 0.0f : (float)(value))

typedef enum
{
    CONTROL_SOURCE_NONE = 0,
    CONTROL_SOURCE_REMOTE,
    CONTROL_SOURCE_VISION,
} Control_Source_e;



static Control_Source_e last_control_source = CONTROL_SOURCE_NONE;

/* cmd应用包含的模块实例指针和交互信息存储*/
#include "can_comm.h"
// static CANCommInstance *cmd_can_comm; // 双板通信
static CANCommInstance *cmd_control_can_comm; // 高频控制数据通信
static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Control_Data_s chassis_fetch_control_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等
static Referee_System_Data_s referee_system_data; // 实际底盘固件目前不发送裁判系统数据。
static FS_RC_ctrl_t *rc_data;                  // 遥控器数据,初始化时返回
static Vision_Send_s vision_send_data;        // 视觉发送数据
static Vision_Recv_s *vision_recv_data;        // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态
// static float chassis_speed_buff;
static float Gimbal[2];
static float tor[2];
static float test_k;
static int one_shoot = 0;

// BMI088Instance *bmi088_test; // 云台IMU
// BMI088_Data_t bmi088_data;

//#define DM_POSITION_T 10000.0f / 360.0f

void RobotCMDInit()
{
    test_k = 0;
    Gimbal[0]=0;
    Gimbal[1]=0;
    tor[0] = 0;
    tor[1] = 0;//初始化gimbal调参数组，以便Ozone寻地址
    rc_data = FS_RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    vision_recv_data = VisionInit(); // 视觉通信初始化
    vision_send_data.bullet_speed = SMALL_AMU_30;
    vision_send_data.work_mode = VISION_MODE_AIM;
    vision_send_data.self_color = COLOR_BLUE;

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    // chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    // 控制数据通信配置 - 高频率
    CANComm_Init_Config_s control_comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Control_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .daemon_count = 10, // DaemonTask为100Hz，约100ms超时
    };
    cmd_control_can_comm = CANCommInit(&control_comm_conf);
#endif // GIMBAL_BOARD
    chassis_cmd_send.rotate_control = 1.0f; // 旋转控制值,默认1.0
    gimbal_cmd_send.pitch = 0;
    //yaw值似乎是角度
    gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data_ch.YawTotalAngle;
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    float offset_angle;
    offset_angle = (gimbal_fetch_data.yaw_angle - YAW_ALIGN_ANGLE) * 180.0 / PI ;
    chassis_cmd_send.offset_angle = offset_angle;
}

/**
 * @brief 在切换控制模式或解除急停时，把控制目标同步到机器人当前的实际位置，避免电机突然追赶旧目标。
 *
 * @return void
 */
static void SyncControlTargetToFeedback(void)
{
    /*
     * Yaw控制使用IMU多圈角，单位为度。
     * Pitch电机反馈pitch_angle单位为度，控制目标单位为弧度。
     */
    gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data_ch.YawTotalAngle;
    gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data_ch.pitch * PI / 180.0f;
    chassis_cmd_send.vx = 0.0f;
    chassis_cmd_send.vy = 0.0f;
    chassis_cmd_send.wz = 0.0f;
    chassis_cmd_send.rotate_control = 0;

    shoot_cmd_send.load_mode = LOAD_STOP;
}

static void StopEverything(void)
{
    robot_state = ROBOT_STOP;
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    // LOGERROR("[CMD] emergency stop!");
}

/**
 * @brief  急停，目前只有遥控器急停
 *
 * @todo   后续增加遥控器离线急停，电机离线急停，
 *         还需要增加发送DM电机失能命令(其实没必要，后面失能还要发送启动命令有点麻烦)
 *
 */
static uint8_t EmergencyHandler()
{
    if (!FS_RemoteControlIsOnline()) // 遥控器离线
    {
        StopEverything();
        BuzzerPlay(BUZZER_MUSIC_HARUHIKAGE, 1);
        return 1;
    }
    // if (!CANCommIsOnline(cmd_control_can_comm))
    // {
    //     // StopEverything();
    //     // BuzzerPlay(BUZZER_MUSIC_BAD_APPLE, 1);
    //     return 1;
    // }

    // 云台电机失能

    // 发射电机失能


    if (fs_switch_is_up (rc_data->rc.SWA)) // 遥控器初始急停
    {
        StopEverything();
        return 1;
    }

    return 0;
}

static void RemoteControlSet()
{
    robot_state = ROBOT_READY;

    /* 遥控器模式下启用云台闭环 */
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    gimbal_cmd_send.aim_mode = AIM_OFF;
    /* 允许发射机构正常响应SWB */
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    shoot_cmd_send.load_mode = LOAD_STOP;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    // if(fs_switch_is_up(rc_data->rc.SWD))
    // {
    //     if(!one_shoot){
    //         one_shoot =1;
    //     }
    // }
    // else if(fs_switch_is_down(rc_data->rc.SWD) && one_shoot)
    // {
    //     shoot_cmd_send.load_mode = LOAD_1_BULLET;
    //     one_shoot = 0;
    // }

    // SWB 拨至下位时连发，上位保持停止。
    if (fs_switch_is_down(rc_data->rc.SWB))
    {
        // shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        // shoot_cmd_send.shoot_rate = (float)abs(rc_data->rc.VRB - FS_VR_LOW + 10) / 150.0f;
        shoot_cmd_send.friction_mode = FRICTION_ON;
    }

    // 底盘模式选择
    if (fs_switch_is_up (rc_data->rc.SWC)) // 全向移动
    {
        // chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        // chassis_cmd_send.wz = 0;
    }
    else if (fs_switch_is_mid (rc_data->rc.SWC)) // 跟随模式
    {
        // chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        // chassis_cmd_send.wz = 0;
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        shoot_cmd_send.shoot_rate = (float)abs(rc_data->rc.VRB - FS_VR_LOW + 10) / 150.0f;
    }
    else if (fs_switch_is_down (rc_data->rc.SWC)) // 小陀螺
    {
        // chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        // chassis_cmd_send.wz = (float)rc_data->rc.VRA * RC_TO_WZ_GAIN; // 用旋钮调整旋转速度
        // shoot_cmd_send.load_mode = LOAD_1_BULLET;
    }

    chassis_cmd_send.vy = (float)RC_APPLY_DEADZONE(rc_data->rc.rocker_l_) * RC_TO_V_GAIN; // 水平方向
    chassis_cmd_send.vx = -(float)RC_APPLY_DEADZONE(rc_data->rc.rocker_l1) * RC_TO_V_GAIN; // 竖直方向
    float v_norm = sqrtf(chassis_cmd_send.vx * chassis_cmd_send.vx + chassis_cmd_send.vy * chassis_cmd_send.vy);
    if (v_norm > CHASSIS_V_MAX)
    {
        float scale = CHASSIS_V_MAX / v_norm;
        chassis_cmd_send.vx *= scale;
        chassis_cmd_send.vy *= scale;
    }
    // 虽然云台板有限位，但这里的限位也是必要的，否则发送数据的pitch会一直增大，想减少需要一段时间才行

    gimbal_cmd_send.pitch -= 0.00004f * (float)(abs(rc_data->rc.rocker_r1) > 20 ? rc_data->rc.rocker_r1 : 0); // 增加的幅度单位为 rad
    gimbal_cmd_send.pitch = LIMIT(gimbal_cmd_send.pitch, PITCH_MIN, PITCH_MAX);

    gimbal_cmd_send.yaw -= 0.002f * (float)(abs(rc_data->rc.rocker_r_) > 10 ? rc_data->rc.rocker_r_ : 0); // 增加的幅度单位为度
    float yaw_now = gimbal_fetch_data.gimbal_imu_data_ch.YawTotalAngle;
    gimbal_cmd_send.yaw = LIMIT(gimbal_cmd_send.yaw,
                                yaw_now - YAW_REMOTE_MAX_LEAD_DEG,
                                yaw_now + YAW_REMOTE_MAX_LEAD_DEG);

}

/**
 * 视觉自瞄协议依次传输 pitch、yaw、delta_pitch、delta_yaw、distance。
 * 调用本函数前已执行 EmergencyHandler()；协议不包含导航字段。
 */
static void VisionControlSet(void)
{
    if (!VisionIsOline()) // 进视觉控制才判断视觉是否离线 而不是放到EmergencyHandler无条件判断
    {
        StopEverything();
        BuzzerPlay(BUZZER_MUSIC_SEE_YOU_AGAIN_2_SPEED, 1);
        return;
    }
    robot_state = ROBOT_READY;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    gimbal_cmd_send.aim_mode = AIM_OFF;
    // 该协议不包含导航或自旋命令。
    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    chassis_cmd_send.vx = chassis_cmd_send.vy = chassis_cmd_send.wz = 0.0f;
    chassis_cmd_send.rotate_control = 0;
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    shoot_cmd_send.load_mode = LOAD_STOP; // 收到有效目标前保持拨弹停止。
    shoot_cmd_send.friction_mode = FRICTION_ON;

    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    chassis_cmd_send.vx = vision_recv_data->vel_x;
    chassis_cmd_send.vy = vision_recv_data->vel_y;
    chassis_cmd_send.wz = 0.0f;
    chassis_cmd_send.rotate_control = 0;

    float speed = sqrtf(chassis_cmd_send.vx * chassis_cmd_send.vx +
                        chassis_cmd_send.vy * chassis_cmd_send.vy);
    if (speed > CHASSIS_V_MAX)
    {
        float scale = CHASSIS_V_MAX / speed;
        chassis_cmd_send.vx *= scale;
        chassis_cmd_send.vy *= scale;
    }
    
    // 状态 0 的角度会被清零；保持上一目标，不向零位转动。
    if (vision_recv_data->target_state == NO_TARGET)
        return;
    gimbal_cmd_send.aim_mode = AIM_ON;
    gimbal_cmd_send.pitch = LIMIT(-vision_recv_data->pitch * PI / 180.0f, PITCH_MIN, PITCH_MAX);
    gimbal_cmd_send.yaw = vision_recv_data->yaw; // 电机角度环按 IMU 度制反馈就近转位。
    
    if (vision_recv_data->target_state == READY_TO_FIRE)
    {
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        shoot_cmd_send.shoot_rate = (float)abs(rc_data->rc.VRB - FS_VR_LOW + 10) / 150.0f;
    }
    if(vision_recv_data->target_state == TARGET_CONVERGING)
    {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
}

static void CmdVisionSend(void)
{
    // 单位为度：BMI088 原始单圈 yaw、取负后的 pitch、原始 roll。
    vision_send_data.yaw = gimbal_fetch_data.gimbal_imu_data_ch.eul[2];
    vision_send_data.pitch = -gimbal_fetch_data.gimbal_imu_data_ch.pitch;
    VisionSend(&vision_send_data);
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // 从其他应用获取回传数据
    // 接收高频控制数据-双板通信底盘消息
    chassis_fetch_control_data = *(Chassis_Control_Data_s *)CANCommGet(cmd_control_can_comm);

    // 实际底盘不发送裁判系统数据，视觉报文中的这些字段保持为零。
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();

    // 控制逻辑
    if (EmergencyHandler())
    {
        last_control_source = CONTROL_SOURCE_NONE;
    }
    else
    {
        Control_Source_e current_control_source;

        // if (fs_switch_is_up(rc_data->rc.SWD))
        // {
        //     current_control_source = CONTROL_SOURCE_REMOTE; // 遥控器控制
        // }
        // else if (fs_switch_is_down(rc_data->rc.SWD))
        // {
        //     current_control_source = CONTROL_SOURCE_VISION; // 视觉控制
        // }
        // else
        // {
        //     current_control_source = CONTROL_SOURCE_NONE;
        // }

        // if (current_control_source != last_control_source) // 处理模式切换 保护
        // {
        //     SyncControlTargetToFeedback(); // 把控制目标同步到机器人当前的实际位置
        //     last_control_source = current_control_source;
        // }
        current_control_source = CONTROL_SOURCE_REMOTE; // 遥控器控制

        switch (current_control_source)
        {
        case CONTROL_SOURCE_REMOTE:
            RemoteControlSet();
            break;

        case CONTROL_SOURCE_VISION:
            VisionControlSet();
            break;

        case CONTROL_SOURCE_NONE:
        default:
            StopEverything();
            break;
        }
    }

    // 向视觉发送数据
    CmdVisionSend();

    shoot_cmd_send.bullet_speed = referee_system_data.bullet_speed;
    // 实际底盘会用 yaw_angle 覆盖 offset_angle；保持合并前的电机原始角度映射。
    chassis_cmd_send.yaw_angle = gimbal_fetch_data.yaw_angle * 180.0f / PI;
    chassis_cmd_send.init_totalangle = shoot_fetch_data.init_totalangle;
    chassis_cmd_send.totalangle = shoot_fetch_data.totalangle;

    CANCommSend(cmd_control_can_comm, (void *)&chassis_cmd_send);
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
