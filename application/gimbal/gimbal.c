#include "gimbal.h"
#include "robot_def.h"
#include "dmmotor.h"
#include "ins_task.h"
#include "message_center.h"
#include "buzzer.h"
#include "motor_def.h"
#include "bsp_dwt.h"

#define GIMBAL_PITCH_REF_MAX_SPEED 2.0f // 目标最大变化速率，单位 rad/s

static attitude_t *gimbal_IMU_data; // 云台IMU数据
static DMMotorInstance *pitch_motor;
static DMMotorInstance *yaw_motor;
static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static float pitch_feedback_rad;
static float pitch_ref_smooth;
static uint32_t gimbal_ref_dwt_cnt;
static uint8_t gimbal_ref_inited = 0;
volatile float test_gimbal_cmd = 0.0f; // Ozone 调试 Pitch 目标，单位 rad

// 按最大变化速率逐步逼近目标，避免角度参考值阶跃。
static void Ramp(float *current, float target, float max_rate, float dt)
{
    float err = target - *current;
    float max_step = max_rate * dt;

    if (err > max_step)
        err = max_step;
    else if (err < -max_step)
        err = -max_step;

    *current += err;
}

void GimbalInit()
{
    // gimbal_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源

    // BuzzerInit();

    // YAW,达妙电机
    DMMotor_Init_Config_s yaw_config = {
        // .can_init_config = {
        //     .can_handle = &hcan2, // 1--->2 O.o
        //     .tx_id = 0x08,//can_id
        //     .rx_id = 0x32,//master_id
        // },
        // .controller_param_init_config = {
        //     .angle_PID = {
        //         .Kp = 44,
        //         .Ki = 29,
        //         .Kd = 1.6,
        //         .MaxOut = 8000,
        //         .IntegralLimit = 1000,
        //         .Improve = PID_Integral_Limit | PID_DerivativeFilter | PID_Derivative_On_Measurement,
        //         .Derivative_LPF_RC = 0.001,
        //     },
        //     .speed_PID = {
        //         .Kp = 0.1,
        //         .Ki = 0,
        //         .MaxOut = 53,
        //     },
        //     .other_angle_feedback_ptr = &gimbal_IMU_data->YawTotalAngle,
        //     .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[2],
        // },
        // .controller_setting_init_config = {
        //     .angle_feedback_source = OTHER_FEED,
        //     .speed_feedback_source = OTHER_FEED,
        //     .outer_loop_type = ANGLE_LOOP,
        //     .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        //     .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        //     .angle_nearest_mode = DM_ANGLE_NEAREST_DEG, // IMU YawTotalAngle 为度
        // },
    };
    // PITCH
    // DMMotor_Init_Config_s pitch_config = {
    //     .can_init_config = {
    //         .can_handle = &hcan2,
    //         .tx_id = 0x09,
    //         .rx_id = 0x19,
    //     },
    //     .controller_param_init_config = {
    //         .angle_PID = {
    //             .Kp = 15,
    //             .Ki = 0,
    //             .Kd = 0.001,
    //             .CoefA = 0.2,
    //             .CoefB = 0.4,
    //             .Output_LPF_RC = 0.001,
    //             Derivative_LPF_RC = 0.001,
    //             .MaxOut = 1000,
    //             .Improve = PID_DerivativeFilter,
    //         },
    //         .speed_PID = {
    //             .Kp = 2.8,
    //             .Ki = 0,
    //             .Kd = 0,
    //             .MaxOut = 300,
    //         },

    //         .other_angle_feedback_ptr = &pitch_feedback_rad,
    //         .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[0],
    //     },
    //     .controller_setting_init_config = {
    //         .angle_feedback_source = OTHER_FEED,
    //         .speed_feedback_source = OTHER_FEED,
    //         .outer_loop_type = ANGLE_LOOP,
    //         .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
    //         .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    //     },
    // };
        // PITCH
    DMMotor_Init_Config_s pitch_config = {
        // .can_init_config = {
        //     .can_handle = &hcan2,
        //     .tx_id = 0x01,
        //     .rx_id = 0x10,
        // },
        // .controller_param_init_config = {
        //     .angle_PID = {
        //         .Kp = 35,
        //         .Ki = 10,
        //         .Kd = 0.1,
        //         .CoefA = 0.2,
        //         .CoefB = 0.4,
        //         .Output_LPF_RC = 0,
        //         .MaxOut = 1000,
        //         .Derivative_LPF_RC = 0.01,
        //         .Improve = PID_DerivativeFilter,
        //     },
        //     .speed_PID = {
        //         .Kp = 2.7,
        //         .Ki = 0,
        //         .Kd = 0,
        //         .MaxOut = 300,
        //     },

        //     .other_angle_feedback_ptr = &pitch_feedback_rad,
        //     .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[0],
        // },
        // .controller_setting_init_config = {
        //     .angle_feedback_source = OTHER_FEED,
        //     .speed_feedback_source = OTHER_FEED,
        //     .outer_loop_type = ANGLE_LOOP,
        //     .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        //     .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        // },
    };

    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    // pitch_motor = DMMotorInit(&pitch_config);
    // yaw_motor = DMMotorInit(&yaw_config);


    DWT_Delay(0.1);
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    // pitch_feedback_rad = gimbal_IMU_data->Pitch * PI / 180.0f;
    // float gimbal_ref_dt = DWT_GetDeltaT(&gimbal_ref_dwt_cnt);
    // if (gimbal_ref_dt <= 0.0f || gimbal_ref_dt > 0.05f)
    //     gimbal_ref_dt = 0.005f; // 云台任务正常周期约为 5 ms

    // switch (gimbal_cmd_recv.gimbal_mode)
    // {
    // // 停止或未实现的模式：清除斜坡状态，重新使能时从当前反馈起步。
    // case GIMBAL_ZERO_FORCE:
    // default:
    //     DMMotorStop(yaw_motor);
    //     DMMotorStop(pitch_motor);
    //     gimbal_ref_inited = 0;
    //     break;
    // // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    // case GIMBAL_GYRO_MODE: // 后续只保留此模式
    // {
    //     if (!gimbal_ref_inited)
    //     {
    //         pitch_ref_smooth = pitch_feedback_rad;
    //         gimbal_ref_inited = 1;
    //     }

    //     float pitch_target = gimbal_cmd_recv.pitch;
    //     // pitch_target = test_gimbal_cmd; // Ozone 调试时取消本行注释，目标仍经过限位和斜坡
    //     if (pitch_target > PITCH_MAX)
    //         pitch_target = PITCH_MAX;
    //     else if (pitch_target < PITCH_MIN)
    //         pitch_target = PITCH_MIN;
    //     Ramp(&pitch_ref_smooth, pitch_target, GIMBAL_PITCH_REF_MAX_SPEED, gimbal_ref_dt);

    //     // 斜坡输出仍受 Pitch 弧度限位约束。
    //     if (pitch_ref_smooth > PITCH_MAX)
    //         pitch_ref_smooth = PITCH_MAX;
    //     else if (pitch_ref_smooth < PITCH_MIN)
    //         pitch_ref_smooth = PITCH_MIN;

    //     DMMotorSetRef(pitch_motor, pitch_ref_smooth);
    //     DMMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
    //     DMMotorEnable(yaw_motor);
    //     DMMotorEnable(pitch_motor);
    //     break;
    // }
    // }

    // gimbal_feedback_data.yaw_angle = yaw_motor->measure.position; //yaw_motor->measure.position单位为弧度
    // gimbal_feedback_data.pitch_angle = pitch_motor->measure.position*180.0f/PI;
    // gimbal_feedback_data.gimbal_imu_data_ch.YawTotalAngle = gimbal_IMU_data->YawTotalAngle;
    // gimbal_feedback_data.gimbal_imu_data_ch.eul[2] = gimbal_IMU_data->Yaw;
    // gimbal_feedback_data.gimbal_imu_data_ch.pitch = gimbal_IMU_data->Pitch;
    // gimbal_feedback_data.gimbal_imu_data_ch.eul[0] = gimbal_IMU_data->Pitch;
    // gimbal_feedback_data.gimbal_imu_data_ch.eul[1] = gimbal_IMU_data->Roll;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
