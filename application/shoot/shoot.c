#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "referee_task.h"

#define LOADER_STALL_CURRENT_THRESHOLD 3300 // 堵转电流阈值(已测试)
#define LOADER_STALL_DETECT_TIME_MS    500  // 电流过大持续500ms则堵转
#define LOADER_MAX_STALL_TIMES 10
typedef enum 
{
    LOADER_NORMAL = 0, // 未堵转
    LOADER_OVER_CURRENT_THRESHOLD = 1, // 电流超过阈值
    LOADER_STALL = 2, // 堵转
    LOADER_DEAD = 3, // 机械性堵转无法修复，停止拨盘
} LOADER_STALL_State_e;
static LOADER_STALL_State_e loader_stall_state;
static float stall_start_time;
static float reverse_start_time;
static float reverse_stall_start_time;
static uint8_t loader_stall_cnt; // 堵转，堵的次数超过10也判断拨盘死掉
static referee_info_t *referee_data;       // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r, *loader,*friction_m; // 拨盘电机

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息
static uint8_t First_flag;
static float bullet_speed = 0; //没测，应该是读裁判系统
static float last_bullet_speed = 0;
// static int fric_rate = 38000;
volatile int fric_rate = 42000;
// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

#define DEBUGSHOOT

void ShootInit()
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config_l = {
        .can_init_config = {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 32, // 20//2.4：单发18
                .Ki = 0, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 12000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508
    };
    // 左摩擦轮
    Motor_Init_Config_s friction_config_r = {
        .can_init_config = {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 32, // 20//2.4：单发18
                .Ki = 0, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 12000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508
    };
    Motor_Init_Config_s friction_config_m = {
        .can_init_config = {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 32, // 20//2.4：单发18
                .Ki = 0, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 12000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508
    };
    friction_config_l.can_init_config.tx_id = 4,
    friction_l = DJIMotorInit(&friction_config_l);
    friction_config_r.can_init_config.tx_id = 5; // 右摩擦轮,改txid和方向就行
    friction_r = DJIMotorInit(&friction_config_r);
    friction_config_m.can_init_config.tx_id = 6; // 右摩擦轮,改txid和方向就行+
    friction_m = DJIMotorInit(&friction_config_m);
    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan2,//1--->2 O.o
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 100, // 10
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 10000,
                .DeadBand = 10, // 不给死区会震荡
            },
            .speed_PID = {
                .Kp = 10, // 10
                .Ki = 11, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 250,
                .MaxOut = 10000,
            },
            .current_PID = {
                .Kp = 1, // 0.7
                .Ki = 0.3, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 6000,
                .MaxOut = 10000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP | CURRENT_LOOP,
            // .motor_reverse_flag = MOTOR_DIRECTION_REVERSE, // 注意方向设置为拨盘的拨出的击发方向
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = M3508 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);
    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI
}

/* 机器人发射机构控制核心任务 */

void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(friction_m);
        DJIMotorStop(loader);
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(friction_m);
        DJIMotorEnable(loader);
        // DJIMotorStop(loader); // 拨盘爆了，打印件都爆出来了!!!!! 增加堵转检测!!!!!!
    }

    // // 拨盘堵转检测 电流超过阈值大于500ms则判断为堵转
    // if (loader_stall_cnt > LOADER_MAX_STALL_TIMES) 
    //     loader_stall_state = LOADER_DEAD;
    // switch(loader_stall_state)
    // {
    //     case LOADER_NORMAL:
    //         if (abs(loader->measure.real_current) > LOADER_STALL_CURRENT_THRESHOLD)
    //         {
    //             loader_stall_state = LOADER_OVER_CURRENT_THRESHOLD;
    //             stall_start_time = DWT_GetTimeline_ms();
    //         }
    //         else loader_stall_state = LOADER_NORMAL;
    //         break;
    //     case LOADER_OVER_CURRENT_THRESHOLD:
    //         if (abs(loader->measure.real_current) > LOADER_STALL_CURRENT_THRESHOLD)
    //         {
    //             if (DWT_GetTimeline_ms() - stall_start_time > LOADER_STALL_DETECT_TIME_MS)
    //             {
    //                 loader_stall_state = LOADER_STALL;
    //                 reverse_start_time = DWT_GetTimeline_ms();
    //                 reverse_stall_start_time = reverse_start_time;
    //                 loader_stall_cnt++;
    //             }
    //         }
    //         else loader_stall_state = LOADER_NORMAL;
    //         break;
    //     case LOADER_STALL: // 反转1s后再正转1s判断是否解除堵转
    //         shoot_cmd_recv.load_mode = 5; // 接管loader的控制权，防止被后面的switchload_mode修改，后续梳理一下整个逻辑，写的更顺一点
    //         if (DWT_GetTimeline_ms() - reverse_start_time <= 1500) // 1.5s内反转并检查反转是否也堵转
    //         {
    //             DJIMotorOuterLoop(loader, SPEED_LOOP);
    //             DJIMotorSetRef(loader, -10000); // 反转10000rad/s，后续要改为宏定义测试速度大小
    //             if (abs(loader->measure.real_current) > LOADER_STALL_CURRENT_THRESHOLD) // 电流可能为负值吗？
    //             {
    //                 if (DWT_GetTimeline_ms() - reverse_stall_start_time > 1000) // 超1s判断反转也无法恢复正常
    //                 {
    //                     loader_stall_state = LOADER_DEAD;
    //                 }
    //                 else 
    //                 {
    //                     reverse_stall_start_time = DWT_GetTimeline_ms();
    //                 }
    //             }
    //         }
    //         else loader_stall_state = LOADER_NORMAL; // 直接返回吧
    //         break;
    //     case LOADER_DEAD: // 私了
    //                 DJIMotorStop(loader);
    //         break;
    //     default:
    //         break;
    // }

    // 设置拨盘模式
    switch (shoot_cmd_recv.load_mode)
    {
        case LOAD_STOP:
            DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
            DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
            PIDClear(&loader->motor_controller.angle_PID);
            PIDClear(&loader->motor_controller.speed_PID);
            break;
        case LOAD_1_BULLET:                                                                     // 激活能量机关/干扰对方用,英雄用.
            if (DWT_GetTimeline_ms() - hibernate_time < dead_time)
            {
                break;
            }
            else
            {
                DJIMotorOuterLoop(loader, ANGLE_LOOP);// 切换到角度环                                               
                DJIMotorSetRef(loader, loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER); // 控制量增加一发弹丸的角度，其中90为36减速比*1.5
                hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
                dead_time = 1000;                                                                   // 完成1发弹丸发射的时间
            }
        break;
        case LOAD_BURSTFIRE:
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            //12拨盘容弹量，2.5是2006与拨盘的减速比
            DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 2.5 * 360 * REDUCTION_RATIO_LOADER / 12);
            // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
            break;
        case 5: // 跳过，不执行任何操作
            break;
        default: 
            DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
            DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
            break;
    }

    // 设置摩擦轮模式
    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
	    last_bullet_speed = bullet_speed;
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(friction_m);
        DJIMotorSetRef(friction_l, fric_rate);
        DJIMotorSetRef(friction_r, fric_rate);
        DJIMotorSetRef(friction_m, fric_rate);
    }
    // 弹速反馈控制，后续可以再调整射速过慢？还是不需要？
    else // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
        DJIMotorSetRef(friction_m, 0);
        
    }

    if(First_flag==100){
        //初始化角度
        // shoot_feedback_data.init_totalangle=loader->measure.init_totalangle;
        First_flag=0;
    }
    if(First_flag<100)
        First_flag++;
    // 用于初始化拨盘角度，用于底盘板计算弹量与热量，其中init_totalangle来自于dji_motor.c的decode函数
    //反馈数据更新
    shoot_feedback_data.totalangle=loader->measure.total_angle;
    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}