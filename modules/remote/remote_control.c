#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"

#define REMOTE_CONTROL_FRAME_SIZE 18u // 遥控器接收的buffer大小
#define SBUS_FRAME_SIZE 25u//fs遥控器的buffer
#define FS_RC_STAGE LAST
static uint8_t rc_lost_handled = 0; //离线回调标志位



// 遥控器数据
static RC_ctrl_t rc_ctrl[2];     //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static FS_RC_ctrl_t fs_rc_ctrl[2]; //stage做临时输入验证帧是否正确，正确后复制给temp
static uint8_t rc_init_flag = 0; // 遥控器初始化标志位
/* ---- 诊断计数:仅供Ozone观察,不参与控制逻辑 ----
 * used + 至少一处写入,保证不会被 -fdata-sections/--gc-sections 回收
 * [0]收帧 [1]提交成功 [2]长度错 [3]帧头错 [4]帧尾错
 * [5]拨杆异常 [6]failsafe [7]Daemon超时离线 [8]最近提交帧的failsafe位 */
__attribute__((used)) volatile uint32_t fs_rc_diag[9];
volatile uint16_t fs_rc_last_size;


// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *rc_usart_instance;
static DaemonInstance *rc_daemon_instance;

/**
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 5; ++i)
        if (abs(*(&rc_ctrl[TEMP].rc.rocker_l_ + i)) > 660)
            *(&rc_ctrl[TEMP].rc.rocker_l_ + i) = 0;
}

/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
static void sbus_to_rc(const uint8_t *sbus_buf)
{
    // 摇杆,直接解算时减去偏置
    rc_ctrl[TEMP].rc.rocker_r_ = ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;                              //!< Channel 0
    rc_ctrl[TEMP].rc.rocker_r1 = (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 1
    rc_ctrl[TEMP].rc.rocker_l_ = (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET; //!< Channel 2
    rc_ctrl[TEMP].rc.rocker_l1 = (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 3
    rc_ctrl[TEMP].rc.dial = ((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;                                 // 左侧拨轮
    RectifyRCjoystick();
    // 开关,0左1右
    rc_ctrl[TEMP].rc.switch_right = ((sbus_buf[5] >> 4) & 0x0003);     //!< Switch right
    rc_ctrl[TEMP].rc.switch_left = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //!< Switch left

    // 鼠标解析
    rc_ctrl[TEMP].mouse.x = (sbus_buf[6] | (sbus_buf[7] << 8)); //!< Mouse X axis
    rc_ctrl[TEMP].mouse.y = (sbus_buf[8] | (sbus_buf[9] << 8)); //!< Mouse Y axis
    rc_ctrl[TEMP].mouse.press_l = sbus_buf[12];                 //!< Mouse Left Is Press ?
    rc_ctrl[TEMP].mouse.press_r = sbus_buf[13];                 //!< Mouse Right Is Press ?

    //  位域的按键值解算,直接memcpy即可,注意小端低字节在前,即lsb在第一位,msb在最后
    rc_ctrl[TEMP].key[KEY_PRESS].keys = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8));
    if (rc_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (rc_ctrl[TEMP].key[KEY_PRESS].shift) // shift键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    uint16_t key_now = rc_ctrl[TEMP].key[KEY_PRESS].keys,                   // 当前按键是否按下
        key_last = rc_ctrl[LAST].key[KEY_PRESS].keys,                       // 上一次按键是否按下
        key_with_ctrl = rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
        key_with_shift = rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  当前shift组合键是否按下
        key_last_with_ctrl = rc_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
        key_last_with_shift = rc_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下

    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
            continue;
        // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS][i]++;
        // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }

    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断
}

/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
static uint8_t fs_to_rc(const uint8_t *sbus_buf, uint16_t recv_size)
{
    FS_RC_ctrl_t *stage = &fs_rc_ctrl[FS_RC_STAGE];   // 暂存区,先验帧再传值
    fs_rc_diag[0]++;                                 // ← 插入①:收到一帧
    fs_rc_last_size = recv_size;
    // ========== 验证1：长度检查 ==========
    if (recv_size != SBUS_FRAME_SIZE) 
    {
        fs_rc_diag[2]++;
        return 0;  // 长度错误
    }
    stage->rc.start_byte = sbus_buf[0];
    stage->rc.rocker_r_ = ((sbus_buf[1]     | sbus_buf[2] << 8) & 0x07FF)-RC_CH_VALUE_OFFSET;
    stage->rc.rocker_r1 = ((sbus_buf[2] >> 3 | sbus_buf[3] << 5) & 0x07FF) -RC_CH_VALUE_OFFSET;
    stage->rc.rocker_l1 = ((sbus_buf[3] >> 6 | sbus_buf[4] << 2 | sbus_buf[5] << 10) & 0x07FF) - RC_CH_VALUE_OFFSET;
    stage->rc.rocker_l_ = ((sbus_buf[5] >> 1 | sbus_buf[6] << 7) & 0x07FF) - RC_CH_VALUE_OFFSET;

    //旋钮
    stage->rc.VRA = ((sbus_buf[6] >> 4 | sbus_buf[7] << 4) & 0x07FF)-RC_CH_VALUE_OFFSET;
    stage->rc.VRB = ((sbus_buf[7] >> 7 | sbus_buf[8] << 1 | sbus_buf[9] << 9) & 0x07FF)-RC_CH_VALUE_OFFSET;
    //拨杆0上，1中，2下
    stage->channel[6] = ((sbus_buf[9] >> 2 | sbus_buf[10] << 6) & 0x07FF);
    stage->channel[7] = ((sbus_buf[10] >> 5 | sbus_buf[11] << 3) & 0x07FF);
    stage->channel[8] = ((sbus_buf[12] | sbus_buf[13] << 8 ) & 0x07FF);
    stage->channel[9] = ((sbus_buf[13] >> 3 | sbus_buf[14] << 5) & 0x07FF);
    stage->rc.err_protect = (sbus_buf[23] >> 3) & 0x01;
    stage->rc.stop_byte = sbus_buf[24];
    if (stage->rc.start_byte != 0x0F) 
    {
        fs_rc_diag[3]++;
        return 0;  // 帧头错误
    }
    if (stage->rc.stop_byte != 0x00) 
    {
        fs_rc_diag[4]++;
        return 0;  // 帧尾错误
    }
    for (int i = 6; i <= 9; i++) 
    {
        uint16_t val = stage->channel[i];
        if (val > FS_SWITCH_HIGH - SWITCH_TOLERANCE && val < FS_SWITCH_HIGH + SWITCH_TOLERANCE)
        {
            stage->channel[i] = 2; // 下挡
        }
        else if (val > FS_SWITCH_MID - SWITCH_TOLERANCE && val < FS_SWITCH_MID + SWITCH_TOLERANCE)
        {
            stage->channel[i] = 1; // 中挡
        }
        else if (val > FS_SWITCH_LOW - SWITCH_TOLERANCE && val < FS_SWITCH_LOW + SWITCH_TOLERANCE)
        {
            
            stage->channel[i] = 0; // 上挡
        }
        else
        {
            fs_rc_diag[5]++;
            return 0;
        }
    }

    stage->rc.SWA = stage->channel[6];
    stage->rc.SWB = stage->channel[7];
    stage->rc.SWC = stage->channel[8];
    stage->rc.SWD = stage->channel[9];
    if(stage->rc.err_protect){
        fs_rc_diag[6]++;
        // 返回0，让callback不喂狗，多次错误自动下电，不让偶发性的错误帧影响实际操控
        return 0;
    }
    memcpy(&fs_rc_ctrl[TEMP], stage, sizeof(FS_RC_ctrl_t)); 
    fs_rc_diag[1]++;                                 // ← 插入⑨:提交成功
    fs_rc_diag[8] = stage->rc.err_protect;           // ← 插入⑩:恒0,表示"最近提交帧的failsafe位"
    fs_rc_diag[6] = 0;
    return 1;
}

/**
 * @brief 对sbus_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void RemoteControlRxCallback(uint16_t recv_size)
{
    DaemonReload(rc_daemon_instance);         // 先喂狗
    sbus_to_rc(rc_usart_instance->recv_buff); // 进行协议解析
}

/**
 * @brief 对fs_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void FS_RemoteControlRxCallback(uint16_t recv_size)
{

    if(fs_to_rc(rc_usart_instance->recv_buff,recv_size))
    {
        rc_lost_handled = 0;
        DaemonReload(rc_daemon_instance);
    };
    // DaemonReload(rc_daemon_instance);         // 先喂狗
    // fs_to_rc(rc_usart_instance->recv_buff); // 进行协议解析
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void RCLostCallback(void *id)
{
    memset(rc_ctrl, 0, sizeof(rc_ctrl)); // 清空遥控器数据
    USARTServiceInit(rc_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
}

static void FS_RCLostCallback(void *id)
{
    if (rc_lost_handled)
        return;

    rc_lost_handled = 1;
    fs_rc_diag[7]++;
    memset(fs_rc_ctrl, 0, sizeof(fs_rc_ctrl)); // 清空遥控器数据
    USARTServiceInit(rc_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = REMOTE_CONTROL_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);

    rc_init_flag = 1;
    return rc_ctrl;
}

FS_RC_ctrl_t *FS_RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = FS_RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = SBUS_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&conf);
    

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = FS_RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);

    rc_init_flag = 1;
    return fs_rc_ctrl;
}

uint8_t RemoteControlIsOnline()
{
    if (rc_init_flag)
        return DaemonIsOnline(rc_daemon_instance);
    return 0;
}

uint8_t FS_RemoteControlIsOnline(){
    if (rc_init_flag)
        return DaemonIsOnline(rc_daemon_instance);
    return 0;
}
