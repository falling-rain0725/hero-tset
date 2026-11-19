#include "ch_imu.h"
#include "string.h"
CH_IMU_Data_s  chimu_data = {0}; // CH-IMU数据
static USARTInstance *chimu_usart_instance; // CH-IMU串口通信实例


static void DecodeCHIMU(uint16_t recv_size)
{
    uint8_t *rxbuff;
    rxbuff = chimu_usart_instance->recv_buff;
    if (rxbuff[0] != CH_IMU_FRAME_HEAD_1 || rxbuff[1] != CH_IMU_FRAME_HEAD_2) // 帧头判断
    {
        return; // 帧头不匹配，直接返回
    }
    if (U2(rxbuff + 2) != CH_IMU_DATASIZE) // 数据长度判断
    {
        return; // 数据长度不匹配，直接返回
    }
    int offset = 6;

    chimu_data.last_yaw = chimu_data.eul[2];

    chimu_data.tag = U1(rxbuff + offset + 0); // 
    chimu_data.pressure = R4(rxbuff + offset + 4); // 
    chimu_data.timestamp = U4(rxbuff + offset + 8); // 时间戳
    chimu_data.acc[0] = R4(rxbuff + offset + 12); // 加速度X
    chimu_data.acc[1] = R4(rxbuff + offset + 16); // 加速度Y
    chimu_data.acc[2] = R4(rxbuff + offset + 20); // 加速度Z
    chimu_data.gyr[0] = R4(rxbuff + offset + 24); // 角速度X
    chimu_data.gyr[1] = R4(rxbuff + offset + 28); // 角速度Y
    chimu_data.gyr[2] = R4(rxbuff + offset + 32); // 角速度Z
    chimu_data.mag[0] = R4(rxbuff + offset + 36); // 磁力计X
    chimu_data.mag[1] = R4(rxbuff + offset + 40); // 磁力计Y
    chimu_data.mag[2] = R4(rxbuff + offset + 44); // 磁力计Z
    chimu_data.eul[0] = R4(rxbuff + offset + 48); // 绕Y轴角度
    chimu_data.eul[1] = R4(rxbuff + offset + 52); // 绕X轴角度
    chimu_data.eul[2] = R4(rxbuff + offset + 56); // 绕Z轴角度
    chimu_data.quat[0] = R4(rxbuff + offset + 60); // 四元数W
    chimu_data.quat[1] = R4(rxbuff + offset + 64); // 四元数X
    chimu_data.quat[2] = R4(rxbuff + offset + 68); // 四元数Y
    chimu_data.quat[3] = R4(rxbuff + offset + 72); // 四元数Z
    deg_change_rad(&chimu_data);
    CH_YawTotalAngle_Calculation();
}

CH_IMU_Data_s *CH_IMU_Init(UART_HandleTypeDef *chimu_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeCHIMU; // 这里可以设置接收回调函数
    conf.usart_handle = chimu_usart_handle;
    conf.recv_buff_size = CH_IMU_BUFFERSIZE;
   chimu_usart_instance = USARTRegister(&conf);

    return &chimu_data;
}

void CH_YawTotalAngle_Calculation()
{
    if(chimu_data.eul[2] - chimu_data.last_yaw > 180.0f)
    {
        chimu_data.YawRoundCount--;
    }else if(chimu_data.eul[2] - chimu_data.last_yaw < -180.0f)
    {
        chimu_data.YawRoundCount++;
    }
    chimu_data.YawTotalAngle = 360.0f * chimu_data.YawRoundCount + chimu_data.eul[2];
}

void deg_change_rad(CH_IMU_Data_s *imu_data)
{
    imu_data->pitch = imu_data->eul[0]-5.2f; // pitch偏差校准
    imu_data->eul[0] = imu_data->eul[0] * 3.14159265358979f / 180.0f;
}
