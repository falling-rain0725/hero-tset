#ifndef CH_IMU_H
#define CH_IMU_H

#include "bsp_usart.h"
#include "stdint.h"
#include "string.h"
#define CH_IMU_BUFFERSIZE  82 // CH-IMU接收数据buffer大小，不得大于256
#define CH_IMU_DATASIZE   76 // CH-IMU接收数据长度
#define CH_IMU_FRAME_HEAD_1 0x5A // 帧头1
#define CH_IMU_FRAME_HEAD_2 0xA5 // 帧头2
#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t  *)(p)))
#define I2(p) (*((int16_t  *)(p)))
static uint16_t U2(uint8_t *p) {uint16_t u; memcpy(&u,p,2); return u;}
static uint32_t U4(uint8_t *p) {uint32_t u; memcpy(&u,p,4); return u;}
static int32_t  I4(uint8_t *p) {int32_t  u; memcpy(&u,p,4); return u;}
static float    R4(uint8_t *p) {float    r; memcpy(&r,p,4); return r;}

typedef struct
{
    uint8_t     tag;              /* item tag: 0x91        */
    float       acc[3];           /* acceleration          */
    float       gyr[3];           /* angular velocity      */  
    float       mag[3];           /* magnetic field        */
    float       eul[3];           /* attitude: eular angle */
    float       quat[4];          /* attitude: quaternion  */
    float       pressure;         /* air pressure          */
    uint32_t    timestamp;      /* timestamp             */
    float last_yaw;
    float YawTotalAngle;
    float YawRoundCount;
    float pitch;
} CH_IMU_Data_s;


CH_IMU_Data_s *CH_IMU_Init(UART_HandleTypeDef *chimu_usart_handle);
void CH_YawTotalAngle_Calculation();
void deg_change_rad(CH_IMU_Data_s *imu_data);
#endif // !CH_IMU_H
