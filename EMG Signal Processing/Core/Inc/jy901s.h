/*
 * jy901s.h
 *
 *  Created on: Dec 8, 2025
 *      Author: 23600
 */

#ifndef INC_JY901S_H_
#define INC_JY901S_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include"string.h"
#include"stdio.h"

/* 串口句柄声明*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/* 配置指令宏定义 */
#define SAVE        0x00
#define CALSW       0x01
#define RSW         0x02
#define RRATE       0x03
#define BAUD        0x04
#define AXOFFSET    0x05
#define AYOFFSET    0x06
#define AZOFFSET    0x07
#define GXOFFSET    0x08
#define GYOFFSET    0x09
#define GZOFFSET    0x0a
#define HXOFFSET    0x0b
#define HYOFFSET    0x0c
#define HZOFFSET    0x0d
#define D0MODE      0x0e
#define D1MODE      0x0f
#define D2MODE      0x10
#define D3MODE      0x11
#define D0PWMH      0x12
#define D1PWMH      0x13
#define D2PWMH      0x14
#define D3PWMH      0x15
#define D0PWMT      0x16
#define D1PWMT      0x17
#define D2PWMT      0x18
#define D3PWMT      0x19
#define IICADDR     0x1a
#define LEDOFF      0x1b
#define GPSBAUD     0x1c

/* 数据域宏定义 */
#define YYMM        0x30
#define DDHH        0x31
#define MMSS        0x32
#define MS          0x33
#define AX          0x34
#define AY          0x35
#define AZ          0x36
#define GX          0x37
#define GY          0x38
#define GZ          0x39
#define HX          0x3a
#define HY          0x3b
#define HZ          0x3c
#define Roll        0x3d
#define Pitch       0x3e
#define Yaw         0x3f
#define TEMP        0x40
#define D0Status    0x41
#define D1Status    0x42
#define D2Status    0x43
#define D3Status    0x44
#define PressureL   0x45
#define PressureH   0x46
#define HeightL     0x47
#define HeightH     0x48
#define LonL        0x49
#define LonH        0x4a
#define LatL        0x4b
#define LatH        0x4c
#define GPSHeight   0x4d
#define GPSYAW      0x4e
#define GPSVL       0x4f
#define GPSVH       0x50
#define q0          0x51
#define q1          0x52
#define q2          0x53
#define q3          0x54

/* DIO模式宏定义 */
#define DIO_MODE_AIN     0
#define DIO_MODE_DIN     1
#define DIO_MODE_DOH     2
#define DIO_MODE_DOL     3
#define DIO_MODE_DOPWM   4
#define DIO_MODE_GPS     5

/* 传感器数据结构体 */
typedef struct
{
    uint8_t  ucYear;
    uint8_t  ucMonth;
    uint8_t  ucDay;
    uint8_t  ucHour;
    uint8_t  ucMinute;
    uint8_t  ucSecond;
    uint16_t usMiliSecond;
} JY901S_TimeDef;

typedef struct
{
    int16_t a[3];  // X/Y/Z 轴加速度
    int16_t T;     // 温度
} JY901S_AccDef;

typedef struct
{
    int16_t w[3];  // X/Y/Z 轴角速度
    int16_t T;     // 温度
} JY901S_GyroDef;

typedef struct
{
    int16_t Angle[3];  // Roll/Pitch/Yaw 角度
    int16_t T;         // 温度
} JY901S_AngleDef;

typedef struct
{
    int16_t h[3];  // X/Y/Z 轴磁力计
    int16_t T;     // 温度
} JY901S_MagDef;

typedef struct
{
    int16_t sDStatus[4];  // D0-D3 状态
} JY901S_DStatusDef;

typedef struct
{
    int32_t lPressure;  // 气压
    int32_t lAltitude;  // 高度
} JY901S_PressDef;

typedef struct
{
    int32_t lLon;  // 经度
    int32_t lLat;  // 纬度
} JY901S_LonLatDef;

typedef struct
{
    int16_t sGPSHeight;  // GPS高度
    int16_t sGPSYaw;     // GPS航向
    int32_t lGPSVelocity;// GPS速度
} JY901S_GPSVDef;

typedef struct
{
    int16_t q[4];  // 四元数
} JY901S_QDef;

/* 全局数据结构体（外部可通过接口访问） */
extern JY901S_TimeDef     JY901S_Time;
extern JY901S_AccDef      JY901S_Acc;
extern JY901S_GyroDef     JY901S_Gyro;
extern JY901S_AngleDef    JY901S_Angle;
extern JY901S_MagDef      JY901S_Mag;
extern JY901S_DStatusDef  JY901S_DStatus;
extern JY901S_PressDef    JY901S_Press;
extern JY901S_LonLatDef   JY901S_LonLat;
extern JY901S_GPSVDef     JY901S_GPSV;
extern JY901S_QDef        JY901S_Q;

/* 函数声明 */
void JY901S_SendCmd(uint8_t cmd[5]);                  // 发送配置指令
void JY901S_ReceiveData(uint8_t ucData);              // 解析接收数据
void JY901S_AccCalibrate(void);                       // 加速度计校准
void JY901S_MagCalibrate(void);                       // 磁力计校准
void JY901S_Init(void);                               // JY901S初始化（开启串口中断）
void JY901S_GetAttitude(float *roll, float *pitch, float *yaw);  // 获取姿态角
void JY901S_GetAccelerometer(float *acc_x, float *acc_y, float *acc_z);  // 获取加速度
void JY901S_GetGyroscope(float *gyro_x, float *gyro_y, float *gyro_z);    // 获取角速度


#endif /* INC_JY901S_H_ */
