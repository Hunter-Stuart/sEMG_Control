#include "jy901s.h"
#include <string.h>

/* 全局数据结构体实例 */
JY901S_TimeDef     JY901S_Time;
JY901S_AccDef      JY901S_Acc;
JY901S_GyroDef     JY901S_Gyro;
JY901S_AngleDef    JY901S_Angle;
JY901S_MagDef      JY901S_Mag;
JY901S_DStatusDef  JY901S_DStatus;
JY901S_PressDef    JY901S_Press;
JY901S_LonLatDef   JY901S_LonLat;
JY901S_GPSVDef     JY901S_GPSV;
JY901S_QDef        JY901S_Q;

/* 静态接收缓冲区 */
static uint8_t ucRxBuffer[256];
static uint8_t ucRxCount = 0;
static uint8_t RxBuffer=0;///
char message_uart[100]="";

/* 校准指令定义 */
static const uint8_t ACCCALSW[5]     = {0XFF,0XAA,0X01,0X01,0X00};  // 进入加速度校准模式
static const uint8_t SAVACALSW[5]    = {0XFF,0XAA,0X00,0X00,0X00};  // 保存加速度校准配置
static const uint8_t MAGNETICCALAM[5]= {0XFF,0XAA,0X01,0X07,0X00};  // 进入磁力计校准模式
static const uint8_t SAVEMAGNETICCALAM[5]={0XFF,0XAA,0X00,0X00,0X00};// 保存磁力计校准配置

/**
 * @brief  发送JY901S配置指令
 * @param  cmd: 5字节指令数组
 * @retval 无
 */
void JY901S_SendCmd(uint8_t cmd[5])
{
    HAL_UART_Transmit(&huart1, cmd, 5, 100);  // HAL库串口发送（超时100ms）
}

/**
 * @brief  解析JY901S串口接收数据
 * @param  ucData: 接收到的单字节数据
 * @retval 无
 */
void JY901S_ReceiveData(uint8_t ucData)
{
    ucRxBuffer[ucRxCount++] = ucData;

    /* 数据头校验：必须以0x55开头 */
    if (ucRxBuffer[0] != 0x55)
    {
        ucRxCount = 0;
        return;
    }

    /* 数据包长度校验（JY901S单包数据为11字节） */
    if (ucRxCount < 11)
    {
        return;
    }

    /* 数据类型解析 */
    switch (ucRxBuffer[1])
    {
        case 0x50: memcpy(&JY901S_Time,    &ucRxBuffer[2], 8); break;  // 时间数据
        case 0x51: memcpy(&JY901S_Acc,     &ucRxBuffer[2], 8); break;  // 加速度数据
        case 0x52: memcpy(&JY901S_Gyro,    &ucRxBuffer[2], 8); break;  // 陀螺仪数据
        case 0x53: memcpy(&JY901S_Angle,   &ucRxBuffer[2], 8); break;  // 角度数据
        case 0x54: memcpy(&JY901S_Mag,     &ucRxBuffer[2], 8); break;  // 磁力计数据
        case 0x55: memcpy(&JY901S_DStatus, &ucRxBuffer[2], 8); break;  // DIO状态数据
        case 0x56: memcpy(&JY901S_Press,   &ucRxBuffer[2], 8); break;  // 气压高度数据
        case 0x57: memcpy(&JY901S_LonLat,  &ucRxBuffer[2], 8); break;  // 经纬度数据
        case 0x58: memcpy(&JY901S_GPSV,    &ucRxBuffer[2], 8); break;  // GPS数据
        case 0x59: memcpy(&JY901S_Q,       &ucRxBuffer[2], 8); break;  // 四元数数据
        default: break;
    }

    ucRxCount = 0;  // 清空接收计数
}

/**
 * @brief  加速度计校准（自动发送校准指令+保存）
 * @retval 无
 */
void JY901S_AccCalibrate(void)
{
    JY901S_SendCmd((uint8_t*)ACCCALSW);
    HAL_Delay(100);
    JY901S_SendCmd((uint8_t*)SAVACALSW);
    HAL_Delay(100);
}

/**
 * @brief  磁力计校准（自动发送校准指令+保存）
 * @retval 无
 */
void JY901S_MagCalibrate(void)
{
    JY901S_SendCmd((uint8_t*)MAGNETICCALAM);
    HAL_Delay(100);
    JY901S_SendCmd((uint8_t*)SAVEMAGNETICCALAM);
    HAL_Delay(100);
}

/**
 * @brief  JY901S初始化（开启串口中断接收）
 * @retval 无
 */
void JY901S_Init(void)
{
    /* 开启串口3中断接收（每次接收1字节） */
    HAL_UART_Receive_IT(&huart1, &RxBuffer, 1);
}

/**
 * @brief  获取姿态角（Roll/Pitch/Yaw）
 * @param  roll: 横滚角（输出，单位°）
 * @param  pitch: 俯仰角（输出，单位°）
 * @param  yaw: 航向角（输出，单位°）
 * @retval 无
 */
void JY901S_GetAttitude(float *roll, float *pitch, float *yaw)
{
    if(roll != NULL)  *roll  = JY901S_Angle.Angle[0] / 32768.0f * 180.0f;
    if(pitch != NULL) *pitch = JY901S_Angle.Angle[1] / 32768.0f * 180.0f;
    if(yaw != NULL)   *yaw   = JY901S_Angle.Angle[2] / 32768.0f * 180.0f;
}

/**
 * @brief  获取加速度计数据
 * @param  acc_x: X轴加速度（输出，单位g）
 * @param  acc_y: Y轴加速度（输出，单位g）
 * @param  acc_z: Z轴加速度（输出，单位g）
 * @retval 无
 */
void JY901S_GetAccelerometer(float *acc_x, float *acc_y, float *acc_z)
{
    if(acc_x != NULL) *acc_x = JY901S_Acc.a[0] / 32768.0f * 16.0f;
    if(acc_y != NULL) *acc_y = JY901S_Acc.a[1] / 32768.0f * 16.0f;
    if(acc_z != NULL) *acc_z = JY901S_Acc.a[2] / 32768.0f * 16.0f;
}

/**
 * @brief  获取陀螺仪角速度数据
 * @param  gyro_x: X轴角速度（输出，单位°/s）
 * @param  gyro_y: Y轴角速度（输出，单位°/s）
 * @param  gyro_z: Z轴角速度（输出，单位°/s）
 * @retval 无
 */
void JY901S_GetGyroscope(float *gyro_x, float *gyro_y, float *gyro_z)
{
    if(gyro_x != NULL) *gyro_x = JY901S_Gyro.w[0] / 32768.0f * 2000.0f;
    if(gyro_y != NULL) *gyro_y = JY901S_Gyro.w[1] / 32768.0f * 2000.0f;
    if(gyro_z != NULL) *gyro_z = JY901S_Gyro.w[2] / 32768.0f * 2000.0f;
}

/**
 * @brief  HAL库串口接收完成回调函数
 * @param  huart: 串口句柄
 * @retval 无
 * @note   需在stm32f4xx_it.c中重写该函数，或直接放在此处（需确保编译优先级）
 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//HAL_UART_Transmit(&huart2, (uint8_t*)"success", strlen("success"), 100);
    if (huart->Instance == USART1)
    {
    	static int i=0;
        /* 解析接收到的数据 */
        JY901S_ReceiveData(RxBuffer);

        /* 重新开启中断接收（必须重新开启，否则只接收1次） */
        HAL_UART_Receive_IT(&huart1, &RxBuffer, 1);
        if(i==10)
        {
        	sprintf(message_uart,"%.2f,%.2f,%.2f\n",JY901S_Angle.Angle[0] / 32768.0f * 180.0f,
        			JY901S_Angle.Angle[1] / 32768.0f * 180.0f,
					JY901S_Angle.Angle[2] / 32768.0f * 180.0f);
        	HAL_UART_Transmit(&huart2, (uint8_t*)message_uart, strlen(message_uart), 100);
        	i=0;
        }
        i++;
    }
}
