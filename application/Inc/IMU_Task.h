/**
 *******************************************************************************
 * @file        IMU_Task.h
 * @brief       IMU姿态解算任务 - 头文件
 * @details     基于BMI088 IMU传感器的姿态解算模块
 *              - 利用Mahony互补滤波器融合加速度计与陀螺仪数据
 *              - 上电自动完成陀螺仪零偏校准
 *              - PID温控回路维持传感器工作温度
 *******************************************************************************
 */
#ifndef _RM_IMU_TASK_H_
#define _RM_IMU_TASK_H_

#include "freertos.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "tim.h"
#include "BMI088driver.h"
#include "BMI088Middleware.h"
#include "BMI088reg.h"
#include "MahonyAHRS.h"
#include "pid.h"

/*============================================================================
 * 参数设定
 *============================================================================*/
#define IMU_TEMP_LIMIT       60        /**< 温度阈值（℃），超温时关闭加热并报错  */
#define IMU_TEMP_SET         47.0f     /**< 温度PID设定值（℃）                 */
#define correct_Time_define  5000      /**< 上电零偏校准采样次数（5000ms）      */

/*============================================================================
 * 函数声明
 *============================================================================*/

/**
 * @brief   IMU任务入口（RTOS线程）
 * @param   argument   RTOS线程参数（未使用）
 * @retval  void
 */
void IMU_Task(void *argument);

/**
 * @brief   IMU系统初始化
 * @details 初始化温度PID控制器、启动加热PWM、等待传感器稳定后检测BMI088在线状态
 * @retval  void
 */
void IMU_Init(void);

/**
 * @brief   IMU温度PID控制（每300ms执行一次）
 * @details 读取BMI088温度 → 过温保护判断 → PID计算 → 更新加热PWM占空比
 * @retval  void
 */
void IMU_temp_Ctrl(void);

/**
 * @brief   陀螺仪上电零偏校准状态机
 * @details 校准阶段累加原始数据计算均值；完成后不在此函数内应用修正
 *          （零偏修正已在 IMU_Data_Read 内部执行）
 * @param   status  校准状态指针（Gyro_Init_Flag=校准中, Gyro_Read_Flag=完成）
 * @retval  void
 */
void Gyro_Init(uint8_t *status);

void IMU_Data_Read(void);

#endif /* _RM_IMU_TASK_H_ */