/**
 *******************************************************************************
 * @file        IMU_Task.c
 * @brief       IMU姿态解算任务 - 实现
 * @details     基于Mahony互补滤波器的IMU姿态解算方案
 *
 *  系统工作流程
 *  ============
 *  1. 上电 → IMU_Init() 初始化PID、启动PWM加热、等待传感器就绪
 *  2. 以 1kHz 循环执行：
 *     a) IMU_temp_Ctrl()       — 每 300ms 一次温度PID控制
 *     b) Gyro_Init()           — 前 1000ms 采集零偏均值
 *     c) IMU_Data_Read()       — 读取传感器 → 减零偏 → Mahony解算姿态角
 *  3. 校准完成后，roll/pitch/yaw 姿态角开始更新
 *
 *  关键数据结构
 *  ============
 *  - IMU_Data:    存储加速度、陀螺仪、温度、姿态角（roll/pitch/yaw）
 *  - MahonyAHRS:  Mahony互补滤波器状态（四元数 + 中间变量）
 *  - gyro_init:   陀螺仪三轴零偏值，校准完成后不再变化
 *  - pid_temp:    温度PID控制器实例
 *******************************************************************************
 */
#include "IMU_Task.h"

/*============================================================================
 * 陀螺仪校准状态宏
 *============================================================================*/
#define Gyro_Init_Flag 0    /**< 校准中：累加原始数据计算零偏均值       */
#define Gyro_Read_Flag 1    /**< 校准完成：正常运行，使用修正后数据     */

/*============================================================================
 * 全局变量
 *============================================================================*/
Mahony_t    MahonyAHRS    = {0};           /**< Mahony滤波器状态结构体         */
IMU_Data_t  IMU_Data      = {0};           /**< IMU传感器数据与姿态角          */
float       gyro_init[3]  = {0};           /**< 陀螺仪三轴零偏值              */
pid_para_t pid_temp;                      /**< 温度PID控制器                 */

float       pid_temp_args[3] = {            /**< 温度PID参数 {Kp, Ki, Kd}      */
    200.0f,                                 /**< 比例系数                      */
    0.5f,                                  /**< 积分系数                      */
    0.2f                                   /**< 微分系数                      */
};
float gyro_raw[3] = {0};                    /**< 陀螺仪原始数据缓存            */
float gyro_sum[3] = {0};                    /**< 陀螺仪原始数据累加和缓存   */
/*============================================================================
 * 调试标志
 *============================================================================*/
#define DEBUG 1
#if DEBUG
int is_temp_err = 0;    /**< 温度异常标志（超温时置1） */
int is_err      = 0;    /**< 传感器离线标志（初始化失败时置1） */
float max_out1 =500.0f;
float max_iout1 = 50.0f;
int safe_pulse = 120;
#endif

/*============================================================================
 * 模块内部状态变量
 *============================================================================*/
uint8_t Init_Status = Gyro_Init_Flag;   /**< 陀螺仪校准状态，初值为"校准中" */

/**
 *******************************************************************************
 * @brief       IMU_Task - IMU主任务（RTOS线程入口）
 * @param       argument  RTOS线程参数（未使用）
 * @retval      void
 * @details     上电初始化后以1kHz频率循环执行温控、零偏校准和姿态解算。
 *              校准完成后（~1秒）姿态角开始正常输出。
 *******************************************************************************
 */
void IMU_Task(void *argument)
{
    IMU_Init();                             /* 传感器及外设初始化              */
    while (1)
    {
        osDelay(1);                         /* 1ms 周期 = 1kHz 控制/解算频率  */
        IMU_Data_Read();                    /* 读取传感器 + Mahony姿态解算    */
        IMU_temp_Ctrl();                    /* 温度PID控制（内部300ms分频）    */
        Gyro_Init(&Init_Status);            /* 陀螺仪零偏校准状态机           */
    }
}

/**
 *******************************************************************************
 * @brief       IMU_Init - IMU系统初始化
 * @param       void
 * @retval      void
 * @details     执行以下初始化步骤：
 *              1. 初始化温度PID控制器（位置式PID）
 *              2. 启动TIM3 CH4 PWM输出（加热元件驱动）
 *              3. 等待2秒让BMI088传感器内部上电稳定
 *              4. 配置Mahony滤波器采样频率为1kHz
 *              5. 检测BMI088在线状态，若离线则循环重试（间隔1秒）
 *******************************************************************************
 */
void IMU_Init(void)
{
    /* ---- 初始化温度PID控制器 ---- */
    pid_para_init(&pid_temp);
	pid_limit_init(&pid_temp, max_iout1, -max_iout1, max_out1, 0.0f);
	pid_reset(&pid_temp, pid_temp_args[0], pid_temp_args[1], pid_temp_args[2]);
    /* ---- 启动加热PWM输出 ---- */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, safe_pulse);
    /* ---- 等待传感器上电稳定（3秒） ---- */
    osDelay(1000);

    /* ---- 配置Mahony滤波器采样频率 ---- */
    MahonyAHRS.invSampleFreq = 1 / 1000.0f;    /* 1kHz = 1ms 采样周期 */
    /* ---- BMI088在线检测 ---- */
    while (BMI088_init())
    {
        is_err = 1;                             /* 置位离线标志 */
        osDelay(1000);                          /* 每隔1秒重试   */
    }
}

/**
 *******************************************************************************
 * @brief       IMU_temp_Ctrl - IMU温度PID控制
 * @param       void
 * @retval      void
 * @details     每300ms执行一次：
 *              1. 通过BMI088读取当前温度
 *              2. 若温度超过 IMU_TEMP_LIMIT 阈值：
 *                 - 清除PID积分项，防止积分饱和
 *                 - 关闭加热PWM输出（CCR4 = 0）
 *                 - 置位错误标志，跳过本次PID计算
 *              3. 正常时：执行PID计算，将输出绝对值写入PWM比较寄存器
 *
 * @note        BMI088_read 会同时更新 IMU_Data.gyro 和 IMU_Data.accel，
 *              作为副作用为后续的 Gyro_Init 提供最新的原始传感器数据
 *******************************************************************************
 */
void IMU_temp_Ctrl(void)
{
    static int cnt = 0;                     /* 分频计数器（1ms → 300ms）       */
    cnt++;
    if (cnt == 300)                         /* 每300ms执行一次温度控制        */
    {
        /* ---- 过温保护 ---- */
        if (IMU_Data.temp >= IMU_TEMP_LIMIT)
        {
            pid_clear(&pid_temp);           /* 清除PID历史状态（防积分饱和）   */
            is_temp_err = 1;                /* 置位过温标志                    */
            htim3.Instance->CCR4 = 0;       /* 关闭加热PWM输出                 */
            cnt = 0;                        /* 重置分频计数器                  */
            return;                         /* 跳过本次PID计算                 */
        }

        /* ---- 正常温度控制 ---- */
        parallel_pid_ctrl(&pid_temp, IMU_TEMP_SET, IMU_Data.temp);
        htim3.Instance->CCR4 = (int)pid_temp.out_value; /* 更新加热PWM占空比      */
        cnt = 0;                            /* 重置分频计数器                  */
        
	}
}

/**
 *******************************************************************************
 * @brief       IMU_Data_Read - 读取IMU数据并解算姿态角
 * @param       void
 * @retval      void
 * @details     每个采样周期（1kHz）执行：
 *              1. 读取BMI088加速度计和陀螺仪原始数据
 *              2. 首次调用时使用当前加速度计数据初始化Mahony滤波器
 *              3. 减去陀螺仪零偏值（gyro_init[]），并对Z轴做死区滤波
 *              4. 执行Mahony互补滤波更新（融合陀螺仪+加速度计）
 *              5. 计算欧拉角（roll/pitch/yaw）
 *              6. 仅在零偏校准完成后输出姿态角到 IMU_Data
 *
 * @note        Z轴死区阈值 0.012 rad/s ≈ 0.69°/s，用于抑制静态噪声
 * @note        yaw 加 180° 将范围从 [-180°, 180°] 映射到 [0°, 360°]
 *******************************************************************************
 */
void IMU_Data_Read(void)
{
    /* ---- 读取BMI088原始数据 ---- */
    BMI088_read(IMU_Data.gyro, IMU_Data.accel, &IMU_Data.temp);
    gyro_raw[0] = IMU_Data.gyro[0];
    gyro_raw[1] = IMU_Data.gyro[1];
    gyro_raw[2] = IMU_Data.gyro[2];
    /* ---- 首次调用时初始化Mahony姿态解算 ---- */
    if (MahonyAHRS.is_Init == 0)
    {
        MahonyAHRS.is_Init = 1;
        MahonyAHRS_init(IMU_Data.accel[0], IMU_Data.accel[1], IMU_Data.accel[2],
                        0, 0, 0, &MahonyAHRS);
    }

    /* ---- 应用陀螺仪零偏修正 ---- */
    IMU_Data.gyro[0] -= gyro_init[0];
    IMU_Data.gyro[1] -= gyro_init[1];
    IMU_Data.gyro[2] -= gyro_init[2];

    /* ---- Z轴死区滤波：滤除微小角速度噪声 ---- */
    if (fabsf(IMU_Data.gyro[2]) < 0.012f)
    {
        IMU_Data.gyro[2] = 0;
    }

    /* ---- Mahony互补滤波：融合陀螺仪和加速度计数据 ---- */
    MahonyAHRS_update(IMU_Data.gyro[0], IMU_Data.gyro[1], IMU_Data.gyro[2],
                      IMU_Data.accel[0], IMU_Data.accel[1], IMU_Data.accel[2],
                      &MahonyAHRS);

    /* ---- 计算欧拉角 ---- */
    Mahony_computeAngles(&MahonyAHRS);

    /* ---- 仅在校准完成后更新姿态角输出 ---- */
    if (Init_Status == Gyro_Read_Flag)
    {
        IMU_Data.roll  = MahonyAHRS.roll_Mahony;
        IMU_Data.pitch = MahonyAHRS.pitch_Mahony;
        IMU_Data.yaw   = MahonyAHRS.yaw_Mahony + 180.0f;  /* 映射到 [0,360] */
    }
}

/**
 *******************************************************************************
 * @brief       Gyro_Init - 陀螺仪上电零偏校准状态机
 * @param       status  校准状态指针
 *                      - 输入/输出：Gyro_Init_Flag → Gyro_Read_Flag
 * @retval      void
 * @details     两阶段状态机：
 *
 *              【校准阶段】status == Gyro_Init_Flag
 *              - 累加 correct_Time_define 次陀螺仪原始数据到 gyro_init[]
 *              - 累加满后除以采样次数，得到零偏平均值
 *              - 将 *status 切换为 Gyro_Read_Flag，校准完成
 *
 *              【运行阶段】status == Gyro_Read_Flag
 *              - 此函数不再执行任何操作
 *              - 零偏修正已移至 IMU_Data_Read() 内部执行（Mahony更新前）
 *
 * @note        校准期间姿态角不更新（IMU_Data.roll/pitch/yaw 保持为0）
 * @note        零偏修正不在此函数内进行，而是在 IMU_Data_Read 中减去 gyro_init[]
 *******************************************************************************
 */
void Gyro_Init(uint8_t *status)
{
    static int cnt = 0;                 /* 校准采样计数器                  */

    if (*status == Gyro_Init_Flag)
    {
        /* ---- 校准阶段：累加原始陀螺仪数据 ---- */
        gyro_sum[0] += gyro_raw[0];
        gyro_sum[1] += gyro_raw[1];
        gyro_sum[2] += gyro_raw[2];
        cnt+=1;

        /* ---- 达到指定采样次数后计算零偏均值 ---- */
        if (cnt >= correct_Time_define)
        {
            gyro_init[0] = gyro_sum[0] / correct_Time_define;    /* X轴零偏均值              */
            gyro_init[1] = gyro_sum[1] / correct_Time_define;    /* Y轴零偏均值              */
            gyro_init[2] = gyro_sum[2] / correct_Time_define;    /* Z轴零偏均值              */
            *status = Gyro_Read_Flag;               /* 切换至运行状态            */
        }
    }
    /* 运行阶段：零偏修正由 IMU_Data_Read() 负责，此处无操作 */
}
