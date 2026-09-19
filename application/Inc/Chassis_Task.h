#ifndef RM_CHASSIS_TASK_H
#define RM_CHASSIS_TASK_H 

#include "freertos.h"
#include "cmsis_os.h" 
#include "Trans_Task.h"

#define CHASSIS_MAX_SPEED 100
#define CHASSIS_WHELL_RADIUS 0.076
#define SIN_45 0.7071
#define CHASSIS_LX 0.16
#define CHASSIS_LY 0.22

typedef enum
{
    Chassis_mode_silence = 0,
    Chassis_mode_normal,
    Chassis_mode_follow,
}Chassis_mode_t;

typedef enum
{
    REMOTE_CTRL = 0,
    KEYBOARD_MOUSE,
    CUSTOM_CONTROLLOR,
    SLAM,
}Chassis_ctrl_mode_t;

typedef struct
{
    float Wz;
    float Chassis_yaw;
    float Vx;
    float Vy;
}Chassis_ctrl_t;

typedef struct
{
    Chassis_mode_t Chassis_mode;
    Chassis_ctrl_mode_t ctrl_mode;
    Chassis_ctrl_t  Ctrl;

    float motor_val[4];
    float Chassis_yaw;
    float Chassis_Wz;
    float Foward_speed;
    float Lateral_speed;

    uint8_t is_offline;
}Chassis_Data_t;

extern Chassis_Data_t Chassis_Data;

void Chassis_Task_main(void *argument);
void Chassis_Init(void);
void Ctrl_Check(void);
void Chassis_normal_mode(float Vx, float Vy, float Wz);
void Chassis_Ctrl(Chassis_Data_t *Chassis_Data_p);
void Chassis_Mode_Loop(Chassis_Data_t *Chassis_Data_p);
void Chassis_Fwd_solution(Chassis_Motor_t *data,Velocity_t *current_V);
#endif