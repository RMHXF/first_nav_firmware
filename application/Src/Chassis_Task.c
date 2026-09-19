#include "Chassis_Task.h"

Chassis_Data_t Chassis_Data;

float temp = 0;
void Chassis_Task_main(void *argument)
{
    Chassis_Init();
    while (1)
    {   
        Ctrl_Check();
        Chassis_Ctrl(&Chassis_Data);
        Chassis_Mode_Loop(&Chassis_Data);

        osDelay(1);
    }
}

void Chassis_Init(void)
{
    Chassis_Data.Chassis_mode = Chassis_mode_normal;
    Chassis_Data.Chassis_yaw = 0;
    Chassis_Data.Foward_speed = 0;
    Chassis_Data.Lateral_speed = 0;
    Chassis_Data.ctrl_mode = REMOTE_CTRL;
    for (int i = 0; i < 4; i++)
    {
        Chassis_Data.motor_val[i] = 0;
    }
}

void Ctrl_Check(void)
{
    if(remote_ctrl.s[0] == 0)
    {
         Chassis_Data.Chassis_mode = Chassis_mode_silence;
    }else{
         Chassis_Data.Chassis_mode = Chassis_mode_normal;
    }
    if(remote_ctrl.s[1] == 1)
    {
        Chassis_Data.ctrl_mode = SLAM;
    }else{
        Chassis_Data.ctrl_mode = REMOTE_CTRL;
    }
}

void Chassis_normal_mode(float Vx, float Vy, float Wz)
{
    Chassis_Data.motor_val[0] = -Vx + Vy + Wz;
    Chassis_Data.motor_val[1] = -Vx - Vy + Wz;
    Chassis_Data.motor_val[2] =  Vx - Vy + Wz;
    Chassis_Data.motor_val[3] =  Vx + Vy + Wz;
}

void Chassis_Ctrl(Chassis_Data_t *Chassis_Data_p)
{
    
    switch (Chassis_Data_p->ctrl_mode)
    {
    case REMOTE_CTRL:
        temp = (remote_ctrl.ch[2] + 660) / 1320.0f;
        Chassis_Data_p->Ctrl.Vx = remote_ctrl.ch[1] / (50 - 30 * temp);
        Chassis_Data_p->Ctrl.Vy = - remote_ctrl.ch[3] / 50;
        Chassis_Data_p->Ctrl.Wz = - remote_ctrl.ch[0] / 50;
        break;
    case SLAM:
        Chassis_Data_p->Ctrl.Vx = Rx_miniPC.Target_V.vx;
        Chassis_Data_p->Ctrl.Vy = Rx_miniPC.Target_V.vy;
        Chassis_Data_p->Chassis_Wz = Rx_miniPC.Target_V.wz;
        break;
    default:
        break;
    }
}

void Chassis_Mode_Loop(Chassis_Data_t *Chassis_Data_p)
{
    switch (Chassis_Data_p->Chassis_mode)
    {
    case Chassis_mode_silence:
        for (int i = 0; i < 4; i++)
        {
            Chassis_Data_p->motor_val[i] = 0;
        }
        break;
    case Chassis_mode_normal:
        Chassis_normal_mode(Chassis_Data_p->Ctrl.Vx, Chassis_Data_p->Ctrl.Vy, Chassis_Data_p->Ctrl.Wz);
        break;
        case Chassis_mode_follow:
        
        break;
    default:
        break;
    }
}

//默认电机顺时针转为正转
void Chassis_Fwd_solution(Chassis_Motor_t *data,Velocity_t *current_V)
{
    float V_temp[4];
    V_temp[0] = data->motor[0].para.vel * CHASSIS_WHELL_RADIUS;
    V_temp[1] = data->motor[1].para.vel * CHASSIS_WHELL_RADIUS;
    V_temp[2] = data->motor[2].para.vel * CHASSIS_WHELL_RADIUS;
    V_temp[3] = data->motor[3].para.vel * CHASSIS_WHELL_RADIUS;
    current_V->vx = 0.25 *(- V_temp[0] * SIN_45 - V_temp[1] * SIN_45 + V_temp[2] * SIN_45 + V_temp[3] * SIN_45);
    current_V->vy = 0.25 *(V_temp[0] * SIN_45 + V_temp[1] * SIN_45 + V_temp[2] * SIN_45 + V_temp[3] * SIN_45);
    current_V->wz = 0.25 * (V_temp[0] + V_temp[1] + V_temp[2] + V_temp[3]) /(CHASSIS_LX + CHASSIS_LY);
}