#include "Trans_Task.h"
#include "Chassis_Task.h"
#include "CRC_Check.h"
#include "string.h"
extern DMA_HandleTypeDef hdma_uart5_rx;
extern UART_HandleTypeDef huart5;

ReceivePacket_t Rx_miniPC;
SendPacket_t Tx_miniPC;

Chassis_Motor_t Chassis_Motor;
Velocity_t Chassis_current_V;
uint8_t sbus_rx_done = 0;
uint8_t remote_loss = 0;
i6x_ctrl_t remote_ctrl;
float test_val = 1;
uint8_t usart5_buf[I6X_FRAME_LENGTH] __attribute__((section(".dma_buffers")));

void Trans_Task_main(void *argument)
{
    Trans_Init();
    osDelay(1000);
    chassis_motor_enable();
    while (1)
    {
        ctrl_dm_motor();
        if(!remote_loss && remote_ctrl.frame_lost)
        {
            chassis_motor_disable();
        }
        else if(remote_loss && !remote_ctrl.frame_lost)
        {
            chassis_motor_enable();
        }
        remote_loss = remote_ctrl.frame_lost;
        Chassis_Fwd_solution(&Chassis_Motor,&Chassis_current_V);
        MiniPC_Data_Send_Process(&Tx_miniPC,&Chassis_current_V);
        MiniPC_Data_Transmit(&Tx_miniPC);

    }
}

void remoter_start(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_buf, I6X_FRAME_LENGTH);
}

void chassis_motor_enable(void)
{
    dm_motor_enable(&hfdcan1, &motor[Motor1]);
    osDelay(10);
    dm_motor_enable(&hfdcan1, &motor[Motor2]);
    osDelay(10);
    dm_motor_enable(&hfdcan1, &motor[Motor3]);
    osDelay(10);
    dm_motor_enable(&hfdcan1, &motor[Motor4]);
    osDelay(10);  
}

void chassis_motor_disable(void)
{
    dm_motor_disable(&hfdcan1, &motor[Motor1]);
    osDelay(10);
    dm_motor_disable(&hfdcan1, &motor[Motor2]);
    osDelay(10);
    dm_motor_disable(&hfdcan1, &motor[Motor3]);
    osDelay(10);
    dm_motor_disable(&hfdcan1, &motor[Motor4]);
    osDelay(10);  
}

void Trans_Init(void)
{
    remoter_start();
    dm_motor_init();
    bsp_can_init();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART5)
    {
        sbus_to_i6x(&remote_ctrl, usart5_buf);
        sbus_rx_done = 1;
        // 立即重新启动 DMA，在下一个 SBUS 帧到来前完成重啟
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_buf, I6X_FRAME_LENGTH);
    }
}

void ctrl_dm_motor(void)
{
    spd_ctrl(&hfdcan1, 0x01,Chassis_Data.motor_val[0]);
    osDelay(1);
    spd_ctrl(&hfdcan1, 0x02,Chassis_Data.motor_val[1]);
    osDelay(1);
    spd_ctrl(&hfdcan1, 0x03,Chassis_Data.motor_val[2]);
    osDelay(1);
    spd_ctrl(&hfdcan1, 0x04,Chassis_Data.motor_val[3]);
    osDelay(1);
}

void MiniPC_Data_Read(uint8_t *buf,ReceivePacket_t *Rx_miniPC)
{
    if(buf == NULL) return;
    if(buf[0] == MINIPC_RECV_HEADER)
    {
        ReceivePacket_t RX_miniPC_Data_temp;
        memcpy(&RX_miniPC_Data_temp,buf,RX_MINIPC_DATA_LEN);
        uint16_t checksum;
        checksum = RX_miniPC_Data_temp.checksum;
        if(Verify_CRC16_Check_Sum(buf,RX_MINIPC_DATA_LEN) == checksum)
        {
            memcpy(&Rx_miniPC,buf,RX_MINIPC_DATA_LEN);
        }
    }
    if(*(buf + RX_MINIPC_DATA_LEN) == 0xA5)
    {
        MiniPC_Data_Read((buf + RX_MINIPC_DATA_LEN),Rx_miniPC);
    }
}

void MiniPC_Data_Send_Process(SendPacket_t *Data,Velocity_t *Current_V)
{
    Data->header = MINIPC_SEND_HEADER;
    Data->timestamp = HAL_GetTick();
    Data->Current_V.vx = Current_V->vx;
    Data->Current_V.vy = Current_V->vy;
    Data->Current_V.wz = Current_V->wz;
    Data->tail = MINIPC_SEND_TAIL;
    Data->checksum = Get_CRC16_Check_Sum((uint8_t *)Data,TX_MINIPC_DATA_LEN-2,0);
}

void MiniPC_Data_Transmit(SendPacket_t *Data)
{   
    uint8_t buf[TX_MINIPC_DATA_LEN] = {0};
    memcpy(buf,&Data,TX_MINIPC_DATA_LEN);
    CDC_Transmit_HS(buf,TX_MINIPC_DATA_LEN);
}
