#include "Trans_Task.h"
#include "CRC_Check.h"
extern DMA_HandleTypeDef hdma_uart5_rx;
extern UART_HandleTypeDef huart5;

ReceivePacket_t Rx_miniPC;
SendPacket_t Tx_miniPC;

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
//            chassis_motor_disable();
        }
        else if(remote_loss && !remote_ctrl.frame_lost)
        {
//            chassis_motor_enable();
        }
        remote_loss = remote_ctrl.frame_lost;
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
//    spd_ctrl(&hfdcan1, 0x01,Chassis_Data.motor_val[0]);
    spd_ctrl(&hfdcan1, 0x01,test_val);
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

}