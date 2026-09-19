#ifndef RM__TRANS_TASK_H
#define RM__TRANS_TASK_H

#include "freertos.h"
#include "cmsis_os.h"
#include "usart.h"
#include "dma.h"
#include "i6x.h"
#include "usbd_cdc_if.h"

#include "bsp_fdcan.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"

#define USART5_BUFLEN 18
#define RX_MINIPC_DATA_LEN 32
#define TX_MINIPC_DATA_LEN 20
#define MINIPC_SEND_HEADER 0xA5
#define MINIPC_RECV_HEADER 0x5A
#define MINIPC_SEND_TAIL 0xB5
#define MINIPC_RECV_TAIL 0x5B

extern i6x_ctrl_t remote_ctrl;

typedef struct
{
    motor_t motor[4];
} Chassis_Motor_t;

typedef struct
{
    float vx;
    float vy;
    float wz;
} Velocity_t;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} IMU_t;

struct ReceivePacket
{
    uint8_t header;
    Velocity_t Target_V;
    IMU_t IMU_lidar;
    uint32_t timestamp;  // (ms) board time
	uint8_t tail;
    uint16_t checksum;
}__attribute__((packed));

struct SendPacket
{
    uint8_t header;
    Velocity_t Current_V;
    uint32_t timestamp;  // (ms) board time
	uint8_t tail;
    uint16_t checksum;
}__attribute__((packed));

typedef struct ReceivePacket ReceivePacket_t;
typedef struct SendPacket SendPacket_t;

extern ReceivePacket_t Rx_miniPC;
extern SendPacket_t Tx_miniPC;

void Trans_Task_main(void *argument);
void Trans_Init(void);
void remoter_start(void);
void chassis_motor_enable(void);
void chassis_motor_disable(void);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void ctrl_dm_motor(void);
void MiniPC_Data_Read(uint8_t *buf,ReceivePacket_t *Rx_miniPC);
void MiniPC_Data_Send_Process(SendPacket_t *Data,Velocity_t *Current_V);
void MiniPC_Data_Transmit(SendPacket_t *Data);
#endif