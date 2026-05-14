/**
 * @file tsk_config_and_callback.cpp
 * @brief Task init and CAN callback dispatch.
 */

#include "tsk_config_and_callback.h"

#include "ChassisTask.h"
#include "ClampingTask.h"
#include "climbingTask2.h"
#include "drv_bsp.h"
#include "drv_can.h"
#include "drv_tim.h"
#include "drv_uart.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

extern "C" void Chassis_CAN_Rx_Dispatch(CAN_HandleTypeDef *hcan, Struct_CAN_Rx_Buffer *Rx_Buffer);

extern Class_Motor_C620 Motor_Z;
extern Class_Motor_C610 Motor_X;
extern Class_Motor_C610 Motor_R;

static void CAN1_Global_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer) {
    switch (Rx_Buffer->Header.StdId) {
        case (0x202): {
            Climbing_CAN_Rx_Dispatch(Rx_Buffer);
            break;
        }
        case (0x203): {
            Climbing_CAN_Rx_Dispatch(Rx_Buffer);
            break;
        }
        case (0x204): {
            Climbing_CAN_Rx_Dispatch(Rx_Buffer);
            break;
        }
        case (0x206): {
            Motor_X.CAN_RxCpltCallback(Rx_Buffer->Data);
            break;
        }
        case (0x207): {
            Motor_R.CAN_RxCpltCallback(Rx_Buffer->Data);
            break;
        }
        case (0x205): {
            Clamping_CAN_Rx_Dispatch(Rx_Buffer);
            break;
        }
        default:
            break;
    }
}

void CAN2_Global_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer) {
    switch (Rx_Buffer->Header.StdId) {
        case (0x201):
        case (0x202):
        case (0x203):
        case (0x204):
            Chassis_CAN_Rx_Dispatch(&hcan2, Rx_Buffer);
            break;
        case (0x205):
            Motor_Z.CAN_RxCpltCallback(Rx_Buffer->Data);
            break;
            case (0x206): {
            Climbing_CAN_Rx_Dispatch(Rx_Buffer);
            break;
        }
    }
}



void Task_Init(void) {
    BSP_Init(BSP_DC24_LU_ON | BSP_DC24_LD_ON | BSP_DC24_RU_ON | BSP_DC24_RD_ON | BSP_LED_1_ON | BSP_LED_8_ON);

    CAN_Init(&hcan1, CAN1_Global_Call_Back);
    CAN_Init(&hcan2, CAN2_Global_Call_Back);

    HAL_TIM_Base_Start_IT(&htim6);
    TIM_Init(&htim6,TIM_CAN_PeriodElapsedCallback);
    //Chassis_Task_Init();
    //Climbing_Task_Init();
    //Clamping_Task_Init();

    //UART_Init(&huart6, MAVLink_UART6_Rx_Call_Back, UART_BUFFER_SIZE);
}
