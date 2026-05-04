#include "ClampingTask.h"

#include "cmsis_os2.h"
#include "main.h"

ClampingController clampingCtrl;

ClampingController &Clamping_Get_Controller(void)
{
    return clampingCtrl;
}

void Clamping_CAN_Rx_Dispatch(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    if (Rx_Buffer == nullptr)
    {
        return;
    }
    clampingCtrl.CAN_RxCallback(Rx_Buffer->Header.StdId, Rx_Buffer->Data);
}

void ClampingTask(void)
{
    clampingCtrl.Init(&hcan1);
    for (;;)
    {
        clampingCtrl.TaskEntry1ms();
        osDelay(1);
    }
}

void Clamping_Task_Init(void)
{
    clampingCtrl.Init(&hcan1);
}
