#ifndef R2_USER_MODULE_CLAMPING_CLAMPING_CONTROLLER_H
#define R2_USER_MODULE_CLAMPING_CLAMPING_CONTROLLER_H

#include "stm32f4xx_hal.h"
#include "dvc_motor.h"
#include "arm.h"

#define CLAMPING_MOTOR_CAN_ID           CAN_Motor_ID_0x205
#define CLAMPING_MOTOR_STD_ID           (0x205U)
#define CLAMPING_ANGLE_TO_RAD(deg)      (-(deg) * PI / 180.0f)
#define CLAMPING_TARGET_ANGLE_CLAMP_RAD (-PI / 2.0f)
#define CLAMPING_TARGET_ANGLE_RESET_RAD (0.0f)
#define CLAMPING_TARGET_ANGLE_MAX_ANGLE (180.0f)
#define CLAMPING_TARGET_ANGLE_MIN_ANGLE (0.0f)
#define CLAMPING_PLAN_DT_S              (0.001f)
#define CLAMPING_MOVE_DURATION_S        (0.3f)

#define CLAMPING_PID_OMEGA_KP           (2000.0f)
#define CLAMPING_PID_OMEGA_KI           (500.0f)
#define CLAMPING_PID_ANGLE_KP           (18.0f)
#define CLAMPING_PID_ANGLE_KI           (0.0f)
#define CLAMPING_PID_OMEGA_OUT_MAX      (5000.0f)
#define CLAMPING_PID_OMEGA_I_OUT_MAX    (7000.0f)
#define CLAMPING_PID_ANGLE_OUT_MAX      (10.0f)
#define CLAMPING_PID_ANGLE_I_OUT_MAX    (15.0f)

#define CLAMPING_LESO_FREQUENCY         (8.0f)
#define CLAMPING_LESO_GAIN              (0.1f)
#define CLAMPING_LESO_KP                (12.0f)

class ClampingController
{
private:
    Class_Motor_C610 motor_clamp_2006_;
    QuinticPlanner angle_planner_;

    void PlanToAngle(float target_angle);

public:
    ClampingController();

    void Init(CAN_HandleTypeDef *hcan);
    void TaskEntry1ms(void);
    void CAN_RxCallback(uint32_t std_id, uint8_t *data);

    void MoveToAngle(float angle);
    void MoveToClampAngle(void);
    void MoveToResetAngle(void);

    void OpenSolenoid(void);
    void ReleaseSolenoid(void);

    Class_Motor_C610 &GetMotor(void);
};

#endif
