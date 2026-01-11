#ifndef __ATTACK_TASK_H
#define __ATTACK_TASK_H

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "RUI_DBUS.h"
#include "MY_define.h"
#include "Monitor_Task.h"
#include "Motors.h"
#include "pid_temp.h"
#include "WHW_IRQN.h"
void SupplyContinue(uint8_t mod,float SupplySpeed);
uint8_t Attack_task(MOTOR_Typdef *MOTOR, RUI_ROOT_STATUS_Typedef *Root,float SupplySpeed,uint8_t mod);
uint8_t RUI_F_MOTOR_PID_ATTACK_INIT(MOTOR_Typdef *MOTOR);
void FirctionControl(DJI_MOTOR_Typedef  *MOTOR, uint8_t mod, int32_t Speed);
void CkilckFire_hore(MOTOR_Typdef *MOTOR);
void DailMotorTwoPID(MOTOR_Typdef *  motor);
#endif