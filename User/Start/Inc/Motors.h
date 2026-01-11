#ifndef __MOTORS_H
#define __MOTORS_H

#include "DJI_Motor.h"
#include "DM_Motor.h"

typedef struct
{
  DJI_MOTOR_Typedef DJI_3508_Pitch;
  DJI_MOTOR_Typedef DJI_3508_Yaw;
	DJI_MOTOR_Typedef DJI_3508_RightFriction;
	DJI_MOTOR_Typedef DJI_3508_LeftFriction;
	DJI_MOTOR_Typedef DJI_3508_RightFriction2;
	DJI_MOTOR_Typedef DJI_3508_LeftFriction2;
	DJI_MOTOR_Typedef DJI_3508_supply;
  DJI_MOTOR_Typedef DJI_Motor_Test;
  DJI_MOTOR_Typedef DJI_3508_UP_Friction;
	DJI_MOTOR_Typedef DJI_3508_Right_Friction;
	DJI_MOTOR_Typedef DJI_3508_Left_Friction;
	DJI_MOTOR_Typedef DJI_3508_Three_supply;
	DM_MOTOR_Typdef DM4310_Yaw;
	DM_MOTOR_Typdef DM4340_Pitch;
	DM_MOTOR_Typdef DM4310_SUPPLY;

}MOTOR_Typdef;

typedef struct
{
    /******************************遥控********************************/
    uint8_t RM_DBUS;
    /***************************自定义控制器********************************/
    uint8_t RM_RX;
    /******************************头部电机********************************/
    uint8_t MOTOR_HEAD_Pitch;
         uint8_t    MOTOR_HEAD_Roll;
	  uint8_t MOTOR_HEAD_Yaw;
    /******************************主控位置********************************/
    uint8_t MASTER_LOCATION;
    /******************************运行模式********************************/
    uint8_t RM_MOD;
/*****************************发射机构********************************/
    uint8_t  MOTOR_Shoot_L;//左
	  uint8_t  MOTOR_Shoot_R;//右
	  uint8_t  MOTOR_Shoot_M;//中
	  uint8_t  MOTOR_Shoot_S;//供弹
	
}RUI_ROOT_STATUS_Typedef;



#endif
