#ifndef __ALL_INIT_H
#define __ALL_INIT_H

#include "can_bsp.h"
#include "bsp_dwt.h"
#include "tim.h"
#include "usart.h"
#include "RUI_DBUS.h"
#include "Referee.h"
#include "Image_RX.h"
#include "cmsis_os.h"
#include "Monitor_Task.h"
#include "MY_Define.h"
#include "fdcan.h"
#include "DJI_Motor.h"
#include "IMU_Task.h"
#include "ws2812.h"
#include "string.h"
#include "RUI_MATH.h"
#include "adc.h"
#include "bsp_buzzer.h"
#include "DM_Motor.h"
#include "Gimbal_Task.h"
#include "Motors.h"
#include "usbd_cdc_if.h"
#include "NIMING.h"
#include "K3debug_Task.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "VOFA.h"
#include "DM_test.h"
#include "attack_Task.h"
#include "Vision.h"
#include "stdint.h"
#include "Trance.h"

void Everying_Init(void);
//extern VisionTemp U;
extern uint64_t RunTime;
extern uint8_t DBUS_RX_DATA[18] __attribute__((section(".ARM.__at_0x24000000")));
extern DBUS_Typedef WHW_V_DBUS;
extern CONTAL_Typedef RUI_V_CONTAL;

extern ALL_RX_Data_T ALL_RX;
extern User_Data_T User_data;

extern uint8_t RX[20];

extern MOTOR_Typdef ALL_MOTOR;

extern FuzzyRule_t fuzzyRules;

extern PID_t temppid;

extern IMU_Data_t IMU_Data;
extern float IMU_Last;
extern   float count;
extern  float countImu;
extern V_Input_t V_Input;

extern RUI_ROOT_STATUS_Typedef RUI_ROOT_STATUS;
//extern VisionRxDataUnion VisionRxData;
extern uint8_t VisionRx[30];
extern select_t selects;
extern Data_RX_0x302_t Data_RX_0x302;
extern Data_RX_0x304_t Data_RX_0x304;
extern uint8_t ppp;
extern MOTOR_Typdef Gimbal;
extern uint8_t tx[20];
extern uint8_t fire ;
extern uint8_t fire1 ;
extern  uint8_t rounds;
extern uint8_t roundangel;
#endif
