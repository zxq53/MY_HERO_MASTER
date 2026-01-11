#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "RUI_DBUS.h"
#include "MY_define.h"
#include "Monitor_Task.h"
#include "Motors.h"
union gmTOch_typdef		//使用共用体整合数据
{
	struct {			
		int16_t vx:11;		//平移速度
		int16_t vy:11;		//前进速度
		int16_t vr:11;		//旋转速度
		uint16_t key_q:1;
		uint16_t key_e:1;
		uint16_t key_r:1;
		uint16_t key_z:1;
		uint16_t key_x:1;
		uint16_t key_c:1;
		uint16_t key_v:1;
		uint16_t key_shift:1;
		uint16_t key_ctrl:1;
		uint16_t key_f:1;
		uint16_t key_g:1;
		uint16_t key_b:1;
		uint16_t s1:3;
		uint16_t supUSe :1;			//是否使用电容
		uint16_t romoteOnLine	:1;	//遥控是否在线
		uint16_t s2 :2;			//底盘状态
		uint16_t topSate:1;			//陀螺仪状态
		uint16_t target:1;
		
	}dataNeaten;
	//CAN发送的数据
	uint8_t sendData[8];
	uint8_t getData[8];
};

union chTOgm_typdef		//使用共用体整合数据
{
	struct {
		int16_t pitch;
		int16_t yaw;
		float time;
	}dataNeaten_angle;
	struct{
		uint64_t muzzleColing:16;	//枪口热量
		uint64_t maxSpeed:8;		//最大射速
		uint64_t nowSpeed:8;		//当前射速
		uint64_t target:1;			//是否识别成功标志位
		uint64_t visionMod:3;		//视觉的状态
		uint64_t visionState:1;		//视觉在线的状态
		uint64_t judgeState:1;		//裁判系统的状态
		uint64_t :0;				//保留
	}dataNeaten_another;
	//CAN发送的数据
	uint16_t sendData[4];
	uint8_t getData[8];
};
struct CanCommunit_typedef
{
		union chTOgm_typdef chTOgm;
		union gmTOch_typdef gmTOch;
};
extern struct CanCommunit_typedef CanCommunit_t;

uint8_t RUI_F_MOTOR_PID_HEAD_INIT(MOTOR_Typdef *MOTOR);
uint8_t gimbal_task(MOTOR_Typdef *MOTOR,RUI_ROOT_STATUS_Typedef *Root,uint8_t MOD);
uint8_t DMPitchPID( MOTOR_Typdef * motor,  float TopPitchSpeed, float topAngle);
void p_limit();
void GIMBAL_init();
uint8_t DM_YAWPID(MOTOR_Typdef * MOTOR,  float TopPitchSpeed, float topAngle);
#endif
