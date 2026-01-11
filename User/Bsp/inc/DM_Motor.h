#ifndef __DM_MOTOR_H
#define __DM_MOTOR_H

#include "RUI_MATH.h"
#include "controller.h"
#include "MY_define.h"
#include "can_bsp.h"
#define MIT_MODE  0x000
#define POS_MODE  0x100
#define SPEED_MODE  0x200

#define P_MIN   -12.5f
#define P_MAX   12.5f
#define V_MIN   -30.0f
#define V_MAX   30.0f
#define KP_MIN  0.0f
#define KP_MAX  500.0f
#define KD_MIN  0.0f
#define KD_MAX  5.0f
#define T_MIN   -10.0f
#define T_MAX   10.0f

typedef struct 
{
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
	float aim;
	float cur;
	int16_t pos[2];
  int16_t vel[2];
	int16_t round;
	int32_t reality;
	uint16_t initialAngle;
	float ralativeAngle;
	uint8_t ONLINE_JUDGE_TIME;

}motor_fbpara_t;

typedef struct
{
    uint16_t mode;
    uint16_t id;
    uint16_t state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float pos;
    float vel;
    float tor;
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;
    int8_t ONLINE_JUDGE_TIME;
}DM_MOTOR_DATA_Typdef;
typedef struct {
	float A;
	float B;
    float rin;
    float lastRin;
    float perrRin;
}FFC_typedef;
typedef struct
{
	uint16_t mode[2];
	motor_fbpara_t para;
	DM_MOTOR_DATA_Typdef DM_DATA;
  FFC_typedef FFC;
  PID_t PID_P;
  PID_t PID_S;
	
}DM_MOTOR_Typdef ;


typedef struct
{
    float position_des;
    float velocity_des;
    float torque_des;
    float Kp;
    float Kd;
}DM_MOTOR_MIT_Typdef;

typedef struct
{
    float POS;
    float SPE;
}DM_MOTOR_PV_Typdef;

typedef struct
{
    float SPE;
    float none_null;
}DM_MOTOR_V_Typdef;



typedef enum
{
  DM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
  DM_CMD_RESET_MODE = 0xfd,   // 停止
  DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
  DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
}DMMotor_Mode_e;
int16_t OneFilter1(int16_t last, int16_t now, float thresholdValue);
void motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id, DMMotor_Mode_e what);
void pos_speed_ctrl(hcan_t* hcan,uint16_t motor_id, DM_MOTOR_PV_Typdef *PVS);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void dm4310_current_set(hcan_t* hcan, uint16_t id, float m1_cur_set, float m2_cur_set, float m3_cur_set, float m4_cur_set);
void dm4310_RXdata(DM_MOTOR_Typdef  *motor, uint8_t *rx_data);
void dm4310_fbdata(DM_MOTOR_Typdef *motor, uint8_t *rx_data);
void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
#endif
