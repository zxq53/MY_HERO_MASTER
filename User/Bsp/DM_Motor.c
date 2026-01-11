#include "DM_Motor.h"

/**
************************************************************************
* @brief:      	dm4310_fbdata: 获取DM4310电机反馈数据函数
* @param[in]:   motor:    指向motor_t结构的指针，包含电机相关信息和反馈数据
* @param[in]:   rx_data:  指向包含反馈数据的数组指针
* @param[in]:   data_len: 数据长度
* @retval:     	void
* @details:    	从接收到的数据中提取DM4310电机的反馈信息，包括电机ID、
*               状态、位置、速度、扭矩相关温度参数、寄存器数据等
************************************************************************
**/
//void dm4310_fbdata(DM_MOTOR_DATA_Typdef *motor, uint8_t *rx_data,uint32_t data_len)
//{
//  if(data_len==FDCAN_DLC_BYTES_8)
//  {
//    //返回的数据有8个字节
//    motor->id = (rx_data[0])&0x0F;
//    motor->state = (rx_data[0])>>4;
//    motor->p_int=(rx_data[1]<<8)|rx_data[2];
//    motor->v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
//    motor->t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
//    motor->pos = uint_to_float(motor->p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
//    motor->vel = uint_to_float(motor->v_int, V_MIN, V_MAX, 12); // (-30.0,30.0)
//    motor->tor = uint_to_float(motor->t_int, T_MIN, T_MAX, 12);  // (-10.0,10.0)
//    motor->Tmos = (float)(rx_data[6]);
//    motor->Tcoil = (float)(rx_data[7]);
//    motor->ONLINE_JUDGE_TIME = RUI_DF_MOTOR_OFFLINE_TIME;
//  }
//}
int16_t angleError = 0;
void dm4310_RXdata(DM_MOTOR_Typdef  *motor, uint8_t *rx_data) //一拖四模式下
{
	 
	motor->para.pos[df_last] = motor->para.pos[df_now];
	motor->para.pos[df_now]  = ((rx_data[0] << 8)|(rx_data[1]));
	int16_t spd_int16= ((rx_data[2] << 8)|(rx_data[3]));
	int16_t cur_int16 = (rx_data[4] << 8)|(rx_data[5]);
	angleError =	motor->para.pos[df_now] -motor->para.initialAngle;
	if(angleError > 4096){
			angleError -= 8192;
	}
	else if (angleError < -4096){
			angleError += 8192;
	}
	motor->para.ralativeAngle = angleError * 0.044f;
	if(( motor->para.pos[df_now] - 	motor->para.pos[df_last])<-4096)
	{
		motor->para.round++;
	}
	else if(( motor->para.pos[df_now] - 	motor->para.pos[df_last])>4096)
	{
	  motor->para.round--;
	}
	
	/*圈数清零保证不会疯转*/
	if((motor->para.round > 32000) | (motor->para.round < -32000))
	{
    motor->para.round = 0;
		motor->para.aim = motor->para.pos[df_now] ;
	}
	motor->para.vel[df_last] = motor->para.vel[df_now];
	motor->para.vel[df_now] =  spd_int16/100;
	
	motor->para.vel[df_now]= OneFilter1(motor->para.vel[df_now],motor->para.vel[df_last] , 500);
	motor->para.cur =  ((float)cur_int16);//(16384.0f/20.0f);
	motor->para.Tcoil = (float)(rx_data[6]);
	motor->para.Tmos = (float)(rx_data[7]);
  motor->para.reality = (int32_t)(( motor->para.round * 8192)+(float)(motor->para.pos[df_now]));///8192.0f*360.0f;	
	 motor->para.ONLINE_JUDGE_TIME=50;
}

//电机模式选择
void motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id, DMMotor_Mode_e what)
{
  uint8_t data[8];
  uint16_t id = motor_id + mode_id;

  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = what;

  canx_send_data(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	mit_ctrl: MIT模式下的电机控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
  uint8_t data[8];
  uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
  uint16_t id = motor_id + MIT_MODE;

  pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);
  vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);
  kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);
  kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
  tor_tmp = float_to_uint(torq, T_MIN,  T_MAX,  12);

  data[0] = (pos_tmp >> 8);
  data[1] = pos_tmp;
  data[2] = (vel_tmp >> 4);
  data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
  data[4] = kp_tmp;
  data[5] = (kd_tmp >> 4);
  data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
  data[7] = tor_tmp;

  canx_send_data(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	pos_speed_ctrl: 位置速度控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   vel:			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
void pos_speed_ctrl(hcan_t* hcan,uint16_t motor_id, DM_MOTOR_PV_Typdef *PVS)
{
  uint16_t id;
  uint8_t *pbuf, *vbuf;
  uint8_t data[8];

  id = motor_id + POS_MODE;
  pbuf=(uint8_t*)&PVS->POS;
  vbuf=(uint8_t*)&PVS->SPE;

  data[0] = *pbuf;
  data[1] = *(pbuf+1);
  data[2] = *(pbuf+2);
  data[3] = *(pbuf+3);

  data[4] = *vbuf;
  data[5] = *(vbuf+1);
  data[6] = *(vbuf+2);
  data[7] = *(vbuf+3);

  canx_send_data(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	speed_ctrl: 速度控制函数
* @param[in]:   hcan: 		指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   vel: 			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送速度控制命令
************************************************************************
**/
void speed_ctrl(hcan_t* hcan,uint16_t motor_id, DM_MOTOR_V_Typdef *vel)
{
  uint16_t id;
  uint8_t *vbuf;
  uint8_t data[4];

  id = motor_id + SPEED_MODE;
  vbuf=(uint8_t*)&vel->SPE;

  data[0] = *vbuf;
  data[1] = *(vbuf+1);
  data[2] = *(vbuf+2);
  data[3] = *(vbuf+3);

  canx_send_data(hcan, id, data, 4);
}
void dm4310_current_set(hcan_t* hcan, uint16_t id, float m1_cur_set, float m2_cur_set, float m3_cur_set, float m4_cur_set)
{
	uint8_t data[8];
	
	int16_t m1_cur_tmp = m1_cur_set; 
	int16_t m2_cur_tmp = m2_cur_set; 
	int16_t m3_cur_tmp = m3_cur_set; 
	int16_t m4_cur_tmp = m4_cur_set;
	
	data[0] = (m1_cur_tmp >> 8);
	data[1] =  m1_cur_tmp;
	
	data[2] = (m2_cur_tmp >> 8);
	data[3] =  m2_cur_tmp;
	
	data[4] = (m3_cur_tmp >> 8);
	data[5] =  m3_cur_tmp;
	
	data[6] = (m4_cur_tmp >> 8);
	data[7] =  m4_cur_tmp;
	
	canx_send_data(hcan, id, data, 8);
}



int16_t OneFilter1(int16_t last, int16_t now, float thresholdValue)
{
		//减小平滑滤波值会增大对于细小毛刺的过滤程度
		//增加尖峰滤波值会增大对于尖峰数值的响应程度
	const float sensitivlFilter = 0.8f;	//尖峰滤波值//小于1//一般为最大值的20%
	const float numbFilter = 0.8f;	//平滑滤波值//小于1
	
	if((last - now)>= thresholdValue){
			return (float)( now*sensitivlFilter + last*(1-sensitivlFilter) );
	}
	else{
			return (float)( now*numbFilter + last*(1-numbFilter) );
	}
}
/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //电机清空函数
 *	@parameter:		    //
 *	@time:				//23-04-13 19:23
 *	@ReadMe:			//
 *  @LastUpDataTime:    //2023-05-07 17:06    bestrui
 *  @UpData：           //不太好描述
 ************************************************************万能分隔符**************************************************************/


/**
************************************************************************
* @brief:      	dm4310_fbdata: 获取DM4310电机反馈数据函数
* @param[in]:   motor:    指向motor_t结构的指针，包含电机相关信息和反馈数据
* @param[in]:   rx_data:  指向包含反馈数据的数组指针
* @param[in]:   data_len: 数据长度
* @retval:     	void
* @details:    	从接收到的数据中提取DM4310电机的反馈信息，包括电机ID、
*               状态、位置、速度、扭矩相关温度参数、寄存器数据等
************************************************************************
**/
void dm4310_fbdata(DM_MOTOR_Typdef *motor, uint8_t *rx_data)
{
    //返回的数据有8个字节
    motor->DM_DATA.id = (rx_data[0])&0x0F;
    motor->DM_DATA.state = (rx_data[0])>>4;
    motor->DM_DATA.p_int=(rx_data[1]<<8)|rx_data[2];
    motor->DM_DATA.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
    motor->DM_DATA.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
    motor->DM_DATA.pos = uint_to_float(motor->DM_DATA.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
    motor->DM_DATA.vel = uint_to_float(motor->DM_DATA.v_int, V_MIN, V_MAX, 12); // (-30.0,30.0)
    motor->DM_DATA.tor = uint_to_float(motor->DM_DATA.t_int, T_MIN, T_MAX, 12);  // (-10.0,10.0)
    motor->DM_DATA.Tmos = (float)(rx_data[6]);
    motor->DM_DATA.Tcoil = (float)(rx_data[7]);
    motor->DM_DATA.ONLINE_JUDGE_TIME = RUI_DF_MOTOR_OFFLINE_TIME;
  
}
