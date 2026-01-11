#ifndef _CAN_BSP_H
#define _CAN_BSP_H

#include "fdcan.h"

typedef FDCAN_HandleTypeDef hcan_t;
union Speed_Data_t
{
	struct __packed
	{
		float speed;
		
	}speed_data;
	uint8_t Data[4];
};
extern float reality_Firctionspeed;
extern void FDCAN1_Config(void);
extern void FDCAN2_Config(void);
extern void FDCAN3_Config(void);
extern uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t GimbalRXResolve(uint8_t * buff,uint16_t CANID) ;
#endif

