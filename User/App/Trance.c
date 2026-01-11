#include "Trance.h"


uint8_t GimbalTXResovle() 
{
		CanCommunit_t.gmTOch.dataNeaten.vx = WHW_V_DBUS.Remote.CH0_int16 ;
		//CanCommunit_t.gmTOch.dataNeaten.vx += gimbal_t.Keyboard.vx;
		CanCommunit_t.gmTOch.dataNeaten.vy =  WHW_V_DBUS.Remote.CH1_int16;
		//CanCommunit_t.gmTOch.dataNeaten.vy += gimbal_t.Keyboard.vy;
		CanCommunit_t.gmTOch.dataNeaten.vr =  WHW_V_DBUS.Remote.Dial_int16;
		//CanCommunit_t.gmTOch.dataNeaten.vr += gimbal_t.Keyboard.vr;
		CanCommunit_t.gmTOch.dataNeaten.s1 = WHW_V_DBUS.Remote.S1_u8;
	  CanCommunit_t.gmTOch.dataNeaten.s2 = WHW_V_DBUS.Remote.S2_u8;
	
		
//		IMU_Data.angle.pitch = INS.Pitch;
//		IMU_Data.angle.yaw = INS.YawTotalAngle;
		
		return 1;
}



uint8_t CANGimbalTX(FDCAN_GlobalTypeDef * canx)
{	
	//先将要发送的数据解算出来	
	GimbalTXResovle();
	ppp++;	
	//数据发送部分
	if(ppp%2 == 0)
	{
		canx_send_data(&hfdcan1, df_GMControl_ID, CanCommunit_t.gmTOch.sendData,8);		//数据发送
	}
	
	if(ppp%2 == 1)
	{
		//canx_send_data(&hfdcan1, df_GMIMU_ID, IMU_Data.Data,8);
		                                      
	}
	
	

	return 1;
}
