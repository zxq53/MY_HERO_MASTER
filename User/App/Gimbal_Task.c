/*
 *                                                     __----~~~~~~~~~~~------___
 *                                    .  .   ~~//====......          __--~ ~~
 *                    -.            \_|//     |||\\  ~~~~~~::::... /~
 *                 ___-==_       _-~o~  \/    |||  \\            _/~~-
 *         __---~~~.==~||\=_    -_--~/_-~|-   |\\   \\        _/~
 *     _-~~     .=~    |  \\-_    '-~7  /-   /  ||    \      /
 *   .~       .~       |   \\ -_    /  /-   /   ||      \   /
 *  /  ____  /         |     \\ ~-_/  /|- _/   .||       \ /
 *  |~~    ~~|--~~~~--_ \     ~==-/   | \~--===~~        .\
 *           '         ~-|      /|    |-~\~~       __--~~
 *                       |-~~-_/ |    |   ~\_   _-~            /\
 *                            /  \     \__   \/~                \__
 *                        _--~ _/ | .-~~____--~-/                  ~~==.
 *                       ((->/~   '.|||' -_|    ~~-/ ,              . _||
 *                                  -_     ~\      ~~---l__i__i__i--~~_/
 *                                  _-~-__   ~)  \--______________--~~
 *                                //.-~~~-~_--~- |-------~~~~~~~~
 *                                       //.-~~~--\
 *                       ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 *                               神兽保佑            永无BUG
 */

#include "Gimbal_Task.h"
#include "All_Init.h"

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //头部PID+前馈总初始化函数
 *	@parameter:		    //
 *	@time:				//23-04-13 12:42
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
uint8_t RUI_F_MOTOR_PID_HEAD_INIT(MOTOR_Typdef *MOTOR)
{
    //电机一切初始化
    float PID_P_Pitch[3] = WHW_DF_PID_P_Pitch;
    float PID_S_Pitch[3] = WHW_DF_PID_S_Pitch;
	
    PID_Init(&MOTOR->DM4340_Pitch.PID_P, 1000.0f, 50.0f,
              PID_P_Pitch, 0, 0,
             0, 0, 0,
             Integral_Limit);
    PID_Init(&MOTOR->DM4340_Pitch.PID_S, 1000.0f, 50.0f,
             PID_S_Pitch, 0, 0,
             0, 0, 0,
             Integral_Limit);

	
	
    float PID_P_YAW[3] = WHW_DF_PID_P_YAW;
    float PID_S_YAW[3] = WHW_DF_PID_S_YAW;
    PID_Init(&MOTOR->DM4310_Yaw.PID_P, 1000.0f, 20.0f,
             PID_P_YAW, 0, 0,
             0, 0, 0,
             Integral_Limit);
    PID_Init(&MOTOR->DM4310_Yaw.PID_S, 1000.0f, 20.0f,
              PID_S_YAW, 0, 0,
             0, 0, 0,
             Integral_Limit);
    return RUI_DF_READY;
}

uint8_t RUI_F_HEAD_AIM_INIT(RUI_ROOT_STATUS_Typedef *Root, MOTOR_Typdef *MOTOR)
{
//    //检查离线
//    if (Root->DM4340_Pitch     == RUI_DF_OFFLINE ||
//        Root->DM4310_Yaw     == RUI_DF_OFFLINE)
//        return RUI_DF_ERROR;
   

    return RUI_DF_READY;
}

uint8_t gimbal_task(MOTOR_Typdef *MOTOR,RUI_ROOT_STATUS_Typedef *Root,uint8_t MOD)
{
    static uint8_t PID_INIT=RUI_DF_ERROR;
    static uint8_t AIM_INIT = RUI_DF_ERROR;
	

    /*电机PID赋值*/
    if (PID_INIT != RUI_DF_READY)
    {
		PID_INIT = RUI_F_MOTOR_PID_HEAD_INIT(MOTOR);
		return RUI_DF_ERROR;
    }

    
//		if(RUI_ROOT_STATUS.DM4310_Yaw==0&&RUI_ROOT_STATUS.DM4340_Pitch==0)
//		{
//			return 0;
//		}else{
//		  GIMBAL_init();}

   
//      if(RUI_ROOT_STATUS.RM_DBUS==0)
//    {
//        MOTOR->DM4340_Pitch.PID_P.Ref = IMU_Data.roll*22.7555556f;
//       MOTOR->DM4310_Yaw.para.aim =IMU_Data.YawTotalAngle*22.7555556f ;
//		  	Gimbal.DM4340_Pitch.PID_S.Output=0;
////			  Gimbal.DM4310_Yaw.PID_S.Output=0;
//		}else
//	    {  
		  p_limit();
			switch (MOD)
		{
			/************* 遥控模式 *************/
			case GMMODRemote:{	
					
					//pitch//yaw轴控制
				MOTOR->DM4310_Yaw.para.aim += -WHW_V_DBUS.Remote.CH2_int16*0.004;
				MOTOR->DM4340_Pitch.PID_P.Ref +=  WHW_V_DBUS.Remote.CH3_int16*0.003;
			}break;
			/************* 停止模式 *************/
			case GMMODStop:{			
					
					//pitch//yaw轴控制
				MOTOR->DM4310_Yaw.para.aim = IMU_Data.YawTotalAngle*22.7555556f;
        MOTOR->DM4340_Pitch.PID_P.Ref= IMU_Data.roll*22.7555556f;
					
				
			}break;
			/************* 视觉模式 *************/ 
			case GMMODVision:{		
					//pitch//yaw轴控制
					MOTOR->DM4310_Yaw.para.aim = IMU_Data.YawTotalAngle*22.7555556f;
          MOTOR->DM4340_Pitch.PID_P.Ref = IMU_Data.roll*22.7555556f;

		}break;
			/************* 错误处理 *************/
			default :
			{
			MOTOR->DM4310_Yaw.para.aim = IMU_Data.YawTotalAngle*22.7555556f;
      MOTOR->DM4340_Pitch.PID_P.Ref= IMU_Data.roll*22.7555556f;
			}
		}
			
	//}

    /*Pitch*/
	   DMPitchPID(&Gimbal , IMU_Data.gyro[0], IMU_Data.roll);
    /*Yaw*/  
	   DM_YAWPID(&Gimbal,  IMU_Data.gyro[2], IMU_Data.YawTotalAngle);
//     PID_Calculate(&MOTOR->DM4310_Yaw.PID_P,
//                  (float) IMU_Data.YawTotalAngle*22.7555556f,
//                  MOTOR->DM4310_Yaw.para.aim);
//    PID_Calculate(&MOTOR->DM4310_Yaw.PID_S,
//                  IMU_Data.gyro[2],
//                  MOTOR->DM4310_Yaw.PID_P.Output);

    /*FDCAN发送*/
      dm4310_current_set(&hfdcan1, 0x4FE,0 ,Gimbal.DM4310_SUPPLY.PID_S.Output, 0,  Gimbal.DM4310_Yaw.PID_S.Output);
      mit_ctrl(&hfdcan1, 0x01, 0, 0,0, 0, Gimbal.DM4340_Pitch.PID_S.Output);
    return RUI_DF_READY;
}

uint8_t DMPitchPID(MOTOR_Typdef * MOTOR,  float TopPitchSpeed, float topAngle)
{
	//TopPitchSpeed *= 100;    //陀螺仪角速度放大
	topAngle *= 22.7555556f;
	
	/**************************角度环处理**************************/
	MOTOR->DM4340_Pitch.PID_P.Err = (MOTOR->DM4340_Pitch.PID_P.Ref- topAngle);
       
	/**************************角度环处理**************************/
	/*比例输出*/
	MOTOR->DM4340_Pitch.PID_P.Pout  = (MOTOR->DM4340_Pitch.PID_P.Err * MOTOR->DM4340_Pitch.PID_P.Kp);
	/*积分输出*/
	MOTOR->DM4340_Pitch.PID_P.Iout += (MOTOR->DM4340_Pitch.PID_P.Err * MOTOR->DM4340_Pitch.PID_P.Ki);
	/*积分限幅*/
if(MOTOR->DM4340_Pitch.PID_P.Iout>=1)
{
	MOTOR->DM4340_Pitch.PID_P.Iout=1;
}else if(MOTOR->DM4340_Pitch.PID_P.Iout<=-1)
{
	MOTOR->DM4340_Pitch.PID_P.Iout=-1;
}
	/*微分输出*/
	MOTOR->DM4340_Pitch.PID_P.Dout = (MOTOR->DM4340_Pitch.PID_P.Err - MOTOR->DM4340_Pitch.PID_P.Last_Err) * MOTOR->DM4340_Pitch.PID_P.Kd;
	//数据迭代
	MOTOR->DM4340_Pitch.PID_P.Last_Err = MOTOR->DM4340_Pitch.PID_P.Err ;
	
//	//角度环总输出
  MOTOR->DM4340_Pitch.PID_P.Output = (MOTOR->DM4340_Pitch.PID_P.Pout + MOTOR->DM4340_Pitch.PID_P.Iout + MOTOR->DM4340_Pitch.PID_P.Dout  );
	//角度环总输出
	/*总输出限幅*/
//	/*总输出限幅*/
	//motor->PID_P.all_out = SectionLimit_f(motor->PID_P.alLt, -motor->PID_P.alLt, motor->PID_P.all_out);
	
	/**************************速度环处理**************************/

	MOTOR->DM4340_Pitch.PID_S.Err = MOTOR->DM4340_Pitch.PID_P.Output - TopPitchSpeed;
	//*英雄开启*//
	
	/*比例输出*/
	MOTOR->DM4340_Pitch.PID_S.Pout  = (MOTOR->DM4340_Pitch.PID_S.Err * MOTOR->DM4340_Pitch.PID_S.Kp);
	/*积分输出*/
	MOTOR->DM4340_Pitch.PID_S.Iout += (MOTOR->DM4340_Pitch.PID_S.Err *MOTOR->DM4340_Pitch.PID_S.Ki);
	/*积分限幅*/
	//motor->PID_S.i_out = SectionLimit_f(motor->PID_S.iLt, -motor->PID_S.iLt, motor->PID_S.i_out); 
	/*微分输出*/
	MOTOR->DM4340_Pitch.PID_S.Dout = (MOTOR->DM4340_Pitch.PID_S.Err - MOTOR->DM4340_Pitch.PID_S.Last_Err) * MOTOR->DM4340_Pitch.PID_S.Kd;
	//数据迭代
	MOTOR->DM4340_Pitch.PID_S.Last_Err = MOTOR->DM4340_Pitch.PID_S.Err ;
	//重力补偿
	//forward_out = (1.664*cosf((INS.Pitch + 19.66f-14.7f) *0.0174533)*k) ;
	//速度环总输出
//	motor->PID_S.all_out = (motor->PID_S.p_out + motor->PID_S.i_out);// + motor->PID_S.d_out);
	MOTOR->DM4340_Pitch.PID_S.Output = (MOTOR->DM4340_Pitch.PID_S.Pout + MOTOR->DM4340_Pitch.PID_S.Iout + MOTOR->DM4340_Pitch.PID_S.Dout )/20;//+ 	FeedforwardController(& m_dm4310_p_t));
	/*总输出限幅*/
	if(MOTOR->DM4340_Pitch.PID_S.Output>=5)
	{MOTOR->DM4340_Pitch.PID_S.Output=5;
	}else if(MOTOR->DM4340_Pitch.PID_S.Output<=-5)
	MOTOR->DM4340_Pitch.PID_S.Output =-5;
	
	return 1;
}
uint8_t DM_YAWPID(MOTOR_Typdef * MOTOR,  float TopPitchSpeed, float topAngle)
{
	TopPitchSpeed *= 100;    //陀螺仪角速度放大
	topAngle *= 22.7555556f;
	
	/**************************角度环处理**************************/
	MOTOR->DM4310_Yaw.PID_P.Err = (MOTOR->DM4310_Yaw.para.aim- topAngle);
       
	/**************************角度环处理**************************/
	/*比例输出*/
	MOTOR->DM4310_Yaw.PID_P.Pout  = (MOTOR->DM4310_Yaw.PID_P.Err * MOTOR->DM4310_Yaw.PID_P.Kp);
	/*积分输出*/
	MOTOR->DM4310_Yaw.PID_P.Iout += (MOTOR->DM4310_Yaw.PID_P.Err * MOTOR->DM4310_Yaw.PID_P.Ki);
	/*积分限幅*/
if(MOTOR->DM4310_Yaw.PID_P.Iout>=100)
{
	MOTOR->DM4310_Yaw.PID_P.Iout=100;
}else if(MOTOR->DM4310_Yaw.PID_P.Iout<=-100)
{
	MOTOR->DM4310_Yaw.PID_P.Iout=-100;
}
	/*微分输出*/
	MOTOR->DM4310_Yaw.PID_P.Dout = (MOTOR->DM4310_Yaw.PID_P.Err - MOTOR->DM4310_Yaw.PID_P.Last_Err) * MOTOR->DM4310_Yaw.PID_P.Kd;
	//数据迭代
	MOTOR->DM4310_Yaw.PID_P.Last_Err = MOTOR->DM4310_Yaw.PID_P.Err ;
	
//	//角度环总输出
  MOTOR->DM4310_Yaw.PID_P.Output = (MOTOR->DM4310_Yaw.PID_P.Pout + MOTOR->DM4310_Yaw.PID_P.Iout + MOTOR->DM4310_Yaw.PID_P.Dout  );
	//角度环总输出
	/*总输出限幅*/
//	/*总输出限幅*/
	//motor->PID_P.all_out = SectionLimit_f(motor->PID_P.alLt, -motor->PID_P.alLt, motor->PID_P.all_out);
	
	/**************************速度环处理**************************/

	MOTOR->DM4310_Yaw.PID_S.Err = MOTOR->DM4310_Yaw.PID_P.Output - TopPitchSpeed;
	//*英雄开启*//
	
	/*比例输出*/
	MOTOR->DM4310_Yaw.PID_S.Pout  = (MOTOR->DM4310_Yaw.PID_S.Err * MOTOR->DM4310_Yaw.PID_S.Kp);
	/*积分输出*/
	MOTOR->DM4310_Yaw.PID_S.Iout += (MOTOR->DM4310_Yaw.PID_S.Err *MOTOR->DM4310_Yaw.PID_S.Ki);
	/*积分限幅*/
	//motor->PID_S.i_out = SectionLimit_f(motor->PID_S.iLt, -motor->PID_S.iLt, motor->PID_S.i_out); 
	/*微分输出*/
	MOTOR->DM4310_Yaw.PID_S.Dout = (MOTOR->DM4310_Yaw.PID_S.Err - MOTOR->DM4310_Yaw.PID_S.Last_Err) * MOTOR->DM4310_Yaw.PID_S.Kd;
	//数据迭代
	MOTOR->DM4310_Yaw.PID_S.Last_Err = MOTOR->DM4310_Yaw.PID_S.Err ;
	//重力补偿
	//forward_out = (1.664*cosf((INS.Pitch + 19.66f-14.7f) *0.0174533)*k) ;
	//速度环总输出
//	motor->PID_S.all_out = (motor->PID_S.p_out + motor->PID_S.i_out);// + motor->PID_S.d_out);
	MOTOR->DM4310_Yaw.PID_S.Output = (MOTOR->DM4310_Yaw.PID_S.Pout + MOTOR->DM4310_Yaw.PID_S.Iout + MOTOR->DM4310_Yaw.PID_S.Dout )/20;//+ 	FeedforwardController(& m_dm4310_p_t));
	/*总输出限幅*/
//	if(MOTOR->DM4310_Yaw.PID_S.Output>=30)
//	{MOTOR->DM4310_Yaw.PID_S.Output=30;
//	}else if(MOTOR->DM4310_Yaw.PID_S.Output<=-30)
//	MOTOR->DM4310_Yaw.PID_S.Output =-30;
	
	return 1;
}
void p_limit()
{
if(Gimbal.DM4340_Pitch.PID_P.Ref>	PitchMax)
	{
	     Gimbal.DM4340_Pitch.PID_P.Ref=	PitchMax;
		if(IMU_Data.roll*22.7555556f>=PitchMax)
		{Gimbal.DM4340_Pitch.PID_S.Iout=0;
			Gimbal.DM4340_Pitch.PID_S.Output=0;
		}
	}
	  if(Gimbal.DM4340_Pitch.PID_P.Ref<PitchMin)
		{
			 Gimbal.DM4340_Pitch.PID_P.Ref=PitchMin 	; 
			
			if(IMU_Data.roll*22.7555556f<=PitchMin)
		{Gimbal.DM4340_Pitch.PID_S.Iout=0;
			Gimbal.DM4340_Pitch.PID_S.Output=0;
		}
		
   }
 }
void GIMBAL_init()
{
	Gimbal.DM4340_Pitch.PID_P.Ref=IMU_Data.roll*22.7555556f;
	Gimbal.DM4310_Yaw.para.aim=IMU_Data.YawTotalAngle*22.7555556f;
}