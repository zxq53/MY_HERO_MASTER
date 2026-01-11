/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2024-07-04 16:39:33
 * @LastEditors: Andy
 * @LastEditTime: 2024-07-05 15:48:31
 */
#ifndef __MY_DEFINE
#define __MY_DEFINE

//INIT成功
#define RUI_DF_READY 1
//INIT错误
#define RUI_DF_ERROR 0

#define RUI_DF_NOW  1
#define RUI_DF_LAST 0

//PID双环
#define RUI_DF_PID_DOUBLE 1
//PID单环
#define RUI_DF_PID_SINGLE 0
//正在判断主控位置
#define RUI_DF_MASTER_LOCATION_JUDGEING 0
//主控位置判断完成，在头部
#define RUI_DF_MASTER_LOCATION_HEAD 1
//主控位置判断完成，在底盘
#define RUI_DF_MASTER_LOCATION_BOTTOM 2

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	//LED
 *	@parameter:		//
 *	@time:				//23-04-13 15:57
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
#define RUI_DF_LED_RED    0
#define RUI_DF_LED_GREEN  1
#define RUI_DF_LED_BLUE   2
#define RUI_DF_LED_YELLOW 3
#define RUI_DF_LED_PURPLE 4
#define RUI_DF_LED_CYAN   5
#define RUI_DF_LED_WRITE  6



/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //离线检测宏定义
 *	@parameter:		    //
 *	@time:				//23-03-28 20:13
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
//设备离线
#define RUI_DF_OFFLINE 0
//设备在线
#define RUI_DF_ONLINE  1 
//遥控器离线检测时间
#define RUI_DF_DBUS_OFFLINE_TIME 30  
//电机离线检测时间
#define RUI_DF_MOTOR_OFFLINE_TIME 15
//自定义控制器离线检测时间
#define DF_0x302_OFFLINE_TIME 60
#define PitchInitialAngle 0
#define YawInitialAngle 0

#define WHW_DF_PID_F_Pitch  {   1.0f,   0.0f,   0.0f   }
#define WHW_DF_PID_P_Pitch  {   0.02500f,   0.000300f,   0.0001f   }
#define WHW_DF_PID_S_Pitch  {   25.0f,   0.0f,   0.0f   }

#define WHW_DF_PID_F_Roll  {   1.0f,   0.0f,   0.0f   }
#define WHW_DF_PID_P_Roll  {   1.0f,   0.0f,   0.0f   }
#define WHW_DF_PID_S_Roll  {   1.0f,   0.0f,   0.0f   }

#define WHW_DF_PID_F_YAW  {   1.0f,   0.0f,   0.0f   }
#define WHW_DF_PID_P_YAW  {   0.8f,   0.0010f,   0.0f   }
#define WHW_DF_PID_S_YAW  {   5.0f,   0.0f,   0.0f   }

//供弹（三）
#define WHW_DF_PID_F_UP_Friction  {   1.0f,   0.001f,   0.0f   }
#define WHW_DF_PID_P_UP_Friction {   7.7f,   0.004f,   0.0f   }
#define WHW_DF_PID_S_UP_Friction  {   15,				0.8f,		1,   }

#define WHW_DF_PID_F_Right_Friction  {   1.0f,   0.001f,   0.0f   }
#define WHW_DF_PID_P_Right_Friction {   7.7f,   0.002f,   0.0f   }
#define WHW_DF_PID_S_Right_Friction  {  15,				0.8f,		1, }

#define WHW_DF_PID_F_Left_Friction  {   1.0f,   0.0f,   0.0f   }
#define WHW_DF_PID_P_Left_Friction {   5.7f,   0.002f,   0.0f   }
#define WHW_DF_PID_S_Left_Friction  {   15,				0.8f,		1,   }//29,0.005 31


#define WHW_DF_PID_F_Three_supply  {   1.0f,   0.0f,   0.0f   }
#define WHW_DF_PID_P_Three_supply {   0.130f,	0.00005f,			0.0001f,  }
#define WHW_DF_PID_S_Three_supply {   0.34f,   0.0f,   0.0f   }
#define PitchMax 647
	#define PitchMin -278
#define TEMPERATURE_Fuzzy 	{   500.0f,    0.02f,  5.0f    }
#define TEMPERATURE_PID 	{   80.0f,    0.0f,  0.0f    }
#define TEMPERATURE_PID_MAX_OUT 4500.0f  //max out of temperature control PID
#define TEMPERATURE_PID_MAX_IOUT 100.0f //max iout of temperature control PID
/************************************************************CAN地址**************************************************************/
#define df_CHControlData_ID 0x232	//各种标志位以及状态
#ifndef df_ID
	#define df_ID
	//发射机构
	#define df_RightFriction_ID 0x202    //CAN2
	#define df_LeftFriction_ID 0x201	 //CAN2
	#define df_RightFriction2_ID 0x204	 //CAN2
	#define df_LeftFriction2_ID 0x203	 //CAN2
	#define df_GiveAmmunitDown_ID 0x204	 //CAN1
	//两个云台电机
	//两个云台电机
	#define df_DM4310_p_ID 0x11	 //CAN1
	#define df_DM4310_y_ID 0x308	 //CAN1
#define df_DM4310_d_ID 0x306	 //CAN1
	//云台主控发送的数据
	#define df_GMControl_ID 0x221		//云台主控板
	#define df_GMIMU_ID 0x222		//陀螺仪数据
	#define df_CHControlAngle_ID 0x231		//底盘主控板//该ID发送两个角度
	#define df_CHControlData_ID 0x232	//各种标志位以及状态
	//超级电容
	#define df_SuperCap_ID 0x307 
	#define  df_2006Lens_ID	  0x206
	#define  df_2006Image_ID	  0x205
	//读取微动开关的电平
	#define ReadAMPin HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)		//读取微动开关的电平
	/************************************************************发射机构**************************************************************/
	#ifndef df_attack
	#define df_stopRun 0      //停止
	#define df_continueRun 1  //供弹
	#define df_clickFire 2    //单发
	#define df_pressFire 3    //多发
	#define df_backRun 4      //倒弹
#endif
#define Firctionspeed2  -5020//-5000 //er级摩擦轮转速   4000//7000//-4950
#define Firctionspeed  4760//-5600//yi级摩擦轮转速5200 5300//4695 4460 4560
#define SeconedSpeed 3       //二次拨弹速度
#define SecondTime 150		 //二次拨弹时间
#define ShoAdjust 50		//每次调节摩擦轮转速的阈值
#define  FirctionspeedUP 0     //上摩擦轮
#define  Firctionspeedright 0  //右摩擦轮
#define  Firctionspeedleft 0  //左摩擦轮
#ifndef	FeedAmmoAngle
	#define	FeedAmmoAngle
	#define FirelSpeed 50//单发拨弹速度
	#define ClickAngle -1350//-4000//(360/6 * 19)		//六齿拨弹机构//减少供弹时间   1300//白色打弹丸  -1000 1:19
	#define MoreNumber 1	//连发子弹数//键鼠操作时会先执行单发程序，所以实际连发数量是 1+MoreNumber
	#define BackAngle 3000	//卡弹后倒退的角度   850
#endif
	#define LeftFriction &Gimbal.DJI_3508_LeftFriction
	#define RightFriction &Gimbal.DJI_3508_RightFriction
	#define LeftFriction2 &Gimbal.DJI_3508_LeftFriction2
	#define RightFriction2 &Gimbal.DJI_3508_RightFriction2
	
	#define Right_Friction &Gimbal.DJI_3508_Right_Friction
	#define UP_Friction &Gimbal.DJI_3508_UP_Friction
	#define Left_Friction &Gimbal.DJI_3508_Left_Friction
#endif
#endif
#ifndef df_now
	#define df_now 0
	#define df_last 1
	#define df_llast 2
#endif
#ifndef GMMOD
	#define GMMOD
	#define GMMODVision 1
	#define GMMODStop 3
	#define GMMODRemote 2
	#define GMMODNone 0
	#define GMMODJuji 4
	
	#define CarKeyboard 1		//与遥控值挂钩
#endif