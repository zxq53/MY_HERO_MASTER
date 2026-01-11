 #include "attack_Task.h"
#include "All_Init.h"
#include "MY_Define.h"
 int err;
 
uint8_t RUI_F_MOTOR_PID_ATTACK_INIT(MOTOR_Typdef *MOTOR)
{

  //电机一切初始化
		float PID_F_UP_Friction[3] = WHW_DF_PID_F_UP_Friction;
    float PID_P_UP_Friction[3] = WHW_DF_PID_P_UP_Friction;
    float PID_S_UP_Friction[3] = WHW_DF_PID_S_UP_Friction;
    
    PID_Init(&MOTOR->DJI_3508_UP_Friction.PID_P, 6000.0f, 2.0f,
               PID_P_UP_Friction, 0, 0,
             0, 0, 0,
             Integral_Limit);
   PID_Init(&MOTOR->DJI_3508_UP_Friction.PID_S, 20000.0f, 2.0f,
              PID_S_UP_Friction, 0, 0,
             0, 0, 0,
             Integral_Limit);
		float PID_F_Right_Friction[3] = WHW_DF_PID_F_Right_Friction;
    float PID_P_Right_Friction[3] = WHW_DF_PID_P_Right_Friction;
    float PID_S_Right_Friction[3] = WHW_DF_PID_S_Right_Friction;
		PID_Init(&MOTOR->DJI_3508_Right_Friction.PID_P, 6000.0f, 2.0f,
               PID_P_Right_Friction, 0, 0,
             0, 0, 0,
             Integral_Limit);
    PID_Init(&MOTOR->DJI_3508_Right_Friction.PID_S, 20000.0f, 2.0f,
              PID_S_Right_Friction, 0, 0,
             0, 0, 0,
             Integral_Limit); 
		float PID_F_Left_Friction[3] = WHW_DF_PID_F_Left_Friction;
    float PID_P_Left_Friction[3] = WHW_DF_PID_P_Left_Friction;
    float PID_S_Left_Friction[3] = WHW_DF_PID_S_Left_Friction ;
		PID_Init(&MOTOR->DJI_3508_Left_Friction.PID_P, 6000.0f, 2.0f,
               PID_P_Left_Friction, 0, 0,
             0, 0, 0,
             Integral_Limit);
    PID_Init(&MOTOR->DJI_3508_Left_Friction.PID_S, 20000.0f, 2.0f,
              PID_S_Left_Friction, 0, 0,
             0, 0, 0,
             Integral_Limit); 
						 
		float PID_F_Three_supply[3] = WHW_DF_PID_F_Three_supply;
    float PID_P_Three_supply[3] = WHW_DF_PID_P_Three_supply;
    float PID_S_Three_supply[3] = WHW_DF_PID_S_Three_supply;	 
	 PID_Init(&MOTOR->DM4310_SUPPLY.PID_P, 6000.0f, 2.0f,
              PID_P_Three_supply, 0, 0,
             0, 0, 0,
             Integral_Limit);
    PID_Init(&MOTOR->DM4310_SUPPLY.PID_S, 6000.0f, 2.0f,
             PID_S_Three_supply, 0, 0,
             0, 0, 0,
             Integral_Limit);
  Gimbal.DM4310_SUPPLY.para.aim= Gimbal.DM4310_SUPPLY.para.reality;
    return RUI_DF_READY;
}

uint8_t Shoot_AIM_INIT(RUI_ROOT_STATUS_Typedef *Root, MOTOR_Typdef *MOTOR)
{
    //检查离线
    if (Root->MOTOR_Shoot_L     == RUI_DF_OFFLINE ||
        Root->MOTOR_Shoot_M     == RUI_DF_OFFLINE ||
        Root->MOTOR_Shoot_R     == RUI_DF_OFFLINE)
        return RUI_DF_ERROR;

    return RUI_DF_READY;
}
uint8_t Attack_task(MOTOR_Typdef *MOTOR, RUI_ROOT_STATUS_Typedef *Root,float SupplySpeed,uint8_t mod)
{
	  static uint8_t PID_INIT = RUI_DF_ERROR;
    static uint8_t AIM_INIT = RUI_DF_ERROR;

    //电机PID赋值
    if (PID_INIT != RUI_DF_READY)
    {
        PID_INIT = RUI_F_MOTOR_PID_ATTACK_INIT(MOTOR);
        return RUI_DF_ERROR;
    }
  if (WHW_V_DBUS.Remote.S1_u8 == 1 )
	{
     SupplyContinue(1,SupplySpeed);
		
	}
	if(fire == 0)	//完成一整个开火命令（为了避免拨动拨杆速度过快导致在一个开火命令没完成前mod变为等待模式）
	   {
		   mod = 2;
     }
	switch(mod) {	
		//停止模式//S1在上
		case 1:{	
			
			FirctionControl(&Gimbal.DJI_3508_UP_Friction,	df_stopRun, 0);
			FirctionControl(&Gimbal.DJI_3508_Left_Friction,	df_stopRun, 0);
			FirctionControl(&Gimbal.DJI_3508_Right_Friction,	df_stopRun, 0);
		}break;
		//单发模式//S1在下
		case 2:{			
			CkilckFire_hore(&Gimbal);
 
		}break;
		//单发等待模式//S1在中
		case 3:{	
			fire1 = 0;	//开火准备
			
	    FirctionControl(&Gimbal.DJI_3508_UP_Friction, df_continueRun, FirctionspeedUP);
	    FirctionControl(&Gimbal.DJI_3508_Right_Friction, df_continueRun, Firctionspeedright);
	    FirctionControl(&Gimbal.DJI_3508_Left_Friction, df_continueRun, Firctionspeedleft);
			
		}break;
	}			 
		if(WHW_V_DBUS.Remote.S2_u8 == 3 ){
			FirctionControl(&Gimbal.DJI_3508_UP_Friction,	df_stopRun, 0);
			FirctionControl(&Gimbal.DJI_3508_Left_Friction ,	df_stopRun, 0);
			FirctionControl(&Gimbal.DJI_3508_Right_Friction,df_stopRun, 0);
	}		 							 
	/*摩擦轮*/				
		
	PID_Calculate(&MOTOR->DJI_3508_UP_Friction.PID_S,
                  MOTOR->DJI_3508_UP_Friction.DATA.Speed_now,
                  MOTOR->DJI_3508_UP_Friction.DATA.Aim);
	PID_Calculate(&MOTOR->DJI_3508_Right_Friction.PID_S,
                  MOTOR->DJI_3508_Right_Friction.DATA.Speed_now,
                  Gimbal.DJI_3508_Right_Friction.DATA.Aim);
	PID_Calculate(&MOTOR->DJI_3508_Left_Friction.PID_S,
                  MOTOR->DJI_3508_Left_Friction.DATA.Speed_now,
                  Gimbal.DJI_3508_Left_Friction.DATA.Aim);
	     DailMotorTwoPID(&Gimbal); 
/*fdcan发送*/									
	DJI_Power_Ctrl(&hfdcan2,
                   0x200,
                   Gimbal.DJI_3508_UP_Friction.PID_S.Output,
                   Gimbal.DJI_3508_Right_Friction.PID_S.Output,
                   Gimbal.DJI_3508_Left_Friction.PID_S.Output,
                   0);
									 
  dm4310_current_set(&hfdcan1, 0x4FE,0 ,Gimbal.DM4310_SUPPLY.PID_S.Output, 0,  Gimbal.DM4310_Yaw.PID_S.Output);								 
return 1;

}
void SupplyContinue( uint8_t mod, float SupplySpeed)
{
		
		
		if(ReadAMPin){	//如果检测到弹丸进入指定位置
			return ;
		}
		 Gimbal.DM4310_SUPPLY.para.aim-=3;
		 
}

/**
  * @author: 楠
  * @performance: 摩擦轮控制函数
  * @parameter: @摩擦轮电机结构体 @模式 @期望转速
  * @time: 23-7-9
  * @update; 加入二级摩擦轮控制 //23-8-31
  * @ReadMe: 非调试时期望转速由裁判系统得到
*/
void FirctionControl(DJI_MOTOR_Typedef *motor, uint8_t mod, int32_t Speed)
{
	
		static uint8_t sholock = 0;
	  static int16_t adjustNumber = 0;
	switch(mod)
	{
		case df_continueRun:{
			if((uint32_t)motor == (uint32_t) UP_Friction)
			{	
				motor->DATA.Aim= -Firctionspeed ;
			}
				
			if((uint32_t)motor == (uint32_t) Left_Friction)
				{motor->DATA.Aim = -Firctionspeed;}
			
				if((uint32_t)motor == (uint32_t) Right_Friction)
				{motor->DATA.Aim = -Firctionspeed;}
		}break;
		case df_stopRun:{
			motor->DATA.Aim = 0;
			motor->PID_P.Iout = 0;//aim值等于now可以使pout和dout直接清零只有iout需要手动清零
			motor->PID_S.Iout = 0;
		}break;
}
}
/**
  * @author: 楠
  * @performance: 单发慢拨弹函数
  * @parameter: NO
  * @time: 23-11-18
  * @ReadMe: 以固定速度转到固定角度
*/
void CkilckFire_hore(MOTOR_Typdef *MOTOR)
{	
	FirctionControl(UP_Friction, df_continueRun, FirctionspeedUP);
	FirctionControl(Right_Friction, df_continueRun, Firctionspeedright);
	FirctionControl(Left_Friction, df_continueRun, Firctionspeedleft);
	static float sum;
	if( fire1 == 0 )
	{
		fire = 0;	//正在开火
		
		Gimbal.DM4310_SUPPLY.para.aim-=FirelSpeed;
	
	  DailMotorTwoPID(&Gimbal); 
   dm4310_current_set(&hfdcan1, 0x4FE,0 ,Gimbal.DM4310_SUPPLY.PID_S.Output,0,  Gimbal.DM4310_Yaw.PID_S.Output);
		sum -= FirelSpeed;
		 err=sum;
		if(sum == -1350||sum < -1350)//1350
		{  
			fire = 1;	//完成开火
			sum = 0;
			fire1 = 1; //完成开火
		}
	}
}
void DailMotorTwoPID(MOTOR_Typdef *  motor) 
{
	const int32_t MAXError = 100;			//控制加速度
	
/**************************角度环处理**************************/
	motor->DM4310_SUPPLY.PID_P.Err = (motor->DM4310_SUPPLY.para.aim - motor->DM4310_SUPPLY.para.reality);

	/*比例输出*/
	motor->DM4310_SUPPLY.PID_P.Pout  = (motor->DM4310_SUPPLY.PID_P.Err * motor->DM4310_SUPPLY.PID_P.Kp);
	/*积分输出*/
	motor->DM4310_SUPPLY.PID_P.Iout += (motor->DM4310_SUPPLY.PID_P.Err * motor->DM4310_SUPPLY.PID_P.Ki);
	/*积分限幅*/
  if(motor->DM4310_SUPPLY.PID_P.Iout>50)
	{
		motor->DM4310_SUPPLY.PID_P.Iout=50;
	}else if(motor->DM4310_SUPPLY.PID_P.Iout<-50)
	{
		motor->DM4310_SUPPLY.PID_P.Iout=-50;
	}
	/*微分输出*/
	motor->DM4310_SUPPLY.PID_P.Dout = (motor->DM4310_SUPPLY.PID_P.Err - motor->DM4310_SUPPLY.PID_P.Last_Err) * motor->DM4310_SUPPLY.PID_P.Kd;
	//数据迭代
	motor->DM4310_SUPPLY.PID_P.Last_Err = motor->DM4310_SUPPLY.PID_P.Err ;

	//角度环总输出
	motor->DM4310_SUPPLY.PID_P.Output = (motor->DM4310_SUPPLY.PID_P.Pout +motor->DM4310_SUPPLY.PID_P.Iout + motor->DM4310_SUPPLY.PID_P.Dout);
	//motor->PID_P.all_out =0; 
	/*总输出限幅*/
/**************************速度环处理**************************/
	motor->DM4310_SUPPLY.PID_S.Err = motor->DM4310_SUPPLY.PID_P.Output - motor->DM4310_SUPPLY.para.vel[df_now];
	/*比例输出*/
	motor->DM4310_SUPPLY.PID_S.Pout  = (motor->DM4310_SUPPLY.PID_S.Err * motor->DM4310_SUPPLY.PID_S.Kp);
	/*积分输出*/
	motor->DM4310_SUPPLY.PID_S.Iout += (motor->DM4310_SUPPLY.PID_S.Err * motor->DM4310_SUPPLY.PID_S.Ki);
	/*积分限幅*/
	//motor->PID_S.i_out = SectionLimit_f(motor->PID_S.iLt, -motor->PID_S.iLt, motor->PID_S.i_out); 
	/*微分输出*/
	motor->DM4310_SUPPLY.PID_S.Dout = (motor->DM4310_SUPPLY.PID_S.Err - motor->DM4310_SUPPLY.PID_S.Last_Err) * motor->DM4310_SUPPLY.PID_S.Kd;
	//数据迭代
	motor->DM4310_SUPPLY.PID_S.Last_Err = motor->DM4310_SUPPLY.PID_S.Last_Err ;
	//角度环总输出
	motor->DM4310_SUPPLY.PID_S.Output = (motor->DM4310_SUPPLY.PID_S.Pout + motor->DM4310_SUPPLY.PID_S.Iout + motor->DM4310_SUPPLY.PID_S.Dout);
	/*总输出限幅*/
	if(motor->DM4310_SUPPLY.PID_S.Output>500)
	{
		motor->DM4310_SUPPLY.PID_S.Output=500;
	}else if (motor->DM4310_SUPPLY.PID_S.Output<-500)
	{
		motor->DM4310_SUPPLY.PID_S.Output=-500;
	}

}