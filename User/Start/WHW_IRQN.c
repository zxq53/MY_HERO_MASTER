/*
 *                        _oo0oo_
 *                       o8888888o
 *                       88" . "88
 *                       (| -_- |)
 *                       0\  =  /0
 *                     ___/`---'\___
 *                   .' \\|     |// '.
 *                  / \\|||  :  |||// \
 *                 / _||||| -:- |||||- \
 *                |   | \\\  - /// |   |
 *                | \_|  ''\---/''  |_/ |
 *                \  .-\__  '-'  ___/-. /
 *              ___'. .'  /--.--\  `. .'___
 *           ."" '<  `.___\_<|>_/___.' >' "".
 *          | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *          \  \ `_.   \_ __\ /__ _/   .-` /  /
 *      =====`-.____`.___ \_____/___.-`___.-'=====
 *                        `=---='
 * 
 * 
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 *            佛祖保佑     永不宕机     永无BUG
 * 
 *        佛曰:  
 *                写字楼里写字间，写字间里程序员；  
 *                程序人员写程序，又拿程序换酒钱。  
 *                酒醒只在网上坐，酒醉还来网下眠；  
 *                酒醉酒醒日复日，网上网下年复年。  
 *                但愿老死电脑间，不愿鞠躬老板前；  
 *                奔驰宝马贵者趣，公交自行程序员。  
 *                别人笑我忒疯癫，我笑自己命太贱；  
 *                不见满街漂亮妹，哪个归得程序员？
 */

/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2024-07-04 15:42:35
 * @LastEditors: Andy
 * @LastEditTime: 2024-07-07 11:27:00
 */

#include "WHW_IRQN.h"
#include "All_Init.h"
#include "can_bsp.h"
int a;
uint8_t move;
float t1,t2,dt;
static uint8_t TX[12] = {0x3A,0x98,0xfd,0x90,0x86,0xa7,0xff,0xf1,0xfd,0x90,0x86,0xa7};
int supplykey=0;
//画UI与平时测试
void StartRobotUITask(void const *argument)
{
    portTickType currentTimeRobotUI;
    currentTimeRobotUI = xTaskGetTickCount();
	
    for(;;)
    {
		VOFA_justfloat(  Gimbal.DM4340_Pitch.PID_P.Ref,
			                 IMU_Data.roll*22.7555556f,
			                 Gimbal.DM4310_Yaw.para.aim,
			                 IMU_Data.YawTotalAngle*22.7555556f,
			                Gimbal.DM4340_Pitch.PID_P.Err,
			                Gimbal.DM4340_Pitch.PID_S.Output,
			                 Gimbal.DM4340_Pitch.PID_P.Output,
			                IMU_Data.gyro[0],
			                0,
			                0);
        vTaskDelayUntil(&currentTimeRobotUI, 1);
    }
}

//陀螺仪
void StartIMUTask(void const *argument)
{
    portTickType currentTimeIMU;
    currentTimeIMU = xTaskGetTickCount();

    for(;;)
    {
        t1 = DWT_GetTimeline_ms();
        INS_Task(&IMU_Data, &temppid);
        t2 = DWT_GetTimeline_ms();
        dt = (t2 - t1);
        vTaskDelayUntil(&currentTimeIMU, 1);
    }
}

//云台
void StartGimbalTask(void const *argument)
{
    portTickType currentTimeGimbal;
    currentTimeGimbal = xTaskGetTickCount();
    for(;;)
    {
			  motor_mode(&hfdcan1, 1, 0x00, 0xFC);
			  CANGimbalTX(FDCAN1);
			   gimbal_task(&Gimbal,&RUI_ROOT_STATUS,WHW_V_DBUS.Remote.S2_u8);
//        move = gimbal_task(&RUI_V_CONTAL, &RUI_ROOT_STATUS, &ALL_MOTOR);
			  //Attack_task(&Gimbal,&RUI_ROOT_STATUS,5,WHW_V_DBUS.Remote.S1_u8);
		
        vTaskDelayUntil(&currentTimeGimbal, 1);
    }
}

//监控
void StartMonitorTask(void const * argument)
{
    portTickType currentTimeMonitor;
    currentTimeMonitor = xTaskGetTickCount();

    for(;;) 
    { 
         Attack_task(&Gimbal,&RUI_ROOT_STATUS,5,WHW_V_DBUS.Remote.S1_u8);  			
        RUI_F_ROOT(&RUI_ROOT_STATUS, &WHW_V_DBUS, &Data_RX_0x302, &Gimbal);
        V_Input.vbus = get_vbus_input(V_Input.adc_val[0]);
        vTaskDelayUntil(&currentTimeMonitor, 5);
    }
}

//系统辨识
void StartK3debugTask(void const * argument)
{
    portTickType currentTimeK3debug;
    currentTimeK3debug = xTaskGetTickCount();

	k3debug_init(&ALL_MOTOR);
	//DM_test_init();
	
    for(;;)
    {
//		k3debug_task(&ALL_MOTOR, &WHW_V_DBUS);
//        DM_test(&IMU_Data);
        vTaskDelayUntil(&currentTimeK3debug, 1);
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef RxHeader1;
    uint8_t g_Can1RxData[64];

    FDCAN_RxHeaderTypeDef RxHeader3;
    uint8_t g_Can3RxData[64];
	
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if(hfdcan->Instance == FDCAN1)
        {
            /* Retrieve Rx messages from RX FIFO0 */
            memset(g_Can1RxData, 0, sizeof(g_Can1RxData));	//接收前先清空数组
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1RxData);
            switch(RxHeader1.Identifier)
            {
                case df_DM4310_y_ID :
								{
                   dm4310_RXdata(&Gimbal.DM4310_Yaw,g_Can1RxData);
								}
                    break;
                 case df_DM4310_p_ID:		//pitch轴电机 
			       { 
							 dm4310_fbdata(&Gimbal.DM4340_Pitch, g_Can1RxData);
				    //dm4310_RXdata(&Gimbal.DM4340_Pitch,g_Can1RxData);
			     }break;  
				       case df_DM4310_d_ID:		//供弹电机
			    {
				   dm4310_RXdata(&Gimbal.DM4310_SUPPLY ,g_Can1RxData);
				    if(supplykey==0)
				  {
					  Gimbal.DM4310_SUPPLY.para.aim= Gimbal.DM4310_SUPPLY.para.reality;
					 supplykey+=1;
				 }
            }case df_CHControlData_ID:
			{
				  GimbalRXResolve(g_Can1RxData,df_CHControlData_ID);			
			}break;
					}
        }

        if(hfdcan->Instance == FDCAN3)
        {
            /* Retrieve Rx messages from RX FIFO0 */
            memset(g_Can3RxData, 0, sizeof(g_Can3RxData));
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader3, g_Can3RxData);
            switch(RxHeader3.Identifier)
            {
				case  0x201:
								RUI_F_MOTOR_CAN_RX_3508RM(&Gimbal.DJI_3508_supply.DATA, g_Can3RxData);
					break;
            }
        }
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	FDCAN_RxHeaderTypeDef RxHeader2;
    uint8_t g_Can2RxData[64];
	
	if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
	{
		if(hfdcan->Instance == FDCAN2)
		{
			/* Retrieve Rx messages from RX FIFO0 */
			memset(g_Can2RxData, 0, sizeof(g_Can2RxData));
			HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader2, g_Can2RxData);
			switch(RxHeader2.Identifier)
			{
				case 0x201:
		
				 RUI_F_MOTOR_CAN_RX_3508RM(&Gimbal.DJI_3508_UP_Friction.DATA, g_Can2RxData);
					break;
				case 0x202:
					
				RUI_F_MOTOR_CAN_RX_3508RM(&Gimbal.DJI_3508_Right_Friction.DATA, g_Can2RxData);
					break;
				case 0x203:
				RUI_F_MOTOR_CAN_RX_3508RM(&Gimbal.DJI_3508_Left_Friction.DATA, g_Can2RxData);
					break;
			}
		}
	}
}

extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
	if(huart->Instance == UART5)
	{
		if (Size <= 18)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart5, DBUS_RX_DATA, 90); // 接收完毕后重启
			RUI_F_DUBS_Resovled(DBUS_RX_DATA, &WHW_V_DBUS);
		}
		else  // 接收数据长度大于BUFF_SIZE，错误处理
		{	
			HAL_UARTEx_ReceiveToIdle_DMA(&huart5, DBUS_RX_DATA, 90); // 接收完毕后重启
			memset(DBUS_RX_DATA, 0, 18);							   
		}
	}
	
//	if(huart->Instance == USART10)
//	{
//		       
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart10, (uint8_t *)VisionRx, 40);//上位机串口
//		Vision_Rx_Data(VisionRx, &VisionRxData);
//	}
	
	if(huart->Instance == USART1)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)selects.Data, 510);//图传串口
	}

    if(huart->Instance == UART7)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart7, (uint8_t *)VisionRx, 510);//裁判系统串口(视觉串口)
        
        uint8_t data_length_7;

        data_length_7  = 510 - __HAL_DMA_GET_COUNTER(&hdma_uart7_rx);//计算接收到的数据长度
        Read_Data_first(&ALL_RX , &User_data , data_length_7);//测试函数：待修改
        memset((uint8_t*)ALL_RX.Data,0,data_length_7);//清零接收缓冲区
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == UART5)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, DBUS_RX_DATA, 90); // 接收发生错误后重启
		memset(DBUS_RX_DATA, 0, 18);							   // 清除接收缓存		
	}
	
	if(huart->Instance == USART10)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart10, (uint8_t *)RX, 40);//上位机串口
		memset((uint8_t *)RX, 0, 20);
	}
	
	if(huart->Instance == USART1)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)selects.Data, 510);//图传串口
	}

    if(huart->Instance == UART7)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart7, (uint8_t *)ALL_RX.Data, 510);//裁判系统串口
    }
}

void USER_TIM_IRQHandler(TIM_HandleTypeDef *htim)
{
}
