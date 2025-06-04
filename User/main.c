/******************** (C) COPYRIGHT (2015)BST BALANCECAR **************************
 * �ļ���  ��main.c
**********************************************************************************/
//#include "stm32f10x.h"
#include "mpu6050.h"
#include "i2c_mpu6050.h"
#include "motor.h"
#include "upstandingcar.h"
#include "SysTick.h"
#include "led.h"
#include "adc.h"
#include "usart.h"
#include "i2c.h"
//#include "outputdata.h"
#include "timer.h"
#include "UltrasonicWave.h"
//#include "stm32f10x_usart.h"
#include "line_follow.h" // 加入循跡模組

float gyz;
int acc;
int acc1;

/*Э�����*/
//extern u8 newLineReceived = 0;

/*
 * ��������main
 * ����  ��������
 */
int main(void)
{	
       
	
	SystemInit();                   //=====系統初始化
	Timerx_Init(5000,7199);				   //定時器TIM1
	UltrasonicWave_Configuration(); 	   //超音波模組初始化，IO中斷配置			    

	USART1_Config(); // 串口1初始化，藍芽模組
	USART3_Config(); // 串口3初始化，與其他裝置共用IO

	TIM2_PWM_Init(); // PWM初始化
	MOTOR_GPIO_Config(); // 馬達IO初始化
	LED_GPIO_Config();
	Adc_Init();
	//TIM3_External_Clock_CountingMode(); // 外部時鐘模式，PA7用於TIM3
	//TIM4_External_Clock_CountingMode(); // 外部時鐘模式，PB7用於TIM4
	TIM3_Encoder_Init(); // 編碼器讀取，PA6/7
	TIM4_Encoder_Init(); // 編碼器讀取，PB6/7
	////////////////////DMP/////////////////////////////////
	i2cInit(); // IIC初始化，連接感測器
	delay_nms(10); // 延遲10ms
	MPU6050_Init(); // MPU6050 DMP初始化

	SysTick_Init(); // SysTick定時器初始化
	CarUpstandInit(); // 平衡車初始化
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk; // 啟用演算法定時器
	LineFollow_Init(); // 初始化循跡感測器

	while (1)
	{

// 		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		MPU6050_Pose(); // 取得MPU6050角度狀態
		//gy0=gyro[0];
		//UltrasonicWave_StartMeasure(); // 啟動超音波測距，Trig腳<10us高電位
		//chaoshengbo(); // 計算超音波距離
		//printf("%d",ucBluetoothValue);
		//printf("\t");
		//printf("%f",BST_fSpeedControlOutNew);
		//printf("\t");
		//printf("%f",BST_fCarAngle);
		//printf("\t");
		//printf("%f",BST_fLeftMotorOut);
		//printf("\t");
		//printf("\n");
		if (newLineReceived)
		{
			ProtocolCpyData();
			Protocol();
		}
		/* 狀態輸出到藍芽APP */
		CarStateOut();
		SendAutoUp();
		LineFollow_Task(); // 循跡主任務，需定期呼叫
		
	 }
 								    
}
