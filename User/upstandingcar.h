#ifndef __UPSTANDINGCAR_H
#define __UPSTANDINGCAR_H
#include "stm32f10x.h"

#define CLI()      __set_PRIMASK(1)  
#define SEI()      __set_PRIMASK(0) 


/********** 角度控制相關定義 **********/									
#define    CAR_ZERO_ANGLE (0)         // 小車結構初始角度，車體直立時的角度基準值，可根據實際調整

/****** 速度控制相關定義 ******/
#define CAR_POSITION_SET      0
#define CAR_SPEED_SET         0
#define MOTOR_LEFT_SPEED_POSITIVE  (BST_fLeftMotorOut >0)
#define MOTOR_RIGHT_SPEED_POSITIVE (BST_fRightMotorOut>0)
#define OPTICAL_ENCODE_CONSTANT  13 // 光編碼器常數
#define SPEED_CONTROL_PERIOD  40    // 速度控制週期（ms）
#define CAR_SPEED_CONSTANT    (1000.0/(float)SPEED_CONTROL_PERIOD/(float)OPTICAL_ENCODE_CONSTANT)
// #define CAR_POSITION_MAX    (MOTOR_OUT_MAX*10)//500////20
// #define CAR_POSITION_MIN    (MOTOR_OUT_MIN*10) //-500//

#define CAR_POSITION_MAX  3000       // 位置最大值
#define CAR_POSITION_MIN  (-3000)    // 位置最小值
/****** 馬達輸出相關定義 ******/
#define MOTOR_OUT_DEAD_VAL       0   // 死區值0
#define MOTOR_OUT_MAX           3000 // 輸出最大值
#define MOTOR_OUT_MIN         (-3000) // 輸出最小值

#define MOTOR_LEFT_AIN1_LOW   (GPIO_ResetBits(GPIOB, GPIO_Pin_15))  // PB15對應馬達驅動AIN1為低時PB15為0
#define MOTOR_LEFT_AIN1_HIGH  (GPIO_SetBits(GPIOB, GPIO_Pin_15))    // PB15對應馬達驅動AIN1為高時PB15為1
#define MOTOR_LEFT_AIN2_LOW   (GPIO_ResetBits(GPIOB, GPIO_Pin_14))  // PB14對應馬達驅動AIN2為低時PB14為0
#define MOTOR_LEFT_AIN2_HIGH  (GPIO_SetBits(GPIOB, GPIO_Pin_14))    // PB14對應馬達驅動AIN2為高時PB14為1

#define MOTOR_RIGHT_BIN1_LOW  (GPIO_ResetBits(GPIOB, GPIO_Pin_12))  // PB12對應馬達驅動BIN1為低時PB12為0
#define MOTOR_RIGHT_BIN1_HIGH (GPIO_SetBits(GPIOB, GPIO_Pin_12))    // PB12對應馬達驅動BIN1為高時PB12為1
#define MOTOR_RIGHT_BIN2_LOW  (GPIO_ResetBits(GPIOB, GPIO_Pin_13))  // PB13對應馬達驅動BIN2為低時PB13為0
#define MOTOR_RIGHT_BIN2_HIGH (GPIO_SetBits(GPIOB, GPIO_Pin_13))    // PB13對應馬達驅動BIN2為高時PB13為1

extern float BST_fCarAngle;					//extern���ڱ������ߺ���ǰ���Ա�ʾ�������ߺ����Ķ����ڱ���ļ��У���ʾ�����������˱�������ʱ��������ģ����Ѱ���䶨�塣
extern float BST_fBluetoothSpeed;
extern float BST_fBluetoothDirectionR;
extern float BST_fBluetoothDirectionL;
extern u8 BST_u8MainEventCount;
extern u8 BST_u8SpeedControlCount;
extern float BST_fSpeedControlOut,BST_fCarAngle_P;
extern float  BST_fAngleControlOut;
extern float BST_fSpeedControlOutNew;
extern u8 BST_u8SpeedControlPeriod;
extern u8 BST_u8DirectionControlPeriod;
extern u8 BST_u8DirectionControlCount;
extern u8 BST_u8LEDCount; 
extern u8 BST_u8trig;
extern u8 BST_u8turnPeriod;
extern u8 BST_u8turnCount;
extern u8 ucBluetoothValue;
extern float angle;
extern float anglex;
extern float gyx,gy0;
extern float gyrx;
extern float gyry;
extern float accelx,accely,accelz,gyrx,gyry,gyrz;
extern float BST_fLeftMotorOut,BST_fRightMotorOut,BST_fBluetoothDirectionNew;
extern s16 BST_s16LeftMotorPulse,BST_s16RightMotorPulse;
extern float juli;
extern 	int x,y1,z1,y2,z2,flagbt;
extern float BST_fCarSpeed_I,BST_fCarSpeed_P,BST_fCarAngle_P,BST_fCarAngle_D;

extern void CarStateOut(void);   	//״̬�����Ƴ�״̬
extern void ProtocolCpyData(void); //���ƴ�������
extern void SendAutoUp(void);


void delay_nms(u16 time);
void CarUpstandInit(void);
//void SampleInputVoltage(void);
void AngleControl(void)	 ;
void MotorOutput(void);
void SpeedControl(void);
void BluetoothControl(void)	;
void GetMotorPulse(void);
void SpeedControlOutput(void);
void DirectionControlOutput(void);
void DirectionControl(void); 
void chaoshengbo(void);
void gfcsbOutput(void);
void csbcontrol(void);
void turn(void);
void turnfliteroutput(void);
void InitMPU6050(void);
void kalmanfilter(float Gyro,float Accel);
void kalmanangle(void);

extern u8 newLineReceived;
void Protocol(void);  /*Э�����*/
#endif
