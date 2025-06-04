#include "line_follow.h"
#include "stm32f10x_gpio.h"
#include "upstandingcar.h" // 需有 SetMotorVoltageAndDirection

#define TCRT5000_LEFT_GPIO_PORT  GPIOB
#define TCRT5000_LEFT_GPIO_PIN   GPIO_Pin_0
#define TCRT5000_RIGHT_GPIO_PORT GPIOB
#define TCRT5000_RIGHT_GPIO_PIN  GPIO_Pin_1

// 馬達速度參數（可依實際車體調整）
#define MOTOR_FORWARD_SPEED  2000
#define MOTOR_TURN_SPEED     1500
#define MOTOR_STOP           0

void LineFollow_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    // 初始化左感測器
    GPIO_InitStructure.GPIO_Pin = TCRT5000_LEFT_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉輸入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TCRT5000_LEFT_GPIO_PORT, &GPIO_InitStructure);
    // 初始化右感測器
    GPIO_InitStructure.GPIO_Pin = TCRT5000_RIGHT_GPIO_PIN;
    GPIO_Init(TCRT5000_RIGHT_GPIO_PORT, &GPIO_InitStructure);
}

void LineFollow_Forward(void)
{
    BST_fBluetoothSpeed = 20; // 依車體調整，單位與藍芽遙控一致
    BST_fBluetoothDirectionNew = 0;
}

void LineFollow_Left(void)
{
    BST_fBluetoothSpeed = 10;
    BST_fBluetoothDirectionNew = 20; // 正值左轉
}

void LineFollow_Right(void)
{
    BST_fBluetoothSpeed = 10;
    BST_fBluetoothDirectionNew = -20; // 負值右轉
}

void LineFollow_Stop(void)
{
    BST_fBluetoothSpeed = 0;
    BST_fBluetoothDirectionNew = 0;
}

// 主循跡任務，需在主迴圈中定期呼叫
void LineFollow_Task(void)
{
    static int lost_count = 0;
    static int last_action = 0; // 0:前進, 1:左轉, 2:右轉
    const int lost_count_threshold = 10; // 允許斷線容錯次數
    int left = GPIO_ReadInputDataBit(TCRT5000_LEFT_GPIO_PORT, TCRT5000_LEFT_GPIO_PIN);
    int right = GPIO_ReadInputDataBit(TCRT5000_RIGHT_GPIO_PORT, TCRT5000_RIGHT_GPIO_PIN);
    float pitch = BST_fCarAngle; // 取得車體俯仰角
    int forward_speed = MOTOR_FORWARD_SPEED;
    // 下坡補償：若俯仰角小於-10度，降低速度
    if(pitch < -10) {
        forward_speed = MOTOR_FORWARD_SPEED / 2;
    }
    // 雙感測器循跡判斷
    if(left == 0 && right == 1) {
        LineFollow_Left();
        last_action = 1;
        lost_count = 0;
    } else if(left == 1 && right == 0) {
        LineFollow_Right();
        last_action = 2;
        lost_count = 0;
    } else if(left == 0 && right == 0) {
        LineFollow_Stop(); // 或可改為前進
        last_action = 0;
        lost_count = 0;
    } else if(left == 1 && right == 1) {
        // 斷線容錯：短暫維持前一動作
        lost_count++;
        if(lost_count < lost_count_threshold) {
            if(last_action == 1) LineFollow_Left();
            else if(last_action == 2) LineFollow_Right();
            else LineFollow_Forward();
        } else {
            LineFollow_Stop();
        }
    }
}
