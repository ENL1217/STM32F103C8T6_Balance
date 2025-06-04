#include "line_follow.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "upstandingcar.h"

// 感測器定義：PB0=右感測器，PB1=左感測器
#define TCRT5000_RIGHT_GPIO_PORT GPIOB
#define TCRT5000_RIGHT_GPIO_PIN  GPIO_Pin_0
#define TCRT5000_LEFT_GPIO_PORT  GPIOB
#define TCRT5000_LEFT_GPIO_PIN   GPIO_Pin_1

// 馬達速度參數（與藍芽遙控一致）
#define MOTOR_FORWARD_SPEED  800
#define MOTOR_TURN_SPEED     600
#define MOTOR_DIRECTION      300

void LineFollow_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    // 初始化右感測器（PB0）
    GPIO_InitStructure.GPIO_Pin = TCRT5000_RIGHT_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉輸入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TCRT5000_RIGHT_GPIO_PORT, &GPIO_InitStructure);
    
    // 初始化左感測器（PB1）
    GPIO_InitStructure.GPIO_Pin = TCRT5000_LEFT_GPIO_PIN;
    GPIO_Init(TCRT5000_LEFT_GPIO_PORT, &GPIO_InitStructure);
}

void LineFollow_Forward(void)
{
    BST_fBluetoothSpeed = MOTOR_FORWARD_SPEED; // 前進速度 800
    BST_fBluetoothDirectionNew = 0;
}

void LineFollow_Left(void)
{
    BST_fBluetoothSpeed = MOTOR_TURN_SPEED; // 轉彎速度 600
    BST_fBluetoothDirectionNew = MOTOR_DIRECTION; // 左轉 300
}

void LineFollow_Right(void)
{
    BST_fBluetoothSpeed = MOTOR_TURN_SPEED; // 轉彎速度 600
    BST_fBluetoothDirectionNew = -MOTOR_DIRECTION; // 右轉 -300
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
    
    // 讀取感測器狀態（0=檢測到黑線，1=檢測到白色）
    int left = GPIO_ReadInputDataBit(TCRT5000_LEFT_GPIO_PORT, TCRT5000_LEFT_GPIO_PIN);
    int right = GPIO_ReadInputDataBit(TCRT5000_RIGHT_GPIO_PORT, TCRT5000_RIGHT_GPIO_PIN);
    float pitch = BST_fCarAngle; // 取得車體俯仰角
    
    // 下坡補償：若俯仰角小於-10度，降低速度
    if(pitch < -10.0f) {
        // 臨時降低速度，可在各函式內動態調整
    }
    
    // 雙感測器循跡邏輯
    // 左黑右白(0,1) -> 偏左，需要右轉修正
    // 左白右黑(1,0) -> 偏右，需要左轉修正
    // 左黑右黑(0,0) -> 在線上，直行
    // 左白右白(1,1) -> 脫線，容錯處理
    
    if(left == 0 && right == 1) {
        // 左感測器檢測到黑線，右感測器沒有 -> 車子偏左，需要右轉
        LineFollow_Right();
        last_action = 2;
        lost_count = 0;
    } else if(left == 1 && right == 0) {
        // 右感測器檢測到黑線，左感測器沒有 -> 車子偏右，需要左轉
        LineFollow_Left();
        last_action = 1;
        lost_count = 0;
    } else if(left == 0 && right == 0) {
        // 兩個感測器都檢測到黑線 -> 正在線上或線很寬，直行
        LineFollow_Forward();
        last_action = 0;
        lost_count = 0;
    } else if(left == 1 && right == 1) {
        // 兩個感測器都沒檢測到黑線 -> 脫線，容錯處理
        lost_count++;
        if(lost_count < lost_count_threshold) {
            // 維持前一動作
            if(last_action == 1) LineFollow_Left();
            else if(last_action == 2) LineFollow_Right();
            else LineFollow_Forward();
        } else {
            // 脫線太久，停止
            LineFollow_Stop();
        }
    }
}
