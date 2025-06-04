#include "line_follow.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "upstandingcar.h"

// 感測器定義：PB0=右感測器，PB1=左感測器
#define TCRT5000_RIGHT_GPIO_PORT GPIOB
#define TCRT5000_RIGHT_GPIO_PIN  GPIO_Pin_0
#define TCRT5000_LEFT_GPIO_PORT  GPIOB
#define TCRT5000_LEFT_GPIO_PIN   GPIO_Pin_1

// 馬達速度參數（針對循跡優化，統一提升動力）
#define MOTOR_FORWARD_SPEED     1100    // 統一提升前進速度
#define MOTOR_TURN_SPEED        900     // 統一提升轉彎速度
#define MOTOR_DIRECTION         450     // 統一提升轉向力度
#define MOTOR_START_SPEED       700     // 啟動搜尋速度

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
    BST_fBluetoothSpeed = MOTOR_FORWARD_SPEED; // 前進速度 1100
    BST_fBluetoothDirectionNew = 0;
}

void LineFollow_Left(void)
{
    BST_fBluetoothSpeed = MOTOR_TURN_SPEED; // 轉彎速度 900
    BST_fBluetoothDirectionNew = -MOTOR_DIRECTION; // 左轉 -450（修正方向）
}

void LineFollow_Right(void)
{
    BST_fBluetoothSpeed = MOTOR_TURN_SPEED; // 轉彎速度 900
    BST_fBluetoothDirectionNew = MOTOR_DIRECTION; // 右轉 450（修正方向）
}

void LineFollow_Stop(void)
{
    BST_fBluetoothSpeed = 0;
    BST_fBluetoothDirectionNew = 0;
}

// 新增：初始搜尋模式，讓平衡車主動尋找黑線
void LineFollow_Search(void)
{
    BST_fBluetoothSpeed = MOTOR_START_SPEED; // 搜尋速度 700
    BST_fBluetoothDirectionNew = 0; // 直行搜尋
}

// 主循跡任務，需在主迴圈中定期呼叫
void LineFollow_Task(void)
{
    static int lost_count = 0;
    static int last_action = 0; // 0:前進, 1:左轉, 2:右轉
    static int startup_search_count = 0; // 啟動搜尋計數器
    const int lost_count_threshold = 15; // 增加斷線容錯次數
    const int startup_search_threshold = 50; // 縮短啟動搜尋時間
    
    // 讀取感測器狀態（0=檢測到黑線，1=檢測到白色）
    int left = GPIO_ReadInputDataBit(TCRT5000_LEFT_GPIO_PORT, TCRT5000_LEFT_GPIO_PIN);
    int right = GPIO_ReadInputDataBit(TCRT5000_RIGHT_GPIO_PORT, TCRT5000_RIGHT_GPIO_PIN);
    
    // 雙感測器循跡邏輯（已修正方向定義）
    // 根據馬達控制邏輯：BST_fBluetoothDirectionNew正值=右轉，負值=左轉
    // 左黑右白(0,1) -> 車子偏左，需要右轉修正
    // 左白右黑(1,0) -> 車子偏右，需要左轉修正
    // 左黑右黑(0,0) -> 在線上，直行
    // 左白右白(1,1) -> 脫線，容錯處理
    
    if(left == 0 && right == 1) {
        // 左感測器檢測到黑線，右感測器沒有 -> 車子偏左，需要右轉修正
        LineFollow_Right();
        last_action = 2;
        lost_count = 0;
        startup_search_count = 0; // 找到線，停止搜尋
    } else if(left == 1 && right == 0) {
        // 右感測器檢測到黑線，左感測器沒有 -> 車子偏右，需要左轉修正
        LineFollow_Left();
        last_action = 1;
        lost_count = 0;
        startup_search_count = 0; // 找到線，停止搜尋
    } else if(left == 0 && right == 0) {
        // 兩個感測器都檢測到黑線 -> 正在線上或線很寬，直行
        LineFollow_Forward(); // 統一使用提升後的前進速度
        last_action = 0;
        lost_count = 0;
        startup_search_count = 0; // 找到線，停止搜尋
    } else if(left == 1 && right == 1) {
        // 兩個感測器都沒檢測到黑線 -> 脫線或初始狀態
        
        // 啟動搜尋：如果是剛開始且還沒找到線，主動搜尋
        if(startup_search_count < startup_search_threshold && last_action == 0) {
            LineFollow_Search(); // 主動前進搜尋黑線
            startup_search_count++;
            lost_count = 0;
        } else {
            // 正常脫線處理
            lost_count++;
            if(lost_count < lost_count_threshold) {
                // 維持前一動作，提升響應性
                if(last_action == 1) LineFollow_Left();
                else if(last_action == 2) LineFollow_Right();
                else LineFollow_Search(); // 如果沒有前一動作，繼續搜尋
            } else {
                // 脫線太久，停止
                LineFollow_Stop();
            }
        }
    }
}
