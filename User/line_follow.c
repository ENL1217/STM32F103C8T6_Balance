#include "line_follow.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "upstandingcar.h"

// 感測器定義：PB0=右感測器，PB1=左感測器
#define TCRT5000_RIGHT_GPIO_PORT GPIOB
#define TCRT5000_RIGHT_GPIO_PIN  GPIO_Pin_0
#define TCRT5000_LEFT_GPIO_PORT  GPIOB
#define TCRT5000_LEFT_GPIO_PIN   GPIO_Pin_1
// 新增：中間感測器定義（PB10）
#define TCRT5000_MID_GPIO_PORT GPIOB
#define TCRT5000_MID_GPIO_PIN  GPIO_Pin_10

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
    
    // 初始化中間感測器（PB10）
    GPIO_InitStructure.GPIO_Pin = TCRT5000_MID_GPIO_PIN;
    GPIO_Init(TCRT5000_MID_GPIO_PORT, &GPIO_InitStructure);
}

// 自動補償直行動力（遇到阻力自動加大馬達輸出）
void LineFollow_Forward(void)
{
    // --- 卡住判斷與自動補償 ---
    static int stuck_count = 0;
    static int boost_count = 0;
    static int last_left_pulse = 0;
    static int last_right_pulse = 0;
    int left_pulse = BST_s16LeftMotorPulse;
    int right_pulse = BST_s16RightMotorPulse;
    float left_out = BST_fLeftMotorOut;
    float right_out = BST_fRightMotorOut;
    int left_pulse_delta = abs(left_pulse - last_left_pulse);
    int right_pulse_delta = abs(right_pulse - last_right_pulse);
    last_left_pulse = left_pulse;
    last_right_pulse = right_pulse;

    // 判斷PWM大且脈衝變化小才算卡住
    int pwm_stuck = (fabs(left_out) > 1400.0f || fabs(right_out) > 1400.0f);
    int pulse_stuck = (left_pulse_delta < 2 && right_pulse_delta < 2);
    int is_stuck = (pwm_stuck && pulse_stuck);

    if(is_stuck) {
        stuck_count++;
    } else {
        stuck_count = 0;
        boost_count = 0;
    }

    if(boost_count > 0) {
        BST_fBluetoothSpeed = 1200; // 補償期間速度
        boost_count--;
    } else if(stuck_count > 8) { // 連續判斷卡住才補償，避免誤觸
        BST_fBluetoothSpeed = 1200; // 短暫補償
        boost_count = 8;           // 只補償8個循環
        stuck_count = 0;
    } else {
        BST_fBluetoothSpeed = MOTOR_FORWARD_SPEED; // 正常速度 1100
    }
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
    const int lost_count_threshold = 15;
    const int startup_search_threshold = 50;
    // 讀取感測器狀態（0=黑線，1=白色）
    int left = GPIO_ReadInputDataBit(TCRT5000_LEFT_GPIO_PORT, TCRT5000_LEFT_GPIO_PIN);
    int right = GPIO_ReadInputDataBit(TCRT5000_RIGHT_GPIO_PORT, TCRT5000_RIGHT_GPIO_PIN);
    int mid = GPIO_ReadInputDataBit(TCRT5000_MID_GPIO_PORT, TCRT5000_MID_GPIO_PIN);

    // 三感測器循跡與爬坡補償（優化判斷）
    if(mid == 0 && left == 1 && right == 1) {
        // 只有中間感測器在線上，左右都沒偵測到黑線，完全在線上
        // 啟動爬坡補償
        BST_fBluetoothSpeed = 1350; // 爬坡補償速度
        BST_fBluetoothDirectionNew = 0;
        last_action = 0;
        lost_count = 0;
        startup_search_count = 0;
        return;
    } else if(mid == 0 && left == 0 && right == 1) {
        // 左黑中黑右白，偏左，需右修正
        LineFollow_Right();
        last_action = 2;
        lost_count = 0;
        startup_search_count = 0;
    } else if(mid == 0 && left == 1 && right == 0) {
        // 左白中黑右黑，偏右，需左修正
        LineFollow_Left();
        last_action = 1;
        lost_count = 0;
        startup_search_count = 0;
    } else if(mid == 1 && left == 0 && right == 1) {
        // 左黑中白右白，大幅偏左，需大右轉
        LineFollow_Right();
        last_action = 2;
        lost_count = 0;
        startup_search_count = 0;
    } else if(mid == 1 && left == 1 && right == 0) {
        // 左白中白右黑，大幅偏右，需大左轉
        LineFollow_Left();
        last_action = 1;
        lost_count = 0;
        startup_search_count = 0;
    } else if(mid == 0 && left == 0 && right == 0) {
        // 三黑，正在線上，直行
        LineFollow_Forward();
        last_action = 0;
        lost_count = 0;
        startup_search_count = 0;
    } else if(mid == 1 && left == 1 && right == 1) {
        // 三白，脫線，搜尋/容錯
        if(startup_search_count < startup_search_threshold && last_action == 0) {
            LineFollow_Search(); // 主動前進搜尋黑線
            startup_search_count++;
            lost_count = 0;
        } else {
            lost_count++;
            if(lost_count < lost_count_threshold) {
                if(last_action == 1) LineFollow_Left();
                else if(last_action == 2) LineFollow_Right();
                else LineFollow_Search();
            } else {
                LineFollow_Stop();
            }
        }
    } else {
        // 其他組合，預設直行
        LineFollow_Forward();
        last_action = 0;
        lost_count = 0;
        startup_search_count = 0;
    }
}
