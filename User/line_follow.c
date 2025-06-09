#include "line_follow.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "upstandingcar.h"
#include "adc.h" // 假設你有adc初始化與讀值函式
#include <stdlib.h>
#include <math.h>

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

// --- PID循跡參數 ---
#define LINE_KP 35.0f
#define LINE_KI 0.0f
#define LINE_KD 18.0f

void LineFollow_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // 初始化PB0/PB1為類比輸入
    GPIO_InitStructure.GPIO_Pin = TCRT5000_RIGHT_GPIO_PIN | TCRT5000_LEFT_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // 類比輸入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 初始化中間感測器（PB10）為數位輸入
    GPIO_InitStructure.GPIO_Pin = TCRT5000_MID_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉輸入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
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
    static int last_action = 0;
    static int startup_search_count = 0;
    static float error_sum = 0;
    static float last_error = 0;
    static int last_left_pulse = 0;
    static int last_right_pulse = 0;
    const int lost_count_threshold = 15;
    const int startup_search_threshold = 50;
    // 讀取ADC與中間感測器
    uint16_t adc_left = Get_Adc(9);  // PB1 = ADC1_IN9
    uint16_t adc_right = Get_Adc(8); // PB0 = ADC1_IN8
    int mid = GPIO_ReadInputDataBit(TCRT5000_MID_GPIO_PORT, TCRT5000_MID_GPIO_PIN);
    // 黑白閾值
    const uint16_t ADC_BLACK = 1500;
    const uint16_t ADC_WHITE = 2500;
    // --- 爬坡補償 ---
    int left_is_white = (adc_left > ADC_WHITE);
    int right_is_white = (adc_right > ADC_WHITE);
    int left_pulse = BST_s16LeftMotorPulse;
    int right_pulse = BST_s16RightMotorPulse;
    int left_pulse_delta = abs(left_pulse - last_left_pulse);
    int right_pulse_delta = abs(right_pulse - last_right_pulse);
    last_left_pulse = left_pulse;
    last_right_pulse = right_pulse;
    int stuck = (left_pulse_delta < 2 && right_pulse_delta < 2);
    if(mid == 0 && left_is_white && right_is_white && stuck) {
        // 中間黑、左右白且馬達沒動，啟動爬坡補償
        BST_fBluetoothSpeed = 1350;
        BST_fBluetoothDirectionNew = 0;
        last_action = 0;
        lost_count = 0;
        startup_search_count = 0;
        error_sum = 0;
        last_error = 0;
        return;
    }
    // --- ADC PID循跡 ---
    float left_norm = 1.0f - ((float)(adc_left - ADC_BLACK) / (ADC_WHITE - ADC_BLACK));
    float right_norm = 1.0f - ((float)(adc_right - ADC_BLACK) / (ADC_WHITE - ADC_BLACK));
    if(left_norm < 0) left_norm = 0; if(left_norm > 1) left_norm = 1;
    if(right_norm < 0) right_norm = 0; if(right_norm > 1) right_norm = 1;
    float error = right_norm - left_norm; // 右大於左，偏右
    error_sum += error;
    float d_error = error - last_error;
    last_error = error;
    float output = LINE_KP * error + LINE_KI * error_sum + LINE_KD * d_error;
    if(output > 600) output = 600;
    if(output < -600) output = -600;
    BST_fBluetoothSpeed = MOTOR_FORWARD_SPEED;
    BST_fBluetoothDirectionNew = (int)output;
    last_action = (output > 0) ? 2 : (output < 0) ? 1 : 0;
    lost_count = 0;
    startup_search_count = 0;
    // --- 三白搜尋 ---
    if(adc_left > ADC_WHITE && adc_right > ADC_WHITE && mid == 1) {
        if(startup_search_count < startup_search_threshold && last_action == 0) {
            LineFollow_Search();
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
        error_sum = 0;
        last_error = 0;
    }
}
