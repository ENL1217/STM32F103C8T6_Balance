#ifndef __LINE_FOLLOW_H__
#define __LINE_FOLLOW_H__

#include "stm32f10x.h"

// 初始化循跡感測器（PB0=右感測器，PB1=左感測器）
void LineFollow_Init(void);
// 循跡主任務，需在主迴圈中定期呼叫
void LineFollow_Task(void);

// 馬達控制包裝
void LineFollow_Forward(void);
void LineFollow_Left(void);
void LineFollow_Right(void);
void LineFollow_Stop(void);
void LineFollow_Search(void);  // 搜尋模式
void LineFollow_Uphill(void);  // 爬坡模式

#endif // __LINE_FOLLOW_H__
