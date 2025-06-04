#ifndef __LINE_FOLLOW_H
#define __LINE_FOLLOW_H

#include "stm32f10x.h"

void LineSensor_Init(void);
uint8_t LineSensor_Read(void);
void LineFollow_Run(void);

#endif
