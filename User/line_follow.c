#include "line_follow.h"
#include "motor.h"

void LineSensor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

uint8_t LineSensor_Read(void)
{
    return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0);
}

void LineFollow_Run(void)
{
    if(LineSensor_Read() == 0) // black line detected
    {
        SetMotorVoltageAndDirection(400, 400);
    }
    else
    {
        SetMotorVoltageAndDirection(0, 0);
    }
}
