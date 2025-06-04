#ifndef __UltrasonicWave_H
#define	__UltrasonicWave_H

void UltrasonicWave_Configuration(void);               // 超音波模組初始化
void UltrasonicWave_StartMeasure(void);                // 啟動測距，Trig腳>10us高電位，然後等待回波高電位時間

extern float juli;
#endif /* __UltrasonicWave_H */

