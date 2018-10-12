/*
 * timer.h
 *
 *  Created on: 24. 6. 2016
 *      Author: priesolv
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "stm32f0xx.h"

typedef void(*PtrSysTickCallback) (void);

void Timer_Init();
void Timer_Delay_ms(uint32_t delay_ms);
uint32_t Timer_GetTicks_ms();
void Timer_SetSysTickCallback(PtrSysTickCallback pFunction);


void TimerUs_Init(void);
void TimerUs_Start(void);
uint16_t TimerUs_GetMicroseconds(void);
void TimerUs_Delay(uint16_t microseconds);
void TimerUs_Clear(void);
void TimerUs_Stop(void);
#endif /* TIMER_H_ */
