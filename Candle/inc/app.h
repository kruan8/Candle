/*
 * app.h
 *
 *  Created on: 19. 10. 2018
 *  Author:     Priesol Vladimir
 */

#ifndef APP_H_
#define APP_H_

#include "stm32f0xx.h"
#include "stdbool.h"

void App_Init(void);
void App_Exec(void);
void App_PwmInit(uint32_t nBusClock_Hz);
void App_PwmSet(uint16_t nValue);

void AD_Init(void);
void Gpio_Init(void);
uint32_t GetTrueRandomNumber();
void RTC_Init_();
void StopMode(void);
void RTC_WriteAccess(bool bEnable);


#endif /* APP_H_ */
