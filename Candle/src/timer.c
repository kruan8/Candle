/*
 * timer.c
 *
 *  Created on: 12. 10. 2018
 *      Author: Priesol Vladimir
 */

#include "timer.h"

// casovac pro mereni us intervalu
#define TIMER_US      TIM14
#define TIMER_US_CLK_ENABLE  RCC->APB1ENR |= RCC_APB1ENR_TIM14EN

static volatile uint32_t nDelayTimer;
static volatile uint32_t g_nTicks = 0;

PtrSysTickCallback pSysTickCallback = 0;

void Timer_Init()
{
  SystemCoreClockUpdate();
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    /* Capture error */
    while (1);
  }

  TimerUs_Init();
}

void Timer_Delay_ms(uint32_t delay_ms)
{
  nDelayTimer = delay_ms;
  while (nDelayTimer);
}

uint32_t Timer_GetTicks_ms()
{
  return g_nTicks;
}

void Timer_SetSysTickCallback(PtrSysTickCallback pFunction)
{
  pSysTickCallback = pFunction;
}

void SysTick_Handler(void)
{
  g_nTicks++;
  if (nDelayTimer)
  {
    nDelayTimer--;
  }

  if (pSysTickCallback)
  {
    pSysTickCallback();
  }
}


// timer for us counting
void TimerUs_Init(void)
{
  // Enable clock for TIM
  TIMER_US_CLK_ENABLE;
  TimerUs_Start();

#ifdef DEBUG
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM14_STOP | DBGMCU_APB1_FZ_DBG_RTC_STOP;
#endif
}

void TimerUs_Start(void)
{
  TIMER_US->PSC = SystemCoreClock / 1000000; // 7 instructions
  TIMER_US->CNT = 0;
  TIMER_US->EGR = TIM_EGR_UG;
  TIMER_US->CR1 |= TIM_CR1_CEN;
}

uint16_t TimerUs_GetMicroseconds(void)
{
    return TIMER_US->CNT;
}

void TimerUs_Delay(uint16_t microseconds)
{
  TimerUs_Clear();
  uint16_t t = TimerUs_GetMicroseconds() + microseconds;
  while (TimerUs_GetMicroseconds() < t)
  {
    continue;
  }
}

void TimerUs_Clear(void)
{
  TIMER_US->CNT = 0;
}

void TimerUs_Stop(void)
{
  TIMER_US->CR1 &= ~TIM_CR1_CEN;
}
