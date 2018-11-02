/*
 * app.c
 *
 *  Created on: 19. 10. 2018
 *  Author:     Priesol Vladimir
 */

#include "app.h"
#include "timer.h"

#define LED_PORT  GPIOC
#define LED_PIN   GPIO_Pin_9

#define LED_ON    LED_PORT->BSRR = LED_PIN
#define LED_OFF   LED_PORT->BRR = LED_PIN

#define APP_SYSTICK_ISR_OFF     SysTick->CTRL  &= ~SysTick_CTRL_TICKINT_Msk  // vypnout preruseni od Systick
#define APP_SYSTICK_ISR_ON      SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk   // zapnout preruseni od Systick

#define TIM_PWM              TIM3  // (PC9 = TIM3/CH4)
#define TIM_PWM_CLK_ENABLE   RCC->APB1ENR |= RCC_APB1ENR_TIM3EN
#define PWM_STEPS            16       // pocet kroku PWM

uint8_t g_nFrameCtrl = 0;   // 5 bit-Counter

uint8_t g_nPwmValue = 0;    // 4 bit-Register
uint8_t g_nNextBright = 0;  // 4 bit-Register
uint8_t g_nRand = 0;        // 5 bit Signal
uint8_t g_nRandFlag = 0;    // 1 bit Signal

void App_Init(void)
{
#ifdef DEBUG
  RCC->APB2ENR |= RCC_APB2ENR_DBGMCUEN;
#endif

  Timer_Init();
  Gpio_Init();
  AD_Init();
//  RTC_Init_();

#ifdef HW
  App_PwmInit(8000000);
#endif
}

void App_PwmInit(uint32_t nBusClock_Hz)
{
  TIM_PWM_CLK_ENABLE;

  // nastavit 6kHz
  TIM_PWM->PSC = nBusClock_Hz / 6000;  // cca 6 kHz
  TIM_PWM->ARR = PWM_STEPS;

  /* (3) Set CCRx = 4, , the signal will be high during 4 us */
  /* (4) Select PWM mode 1 on OC1 (OC1M = 110),
  enable preload register on OC1 (OC1PE = 1) */
  /* (5) Select active high polarity on OC1 (CC1P = 0, reset value),
  enable the output on OC1 (CC1E = 1)*/
  /* (6) Enable output (MOE = 1)*/
  /* (7) Enable counter (CEN = 1)
  select edge aligned mode (CMS = 00, reset value)
  select direction as upcounter (DIR = 0, reset value) */
  /* (8) Force update generation (UG = 1) */
  TIM_PWM->CCR4 = 0; /* (3)  PWM value */
  TIM_PWM->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE; /* (4) */ // PWM mode1 + Preload register on TIMx_CCR4 enabled.
  TIM_PWM->CCER |= TIM_CCER_CC4E; /* (5) */
  TIM_PWM->CR1 |= TIM_CR1_CEN;
  TIM_PWM->EGR |= TIM_EGR_UG; /* (8) */

  TIM_PWM->DIER |= TIM_DIER_UIE;

  // povolit preruseni od TIM3
  NVIC_SetPriority(TIM3_IRQn, 0);    // Set priority
  NVIC_EnableIRQ(TIM3_IRQn);         // Enable RTC_IRQn

#ifdef DEBUG
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM3_STOP;
#endif
}

void App_PwmSet(uint16_t nValue)
{
  TIM_PWM->CCR4 = nValue;
}

void App_Exec(void)
{

#ifdef HW
  while (1)
  {
    SleepMode();
  }
#endif

  uint8_t nPwmCtrl = 0;   // 4 bit-Counter

  while(1)
  {
    TimerUs_Delay(150);
//     StopMode();

    // PWM led
    nPwmCtrl++;
    nPwmCtrl &= 0xf;    // only 4 bit
    if (nPwmCtrl <= g_nPwmValue)
    {
      LED_ON;
    }
    else
    {
      LED_OFF;
    }

    // FRAME
    if (nPwmCtrl == 0)
    {
      g_nFrameCtrl++;
      g_nFrameCtrl &= 0x1f;

      // generate a new random number every 8 cycles. In reality this is most likely bit serial
      if ((g_nFrameCtrl & 0x07) == 0)
      {
        g_nRand = GetTrueRandomNumber() & 0x1f;
        if ((g_nRand & 0x0c) != 0)
        {
          g_nRandFlag = 1;
        }
        else
        {
          g_nRandFlag = 0; // only update if valid
        }
      }

      // NEW FRAME
      if (g_nFrameCtrl == 0)
      {
        // reload PWM
        g_nPwmValue = g_nNextBright;

        // force update at beginning of frame
        g_nRandFlag = 1;
      }

      if (g_nRandFlag)
      {
        g_nNextBright = g_nRand > 15 ? 15 : g_nRand;
      }
    }
  }

}

void AD_Init(void)
{
  ADC_InitTypeDef ADC_InitStructure;

  // HSI14 enable
  RCC->CR2 |= RCC_CR2_HSI14ON;

  // Initialize ADC 14MHz RC
  RCC_ADCCLKConfig(RCC_ADCCLK_HSI14);

  while (!RCC_GetFlagStatus(RCC_FLAG_HSI14RDY));

  ADC_DeInit(ADC1);
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_TRGO; //default
  ADC_Init(ADC1, &ADC_InitStructure);

  //Convert the ADC1 temperature sensor, user shortest sample time to generate most noise
  ADC1->CHSELR |= (uint32_t)ADC_Channel_TempSensor;
  ADC1->SMPR = 0;     // ADC_SampleTime_1_5Cycles

  // HSI14 disable
  RCC->CR2 &= ~RCC_CR2_HSI14ON;
}

void Gpio_Init(void)
{
  GPIO_InitTypeDef InitStruct;
  GPIO_StructInit(&InitStruct);

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

#ifdef HW
  InitStruct.GPIO_Pin = LED_PIN;
  InitStruct.GPIO_Mode = GPIO_Mode_AF;
  InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(LED_PORT, &InitStruct);

  GPIO_PinAFConfig(LED_PORT, GPIO_PinSource9, GPIO_AF_1);
#else
  InitStruct.GPIO_Pin = LED_PIN;
  InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(LED_PORT, &InitStruct);
#endif

}

void RTC_Init_()
{
  RCC->APB1ENR |= RCC_APB1ENR_PWREN; // Enable PWR clock
  RCC->CSR |= RCC_CSR_LSION; // Enable the LSI
  while(!(RCC->CSR & RCC_CSR_LSIRDY)); // Wait while it is not ready
  RTC_WriteAccess(true);
  RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_RTCSEL) | RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_LSI;
  RTC->ISR = RTC_ISR_INIT; // Enable init phase
  while((RTC->ISR & RTC_ISR_INITF) != RTC_ISR_INITF);

  RTC->PRER = 0x00000003;  // do RTC by melo jit 40/3 kHz

  // nastavit alarm na hodnotu 6 impulsu z LSI -> 40kHz/6=6,5kHz (~150 us)
  RTC->ALRMAR = 2 | RTC_ALRMAR_MSK2 | RTC_ALRMAR_MSK3 | RTC_ALRMAR_MSK4;

  // povolit alarm a INT
  RTC->CR |= (RTC_CR_ALRAE | RTC_CR_ALRAIE);

  // vynulovat time registr
  RTC->TR = 0;

  RTC->ISR =~ RTC_ISR_INIT;
  RTC_WriteAccess(false);

  // nastavit EXTI od Alarmu
  EXTI->IMR |= EXTI_EMR_MR17;       // line 17 is connected to RTC Alarm event
  EXTI->RTSR |= EXTI_RTSR_TR17;     // Rising edge for line 17

  // povolit preruseni od RTC
  NVIC_SetPriority(RTC_IRQn, 0);    // Set priority
  NVIC_EnableIRQ(RTC_IRQn);         // Enable RTC_IRQn
}

void SleepMode(void)
{
  APP_SYSTICK_ISR_OFF;
  PWR_EnterSleepMode(PWR_SLEEPEntry_WFI);
  APP_SYSTICK_ISR_ON;
}

void StopMode(void)
{
  RTC_WriteAccess(true);
  RTC->ISR = RTC_ISR_INIT; // Enable init phase

  // vynulovat time registr
  RTC->TR = 0;
  RTC->CR |= (RTC_CR_ALRAE | RTC_CR_ALRAIE);
  RTC->ISR =~ RTC_ISR_INIT;
  RTC_WriteAccess(false);

  APP_SYSTICK_ISR_OFF;

  PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

  APP_SYSTICK_ISR_ON;
}

void RTC_WriteAccess(bool bEnable)
{
  if (bEnable)
  {
    // Enable write in RTC domain control register
    PWR->CR |= PWR_CR_DBP;

    // Enable write access
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
  }
  else
  {
    // Disable write access
    RTC->WPR = 0xFE;
    RTC->WPR = 0x64;

    PWR->CR &= ~PWR_CR_DBP;
  }
}

uint32_t GetTrueRandomNumber()
{
  // HSI14 enable
  RCC->CR2 |= RCC_CR2_HSI14ON;

  //enable ADC1 clock
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  //enable internal temperature sensor
  ADC->CCR |= (uint32_t)ADC_CCR_TSEN;

  // Enable ADCperipheral
  ADC1->CR |= (uint32_t)ADC_CR_ADEN;
  while (!(ADC1->ISR & ADC_FLAG_ADRDY));

  // Enable CRC clock
  RCC->AHBENR |= RCC_AHBENR_CRCEN;

  for (uint8_t i = 0; i < 8; i++)
  {
    //Start ADC1 Software Conversion
    ADC1->CR |= (uint32_t)ADC_CR_ADSTART;

    //wait for conversion complete
    while (!(ADC1->ISR = (uint32_t)ADC_FLAG_EOC));

    CRC->DR = ADC1->DR;

    //clear EOC flag
//    ADC1->ISR = (uint32_t)ADC_FLAG_EOC;
  }

  // ADC disable
  ADC1->CR |= (uint32_t)ADC_CR_ADDIS;

  // HSI14 disable
  RCC->CR2 &= ~RCC_CR2_HSI14ON;

  // disable temperature sensor to save power
  ADC->CCR &= (uint32_t)(~ADC_CCR_TSEN);

  RCC->APB2ENR &= ~RCC_APB2Periph_ADC1;

  CRC->DR = 0xBADA55E5;
  uint32_t nValue = CRC->DR;

  RCC->AHBENR &= ~RCC_AHBENR_CRCEN;

  return nValue;
}

void TIM3_IRQHandler(void)
{
  if (!(TIM_PWM->SR & TIM_SR_UIF))
  {
    return;
  }

  TIM_PWM->SR &= ~TIM_SR_UIF;

  g_nFrameCtrl++;
  g_nFrameCtrl &= 0x1f;

  // generate a new random number every 8 cycles. In reality this is most likely bit serial
  if ((g_nFrameCtrl & 0x07) == 0)
  {
    g_nRand = GetTrueRandomNumber() & 0x1f;
    if ((g_nRand & 0x0c) != 0)
    {
      g_nRandFlag = 1;
    }
    else
    {
      g_nRandFlag = 0; // only update if valid
    }
  }

  // NEW FRAME
  if (g_nFrameCtrl == 0)
  {
    // reload PWM
    g_nPwmValue = g_nNextBright;

    // force update at beginning of frame
    g_nRandFlag = 1;
  }

  if (g_nRandFlag)
  {
    g_nNextBright = g_nRand > 15 ? 15 : g_nRand;
  }

  App_PwmSet(g_nPwmValue);
}

void RTC_IRQHandler(void)
{
  EXTI->PR = EXTI_PR_PR17;
  RTC_WriteAccess(true);
//  RTC->CR &= ~(RTC_CR_ALRAE | RTC_CR_ALRAIE);
  RTC->ISR = (uint32_t)((uint32_t)(~((RTC_ISR_ALRAF | RTC_ISR_INIT)& 0x0001FFFF) | (uint32_t)(RTC->ISR & RTC_ISR_INIT)));
//  RTC_WriteAccess(false);
  RTC->ISR |= RTC_ISR_ALRAF;
}
