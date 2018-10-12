/*
 *
 * Created on: 12.10.2018
 *    Author: Priesol Vladimir
 *
**********************************************************************/

#include "stm32f0xx.h"
#include "stm32f0xx_adc.h"
#include "timer.h"
#include <stdbool.h>

void AD_Init(void);
void Gpio_Init(void);
uint32_t GetTrueRandomNumber();
void RTC_Init_();
void StopMode(void);
void RTC_WriteAccess(bool bEnable);

#define LED_PORT  GPIOC
#define LED_PIN   GPIO_Pin_9

#define LED_ON    LED_PORT->BSRR = LED_PIN
#define LED_OFF   LED_PORT->BRR = LED_PIN

#define APP_SYSTICK_ISR_OFF     SysTick->CTRL  &= ~SysTick_CTRL_TICKINT_Msk  // vypnout preruseni od Systick
#define APP_SYSTICK_ISR_ON      SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk   // zapnout preruseni od Systick

int main(void)
{
  // kontrola hodin
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks); // Get system clocks

  Timer_Init();
  Gpio_Init();
  AD_Init();
  RTC_Init_();

  uint8_t nPwmCtrl = 0;		// 4 bit-Counter
  uint8_t nFrameCtrl = 0;	// 5 bit-Counter

 	uint8_t nPwmValue = 0;		// 4 bit-Register
 	uint8_t nNextBright = 0;	// 4 bit-Register
 	uint8_t nRand = 0;			  // 5 bit Signal
 	uint8_t nRandFlag = 0;		// 1 bit Signal

  while(1)
  {
    TimerUs_Delay(150);
    // StopMode();

 		// PWM - nastavuje jas LED
    nPwmCtrl++;
 		nPwmCtrl &= 0xf;		// only 4 bit
 		if (nPwmCtrl <= nPwmValue)
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
 			nFrameCtrl++;
 			nFrameCtrl &= 0x1f;

 			// generate a new random number every 8 cycles. In reality this is most likely bit serial
 			if ((nFrameCtrl & 0x07) == 0)
 			{
 				nRand = GetTrueRandomNumber() & 0x1f;
 				if ((nRand & 0x0c) != 0)
        {
          nRandFlag = 1;
        }
        else
        {
          nRandFlag = 0; // only update if valid
        }
 			}

			// NEW FRAME
 			if (nFrameCtrl == 0)
 			{
 			  // reload PWM
 				nPwmValue = nNextBright;

 				// force update at beginning of frame
 				nRandFlag = 1;
 			}

 			if (nRandFlag)
 			{
 				nNextBright = nRand > 15 ? 15 : nRand;
 			}
 		}
  }
}

void AD_Init(void)
{
  ADC_InitTypeDef ADC_InitStructure;

  // Initialize ADC 14MHz RC
  RCC_ADCCLKConfig(RCC_ADCCLK_HSI14);
  RCC_HSI14Cmd(ENABLE);
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
}

void Gpio_Init(void)
{
  GPIO_InitTypeDef InitStruct;
  GPIO_StructInit(&InitStruct);

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

  InitStruct.GPIO_Pin = LED_PIN;
  InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(LED_PORT, &InitStruct);
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

void StopMode(void)
{
  RTC_WriteAccess(true);
  RTC->ISR = RTC_ISR_INIT; // Enable init phase

  // vynulovat time registr
  RTC->TR = 0;
  RTC->CR |= (RTC_CR_ALRAE | RTC_CR_ALRAIE);
  RTC->ISR =~ RTC_ISR_INIT;
  RTC_WriteAccess(false);

  EXTI->PR = 0xFFFFFFFF;

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

  CRC->DR = 0xBADA55E5;
  uint32_t nValue = CRC->DR;

  // disable temperature sensor to save power
  ADC->CCR &= (uint32_t)(~ADC_CCR_TSEN);

  // ADC disable
  ADC1->CR |= (uint32_t)ADC_CR_ADDIS;

  RCC->APB2ENR &= ~RCC_APB2Periph_ADC1;
  RCC->AHBENR &= ~RCC_AHBENR_CRCEN;

  return nValue;
}

void RTC_IRQHandler(void)
{
  RTC_WriteAccess(true);
  RTC->CR &= ~(RTC_CR_ALRAE | RTC_CR_ALRAIE);
  RTC->ISR = (uint32_t)((uint32_t)(~((RTC_ISR_ALRAF | RTC_ISR_INIT)& 0x0001FFFF) | (uint32_t)(RTC->ISR & RTC_ISR_INIT)));
  RTC_WriteAccess(false);
//  RTC->ISR |= RTC_ISR_ALRAF;
}
