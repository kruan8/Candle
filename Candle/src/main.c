/*
 *
 * Created on: 12.10.2018
 *    Author: Priesol Vladimir
 *
**********************************************************************/
/*
 * Spotreba:
 * SLEEP HSI 8MHz, HPRE DIV16, HSI14 off -> 0,9mA
 * SLEEP HSI 8MHz,  HPRE DIV1, HSI14 off -> 1,3mA
 *
 *
 */
#include "stm32f0xx.h"
#include "timer.h"
#include "app.h"
#include <stdbool.h>




int main(void)
{
  // po resetu bezi HSI na 8MHz

  // SYSCLK = 8MHz/DIV
//  RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV16;

  // kontrola hodin
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks); // Get system clocks

  App_Init();

  App_Exec();
}

