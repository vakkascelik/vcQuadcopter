 /*************************************************************************
 *    History :
 *    1. Date        : January 12, 2017
 *       Author      : Vakkas Celik
 *       Description : Create
 *
 **************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "includes.h"

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure2;
TIM_ICInitTypeDef  TIM_ICInitStructure;
NVIC_InitTypeDef NVIC_InitStructure;

uint16_t IC1Value 	= 0;
uint16_t IC2Value 	= 0;
uint16_t IC3Value 	= 0;
uint16_t IC4Value 	= 0;
uint16_t DutyCycle1     = 0;
uint32_t Frequency1     = 0;
uint16_t DutyCycle2     = 0;
uint32_t Frequency2     = 0;
uint16_t DutyCycle3     = 0;
uint32_t Frequency3     = 0;
uint16_t DutyCycle4     = 0;
uint32_t Frequency4     = 0;

void initReceiverPWM();

void TIM_2_4_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  /* GPIOA and GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOD, ENABLE);
  
  /* GPIOD Configuration: TIM4 CH1 (PD12) and TIM4 CH3 (PD14) and TIM4 CH3 (PBx) and TIM4 CH4 (PBx) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_14;// | GPIO_Pin_x | GPIO_Pin_x ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
  
  /* GPIOA Configuration: TIM2 CH1 (PA15) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  
  /* GPIOA Configuration: TIM2 CH3 (PB10) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Connect TIM3 pins to AF2 */  
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4); 
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_TIM2); 
}

void initReceiverPWM()
{
  /* TIM Configuration */
  TIM_2_4_Config();
  
  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

   /* --------------------------------------------------------------------------- 
    TIM2 configuration: PWM Input mode
     The external signal is connected to TIM2 CH2 pin (PB.03)
     TIM2 CCR2 is used to compute the frequency value 
     TIM2 CCR1 is used to compute the duty cycle value

    In this example TIM2 input clock (TIM2CLK) is set to APB1 clock (PCLK1), since
    APB1 prescaler is set to 1.
      TIM2CLK = PCLK1 = HCLK = SystemCoreClock

    External Signal Frequency = SystemCoreClock / TIM2_CCR2 in Hz.
    External Signal DutyCycle = (TIM2_CCR1*100)/(TIM2_CCR2) in %.
  Note: 
  SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f0xx.c file.
  Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
  function to update SystemCoreClock variable value. Otherwise, any configuration
  based on this variable will be incorrect.
  --------------------------------------------------------------------------- */
  /* Time base configuration */
  TIM_TimeBaseStructure2.TIM_Period = 65535;//down to 200hz
  TIM_TimeBaseStructure2.TIM_Prescaler = 83;//84-1
  TIM_TimeBaseStructure2.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure2.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure2);
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x00;
  TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_PWMIConfig3(TIM4, &TIM_ICInitStructure);

  /* Select the TIM4 Input Trigger: TI1FP1 */
  TIM_SelectInputTrigger(TIM4, TIM_TS_TI1FP1);

  /* Select the slave Mode: Reset Mode */
  TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM4,TIM_MasterSlaveMode_Enable);

  /* Enable the CC1,CC2 Interrupt Request */
   TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);
   TIM_ITConfig(TIM4, TIM_IT_CC3, ENABLE);
 
  /* TIM enable counter */
  TIM_Cmd(TIM4, ENABLE);  
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_PWMIConfig(TIM2, &TIM_ICInitStructure);
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_PWMIConfig3(TIM2, &TIM_ICInitStructure);

  /* Select the TIM2 Input Trigger: TI2FP2 */
  TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);

  /* Select the slave Mode: Reset Mode */
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM2,TIM_MasterSlaveMode_Enable);

  /* Enable the CC1,CC2,CC3,CC4 Interrupt Request */
   TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
   TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
  
    /* TIM enable counter */
  TIM_Cmd(TIM2, ENABLE);
}




void TIM4_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET) 
  { 
    // Clear TIM2 Capture compare interrupt pending bit 
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);

    // Get the Input Capture value
    IC1Value = TIM_GetCapture1(TIM4);

    if (IC1Value != 0)
    {
      // Duty cycle computation 
      DutyCycle1 = TIM_GetCapture2(TIM4);//(TIM_GetCapture2(TIM4) * 100) / IC1Value;

      // Frequency computation 
      Frequency1 = SystemCoreClock / IC1Value;
    }
    else
    {
      DutyCycle1 = 0;
      Frequency1 = 0;
    }
  }
  
  if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET) 
  { 
    // Clear TIM2 Capture compare interrupt pending bit 
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);

    // Get the Input Capture value
    IC3Value = TIM_GetCapture3(TIM4);

    if (IC3Value != 0)
    {
      // Duty cycle computation 
      DutyCycle2 = TIM_GetCapture4(TIM4);//(IC4Value * 100) / IC3Value;

      // Frequency computation 
      Frequency2 = SystemCoreClock / IC3Value;
    }
    else
    {
      DutyCycle2 = 0;
      Frequency2 = 0;
    }
  }
}

void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) 
  { 
    // Clear TIM2 Capture compare interrupt pending bit 
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

    // Get the Input Capture value
    IC3Value = TIM_GetCapture1(TIM2);

    if (IC3Value != 0)
    {
      // Duty cycle computation 
      DutyCycle3 = (TIM_GetCapture2(TIM2) * 100) / IC3Value;

      // Frequency computation 
      Frequency3 = SystemCoreClock / IC3Value;
    }
    else
    {
      DutyCycle3 = 0;
      Frequency3 = 0;
    }
  }
  
  if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET) 
  { 
    // Clear TIM2 Capture compare interrupt pending bit 
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);

    // Get the Input Capture value
    IC4Value = TIM_GetCapture3(TIM2);

    if (IC4Value != 0)
    {
      // Duty cycle computation 
      DutyCycle4 = (TIM_GetCapture4(TIM2) * 100) / IC4Value;

      // Frequency computation 
      Frequency4 = SystemCoreClock / IC4Value;
    }
    else
    {
      DutyCycle4 = 0;
      Frequency4 = 0;
    }
  }

}