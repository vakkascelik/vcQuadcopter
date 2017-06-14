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
uint16_t Throttle       = 0;      // DutyCycle1
uint32_t Yaw            = 0;      // DutyCycle2
uint32_t Pitch          = 0;      // DutyCycle3
uint32_t Roll           = 0;      // DutyCycle4
uint32_t Frequency     = 0;

void initReceiverPWM();

void TIM_2_4_5_12_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM2 and TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* GPIOA and GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOD, ENABLE);
  
  /* GPIOD Configuration: TIM4 CH1 (PD12)  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;// | GPIO_Pin_x | GPIO_Pin_x ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
  
  /* GPIOA Configuration: TIM2 CH1 (PA15) and TIM5 CH1 (PA0) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_0 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  
  /* GPIOA Configuration: TIM12 CH1 (PB14) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Connect TIM pins to AF2 */  
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM4); 
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM2); 
}

void initReceiverPWM()
{
  /* TIM Configuration */
  TIM_2_4_5_12_Config();
  
  /* Enable the TIM2, TIM4, TIM5, TIM12 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM8_BRK_TIM12_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);


  /* Time base configuration */
  TIM_TimeBaseStructure2.TIM_Period = 65535;//down to 200hz
  TIM_TimeBaseStructure2.TIM_Prescaler = 83;//84-1
  TIM_TimeBaseStructure2.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure2.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure2);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure2);
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure2);
  TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure2);
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x00;
  
  TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);  
  TIM_SelectInputTrigger(TIM4, TIM_TS_TI1FP1);                  // Select the TIM4 Input Trigger: TI1FP1 
  TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);               // Select the slave Mode: Reset Mode 
  TIM_SelectMasterSlaveMode(TIM4,TIM_MasterSlaveMode_Enable); 
  TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);                      // Enable the CC1,CC2 Interrupt Request 
  TIM_Cmd(TIM4, ENABLE);                                        // TIM enable counter
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_PWMIConfig(TIM2, &TIM_ICInitStructure);  
  TIM_SelectInputTrigger(TIM2, TIM_TS_TI1FP1);                  // Select the TIM4 Input Trigger: TI1FP1 
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);               // Select the slave Mode: Reset Mode 
  TIM_SelectMasterSlaveMode(TIM2,TIM_MasterSlaveMode_Enable); 
  TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);                      // Enable the CC1,CC2 Interrupt Request 
  TIM_Cmd(TIM2, ENABLE);                                        // TIM enable counter
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_PWMIConfig(TIM5, &TIM_ICInitStructure);  
  TIM_SelectInputTrigger(TIM5, TIM_TS_TI1FP1);                  // Select the TIM4 Input Trigger: TI1FP1 
  TIM_SelectSlaveMode(TIM5, TIM_SlaveMode_Reset);               // Select the slave Mode: Reset Mode 
  TIM_SelectMasterSlaveMode(TIM5,TIM_MasterSlaveMode_Enable); 
  TIM_ITConfig(TIM5, TIM_IT_CC1, ENABLE);                      // Enable the CC1,CC2 Interrupt Request 
  TIM_Cmd(TIM5, ENABLE);    
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_PWMIConfig(TIM12, &TIM_ICInitStructure);  
  TIM_SelectInputTrigger(TIM12, TIM_TS_TI1FP1);                  // Select the TIM4 Input Trigger: TI1FP1 
  TIM_SelectSlaveMode(TIM12, TIM_SlaveMode_Reset);               // Select the slave Mode: Reset Mode 
  TIM_SelectMasterSlaveMode(TIM12,TIM_MasterSlaveMode_Enable); 
  TIM_ITConfig(TIM12, TIM_IT_CC1, ENABLE);                      // Enable the CC1,CC2 Interrupt Request 
  TIM_Cmd(TIM12, ENABLE);    
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
      Throttle = TIM_GetCapture2(TIM4);//(TIM_GetCapture2(TIM4) * 100) / IC1Value;

      // Frequency computation 
      Frequency = SystemCoreClock / IC1Value;
    }
    else
    {
      Throttle = 0;
      Frequency = 0;
    }
  }
}

void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) 
  { 
    // Clear TIM2 Capture compare interrupt pending bit 
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

    if (TIM_GetCapture1(TIM2) != 0)
    {
      // Duty cycle computation 
      Roll = TIM_GetCapture2(TIM2);// * 100) / IC3Value;
    }
    else
    {
      Roll = 0;
    }
  }
}


void TIM5_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET) 
  { 
    // Clear TIM2 Capture compare interrupt pending bit 
    TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);

    if (TIM_GetCapture1(TIM5) != 0)
    {
      // Duty cycle computation 
      Yaw = TIM_GetCapture2(TIM5);// * 100) / IC3Value;
    }
    else
    {
      Yaw = 0;
    }
  }
}


void TIM12_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM12, TIM_IT_CC1) != RESET) 
  { 
    // Clear TIM2 Capture compare interrupt pending bit 
    TIM_ClearITPendingBit(TIM12, TIM_IT_CC1);

    if (TIM_GetCapture1(TIM12) != 0)
    {
      // Duty cycle computation 
      Pitch = TIM_GetCapture2(TIM12);// * 100) / IC3Value;
    }
    else
    {
      Pitch = 0;
    }
  }
}