///      Change the text below to suit your controller
///------------------------------------------------------------------------------
///     ccd_Sync_init -- TIM16 -- PA6 (CH4) connect to PA9 (TIM1 -- TI2FP2)
///
///  -------- C C D     S y g n a l s  -----------------------------------------
///
///  ICG --- TIM1(CH1) - PA8. TIM1 - OnePulseMode. TIM1 - Master for TIM2 & TIM3
///    
///  SH  --- TIM2(CH1) - PA5. TIM2 - OPM. Master for TIM4 (ITR1) ??? 
///  
///  CLK --- TIM3(CH4) - PB1. TIM3 - ResetMode ITR0. Master for  ???
///  
///  ADC --- TIM4     - PA11
///
/// ------------------- P I N S ------------------------------------------------
///                             |       |             S P I 
///    Sync   ICG    SH    CLK  |  ADC  |   MOSI   MISO   SCK    CS 
///    PA6    PA8    PA5   PB1  |  PA0  |   PB5    PB4    PB3   PA15
///                             |       |     
///------------------------------------------------------------------------------

#ifdef STM32F3
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_ll_tim.h"
#endif

#include "ccd_timers_hw.h"

//------------------------------------------------------------------------------

static void ccd_Sync_init(void);
static void ccd_ICG_init(void);
static void ccd_SH_init(void);
static void ccd_CLK_init(void);

//------------------------------------------------------------------------------
//  ALL TIMERS INIT/START
//------------------------------------------------------------------------------

void ccd_timers_init(void)
{
  ccd_Sync_init();
  ccd_ICG_init();
  ccd_SH_init();
  ccd_CLK_init();
}

void ccd_timers_start(void)
{
  //LL_TIM_EnableCounter(TIM1);
  //LL_TIM_EnableCounter(TIM2);
  LL_TIM_EnableCounter(TIM3); // Clk Tim
  
  LL_TIM_EnableCounter(TIM16); // 
}

//------------------------------------------------------------------------------
//  TIMERS INIT FUNCTIONS
//------------------------------------------------------------------------------
//  Master Clock -- TIM16
//------------------------------------------------------------------------------

static void ccd_Sync_init(void)
{
  //--- TIM 16 INIT ---//
  
  //--- Peripheral clock enable ---//
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM16);

  //--- TIM16 interrupt Init ---//
  //NVIC_SetPriority(TIM1_UP_TIM16_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  //NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  
  //--- Base Init TIM ---//
  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  TIM_InitStruct.Prescaler = 64 - 1;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 21000 - 1;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0; // RCR reg
  LL_TIM_Init(TIM16, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM16);
  
  //--- PWM OC (CH1) -- MASTER ---//
  LL_TIM_OC_EnablePreload(TIM16, LL_TIM_CHANNEL_CH1); /// in CCMRx set bit OCxPE
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 10000 - 1;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM16, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM16, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM16, LL_TIM_CHANNEL_CH1);  //  Enable channel 1
  //TIM16->CCER |= TIM_CCER_CC1E;   /// Enable Ch1 (CC1E) /// | TIM_CCER_CC1P
  
  //---  BDTR  ---//
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH; // BKP = 1
  TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_ENABLE; // MOE = 1
  LL_TIM_BDTR_Init(TIM16, &TIM_BDTRInitStruct);
  
  //--- TIM16 PA6 OUTPUT Configuration ---//  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1; /// PA6 -> TIM16_CH1 -- AF1 - P.44 DT
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  SET_BIT(TIM16->EGR, TIM_EGR_UG);
}

//------------------------------------------------------------------------------
// ICG -- TIM1
//------------------------------------------------------------------------------

static void ccd_ICG_init(void)
{
  //--- TIM 1 INIT ---//
  
  //--- Peripheral clock enable ---//
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
  //--- TIM1 interrupt Init ---//
  //NVIC_SetPriority(TIM1_UP_TIM16_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  //NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

  //--- Init TIM ---//
  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  TIM_InitStruct.Prescaler = 3;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 198;                           // Pulse width time + dead time
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
    
  //--- PWM OC (CH1) -- ICG sygnal ---//
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 4;                       // Dead time
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);  // Enable channel 1
  
  //--- TI2FP2 (CH2)  ??? OC4---//
  LL_TIM_SetOnePulseMode(TIM1, LL_TIM_ONEPULSEMODE_SINGLE);
  LL_TIM_SetTriggerInput(TIM1, LL_TIM_TS_TI2FP2);
  LL_TIM_SetSlaveMode(TIM1, LL_TIM_SLAVEMODE_TRIGGER);
  LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_IC_SetFilter(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
  LL_TIM_DisableIT_TRIG(TIM1);
  LL_TIM_DisableDMAReq_TRIG(TIM1);
  
  //-- MASTER --///
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_ENABLE);
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_OC4_RISINGFALLING); // ??
  LL_TIM_EnableMasterSlaveMode(TIM1);
  
  //---  BDTR  ---//
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
  TIM_BDTRInitStruct.Break2State = LL_TIM_BREAK2_DISABLE;
  TIM_BDTRInitStruct.Break2Polarity = LL_TIM_BREAK2_POLARITY_HIGH;
  TIM_BDTRInitStruct.Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_ENABLE; // MOE = 1  -- no - LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);

  //---- GPIO CONFIG ----// 
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  //--- PA9 -- TI2FP2 (CH2) <-- Trigger mode Input---//
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  //--- PA8 -- ICG Sygnal --- CH1(TIM1)---//
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  SET_BIT(TIM1->EGR, TIM_EGR_UG);
}

//------------------------------------------------------------------------------
// SH -- TIM2
//------------------------------------------------------------------------------

static void ccd_SH_init(void)
{
  //--- TIM 2 INIT ---//
  
  //--- Peripheral clock enable ---//
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  //--- Init TIM ---//
  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  TIM_InitStruct.Prescaler = 3;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 110;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  
  //--- PWM OC (CH1) -- SH sygnal ---//
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 6;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_LOW;
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);  // Enable channel 1
  
  //--- OC (CH2) -- Master for TIM4 (ADC) ---//
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.CompareValue = 107;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2);  // Enable channel 2
  
  //--- Slave Settings -- ITR0 ---//
  LL_TIM_SetOnePulseMode(TIM2, LL_TIM_ONEPULSEMODE_SINGLE);
  LL_TIM_SetTriggerInput(TIM2, LL_TIM_TS_ITR0);
  LL_TIM_SetSlaveMode(TIM2, LL_TIM_SLAVEMODE_TRIGGER);
  LL_TIM_DisableIT_TRIG(TIM2);
  LL_TIM_DisableDMAReq_TRIG(TIM2);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_OC2REF);
  LL_TIM_EnableMasterSlaveMode(TIM2);

  //---- GPIO CONFIG ----// 
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  //--- PA5 -- SH Sygnal --- CH1(TIM2)---//
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  SET_BIT(TIM2->EGR, TIM_EGR_UG);
}

//------------------------------------------------------------------------------
// CLK -- TIM3
//------------------------------------------------------------------------------

static void ccd_CLK_init(void)
{
  //--- TIM 3 INIT ---//

  //--- Peripheral clock enable ---//
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  //--- Init TIM ---//
  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  TIM_InitStruct.Prescaler = 3;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 9-1;     ////----------------------------------- 12-1
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  
  //--- PWM OC (CH4) -- CLK ---//
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH4);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 5-1;   ////-------------------------------- 7-1
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_LOW;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_EnableFast(TIM3, LL_TIM_CHANNEL_CH4);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);  // Enable channel 4
  
  //--- Slave Settings -- ITR0 ---//
  LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR0);
  LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_RESET);
  LL_TIM_DisableIT_TRIG(TIM3);
  LL_TIM_DisableDMAReq_TRIG(TIM3);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_EnableMasterSlaveMode(TIM3);

  //---- GPIO CONFIG ----//
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  //--- PB1 -- CLK --- CH4(TIM3)---//
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  SET_BIT(TIM3->EGR, TIM_EGR_UG);
}


//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------


void ccd_tim_icg(uint16_t *buf, uint16_t size)
{
  if (size < 1) return;
  
  uint16_t cmd = buf[0];
  uint32_t val = ((uint32_t) buf[1]) & 0x0000FFFF;
  
  switch (cmd)
  {
  case 1:
    LL_TIM_OC_SetCompareCH1(TIM1, val); // ICG_SYG (4)
    break;
  case 2:
    LL_TIM_SetAutoReload(TIM1, val); // init val = 198
    break;
  default:
    break;
  }
}

//------------------------------------------------------------------------------

void ccd_tim_sh(uint16_t *buf, uint16_t size)
{
  if (size < 1) return;
  
  uint16_t cmd = buf[0];
  uint32_t val = ((uint32_t) buf[1]) & 0x0000FFFF;
  
  switch (cmd)
  {
  case 1:
    LL_TIM_OC_SetCompareCH1(TIM2, val); // SH_SYG (6)
    break;
  case 2:
    LL_TIM_OC_SetCompareCH2(TIM2, val); // Master for TIM4 (ADC) (107)
    break;
  case 3:
    LL_TIM_SetAutoReload(TIM2, val); // init val = 110
    break;
  default:
    break;
  }
}





//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------



























