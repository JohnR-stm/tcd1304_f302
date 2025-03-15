///------------------------------------------------------------------------------
///
///
///
///
///
///
///
///------------------------------------------------------------------------------

#ifdef STM32F3
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_ll_tim.h"
#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_adc.h"
#endif

#include "ccd_spi_hw.h"
#include "ccd_adc_hw.h"

//------------------------------------------------------------------------------




volatile uint16_t Buf_ADC[CCD] = {0};


volatile uint8_t Counter = 0;
extern uint8_t flag_spi;

extern uint16_t Buf_SPI[CCD];

//------------------------------------------------------------------------------


static void ccd_tim_adc_init(void);
static void ccd_adc_init(void);
static void ccd_dma_adc_init(void);


//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

void ccd_adc_mdl_init(void)
{
  ccd_adc_init();
  ccd_tim_adc_init();
  ccd_dma_adc_init();
  
  LL_ADC_REG_StartConversion(ADC1);
  LL_ADC_Enable(ADC1);
} 



static void ccd_tim_adc_init(void)
{
  //--- TIM 4 INIT ---//
  
  //--- Peripheral clock enable ---//
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

  //--- Base Init TIM ---//
  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  TIM_InitStruct.Prescaler = 3;                        //PSC ------------------
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 47;                      //ARR ------------------
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM4);
  LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_INTERNAL);
  
  //--- PWM OC (CH1) -- debug sygnal ---//
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 23;                  //Pulse ---------------
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);  // Enable channel 1
  
  //--- Slave Settings -- ITR1 (SH - Master)---//
  LL_TIM_SetTriggerInput(TIM4, LL_TIM_TS_ITR1);
  LL_TIM_SetSlaveMode(TIM4, LL_TIM_SLAVEMODE_TRIGGER);  //SMCR
  LL_TIM_DisableIT_TRIG(TIM4); // Trigger interrupt disable
  LL_TIM_DisableDMAReq_TRIG(TIM4); // DMA Request disable 
  //--- Master settings - Disable ---///
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_UPDATE);
  LL_TIM_DisableMasterSlaveMode(TIM4);
  
    
#ifdef DEBUG_CCD
  //--- PA11 -- Debug Sygnal --- TIM4_CH1---//
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_10;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif
  
  SET_BIT(TIM4->EGR, TIM_EGR_UG);
}




static void ccd_adc_init(void)
{
  //--- PA0 -- ADC_Input --- ADC1_IN1---//
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  //--- ADC clock enable ---//
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC12);
  
  //--- DMA ENABLE ---//
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

  //--- Base Init ADC ---//
  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;  //or left
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  
  //---  Regular Channel Settings ---//
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM4_TRGO;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;     // Scan Conversion Mode
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE; // Discontinuous Conversion Mode (only if Scan Mode is enabled)
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;           // End Of Conversion Selection
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;   // DMA Continuous Requests (DMACFG + DMAEN)
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;         //  Overrun behaviour
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  
  //--- Common Settings ---//
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);         // External Trigger Conversion Edge

  //---  Internal Regulator ---//
  LL_ADC_EnableInternalRegulator(ADC1);
  /*Delay for ADC internal voltage regulator stabilization. 
    Compute number of CPU cycles to wait for, from delay in us. 
    Note: Variable divided by 2 to compensate partially 
    CPU processing cycles (depends on compilation optimization). 
    Note: If system core clock frequency is below 200kHz, wait time 
    is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  //--- Configure Regular Channel ---//
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SINGLE_ENDED);
    //sConfig.OffsetNumber = ADC_OFFSET_NONE;
    //sConfig.Offset = 0;
}
  
  
static void ccd_dma_adc_init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  //LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

  /* DMA interrupt init */
  // DMA1_Channel1_IRQn interrupt configuration -- FOR ADC
  NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  
  //DMA2_Channel2_IRQn interrupt configuration  -- FOR SPI 
  //NVIC_SetPriority(DMA2_Channel2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  //NVIC_EnableIRQ(DMA2_Channel2_IRQn);
  
  uint32_t adc_addr = LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA);
  
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t) Buf_ADC);
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t) adc_addr);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, (uint32_t) (CCD - 2));
  
  
  DMA1_Channel1->CCR = 0;
  DMA1_Channel1->CCR |= (DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_PSIZE_0  | DMA_CCR_MSIZE_0);
  //DMA1_Channel1->CCR |= (DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_EN);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}

  
//===== D M A 1   H A N D L E R ================================================

void DMA1_Channel1_IRQHandler(void)
{
  if(LL_DMA_IsActiveFlag_TC1(DMA1))
  {
    LL_DMA_ClearFlag_TC1(DMA1);  /// Transfer complete flag
    LL_DMA_ClearFlag_HT1(DMA1);  /// half transfer complete flag
    LL_DMA_ClearFlag_GI1(DMA1);  /// Clear global interrupt flag
    
    TIM4->CR1 &= ~TIM_CR1_CEN;   /// Disable TIM4

    //---- Send to SPI ----///
    if (flag_spi == 1) Counter++;               // !!!!! New Change
    if(Counter >= 4)
    {
      Counter = 0;
      flag_spi = 0;                             // !!!!! New Change
      for (uint16_t cnt = 0; cnt < CCD; cnt++)
        Buf_SPI[cnt] = Buf_ADC[cnt];
      // ccd_send_SPI_buf ((uint32_t *)((void *)&Buf_SPI[0]), CCD); // !!!!! New Change
    }
    //---- END Send to SPI ----///
    
  }
  else
  {
    LL_DMA_ClearFlag_TC1(DMA1);  /// Transfer complete flag
    LL_DMA_ClearFlag_HT1(DMA1);  /// half transfer complete flag
    LL_DMA_ClearFlag_GI1(DMA1);  /// Clear global interrupt flag
    LL_DMA_ClearFlag_TE1(DMA1);
  }
  /*
  else if(LL_DMA_IsActiveFlag_TE3(DMA1))     /// if transfer error (TE) flag for channel 7 P.308 RM
  {
    LL_DMA_ClearFlag_TE3(DMA1);  /// Transfer error flag
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
  } */
}

//==============================================================================


