
#ifdef STM32F3
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_gpio.h"
//#include "stm32f3xx_ll_tim.h"
//#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_dac.h"
#endif

//#include "ccd_spi_hw.h"
#include "ccd_dac_hw.h"

//------------------------------------------------------------------------------

#define         DAC_VALUE            ((uint32_t)1000)

//------------------------------------------------------------------------------

static void ccd_dac_init(void);

//------------------------------------------------------------------------------

void ccd_dac_mdl_init(void)
{
  ccd_dac_init();
}

//------------------------------------------------------------------------------

void ccd_dac_val(uint32_t Data)
{
  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, Data);
}

//------------------------------------------------------------------------------

void ccd_dac_enable(_Bool val)
{
  if (val)
  {
    LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);
  }
  else
    LL_DAC_Disable(DAC1, LL_DAC_CHANNEL_1);

}

//------------------------------------------------------------------------------


static void ccd_dac_init(void)
{
  
  ///----- PIN INIT (PA4) ------
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  ///------ DAC INIT -----

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1);
  
  LL_DAC_InitTypeDef DAC_InitStruct = {0};
  DAC_InitStruct.TriggerSource = LL_DAC_TRIG_SOFTWARE;
  DAC_InitStruct.WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE;
  DAC_InitStruct.OutputBuffer = LL_DAC_OUTPUT_BUFFER_ENABLE;
  LL_DAC_Init(DAC, LL_DAC_CHANNEL_1, &DAC_InitStruct);
  LL_DAC_DisableTrigger(DAC, LL_DAC_CHANNEL_1);
  
  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, 2048);
}


