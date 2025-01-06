///------------------------------------------------------------------------------
///
///
///
///
///
///
///
///------------------------------------------------------------------------------

#include <stdint.h>

#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_spi.h"


#include "ccd_spi_hw.h"


//------------------------------------------------------------------------------

static void ccd_spi_pins_init(void);
static void ccd_spi_port_init(void);
static void ccd_spi_dma_init(void);


void ccd_set_SPI_addr (void);
void ccd_set_SPI_buf_len (uint32_t data_len);
void ccd_set_SPI_buf_addr (uint32_t * addr);

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

void ccd_spi_init(void)
{
  ccd_spi_pins_init();
  ccd_spi_port_init();
  ccd_spi_dma_init();
  
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

static void ccd_spi_pins_init(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  
  //--- SPI3 GPIO Configuration ---//
  //---- PB3 --> SPI3_SCK  ----//
  //---- PB4 --> SPI3_MISO ----//
  //---- PB5 --> SPI3_MOSI ----//
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct); 
}


//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

static void ccd_spi_port_init(void)
{
  //--- SPI 3 INIT ---//
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
  

  //--- SPI3  configuration ---//
  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;            // Data - 16 bit
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;      // ?? 8 Mhz
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI3, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI3, LL_SPI_PROTOCOL_MOTOROLA);           // CR2, SPI_CR2_FRF
  LL_SPI_EnableNSSPulseMgt(SPI3);                               // CR2, SPI_CR2_NSSP
  
  //--- DMA TX Enable ---//
  LL_SPI_EnableDMAReq_TX(SPI3);
  //--- SPI 3 Enable ---//
  LL_SPI_Enable(SPI3);
}


//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

static void ccd_spi_dma_init(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

  
  //LL_DMA_SetMemoryAddress(DMA2, LL_DMA_CHANNEL_2, (uint32_t) Buf_ADC);
  
  //LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_2, (uint32_t) (CCD - 2));
   
  
  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetChannelPriorityLevel(DMA2, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_MEDIUM);
  LL_DMA_SetMode(DMA2, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA2, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_HALFWORD);
  LL_DMA_SetMemorySize(DMA2, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_HALFWORD);


  
  //DMA1_Channel1->CCR = 0;
  //DMA1_Channel1->CCR |= (DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_PSIZE_0  | DMA_CCR_MSIZE_0);
  //DMA1_Channel1->CCR |= (DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_EN);
 
 
  
}

//------------------------------------------------------------------------------

inline void ccd_set_SPI_addr (void)
{
  uint32_t spi_addr;
  spi_addr = LL_SPI_DMA_GetRegAddr(SPI3);
  LL_DMA_SetPeriphAddress(DMA2, LL_DMA_CHANNEL_2, spi_addr);
}

//------------------------------------------------------------------------------

inline void ccd_set_SPI_buf_len (uint32_t data_len)
{
  LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_2, data_len);
}


//------------------------------------------------------------------------------

inline void ccd_set_SPI_buf_addr (uint32_t * addr)
{
  LL_DMA_SetMemoryAddress(DMA2, LL_DMA_CHANNEL_2, (uint32_t)addr);
}


//------------------------------------------------------------------------------

void ccd_send_SPI_buf (uint32_t * addr, uint32_t data_len)
{
  LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_2);
  
  ccd_set_SPI_addr ();
  ccd_set_SPI_buf_len (data_len);
  ccd_set_SPI_buf_addr (addr);
  
  LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_2);
}



//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------





//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------





//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------



