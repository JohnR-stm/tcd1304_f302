#ifndef _TCD1304_SPI_H_
#define _TCD1304_SPI_H_
#include <stdint.h>

void ccd_spi_init(void);

void ccd_send_SPI_buf (uint32_t * addr, uint32_t data_len);

void ccd_spi_enable_interrupts(void);
void ccd_spi_clear_err(void);
void spi_nss_soft(uint8_t cond);

#define CCD             3650

#define Buf_SZ          10

#endif /* _TCD1304_SPI_H_ */