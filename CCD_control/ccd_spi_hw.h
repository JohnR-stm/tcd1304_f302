#ifndef _TCD1304_SPI_H_
#define _TCD1304_SPI_H_
#include <stdint.h>

void ccd_spi_init(void);

void ccd_send_SPI_buf (uint32_t * addr, uint32_t data_len);

#define CCD 3650

#endif /* _TCD1304_SPI_H_ */