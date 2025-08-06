#ifndef _CCD_CMD_PROC_
#define _CCD_CMD_PROC_

#include <stdint.h>

#define CMD_SZ_MSK              ((uint16_t) 0x003F)
#define CMD_KEY_MSK             ((uint16_t) 0xFF00)
#define CMD_KEY                 ((uint16_t) 0xD500)
#define CMD_SZ_BUF              ((uint16_t) 0x0008)
#define CMD_THRESHOLD           ((uint16_t) 0x0003)
#define CMD_OFFSET              ((uint16_t) 0x6980)

typedef void (*command_handler)(uint16_t *buf, uint16_t size);
// extern from ccd_cmd_table.c
extern command_handler command_table[];

void cmd_reset_buf(void);
uint8_t cmd_buf_create(uint16_t data);
uint8_t cmd_buf_processing(void);


#endif /* _CCD_CMD_PROC_ */