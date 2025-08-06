#include "ccd_cmd_proc.h"

//----------------------------------------------------------------------------

static uint16_t Buf_cmd[CMD_SZ_BUF] = {0};
static uint16_t cnt = 0;
static uint16_t sz = 0;

//----------------------------------------------------------------------------

extern command_handler command_table[];

//----------------------------------------------------------------------------
//
//----------------------------------------------------------------------------

void cmd_reset_buf(void)
{
  cnt = 0;
  for (uint8_t a = 0; a < CMD_SZ_BUF; a++)
    Buf_cmd[a] = 0;
}

//----------------------------------------------------------------------------

uint8_t cmd_buf_create(uint16_t data)
{
  if(cnt > 0)
  {
    cnt--;
    Buf_cmd[sz - cnt] = data;
    if (cnt == 0) 
      return 3;         /// cmd Buf is full
    return 2;           /// store Buf data - OK
  }
  else 
  {
    if((data & CMD_KEY_MSK) == CMD_KEY)
    {
      cmd_reset_buf();
      cnt = (data & CMD_SZ_MSK) ;
      if (cnt > CMD_SZ_BUF)
      {
        cnt = 0;
        return 0;       /// error
      } 
      sz = cnt - 1;
      return 1;         /// calc Buf size - OK
    }
    else return 0;      /// error
  } 
}

//----------------------------------------------------------------------------

uint8_t cmd_buf_processing(void) 
{
  uint16_t command = Buf_cmd[0];
  if (command >= CMD_OFFSET)
  {
    command = command - CMD_OFFSET;
    if (command < CMD_THRESHOLD)
    {
      command_handler handler = command_table[command]; /// extern
      handler(&Buf_cmd[1], sz);
      return 1;
    }
    else return 0;
  }
  else return 0;                /// error
}

//----------------------------------------------------------------------------
//
//----------------------------------------------------------------------------




