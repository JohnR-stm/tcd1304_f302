#include "ccd_cmd_table.h"
#include "ccd_cmd_proc.h"

/// INCLUDES PROCESSING FUNCTIONS

#include "ccd_timers_hw.h"
#include "ccd_adc_hw.h"

//-----------------------------------------------------------

static void NoOp(uint16_t *data, uint16_t len);

//-----------------------------------------------------------

command_handler command_table[CMD_THRESHOLD] = {
    NoOp,               // no operations
    ccd_tim_icg,     // from ccd_timers.c
    ccd_tim_sh   // from ccd_adc.c

};


//-------------------------------------------------------------

static void NoOp(uint16_t *data, uint16_t len) {
    // no operations
}

