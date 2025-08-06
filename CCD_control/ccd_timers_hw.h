#ifndef _TCD1304_TIMERS_H_
#define _TCD1304_TIMERS_H_

#define TIM_EnableCounter(TIMx) SET_BIT(TIMx->CR1, TIM_CR1_CEN)
#define TIM_DisableCounter(TIMx) CLEAR_BIT(TIMx->CR1, TIM_CR1_CEN)

#include <stdint.h>

void ccd_timers_init(void);
void ccd_timers_start(void);

void ccd_tim_icg(uint16_t *buf, uint16_t size);
void ccd_tim_sh(uint16_t *buf, uint16_t size);

#endif /* _TCD1304_TIMERS_H_ */