#ifndef __ENCODER_H
#define __ENCODER_H
#include <sys.h>	 
#include "system.h"

// No larger than 65535, because the timer of STM32F103 is 16 bit
//不可大于65535，因为STM32F103的定时器是16位的
#define ENCODER_TIM_PERIOD (u16)(65535)   

void Encoder_Init_TIM2(void);
void Encoder_Init_TIM3(void);
void Encoder_Init_TIM4(void);
void Encoder_Init_TIM5(void);


int Read_Encoder(u8 TIMX);

void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void TIM5_IRQHandler(void);
void TIM8_BRK_TIM12_IRQHandler(void);

#endif
