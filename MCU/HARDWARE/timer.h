#ifndef __TIMER_H
#define __TIMER_H
#include "system.h"
void TIM8_Cap_Init(u16 arr, u16 psc);
void TIM12_SERVO_Init(u16 arr,u16 psc);
void TIM8_SERVO_Init(u16 arr,u16 psc);

extern int L_Remoter_Ch1,L_Remoter_Ch2,L_Remoter_Ch3,L_Remoter_Ch4;
extern int Remoter_Ch1,Remoter_Ch2,Remoter_Ch3,Remoter_Ch4;

#endif
